/* engine.c — frames in, records out. All orchestration, zero policy of its own: track/bank/agc/sched
 * hold the policy, this file moves data on the frame clock.
 *
 * Per-frame cost is ROI-PROPORTIONAL by design (research.md R5-measured): the dmabuf is write-combine
 * and a full-frame read costs ~4 ms, so the per-frame path touches only the tracked apertures
 * (16 slots × ~576 native px ≈ 9 KB). Full-field work (acquisition) is budgeted through sched.
 */
#include "engine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICK_US 50000u                 /* 20 Hz control tick, exact                                   */

struct Engine {
    BcnConfig cfg;
    Bank bank;
    Agc agc;
    Sched sched;
    EngineEmit emit;
    void *user;

    uint64_t t0_us;                    /* first frame's timestamp = the epoch                          */
    uint64_t epoch_us;                 /* chip-clock epoch (== t0)                                     */
    uint64_t frame_idx;
    uint64_t last_tick;                /* ticks completed                                              */
    uint32_t rec_seq;
    uint16_t frame_w, frame_h;
    uint8_t  roi_peak;                 /* brightest raw px across tracked ROIs since the last tick     */
    int32_t  roi[TRK_MAX_EXTENT * TRK_MAX_EXTENT];   /* scratch: one aperture                         */
};

int engine_open(Engine **out, const BcnConfig *cfg, EngineEmit emit, void *user,
                char *err, size_t err_len)
{
    Engine *e;
    unsigned w, h;
    if (cfg->n_chips != CORR_N) {
        snprintf(err, err_len, "engine: [code] n_chips = %u but the correlator is Gold-%d",
                 (unsigned)cfg->n_chips, CORR_N);
        return -1;
    }
    if (sscanf(cfg->camera_mode, "%ux%u", &w, &h) != 2) {
        snprintf(err, err_len, "engine: [camera] mode \"%s\" is not WxH", cfg->camera_mode);
        return -1;
    }
    e = calloc(1, sizeof *e);          /* the ONLY allocation; slots live inside Bank inside Engine    */
    if (!e) { snprintf(err, err_len, "engine: out of memory"); return -1; }
    e->cfg = *cfg;
    e->emit = emit;
    e->user = user;
    e->frame_w = (uint16_t)w;
    e->frame_h = (uint16_t)h;
    bank_init(&e->bank, &e->cfg);
    agc_init(&e->agc, &e->cfg);
    sched_init(&e->sched, &e->cfg, cfg->fps ? 1000000u / cfg->fps : 3617u);
    *out = e;
    return 0;
}

void engine_controls(const Engine *e, uint32_t *exposure_us, uint16_t *gain_q8)
{
    *exposure_us = e->agc.exposure_us;
    *gain_q8 = e->agc.gain_q8;
}
const Bank *engine_bank(const Engine *e) { return &e->bank; }

int engine_seed(Engine *e, uint8_t code_id, int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8)
{
    return bank_seed(&e->bank, &e->cfg, code_id, TRK_SCALE_MEDIUM, x_q8, y_q8,
                     chip_hz_q8, e->epoch_us, e->t0_us + e->frame_idx * 3617u);
}

/* Extract one aperture at scale factor f centred at plane px (cx,cy): each plane pixel is the sum of
 * f×f raw pixels, normalized by (exposure × gain) so AGC is transparent to the correlator (spec §4).
 * Out-of-frame pixels read as 0 — the mean removal in corr treats them as background. Also tracks the
 * raw peak (saturation + AGC input). */
static void extract(const FrameView *fv, int16_t cx, int16_t cy,
                    uint8_t extent, uint8_t f, int32_t *roi, uint8_t *raw_peak)
{
    /* norm: sample = rawsum * 2^14 / (exposure_us * gain_q8 / 256). Integer, exact, order fixed. */
    int64_t denom = ((int64_t)fv->exposure_us * fv->gain_q8) >> 8;
    int16_t half = (int16_t)(extent / 2u);
    int16_t px, py;
    uint8_t peak = *raw_peak;
    if (denom <= 0) denom = 1;

    for (py = 0; py < extent; py++) {
        for (px = 0; px < extent; px++) {
            int32_t nx0 = (int32_t)(cx - half + px) * f;
            int32_t ny0 = (int32_t)(cy - half + py) * f;
            int32_t sum = 0;
            int32_t dx, dy;
            if (nx0 >= 0 && ny0 >= 0 && nx0 + f <= fv->w && ny0 + f <= fv->h) {
                for (dy = 0; dy < f; dy++) {
                    const uint8_t *r = fv->data + (size_t)(ny0 + dy) * fv->stride + nx0;
                    for (dx = 0; dx < f; dx++) {
                        sum += r[dx];
                        if (r[dx] > peak) peak = r[dx];
                    }
                }
            }
            roi[py * extent + px] = (int32_t)(((int64_t)sum << 14) / denom);
        }
    }
    *raw_peak = peak;
}

static void tick(Engine *e, uint64_t tick_end_us)
{
    BcnRecord rec;
    int i;
    uint32_t dt_us = TICK_US;

    for (i = 0; i < e->bank.max_slots; i++) {
        BankSlot *s = &e->bank.slots[i];
        Track *t = &s->trk;
        if (!s->used) continue;
        track_tick(t, &e->cfg, tick_end_us, dt_us);
        t->t_int_chips = agc_integration(&e->agc, &e->cfg, t->q_q8, t->t_int_chips);
        t->saturated = 0;              /* re-detected each tick from raw peaks in extract()            */
    }
    bank_tick(&e->bank, &e->cfg);
    agc_exposure(&e->agc, &e->cfg, e->roi_peak);
    e->roi_peak = 0;

    bcn_record_init(&rec, e->cfg.build_id, e->cfg.config_hash);
    rec.t_us = tick_end_us;
    rec.seq = e->rec_seq++;
    rec.tick_index = (uint32_t)e->last_tick;
    rec.n_tracks = bank_emit(&e->bank, &e->cfg, &rec);
    rec.n_slots_used = bank_slots_used(&e->bank);
    rec.deadline_margin_us = 0;        /* default-ok: core is clockless (R3) — the LIVE app measures
                                        * and reports the deadline; the record stays deterministic so
                                        * replay parity is byte-exact. */
    /* inav_t_us / gps_time_ms stay 0: zero-when-absent (R10), same rule as the recorder */
    if (e->agc.settling_ticks) {
        uint8_t k;
        for (k = 0; k < rec.n_tracks; k++) rec.tracks[k].flags |= BCN_F_AGC_SETTLING;
    }
    e->emit(&rec, e->user);
}

void engine_frame(Engine *e, const FrameView *fv)
{
    int i;
    uint64_t tk;

    if (e->frame_idx == 0) {
        e->t0_us = fv->t_us;
        e->epoch_us = fv->t_us;
    }
    e->frame_idx++;

    /* per-slot aperture extraction + deposit */
    for (i = 0; i < e->bank.max_slots; i++) {
        BankSlot *s = &e->bank.slots[i];
        Track *t = &s->trk;
        int16_t cx, cy;
        uint8_t f;
        uint8_t peak = 0;
        if (!s->used || t->state == TRK_DEAD) continue;
        f = t->scale == TRK_SCALE_COARSE ? 4u : t->scale == TRK_SCALE_MEDIUM ? 2u : 1u;
        track_roi_center(t, &cx, &cy);
        extract(fv, cx, cy, t->extent, f, e->roi, &peak);
        if (peak >= 250u) t->saturated = 1;               /* spec §5: flat-top estimator engages      */
        if (peak > e->roi_peak) e->roi_peak = peak;
        track_frame(t, &e->cfg, e->roi, cx, cy, fv->t_us);
    }

    /* tick boundaries crossed by this frame's timestamp (usually 0 or 1; catches up after gaps) */
    tk = (fv->t_us - e->t0_us) / TICK_US;
    while (e->last_tick < tk) {
        e->last_tick++;
        tick(e, e->t0_us + e->last_tick * TICK_US);
    }
}

void engine_close(Engine *e) { free(e); }
