/* engine.c — frames in, records out. All orchestration, zero policy of its own: track/bank/agc/sched
 * hold the policy, this file moves data on the frame clock.
 *
 * Per-frame cost is ROI-PROPORTIONAL by design (research.md R5-measured): the dmabuf is write-combine
 * and a full-frame read costs ~4 ms, so the per-frame path touches only the tracked apertures
 * (16 slots × ~576 native px ≈ 9 KB). Full-field work (acquisition) is budgeted through sched.
 */
#include "engine.h"
#include "acquire.h"
#include "trail.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TICK_US 50000u                 /* 20 Hz control tick, exact                                   */

struct Engine {
    BcnConfig cfg;
    Bank bank;
    Agc agc;
    Sched sched;
    Acquire acq;
    Trail trail;
    EngineEmit emit;
    void *user;

    uint64_t t0_us;                    /* first frame's timestamp = the epoch                          */
    uint64_t epoch_us;                 /* chip-clock epoch (== t0)                                     */
    uint64_t frame_idx;
    uint64_t last_tick;                /* ticks completed                                              */
    uint32_t rec_seq;
    uint16_t frame_w, frame_h;
    uint8_t  roi_peak;                 /* brightest raw px across tracked ROIs since the last tick     */
    AcqSeed  pending[ACQ_MAX_SEEDS];   /* seeds found by an in-flight pass, applied at the model frame  */
    uint8_t  pending_n;
    uint64_t next_acq_frame;           /* pass throttle                                                */
    /* warm reacquire: the last CONFIRMED track's place and rate. A re-seed near it inherits the rate —
     * a track that dies for a non-rate reason must not send reacquisition on a rate tour. */
    int32_t  warm_x_q8, warm_y_q8;
    uint32_t warm_chip_hz_q8;
    uint8_t  warm_valid;
    /* ---- T083: the RECEIVER-GLOBAL chip clock, and the loop that measures it -----------------------
     * There is one emitter and one clock, so rate and phase are properties of the RECEIVER, not of a
     * track. That is why this lives here and not in Track: a per-track loop would estimate the same
     * number several times from less evidence each, and new seeds would start from the stale config
     * value while established tracks had moved on. (It is also the groundwork T081 needs, for the same
     * reason.)
     *
     * THE DISCRIMINATOR is the one measured offline on three clips across two days: the tracked
     * chip_phase, unwrapped, is a clean linear ramp whose slope IS the frequency error, with a residual
     * of 0.27-0.36 chip. Accumulating wrapped phase steps over a few seconds gives a drift in chips/s,
     * and chips/s IS Hz. Measured drift at the old nominal was 0.718-0.792 chip/s.
     *
     * WHY IT IS SLOW AND DEADBANDED. The two previous attempts ran per-tick off a noisy statistic and
     * diverged. Over RATE_WIN_US the true drift is large against the 0.3-chip noise (0.72 chip/s x 5 s
     * = 3.6 chips), so a windowed estimate has ample SNR while a per-tick one has none. */
    uint32_t chip_hz_q8;            /* the receiver's clock — seeds and retunes both use THIS          */
    int32_t  rate_acc_chips;        /* accumulated wrapped phase steps since the window opened         */
    uint64_t rate_t0_us;
    uint16_t rate_obs;
    uint8_t  rate_prev_phase;
    uint8_t  rate_prev_valid;
    uint32_t rate_corrections;
    int32_t  roi[TRK_MAX_EXTENT * TRK_MAX_EXTENT];   /* scratch: one aperture                         */
    uint8_t  field_on;                               /* debug field map requested (engine_field_enable) */
    uint8_t  field_valid;
    uint8_t  field[BCN_FIELD_W * BCN_FIELD_H];       /* coarse point-source contrast, for the scope     */
    uint8_t  preview_on;                             /* graphical preview requested (engine_preview_*)  */
    uint8_t  preview_valid;
    uint8_t  preview[BCN_PREVIEW_MAX_W * BCN_PREVIEW_MAX_H];  /* 2x2 max-pooled frame, for the viewer  */
};

/* Coarse point-source contrast for the operator's scope. Per cell: max - mean, clamped to 0..255.
 * Cheap and deliberately dumb -- it is a viewfinder, not a detector, and nothing in the tracker reads
 * it. Cost is one pass over the frame, charged only on ticks and only when enabled. */
static void field_update(Engine *e, const FrameView *fv)
{
    uint16_t cy, cx;
    if (!fv->w || !fv->h) return;
    for (cy = 0; cy < BCN_FIELD_H; cy++) {
        uint32_t y0 = (uint32_t)cy * fv->h / BCN_FIELD_H;
        uint32_t y1 = (uint32_t)(cy + 1) * fv->h / BCN_FIELD_H;
        for (cx = 0; cx < BCN_FIELD_W; cx++) {
            uint32_t x0 = (uint32_t)cx * fv->w / BCN_FIELD_W;
            uint32_t x1 = (uint32_t)(cx + 1) * fv->w / BCN_FIELD_W;
            uint32_t sum = 0, n = 0, y, x;
            uint8_t mx = 0;
            int32_t c;
            for (y = y0; y < y1; y++) {
                const uint8_t *r = fv->data + (size_t)y * fv->stride;
                for (x = x0; x < x1; x++) {
                    if (r[x] > mx) mx = r[x];
                    sum += r[x];
                    n++;
                }
            }
            c = n ? (int32_t)mx - (int32_t)(sum / n) : 0;
            if (c < 0) c = 0;
            if (c > 255) c = 255;
            e->field[cy * BCN_FIELD_W + cx] = (uint8_t)c;
        }
    }
    e->field_valid = 1u;
}

void engine_field_enable(Engine *e, int on) { e->field_on = on ? 1u : 0u; }
const uint8_t *engine_field_map(const Engine *e)
{
    return (e->field_on && e->field_valid) ? e->field : NULL;
}

/* 2x2 MAX pool of the frame. Max, not mean: the beacon is a few saturated pixels, and averaging it with
 * three dark neighbours quarters it -- it would fade out of the display exactly when it matters most.
 * Same tick-gated cost model as field_update(). */
static void preview_update(Engine *e, const FrameView *fv)
{
    const unsigned d = BCN_PREVIEW_DIV;
    unsigned pw = fv->w / d, ph = fv->h / d, py, px;
    if (!pw || !ph || pw > BCN_PREVIEW_MAX_W || ph > BCN_PREVIEW_MAX_H) return;
    for (py = 0; py < ph; py++) {
        const uint8_t *r0 = fv->data + (size_t)(py * d) * fv->stride;
        const uint8_t *r1 = r0 + fv->stride;
        uint8_t *o = e->preview + (size_t)py * pw;
        for (px = 0; px < pw; px++) {
            unsigned sx = px * d;
            uint8_t a = r0[sx], b = r0[sx + 1], c = r1[sx], g = r1[sx + 1];
            uint8_t m = a > b ? a : b;
            uint8_t n = c > g ? c : g;
            o[px] = m > n ? m : n;
        }
    }
    e->preview_valid = 1u;
}

int engine_preview_enable(Engine *e, int on)
{
    if (on && ((unsigned)e->frame_w / BCN_PREVIEW_DIV > BCN_PREVIEW_MAX_W ||
               (unsigned)e->frame_h / BCN_PREVIEW_DIV > BCN_PREVIEW_MAX_H))
        return -1;                       /* fail loud rather than silently rescaling the viewfinder */
    e->preview_on = on ? 1u : 0u;
    return 0;
}

const uint8_t *engine_preview_map(const Engine *e, unsigned *w, unsigned *h)
{
    if (!e->preview_on || !e->preview_valid) return NULL;
    if (w) *w = (unsigned)e->frame_w / BCN_PREVIEW_DIV;
    if (h) *h = (unsigned)e->frame_h / BCN_PREVIEW_DIV;
    return e->preview;
}

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
    e->chip_hz_q8 = cfg->chip_hz_nominal_q8;
    bank_init(&e->bank, &e->cfg);
    agc_init(&e->agc, &e->cfg);
    acquire_init(&e->acq, &e->cfg);
    trail_init(&e->trail, &e->cfg);
    sched_init(&e->sched, &e->cfg, cfg->fps ? 1000000u / cfg->fps : 3617u);
    *out = e;
    return 0;
}

uint32_t engine_chip_hz_q8(const Engine *e) { return e->chip_hz_q8; }
uint32_t engine_rate_corrections(const Engine *e) { return e->rate_corrections; }

/* ---- T083: measure the emitter's period and steer the receiver clock onto it -----------------------
 * Called once per tick, after bank_tick, so every track is in a settled state before any retune. */
#define RATE_WIN_US      5000000u   /* 5 s of accumulation — 3.6 chips of drift vs 0.3 chip of noise  */
#define RATE_MIN_OBS     40u        /* ticks that actually contributed a phase step                   */
#define RATE_DEADBAND_Q8 5          /* 0.02 Hz — below this, stop chasing the residual                */
#define RATE_SLEW_Q8     64         /* 0.25 Hz per correction, so no single step can teleport phase   */

static void rate_track(Engine *e, uint64_t now_us)
{
    const Track *best = NULL;
    int i;
    if (!e->cfg.rate_track) return;

    for (i = 0; i < e->bank.max_slots; i++) {                 /* the strongest confirmed witness */
        const BankSlot *s = &e->bank.slots[i];
        if (!s->used || s->trk.state != TRK_CONFIRMED || s->trk.chip_phase == 0xFF) continue;
        if (!best || s->trk.q_q8 > best->q_q8) best = &s->trk;
    }
    if (!best) { e->rate_prev_valid = 0u; return; }           /* blind: hold the estimate, do not guess */

    if (e->rate_prev_valid) {
        int d = (int)best->chip_phase - (int)e->rate_prev_phase;
        if (d >  CORR_N / 2) d -= CORR_N;                     /* wrap to the shortest step */
        if (d < -CORR_N / 2) d += CORR_N;
        e->rate_acc_chips += d;
        e->rate_obs++;
    } else {
        e->rate_t0_us = now_us;
        e->rate_acc_chips = 0;
        e->rate_obs = 0;
    }
    e->rate_prev_phase = best->chip_phase;
    e->rate_prev_valid = 1u;

    if (now_us - e->rate_t0_us >= RATE_WIN_US) {
        uint64_t el = now_us - e->rate_t0_us;
        if (e->rate_obs >= RATE_MIN_OBS) {
            /* chips/s, in q8: acc * 256 * 1e6 / elapsed_us. acc is small, so this cannot overflow. */
            int32_t drift_q8 = (int32_t)(((int64_t)e->rate_acc_chips * 256 * 1000000) / (int64_t)el);
            if (drift_q8 > RATE_DEADBAND_Q8 || drift_q8 < -RATE_DEADBAND_Q8) {
                if (drift_q8 >  RATE_SLEW_Q8) drift_q8 =  RATE_SLEW_Q8;
                if (drift_q8 < -RATE_SLEW_Q8) drift_q8 = -RATE_SLEW_Q8;
                e->chip_hz_q8 = (uint32_t)((int32_t)e->chip_hz_q8 + drift_q8);
                for (i = 0; i < e->bank.max_slots; i++)       /* one clock: every track moves together */
                    if (e->bank.slots[i].used)
                        track_retune(&e->bank.slots[i].trk, e->chip_hz_q8, now_us);
                e->rate_corrections++;
            }
        }
        e->rate_t0_us = now_us;
        e->rate_acc_chips = 0;
        e->rate_obs = 0;
        e->rate_prev_valid = 0u;      /* phase indices moved under the retune; re-observe before using */
    }
}

void engine_controls(const Engine *e, uint32_t *exposure_us, uint16_t *gain_q8)
{
    *exposure_us = e->agc.exposure_us;
    *gain_q8 = e->agc.gain_q8;
}
const Bank *engine_bank(const Engine *e) { return &e->bank; }

int engine_seed(Engine *e, uint8_t code_id, int32_t x_q8, int32_t y_q8, uint32_t chip_hz_q8)
{
    (void)chip_hz_q8;   /* one emitter, one clock: seeds adopt the RECEIVER's estimate (T083) */
    return bank_seed(&e->bank, &e->cfg, code_id, TRK_SCALE_MEDIUM, x_q8, y_q8,
                     e->chip_hz_q8, e->epoch_us, e->t0_us + e->frame_idx * 3617u);
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

    /* Trail search (T050-lite): when the aperture path failed to measure this tick, test velocity
     * hypotheses against the ring. The aperture stays authoritative when it works — it has the
     * sub-pixel centroid — so this fires exactly where the coherence limit lives. Measured ceiling and
     * search recovery are both 100 % out to 40 °/s (results/stage1-chip-rate-and-q.md). */
    if (e->cfg.trail_enable) {
        for (i = 0; i < e->bank.max_slots; i++) {
            BankSlot *s = &e->bank.slots[i];
            Track *t = &s->trk;
            TrailFix fx;
            if (!s->used || s->role != BANK_ROLE_PRECISION) continue;
            if (t->state != TRK_CONFIRMED && t->state != TRK_HOLD) continue;
            if (t->measured_fix && t->q_q8 >= e->cfg.q_fix_q8) continue;   /* aperture succeeded */
            /* Acceptance is q_fix, NOT q_lock, and the difference was measured (pend5 swing 2,
             * 2026-08-30): a trail fix REPLACES the state, so a marginal hypothesis teleports the
             * track. At 1.5 ft the angular rate ran to 115 deg/s -- 79 % of frames beyond the grid --
             * so the best IN-GRID hypothesis was always wrong, scored 0.55-0.68, cleared q_lock, and
             * scattered the track across the field. q_fix (0.75) is the bar that separates real from
             * junk (on-beacon p05 0.86 vs off-target p95 0.67); below it, HOLD's honest coast beats a
             * confident teleport. */
            if (trail_search(&e->trail, t->tmpl, t->epoch_us, t->chip_hz_q8, tick_end_us,
                             e->frame_w, e->frame_h, t->m2_div, &fx) &&
                fx.q_q8 >= e->cfg.q_fix_q8)
                track_apply_trail_fix(t, &e->cfg, fx.x_q8, fx.y_q8, fx.vx_q8, fx.vy_q8,
                                      fx.q_q8, tick_end_us, dt_us);
            break;                                 /* one precision trail per engine (one ring) */
        }
    }
    rate_track(e, tick_end_us);                   /* T083 — after the bank settles, before we report */
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

    /* Trail ring (T050-lite): a crop follows the precision track so the per-tick velocity-hypothesis
     * search has one word of history to correlate trails against. ROI-proportional: ~9 KB/frame. */
    if (e->cfg.trail_enable) {
        const Track *pt = NULL;
        for (i = 0; i < e->bank.max_slots; i++) {
            const BankSlot *s = &e->bank.slots[i];
            if (s->used && s->role == BANK_ROLE_PRECISION && s->trk.state != TRK_DEAD &&
                (!pt || s->trk.q_q8 > pt->q_q8)) pt = &s->trk;
        }
        if (pt) {
            /* M2 centre-origin q8 -> native px */
            int16_t cx = (int16_t)(((pt->xp_q8 * pt->m2_div) >> 8) + (int32_t)(e->frame_w / 2u));
            int16_t cy = (int16_t)(((pt->yp_q8 * pt->m2_div) >> 8) + (int32_t)(e->frame_h / 2u));
            trail_frame(&e->trail, fv, cx, cy);
        } else if (e->warm_valid) {
            int16_t cx = (int16_t)(((e->warm_x_q8 * 2) >> 8) + (int32_t)(e->frame_w / 2u));
            int16_t cy = (int16_t)(((e->warm_y_q8 * 2) >> 8) + (int32_t)(e->frame_h / 2u));
            trail_frame(&e->trail, fv, cx, cy);
        }
    }

    /* Acquisition (US3-lite): when nothing is CONFIRMED, spend the sched budget on blink-detect
     * passes. sched_try_begin/sched_completed make the completion frame a MODEL, identical live and
     * replayed (R3) — the pass's seeds are only APPLIED at the model's completion frame. */
    {
        int have_confirmed = 0;
        for (i = 0; i < e->bank.max_slots; i++)
            if (e->bank.slots[i].used && e->bank.slots[i].trk.state == TRK_CONFIRMED) {
                const Track *t = &e->bank.slots[i].trk;
                have_confirmed = 1;
                e->warm_x_q8 = t->x_q8;
                e->warm_y_q8 = t->y_q8;
                e->warm_chip_hz_q8 = t->chip_hz_q8;
                e->warm_valid = 1u;
                break;
            }
        /* Throttle + cap, learned live (2026-08-19: the first policy flooded the bank with 12 junk
         * candidates and the WC-read cost of their apertures drove the deadline margin to -233 ms —
         * §11.1's instrumentation catching its first real overload):
         *   - a pass at most every 25 frames (~11 Hz of attempts),
         *   - at most 4 candidates outstanding, 3 seeds per pass (daylight lesson, 2026-08-20: LED-PWM
         *     flicker out-blinks the beacon at short exposure, and with 1 seed/2 slots the lamps owned
         *     the bank while the beacon starved outside it. "The code kills false candidates" is the
         *     designed discriminator — let it see candidates. The A53-era 1/2 throttle was a deadline
         *     precaution; the A76 runs 36 ms of margin),
         *   - candidates seed at MEDIUM scale (576 native px/frame, 16x cheaper than coarse). */
        if (!have_confirmed) {
            int n_cand = 0;
            for (i = 0; i < e->bank.max_slots; i++)
                if (e->bank.slots[i].used && e->bank.slots[i].role == BANK_ROLE_CANDIDATE) n_cand++;
            int done = sched_completed(&e->sched, e->frame_idx);
            /* Every completed pass ends its episode — INCLUDING an empty one. Without this, passes_max
             * empty passes (and the first pass is empty by construction: it primes the diff plane)
             * exhaust the budget and acquisition never runs again. Found live 2026-08-19: replay
             * acquired instantly (burst data seeds on the first diff), the live bench never did. */
            if (done) sched_reset_episode(&e->sched);
            if (done && e->pending_n) {
                uint8_t k;
                for (k = 0; k < e->pending_n; k++) {
                    /* skip seeds that landed on an existing candidate's aperture */
                    int clash = 0;
                    for (i = 0; i < e->bank.max_slots; i++) {
                        const Track *t = &e->bank.slots[i].trk;
                        int32_t dx, dy;
                        if (!e->bank.slots[i].used) continue;
                        dx = t->x_q8 - e->pending[k].x_q8;
                        dy = t->y_q8 - e->pending[k].y_q8;
                        if (dx < 0) dx = -dx;
                        if (dy < 0) dy = -dy;
                        if (dx < (8 << 8) && dy < (8 << 8)) { clash = 1; break; }
                    }
                    if (!clash && n_cand < 4) {
                        uint32_t hz;
                        int32_t wx = e->pending[k].x_q8 - e->warm_x_q8;
                        int32_t wy = e->pending[k].y_q8 - e->warm_y_q8;
                        if (wx < 0) wx = -wx;
                        if (wy < 0) wy = -wy;
                        if (e->warm_valid && wx < (8 << 8) && wy < (8 << 8))
                            hz = e->warm_chip_hz_q8;   /* warm reacquire: inherit the proven rate    */
                        else {
                            (void)acquire_next_rate_q8(&e->acq, &e->cfg,
                                                       e->pending[k].x_q8, e->pending[k].y_q8);
                            hz = e->chip_hz_q8;   /* the receiver's tracked clock, not the constant */
                        }
                        bank_seed(&e->bank, &e->cfg, 1u /* bench: code B; two-code seeds try both when
                                  the cube lands */, TRK_SCALE_MEDIUM,
                                  e->pending[k].x_q8, e->pending[k].y_q8, hz,
                                  e->epoch_us, fv->t_us);
                        n_cand++;
                    }
                }
                e->pending_n = 0;
            }
            if (!e->sched.inflight && n_cand < 4 &&
                e->frame_idx >= e->next_acq_frame &&
                sched_try_begin(&e->sched, e->frame_idx)) {
                /* the pass runs on THIS frame's data; its results wait for the model's frame */
                e->pending_n = acquire_pass(&e->acq, fv, e->pending, 3);
                e->next_acq_frame = e->frame_idx + 25u;
            }
        }
    }

    /* tick boundaries crossed by this frame's timestamp (usually 0 or 1; catches up after gaps) */
    tk = (fv->t_us - e->t0_us) / TICK_US;
    if (e->field_on && e->last_tick < tk) field_update(e, fv);   /* once per tick, not per frame */
    if (e->preview_on && e->last_tick < tk) preview_update(e, fv);       /* same gate, same reason */
    while (e->last_tick < tk) {
        e->last_tick++;
        tick(e, e->t0_us + e->last_tick * TICK_US);
    }
}

void engine_close(Engine *e) { free(e); }
