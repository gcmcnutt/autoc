/* T043 — two replay runs of one clip emit BYTE-IDENTICAL record streams, and — because parity of a
 * tracker that tracks nothing would be vacuous — the clip contains a moving code-B beacon the engine
 * must actually track. This is the whole US2 chain under one deterministic roof:
 * recorder -> replay -> engine(corr/track/bank/agc) -> records.
 *
 * The synthetic scene: 320x200 @ 250 fps, textured background (deterministic xorshift), a code-B
 * point source at 115 Hz chips moving at a constant ~30 px/s diagonal — about 9 M2-px/s, a gentle slew
 * the alpha-beta loop must follow with zero steady-state lag (spec §2.1).
 */
#include "bcn_test.h"
#include "config.h"
#include "corr.h"
#include "engine.h"
#include "sink_record.h"
#include "src_replay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 320
#define H 200
#define FPS 250
#define SECONDS 3
#define NFRAMES (FPS * SECONDS)
#define CHIP_HZ_Q8 (115u * 256u)
#define CODE_B_BITS 0x2333CA5Eu

static const char *INI = "test_parity_tmp.ini";
static const char *CLIP = "test_parity_tmp.bcnr";

static void write_ini(void)
{
    FILE *f = fopen(INI, "w");
    if (!f) { perror("fopen"); exit(2); }
    fprintf(f,
        "[camera]\nmode = %dx%d\nfps = %d\nexposure_min_us = 200\nexposure_max_us = 3000\n"
        "gain_min_q8 = 1024\ngain_max_q8 = 4096\n"
        "[code]\nn_chips = 31\nrate_track = 1\nchip_hz_nominal = 115.0\nchip_hz_candidates = 115.0,200.0\n"
        "code_a = 0000000100011011000011001110011\ncode_b = 0100011001100111100101001011110\n"
        "[bank]\nmax_slots = 16\nscale_extents = 24,12,6\nalpha = 0.35\nbeta = 0.08\n"
        "q_lock = 0.55\nq_drop = 0.30\nq_fix = 0.75\nlock_health_lock = 0.60\nlock_health_drop = 0.35\n"
        "min_mod_depth = 0.0\n"
        "hold_max_age_ms = 150\nhold_max_cep_px = 3.0\n"
        "[agc]\nexposure_target_lo = 40\nexposure_target_hi = 200\nintegration_min_chips = 31\n"
        "integration_max_chips = 124\nroi_driven = 1\n"
        "[record]\nmode = continuous\npath = %s\nring_seconds = 1\nburst_frames = 80\nburst_every = 500\n"
        "trigger = manual\n"
        "[sched]\nacquire_cost_us_per_pass = 12000\nacquire_passes_max = 2\n"
        "[sync]\nfiducial_enabled = 0\nfiducial_period_s = 10\nmsp_uart = /dev/null\nmsp_baud = 115200\n",
        W, H, FPS, CLIP);
    fclose(f);
}

static uint32_t rng = 0xC0FFEE11u;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

/* Beacon truth: native px, moving. Starts at (200, 80), vx = +30 px/s, vy = +12 px/s. */
static double truth_x(double t_s) { return 200.0 + 30.0 * t_s; }
static double truth_y(double t_s) { return 80.0 + 12.0 * t_s; }

static void make_clip(void)
{
    char err[256];
    BcnConfig cfg;
    BcnRecorder *rec = NULL;
    static uint8_t bg[H][W];
    static uint8_t px[H][W];
    int8_t tmpl[CORR_N];
    int i, x, y;

    corr_template(CODE_B_BITS, tmpl);
    rng = 0xC0FFEE11u;
    /* static textured background: smooth-ish blobs + per-pixel noise, all deterministic */
    for (y = 0; y < H; y++)
        for (x = 0; x < W; x++)
            bg[y][x] = (uint8_t)(20 + ((x * 7 + y * 13) % 31) + (rnd() % 7));

    if (bcn_config_load(INI, &cfg, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
    if (bcn_recorder_open(&rec, &cfg, NULL, NULL, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }

    for (i = 0; i < NFRAMES; i++) {
        FrameView fv;
        uint64_t t_us = 1000000ull + (uint64_t)i * (1000000ull / FPS);
        double t_s = (double)i / FPS;
        int64_t chip = corr_chip_at(t_us, 1000000ull, CHIP_HZ_Q8);
        int8_t lit = tmpl[(uint8_t)(chip % CORR_N)] > 0;      /* chip 0 of the code at the epoch */
        int bx = (int)(truth_x(t_s) + 0.5), by = (int)(truth_y(t_s) + 0.5);

        memcpy(px, bg, sizeof px);
        if (lit) {                       /* a 2x2 point source, amplitude 90 over background */
            int dy, dx;
            for (dy = 0; dy < 2; dy++)
                for (dx = 0; dx < 2; dx++)
                    if (by + dy < H && bx + dx < W) {
                        int v = px[by + dy][bx + dx] + 90;
                        px[by + dy][bx + dx] = (uint8_t)(v > 255 ? 255 : v);
                    }
        }
        memset(&fv, 0, sizeof fv);
        fv.data = &px[0][0]; fv.stride = W; fv.w = W; fv.h = H;
        fv.seq = (uint32_t)i;
        fv.t_us = t_us;
        fv.exposure_us = 200u; fv.gain_q8 = 1024u;
        if (bcn_recorder_push(rec, &fv, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
    }
    if (bcn_recorder_close(rec, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
}

/* run the engine over the clip; append every emitted record's wire bytes to a growing buffer */
typedef struct { uint8_t *buf; size_t len, cap; BcnRecord last; uint32_t n; } Sink;
static void emit_cb(const BcnRecord *r, void *user)
{
    Sink *s = user;
    if (s->len + BCN_RECORD_WIRE_BYTES > s->cap) {
        s->cap = s->cap ? s->cap * 2 : 1 << 20;
        s->buf = realloc(s->buf, s->cap);
    }
    bcn_record_encode(r, s->buf + s->len);
    s->len += BCN_RECORD_WIRE_BYTES;
    s->last = *r;
    s->n++;
}

static void run_engine(Sink *sink)
{
    char err[256];
    BcnConfig cfg;
    Engine *e = NULL;
    FrameSource *src = NULL;
    FrameView fv;
    FrameStatus st;
    int seeded = 0;

    memset(sink, 0, sizeof *sink);
    if (bcn_config_load(INI, &cfg, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
    if (engine_open(&e, &cfg, emit_cb, sink, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
    if (bcn_replay_open(&src, CLIP, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); exit(2); }
    while ((st = src->next(src, &fv)) == FRAME_OK) {
        engine_frame(e, &fv);
        if (!seeded) {
            /* US2 demo path: manual seed near (not at) the truth — 3 native px off, exact rate unknown
             * (seed 115.0 vs the clip's 115.0 — candidate list would handle a mismatch; here the DPLL
             * proves pull-in from the phase search alone). M2 coords: native/2 - centre. */
            /* 320-wide frames: native IS the M2 grid (m2_div = 1) */
            int32_t seed_x_q8 = (int32_t)(((200 + 3) - W / 2) * 256);
            int32_t seed_y_q8 = (int32_t)(((80 - 2) - H / 2) * 256);
            CHECK(engine_seed(e, 1u, seed_x_q8, seed_y_q8, CHIP_HZ_Q8) >= 0, "seed must land");
            seeded = 1;
        }
    }
    CHECK(st == FRAME_END, "replay must end cleanly");
    src->close(src);
    engine_close(e);
}

int main(void)
{
    Sink a, b;

    write_ini();
    make_clip();

    run_engine(&a);
    run_engine(&b);

    /* ---- T043 proper: byte-identical record streams ---- */
    CHECK(a.n > 0, "engine must emit records");
    CHECK_EQ_U(b.n, a.n);
    CHECK(a.len == b.len && memcmp(a.buf, b.buf, a.len) == 0,
          "two replay runs must be BYTE-IDENTICAL (%u records)", a.n);

    /* ---- and the tracker must have actually tracked ---- */
    {
        const BcnRecord *last = &a.last;
        double t_end = (double)(NFRAMES - 1) / FPS;
        double tx_m2 = (truth_x(t_end) + 1.0 - W / 2);   /* native==M2 at 320 wide; +1: centre of 2x2 */
        double ty_m2 = (truth_y(t_end) + 1.0 - H / 2);
        const BcnTrack *t = &last->tracks[0];
        double ex, ey;

        CHECK(last->n_tracks >= 1, "a confirmed track must be reporting by the end (n_tracks=%u, slots=%u)",
              last->n_tracks, last->n_slots_used);
        if (last->n_tracks >= 1) {
            CHECK_EQ_U(t->code_id, 1u);                        /* code B                              */
            CHECK(t->flags & BCN_F_VALID, "VALID");
            CHECK(t->flags & BCN_F_LOCK, "LOCK (flags=0x%x)", t->flags);
            CHECK(t->flags & BCN_F_MEASURED_FIX, "MEASURED_FIX (flags=0x%x)", t->flags);
            ex = t->x / 256.0 - tx_m2;
            ey = t->y / 256.0 - ty_m2;
            CHECK(ex * ex + ey * ey < 9.0, "position within 3 M2 px of truth (err %.2f,%.2f)", ex, ey);
            /* zero steady-state lag under constant slew: velocity ~ (30/2, 12/2) M2 px/s */
            CHECK(t->vx > (int32_t)(22 * 256) && t->vx < (int32_t)(38 * 256),
                  "vx ~ 30 M2 px/s (native==M2 at 320 wide), got %.1f", t->vx / 256.0);
            /* DPLL: chip rate within 1% of 115 Hz */
            CHECK(t->chip_hz > 114u * 256u && t->chip_hz < 116u * 256u,
                  "chip_hz ~ 115, got %.2f", t->chip_hz / 256.0);
            CHECK(t->q > 100u, "q healthy, got %u", t->q);
            CHECK(t->lock_health > 128u, "lock_health above prior, got %u", t->lock_health);
        }
    }

    free(a.buf); free(b.buf);
    remove(INI); remove(CLIP);
    BCN_TEST_MAIN_END();
}
