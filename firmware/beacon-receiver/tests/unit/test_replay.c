/* T016 — the replay source yields an IDENTICAL FrameView sequence across two runs.
 * This is the parity contract's replay half: if two opens of the same file differ in anything — bytes,
 * metadata, ordering, count — replay-vs-live parity (T043) is dead before it starts. */
#include "bcn_test.h"
#include "config.h"
#include "frame.h"
#include "sink_record.h"
#include "src_replay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *INI = "test_replay_tmp.ini";
static const char *OUT = "test_replay_tmp.bcnr";
#define W 48
#define H 20
#define NFRAMES 37   /* deliberately not round */

static void write_ini(void)
{
    FILE *f = fopen(INI, "w");
    if (!f) { perror("fopen"); exit(2); }
    fprintf(f,
        "[camera]\nmode = %dx%d\nfps = 250\nexposure_min_us = 50\nexposure_max_us = 3000\n"
        "gain_min_q8 = 256\ngain_max_q8 = 4096\n"
        "[code]\nn_chips = 31\nrate_track = 1\nchip_hz_nominal = 115.0\nchip_hz_candidates = 115.0\n"
        "code_a = 0000000100011011000011001110011\ncode_b = 0100011001100111100101001011110\n"
        "[bank]\nmax_slots = 16\nscale_extents = 24,12,6\nalpha = 0.35\nbeta = 0.08\n"
        "q_lock = 0.55\nq_drop = 0.30\nq_fix = 0.75\nlock_health_lock = 0.60\nlock_health_drop = 0.35\n"
        "min_mod_depth = 0.0\n"
        "hold_max_age_ms = 150\nhold_max_cep_px = 3.0\n"
        "[agc]\nexposure_target_lo = 40\nexposure_target_hi = 200\nintegration_min_chips = 31\n"
        "integration_max_chips = 124\nroi_driven = 1\n"
        "[record]\nmode = continuous\npath = %s\nring_seconds = 1\nburst_frames = 4\nburst_every = 8\n"
        "trigger = manual\n"
        "[trail]\nenable = 1\ncrop_px = 96\nring_frames = 80\nvmax = 64.0\nvstep = 8.0\ncandidates = 5\n"
        "[sched]\nacquire_cost_us_per_pass = 12000\nacquire_passes_max = 2\n"
        "[sync]\nfiducial_enabled = 0\nfiducial_period_s = 10\nmsp_uart = /dev/null\nmsp_baud = 115200\n",
        W, H, OUT);
    fclose(f);
}

/* FNV-1a over everything a FrameView exposes, in a fixed order — one number per run to compare. */
static uint64_t digest_run(const char *path, uint32_t *frames_out, char *err, size_t err_len)
{
    FrameSource *src = NULL;
    FrameView fv;
    uint64_t h = 1469598103934665603ull;
    uint32_t n = 0;
    FrameStatus st;
#define MIX(v) do { uint64_t x = (uint64_t)(v); int b; \
                    for (b = 0; b < 8; b++) { h ^= (x >> (8*b)) & 0xFF; h *= 1099511628211ull; } } while (0)

    if (bcn_replay_open(&src, path, err, err_len) != 0) return 0;
    while ((st = src->next(src, &fv)) == FRAME_OK) {
        uint32_t i;
        MIX(fv.seq); MIX(fv.t_us); MIX(fv.exposure_us); MIX(fv.gain_q8);
        MIX(fv.w); MIX(fv.h); MIX(fv.stride);
        for (i = 0; i < (uint32_t)fv.w * fv.h; i++) { h ^= fv.data[i]; h *= 1099511628211ull; }
        n++;
    }
    src->close(src);
    if (st != FRAME_END) return 0;   /* an ERROR run digests to 0, which can never match a real digest */
    *frames_out = n;
    return h | 1ull;
#undef MIX
}

int main(void)
{
    char err[256];
    BcnConfig cfg;
    BcnRecorder *r = NULL;
    uint8_t px[W * H];
    uint32_t i;
    uint64_t d1, d2;
    uint32_t n1 = 0, n2 = 0;

    write_ini();
    CHECK((bcn_config_load(INI, &cfg, err, sizeof err)) == 0, "config: %s", err);
    CHECK((bcn_recorder_open(&r, &cfg, NULL, NULL, err, sizeof err)) == 0, "open: %s", err);
    for (i = 0; i < NFRAMES; i++) {
        FrameView fv;
        uint32_t k;
        for (k = 0; k < W * H; k++) px[k] = (uint8_t)(i * 131u + k * 17u + (k >> 3));
        memset(&fv, 0, sizeof fv);
        fv.data = px; fv.stride = W; fv.w = W; fv.h = H;
        fv.seq = i * 2u;                                  /* every other frame: gaps everywhere */
        fv.t_us = 77777ull + (uint64_t)i * 4003ull;
        fv.exposure_us = 150u + (i % 7u);
        fv.gain_q8 = (uint16_t)(512u + i * 3u);
        CHECK((bcn_recorder_push(r, &fv, err, sizeof err)) == 0, "push %u: %s", i, err);
    }
    CHECK((bcn_recorder_close(r, err, sizeof err)) == 0, "close: %s", err);

    d1 = digest_run(OUT, &n1, err, sizeof err);
    CHECK(d1 != 0, "run 1 must complete cleanly: %s", err);
    d2 = digest_run(OUT, &n2, err, sizeof err);
    CHECK(d2 != 0, "run 2 must complete cleanly: %s", err);
    CHECK_EQ_U(n1, NFRAMES);
    CHECK_EQ_U(n2, NFRAMES);
    CHECK(d1 == d2, "two runs over one file must be IDENTICAL (got %016llx vs %016llx)",
          (unsigned long long)d1, (unsigned long long)d2);

    remove(INI); remove(OUT);
    BCN_TEST_MAIN_END();
}
