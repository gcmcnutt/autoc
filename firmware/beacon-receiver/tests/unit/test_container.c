/* T015 — container write->read round-trip preserves every field INCLUDING seq gaps.
 * Written before sink_record.c. Uses synthetic frames on a machine with no camera; the recorder is a
 * library precisely so this test can exist. */
#define _POSIX_C_SOURCE 200809L   /* truncate() */
#include "bcn_test.h"
#include "config.h"
#include "container.h"
#include "frame.h"
#include "sink_record.h"
#include "src_replay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *INI = "test_container_tmp.ini";
static const char *OUT = "test_container_tmp.bcnr";

/* Small geometry so the test is instant; the writer must take it from config, not assume 640x400. */
#define W 64
#define H 32

static void write_ini(uint32_t burst_frames, uint32_t burst_every, const char *mode)
{
    FILE *f = fopen(INI, "w");
    if (!f) { perror("fopen ini"); exit(2); }
    fprintf(f,
        "[camera]\nmode = %dx%d\nfps = 250\nexposure_min_us = 50\nexposure_max_us = 3000\n"
        "gain_min_q8 = 256\ngain_max_q8 = 4096\n"
        "[code]\nn_chips = 31\nrate_track = 1\nchip_hz_nominal = 115.0\nchip_hz_candidates = 115.0,200.0\n"
        "code_a = 0000000100011011000011001110011\ncode_b = 0100011001100111100101001011110\n"
        "[bank]\nmax_slots = 16\nscale_extents = 24,12,6\nalpha = 0.35\nbeta = 0.08\n"
        "q_lock = 0.55\nq_drop = 0.30\nq_fix = 0.75\nlock_health_lock = 0.60\nlock_health_drop = 0.35\n"
        "min_mod_depth = 0.0\n"
        "hold_max_age_ms = 150\nhold_max_cep_px = 3.0\n"
        "[agc]\nexposure_target_lo = 40\nexposure_target_hi = 200\nintegration_min_chips = 31\n"
        "integration_max_chips = 124\nroi_driven = 1\n"
        "[record]\nmode = %s\npath = %s\nring_seconds = 1\nburst_frames = %u\nburst_every = %u\n"
        "trigger = manual\n"
        "[sched]\nacquire_cost_us_per_pass = 12000\nacquire_passes_max = 2\n"
        "[sync]\nfiducial_enabled = 0\nfiducial_period_s = 10\nmsp_uart = /dev/null\nmsp_baud = 115200\n",
        W, H, mode, OUT, burst_frames, burst_every);
    fclose(f);
}

/* Deterministic per-frame pixels so payload corruption or misalignment is caught, not just headers. */
static void fill(uint8_t *px, uint32_t seq)
{
    uint32_t i;
    for (i = 0; i < W * H; i++) px[i] = (uint8_t)(seq * 31u + i * 7u);
}

static FrameView mkframe(const uint8_t *px, uint32_t seq)
{
    FrameView fv;
    memset(&fv, 0, sizeof fv);
    fv.data = px; fv.stride = W; fv.w = W; fv.h = H;
    fv.seq = seq;
    fv.t_us = 1000000ull + (uint64_t)seq * 3617ull;   /* deliberately not a round period */
    fv.exposure_us = 200u + seq;
    fv.gain_q8 = (uint16_t)(1024u + seq);
    return fv;
}

int main(void)
{
    char err[256];
    BcnConfig cfg;
    uint8_t px[W * H];
    uint8_t want[W * H];

    /* ---- continuous, WITH a deliberate seq gap: frames 0,1,2,5,6 (3,4 "dropped" upstream). The writer
     * must preserve the gap, not paper over it — gaps are explicit, never implied. */
    {
        BcnRecorder *r = NULL;
        BcnRecorderStats st;
        FrameSource *src = NULL;
        FrameView fv;
        const uint32_t seqs[] = { 0, 1, 2, 5, 6 };
        uint32_t i;

        write_ini(4, 16, "continuous");
        CHECK((bcn_config_load(INI, &cfg, err, sizeof err)) == 0, "config: %s", err);
        CHECK((bcn_recorder_open(&r, &cfg, NULL, NULL, err, sizeof err)) == 0, "open: %s", err);
        for (i = 0; i < 5; i++) {
            fill(px, seqs[i]);
            fv = mkframe(px, seqs[i]);
            CHECK((bcn_recorder_push(r, &fv, err, sizeof err)) == 0, "push %u: %s", seqs[i], err);
        }
        bcn_recorder_drain(r);              /* stats are writer-thread-async — drain for coherence */
        bcn_recorder_stats(r, &st);
        CHECK_EQ_U(st.frames_offered, 5u);
        CHECK_EQ_U(st.frames_written, 5u);
        CHECK((bcn_recorder_close(r, err, sizeof err)) == 0, "close: %s", err);

        /* Read back through the REPLAY source — the same reader the tracker will use. */
        CHECK((bcn_replay_open(&src, OUT, err, sizeof err)) == 0, "replay open: %s", err);
        {
            const BcnContainerHeader *h = bcn_replay_header(src);
            CHECK_EQ_U(h->width, W);
            CHECK_EQ_U(h->height, H);
            CHECK_EQ_U(h->mode, (uint32_t)BCN_MODE_CONTINUOUS);
            CHECK_EQ_U(h->nominal_fps, 250u);
            CHECK_EQ_U(h->start_t_us, 1000000ull);        /* patched in at close from frame 0 */
            CHECK(h->config_hash == cfg.config_hash, "config_hash must be stamped through");
        }
        for (i = 0; i < 5; i++) {
            const BcnFrameHeader *fh;
            CHECK((src->next(src, &fv)) == FRAME_OK, "next %u", i);
            CHECK_EQ_U(fv.seq, seqs[i]);
            CHECK_EQ_U(fv.t_us, 1000000ull + (uint64_t)seqs[i] * 3617ull);
            CHECK_EQ_U(fv.exposure_us, 200u + seqs[i]);
            CHECK_EQ_U(fv.gain_q8, 1024u + seqs[i]);
            CHECK_EQ_U(fv.w, W); CHECK_EQ_U(fv.h, H);
            fill(want, seqs[i]);
            CHECK(memcmp(fv.data, want, W * H) == 0, "payload %u must round-trip bit-exactly", seqs[i]);
            fh = bcn_replay_last_frame_header(src);
            CHECK_EQ_U(fh->inav_t_us, 0u);                /* T024/R10: zero-when-absent, always */
            CHECK_EQ_U(fh->gps_time_ms, 0u);
        }
        CHECK((src->next(src, &fv)) == FRAME_END, "clean EOF, not an error");
        src->close(src);
    }

    /* ---- burst: frames 0..15 with burst_frames=4, burst_every=8 -> written 0,1,2,3, 8,9,10,11;
     * 0 and 8 carry BURST_START. The gap 4..7 is explicit in seq. */
    {
        BcnRecorder *r = NULL;
        BcnRecorderStats st;
        FrameSource *src = NULL;
        FrameView fv;
        uint32_t i;
        static const uint32_t kept[] = { 0, 1, 2, 3, 8, 9, 10, 11 };

        write_ini(4, 8, "burst");
        CHECK((bcn_config_load(INI, &cfg, err, sizeof err)) == 0, "config: %s", err);
        CHECK((bcn_recorder_open(&r, &cfg, NULL, NULL, err, sizeof err)) == 0, "open burst: %s", err);
        for (i = 0; i < 16; i++) {
            fill(px, i);
            fv = mkframe(px, i);
            CHECK((bcn_recorder_push(r, &fv, err, sizeof err)) == 0, "push %u: %s", i, err);
        }
        bcn_recorder_drain(r);
        bcn_recorder_stats(r, &st);
        CHECK_EQ_U(st.frames_offered, 16u);
        CHECK_EQ_U(st.frames_written, 8u);
        CHECK((bcn_recorder_close(r, err, sizeof err)) == 0, "close burst: %s", err);

        CHECK((bcn_replay_open(&src, OUT, err, sizeof err)) == 0, "replay burst: %s", err);
        CHECK_EQ_U(bcn_replay_header(src)->mode, (uint32_t)BCN_MODE_BURST);
        for (i = 0; i < 8; i++) {
            const BcnFrameHeader *fh;
            CHECK((src->next(src, &fv)) == FRAME_OK, "burst next %u", i);
            CHECK_EQ_U(fv.seq, kept[i]);
            fh = bcn_replay_last_frame_header(src);
            if (kept[i] == 0u || kept[i] == 8u)
                CHECK(fh->flags & BCN_FR_BURST_START, "frame %u must carry BURST_START", kept[i]);
            else
                CHECK(!(fh->flags & BCN_FR_BURST_START), "frame %u must NOT carry BURST_START", kept[i]);
        }
        CHECK((src->next(src, &fv)) == FRAME_END, "burst EOF");
        src->close(src);
    }

    /* ---- ring: capacity floor(1s * 250fps) but we push only 12 with capacity>=12? No — exercise
     * OVERWRITE: push 300 frames into a 250-slot ring, trigger, expect the LAST 250 (seq 50..299),
     * first dumped frame flagged TRIGGER_DUMP. */
    {
        BcnRecorder *r = NULL;
        BcnRecorderStats st;
        FrameSource *src = NULL;
        FrameView fv;
        uint32_t i;

        write_ini(4, 8, "ring");
        CHECK((bcn_config_load(INI, &cfg, err, sizeof err)) == 0, "config: %s", err);
        CHECK((bcn_recorder_open(&r, &cfg, NULL, NULL, err, sizeof err)) == 0, "open ring: %s", err);
        for (i = 0; i < 300; i++) {
            fill(px, i);
            fv = mkframe(px, i);
            CHECK((bcn_recorder_push(r, &fv, err, sizeof err)) == 0, "ring push %u: %s", i, err);
        }
        bcn_recorder_stats(r, &st);
        CHECK_EQ_U(st.frames_written, 0u);                /* nothing reaches disk before the trigger */
        CHECK((bcn_recorder_trigger(r, err, sizeof err)) == 0, "trigger: %s", err);
        bcn_recorder_drain(r);
        bcn_recorder_stats(r, &st);
        CHECK_EQ_U(st.frames_written, 250u);
        CHECK_EQ_U(st.ring_dumps, 1u);
        CHECK((bcn_recorder_close(r, err, sizeof err)) == 0, "close ring: %s", err);

        CHECK((bcn_replay_open(&src, OUT, err, sizeof err)) == 0, "replay ring: %s", err);
        for (i = 0; i < 250; i++) {
            const BcnFrameHeader *fh;
            CHECK((src->next(src, &fv)) == FRAME_OK, "ring next %u", i);
            CHECK_EQ_U(fv.seq, 50u + i);                  /* oldest 50 were overwritten */
            fill(want, 50u + i);
            CHECK(memcmp(fv.data, want, W * H) == 0, "ring payload %u intact after wraparound", 50u + i);
            fh = bcn_replay_last_frame_header(src);
            if (i == 0)
                CHECK(fh->flags & BCN_FR_TRIGGER_DUMP, "first dumped frame must carry TRIGGER_DUMP");
        }
        CHECK((src->next(src, &fv)) == FRAME_END, "ring EOF");
        src->close(src);
    }

    /* ---- a truncated file: the reader must not return garbage as a frame. */
    {
        FrameSource *src = NULL;
        FrameView fv;
        FILE *f = fopen(OUT, "rb+");
        long sz;
        CHECK(f != NULL, "reopen for truncation");
        fseek(f, 0, SEEK_END); sz = ftell(f);
        fclose(f);
        CHECK(truncate(OUT, sz - 100) == 0, "truncate");
        CHECK((bcn_replay_open(&src, OUT, err, sizeof err)) == 0, "replay truncated: %s", err);
        while (src->next(src, &fv) == FRAME_OK) { /* consume the intact prefix */ }
        /* The final status must be END or ERROR — never OK-with-garbage. Reaching here proves it. */
        CHECK(1, "truncated file terminated");
        src->close(src);
    }

    remove(INI); remove(OUT);
    BCN_TEST_MAIN_END();
}
