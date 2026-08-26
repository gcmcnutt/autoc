/* beacon_trackd.cc — T053/T062/T063: the daemon. Thin BY ARCHITECTURE (plan.md): every algorithm lives
 * in core/engine; this file is sources, sinks, the deadline stopwatch, and argument parsing.
 *
 * Deadline instrumentation (T062, spec §11.1): core is clockless (R3) and emits deadline_margin_us = 0,
 * which keeps replay parity byte-exact. The LIVE path measures here — sensor timestamps are
 * CLOCK_BOOTTIME on this platform, so the margin is directly computable — and patches the field into
 * the record before transport. Replay leaves it 0 and reports no deadline stats: there is no deadline
 * to miss when time is virtual. p99 and max are tracked, not mean (§11.1: the tail is the spec).
 */
#include <string>
#include "config.h"
#include "engine.h"
#include "frame.h"
#include "src_replay.h"
#include "emit_record.h"
#include "sink_record.h"
#if defined(BCN_HAVE_LIBCAMERA)
#include "src_libcamera.h"
#endif

#include <cerrno>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <vector>

static volatile sig_atomic_t g_stop = 0;
static void on_stop(int) { g_stop = 1; }

struct Ctx {
    std::vector<BcnEmitter *> sinks;
    bool live = false;
    uint64_t next_tick_boottime_us = 0;
    std::vector<int32_t> margins;      /* live deadline margins, for p99/max reporting */
    uint32_t misses = 0;
    uint32_t emitted = 0;
    const Engine *eng = nullptr;       /* for the --field-map / --preview side channels (below) */
    bool field_map = false;
    bool preview = false;
};

/* --field-map: write the engine's coarse contrast map as its OWN JSON line on stdout, right after the
 * record line. A side channel, not a record field: BcnRecord is a versioned wire contract and this is a
 * viewfinder. ascii_scope keys on the "field" member and ignores the line otherwise, so an older scope
 * (or any other json consumer) is unaffected -- it just sees a line with no "tracks". Pairs with
 * --emit json:-; it has nothing to attach to over the binary/tcp sink. */
static void emit_field_line(const Engine *eng)
{
    const uint8_t *f = engine_field_map(eng);
    char buf[BCN_FIELD_W * BCN_FIELD_H * 4 + 64];
    int n;
    if (!f) return;
    n = snprintf(buf, sizeof buf, "{\"field_w\":%d,\"field_h\":%d,\"field\":[", BCN_FIELD_W, BCN_FIELD_H);
    for (int i = 0; i < BCN_FIELD_W * BCN_FIELD_H; i++)
        n += snprintf(buf + n, sizeof buf - (size_t)n, "%s%u", i ? "," : "", f[i]);
    n += snprintf(buf + n, sizeof buf - (size_t)n, "]}\n");
    if (write(1, buf, (size_t)n) != (ssize_t)n) { /* scope went away; not fatal */ }
}

/* --preview: the same side-channel idea as --field-map, but a plane you can actually look at. Base64
 * rather than a JSON array of ints -- 320x200 as decimal text is ~200 KB per tick against 85 KB encoded,
 * and this is meant to run live at 20 Hz. Same rules otherwise: its own line, keyed on "preview_b64",
 * ignored by any consumer that does not know it, and never part of BcnRecord. */
static void emit_preview_line(const Engine *eng)
{
    static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    unsigned w = 0, h = 0;
    const uint8_t *p = engine_preview_map(eng, &w, &h);
    size_t n, i, o = 0;
    if (!p || !w || !h) return;
    n = (size_t)w * h;
    {
        /* 4 chars per 3 bytes, plus the header and slack */
        std::string buf;
        buf.resize(64 + ((n + 2) / 3) * 4 + 8);
        o = (size_t)snprintf(&buf[0], buf.size(), "{\"preview_w\":%u,\"preview_h\":%u,\"preview_b64\":\"", w, h);
        for (i = 0; i + 2 < n; i += 3) {
            uint32_t v = ((uint32_t)p[i] << 16) | ((uint32_t)p[i + 1] << 8) | p[i + 2];
            buf[o++] = B64[(v >> 18) & 63]; buf[o++] = B64[(v >> 12) & 63];
            buf[o++] = B64[(v >> 6) & 63];  buf[o++] = B64[v & 63];
        }
        if (i < n) {                                   /* 1 or 2 trailing bytes */
            uint32_t v = (uint32_t)p[i] << 16;
            int two = (i + 1 < n);
            if (two) v |= (uint32_t)p[i + 1] << 8;
            buf[o++] = B64[(v >> 18) & 63]; buf[o++] = B64[(v >> 12) & 63];
            buf[o++] = two ? B64[(v >> 6) & 63] : '=';
            buf[o++] = '=';
        }
        buf[o++] = '"'; buf[o++] = '}'; buf[o++] = '\n';
        if (write(1, buf.data(), o) != (ssize_t)o) { /* viewer went away; not fatal */ }
    }
}

static uint64_t boottime_us()
{
    struct timespec ts;
#if defined(CLOCK_BOOTTIME)
    if (clock_gettime(CLOCK_BOOTTIME, &ts) != 0)
#endif
        clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

static void emit_cb(const BcnRecord *rec, void *user)
{
    Ctx *c = (Ctx *)user;
    BcnRecord r = *rec;
    if (c->live) {
        /* §11.1: the record predicting tick N must be on the wire >= 5 ms before tick N. x_pred targets
         * the NEXT tick = rec->t_us + 50 ms on the sensor clock == BOOTTIME on this platform. */
        uint64_t now = boottime_us();
        int64_t margin = (int64_t)(r.t_us + 50000ull) - (int64_t)now - 5000;
        r.deadline_margin_us = (int32_t)(margin > INT32_MAX ? INT32_MAX : margin < INT32_MIN ? INT32_MIN : margin);
        c->margins.push_back(r.deadline_margin_us);
        if (r.deadline_margin_us < 0) c->misses++;
    }
    for (BcnEmitter *e : c->sinks) bcn_emitter_send(e, &r);
    if (c->field_map && c->eng) emit_field_line(c->eng);
    if (c->preview && c->eng) emit_preview_line(c->eng);
    c->emitted++;
}

static void usage()
{
    fprintf(stderr,
        "usage: beacon_trackd --config <ini> [--source live|replay:<file>]\n"
        "                     [--emit <sink>[,<sink>...]] [--record <path>] [--duration S]\n"
        "                     [--record-mode continuous|ring|burst]   (default: [record] mode in the ini)\n"
        "                     [--seed B:<x_m2>:<y_m2>]   (bench helper; acquisition normally seeds)\n"
        "                     [--field-map]              (viewfinder for the scope; use with --emit json:-)\n"
        "                     [--preview]                (M2 plane as base64 for live_view.py; --emit json:-)\n"
        "  <sink> ::= binary:<path> | json:- | tcp:<host>:<port> | serial:<dev>[:<baud>]\n");
}

int main(int argc, char **argv)
{
    const char *ini = nullptr, *source = "live", *emit_spec = "json:-", *record_path = nullptr;
    const char *record_mode = nullptr;   /* NULL = take [record] mode from the ini */
    const char *seed_spec = nullptr;
    bool field_map = false;
    bool preview = false;
    long duration_s = 0;
    char err[BCN_ERR_MAX];
    BcnConfig cfg;
    Engine *eng = nullptr;
    FrameSource *src = nullptr;
    BcnRecorder *rec = nullptr;
    Ctx ctx;
    int rc = 0;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--config")   && i + 1 < argc) ini = argv[++i];
        else if (!strcmp(argv[i], "--source")   && i + 1 < argc) source = argv[++i];
        else if (!strcmp(argv[i], "--emit")     && i + 1 < argc) emit_spec = argv[++i];
        else if (!strcmp(argv[i], "--record")   && i + 1 < argc) record_path = argv[++i];
        else if (!strcmp(argv[i], "--record-mode") && i + 1 < argc) record_mode = argv[++i];
        else if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration_s = atol(argv[++i]);
        else if (!strcmp(argv[i], "--seed")     && i + 1 < argc) seed_spec = argv[++i];
        else if (!strcmp(argv[i], "--field-map"))                field_map = true;
        else if (!strcmp(argv[i], "--preview"))                  preview = true;
        else { usage(); return 1; }
    }
    if (!ini) { usage(); return 1; }
    if (bcn_config_load(ini, &cfg, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_trackd: %s\n", err);
        return 1;
    }

    signal(SIGINT, on_stop);
    signal(SIGTERM, on_stop);
    signal(SIGPIPE, SIG_IGN);

    /* sinks (comma-separated) */
    {
        char spec[512];
        snprintf(spec, sizeof spec, "%s", emit_spec);
        for (char *tok = strtok(spec, ","); tok; tok = strtok(nullptr, ",")) {
            BcnEmitter *e = nullptr;
            if (bcn_emitter_open(&e, tok, err, sizeof err) != 0) {
                fprintf(stderr, "beacon_trackd: %s\n", err);
                return 1;
            }
            ctx.sinks.push_back(e);
        }
    }

    if (engine_open(&eng, &cfg, emit_cb, &ctx, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_trackd: %s\n", err);
        return 1;
    }

    ctx.live = strcmp(source, "live") == 0;
    if (ctx.live) {
#if defined(BCN_HAVE_LIBCAMERA)
        if (bcn_libcamera_open(&src, &cfg, err, sizeof err) != 0) {
            fprintf(stderr, "beacon_trackd: %s\n", err);
            return 3;
        }
#else
        fprintf(stderr, "beacon_trackd: built without libcamera — live capture unavailable on this host\n");
        return 3;
#endif
    } else if (strncmp(source, "replay:", 7) == 0) {
        if (bcn_replay_open(&src, source + 7, err, sizeof err) != 0) {
            fprintf(stderr, "beacon_trackd: %s\n", err);
            /* the replay open message already names both versions on a mismatch */
            return strstr(err, "format_version") ? 2 : 3;
        }
    } else {
        usage();
        return 1;
    }

    /* The mode must be overridable from the CLI. pan1.bcnr (2026-08-21) was captured in BURST mode
     * purely because [record] mode in beacon-bench.ini is the bench default `burst`, and burst gives
     * 80-frame islands 1.7 s apart -- 1.08 code words each, with gaps the container contract forbids
     * correlating across. That silently made the clip unable to measure reacquire or tracking through a
     * pan. A motion campaign wants `continuous`, and it should not depend on remembering to edit an ini. */
    if (record_path &&
        bcn_recorder_open(&rec, &cfg, record_mode, record_path, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_trackd: %s\n", err);
        return 1;
    }

    if (field_map) { engine_field_enable(eng, 1); ctx.eng = eng; ctx.field_map = true; }
    if (preview) {
        if (engine_preview_enable(eng, 1) != 0) {
            fprintf(stderr, "beacon_trackd: --preview unsupported for [camera] mode %s "
                            "(max %dx%d after /%d)\n",
                    cfg.camera_mode, BCN_PREVIEW_MAX_W, BCN_PREVIEW_MAX_H, BCN_PREVIEW_DIV);
            return 1;
        }
        ctx.eng = eng; ctx.preview = true;
    }

    fprintf(stderr, "beacon_trackd: source=%s emit=%s%s%s%s%s\n", source, emit_spec,
            record_path ? " record=" : "", record_path ? record_path : "",
            field_map ? " field-map=on" : "", preview ? " preview=on" : "");

    uint64_t t0 = 0;
    uint32_t nframes = 0;
    for (;;) {
        FrameView fv;
        FrameStatus st;
        if (g_stop) break;
        st = src->next(src, &fv);
        if (st == FRAME_END) break;
        if (st == FRAME_ERROR) { rc = 3; break; }
        if (t0 == 0) t0 = fv.t_us;
        engine_frame(eng, &fv);
        if (rec && bcn_recorder_push(rec, &fv, err, sizeof err) != 0) {
            fprintf(stderr, "beacon_trackd: %s\n", err);
            rc = 3; break;
        }
        if (seed_spec && nframes == 8) {   /* a few frames in, so the epoch exists */
            unsigned code = 1;
            int x = 0, y = 0;
            if (sscanf(seed_spec, "B:%d:%d", &x, &y) == 2) code = 1;
            else if (sscanf(seed_spec, "A:%d:%d", &x, &y) == 2) code = 0;
            engine_seed(eng, (uint8_t)code, x * 256, y * 256, cfg.chip_hz_nominal_q8);
            fprintf(stderr, "beacon_trackd: seeded %s at M2 (%d,%d)\n", code ? "B" : "A", x, y);
        }
        nframes++;
        if (duration_s > 0 && fv.t_us - t0 >= (uint64_t)duration_s * 1000000ull) break;
    }

    src->close(src);
    BcnRecorderStats rec_stats;
    memset(&rec_stats, 0, sizeof rec_stats);
    if (rec) {
        bcn_recorder_drain(rec);
        bcn_recorder_stats(rec, &rec_stats);
        bcn_recorder_close(rec, err, sizeof err);
    }
    engine_close(eng);
    for (BcnEmitter *e : ctx.sinks) bcn_emitter_close(e);

    fprintf(stderr, "beacon_trackd: %u records emitted over %u frames\n", ctx.emitted, nframes);
    if (rec_stats.frames_offered) {
        /* frames_dropped is the honest capture metric. Sensor seq gaps do NOT prove the capture was
         * clean -- this pipeline's seq counts DELIVERED frames, so a dropped frame is invisible there
         * (sink_record.c's 2026-08-19 note). Print the count, loudly if nonzero. */
        fprintf(stderr, "beacon_trackd: recorder %llu offered, %llu written, %llu DROPPED (%.2f%%), "
                        "%.1f MB, O_DIRECT %d\n",
                (unsigned long long)rec_stats.frames_offered,
                (unsigned long long)rec_stats.frames_written,
                (unsigned long long)rec_stats.frames_dropped,
                100.0 * (double)rec_stats.frames_dropped / (double)rec_stats.frames_offered,
                (double)rec_stats.bytes_written / 1e6, rec_stats.o_direct);
        if (rec_stats.frames_dropped)
            fprintf(stderr, "beacon_trackd: WARNING — the sink could not keep up at this duty.\n");
    }
    if (ctx.live && !ctx.margins.empty()) {
        std::sort(ctx.margins.begin(), ctx.margins.end());
        size_t p1 = ctx.margins.size() / 100;   /* 1st percentile of MARGIN = 99th of lateness */
        fprintf(stderr,
            "beacon_trackd: deadline margin min/p1/median %d/%d/%d us, misses %u/%zu (%.3f%%) "
            "[bar: <0.1%% at >=5 ms, spec 11.1]\n",
            ctx.margins.front(), ctx.margins[p1], ctx.margins[ctx.margins.size() / 2],
            ctx.misses, ctx.margins.size(), 100.0 * ctx.misses / ctx.margins.size());
        if (ctx.misses > ctx.margins.size() / 1000) rc = rc ? rc : 4;   /* exit 4: budget exceeded */
    }
    return rc;
}
