/* beacon_record.c — T022 CLI + T023 real-time hygiene. The recorder that flies before the tracker
 * (spec §7.1): it exercises the ENTIRE libcamera path with no algorithm attached, so capture, thermals,
 * storage and the container format are proven before anything depends on them.
 *
 * Exit codes per contracts/cli.md: 0 clean · 1 config error · 2 version mismatch · 3 camera/source
 * failure. (4, deadline budget, belongs to beacon_trackd.)
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <time.h>
#include "config.h"
#include "frame.h"
#include "sink_record.h"
#include "src_libcamera.h"

#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static volatile sig_atomic_t g_trigger = 0;
static void on_stop(int sig)    { (void)sig; g_stop = 1; }
static void on_trigger(int sig) { (void)sig; g_trigger = 1; }

/* T023. Best-effort by design: a bench capture run as user `pi` lacking CAP_SYS_NICE should WARN and
 * degrade, not die — RT scheduling affects jitter, never correctness of the bytes. Each miss is printed
 * so a flight build can grep for the warnings and add the caps. */
static void rt_hygiene(void)
{
    struct sched_param sp;
    cpu_set_t cpus;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "beacon_record: warning: mlockall failed (%s) — page faults may add jitter\n",
                strerror(errno));

    memset(&sp, 0, sizeof sp);
    sp.sched_priority = 10;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "beacon_record: warning: SCHED_FIFO refused (%s) — running best-effort "
                        "(sudo, or setcap cap_sys_nice)\n", strerror(errno));

    /* Pin to core 3: core 0 takes most IRQs on the Pi. One core is plenty — the loop is a memcpy. */
    CPU_ZERO(&cpus);
    CPU_SET(3, &cpus);
    if (sched_setaffinity(0, sizeof cpus, &cpus) != 0)
        fprintf(stderr, "beacon_record: warning: core pinning refused (%s)\n", strerror(errno));
}

static void usage(void)
{
    fprintf(stderr,
        "usage: beacon_record --config <ini> [--mode continuous|ring|burst] [--out <path>]\n"
        "                     [--ring-seconds N] [--burst-frames N --burst-every M]\n"
        "                     [--duration S] [--trigger manual|gpio]\n"
        "  SIGUSR1 = dump the ring now (ring mode). SIGINT/SIGTERM = finish cleanly.\n");
}

/* --read-bench N: time a FULL-FRAME read of the capture buffer with N threads.
 *
 * WHY THIS EXISTS. research.md measured the dmabuf mmap as WRITE-COMBINE, one uncached 256 KB frame read
 * costing ~4 ms against a 3.47 ms frame period. That single number is what forced the whole receiver to
 * be ROI-proportional, and it is what blocks a full-field per-frame ("event camera") detector -- the
 * arithmetic is cheap (reduce2 NEON is 0.65 ms) but touching every pixel is not.
 *
 * The number was measured single-threaded, and that may understate what the hardware can do: a
 * write-combine read is LATENCY-bound rather than bandwidth-bound, because uncached reads do not
 * prefetch. Several threads can therefore have several misses outstanding at once and hide latency that
 * one thread cannot. If that holds, 4 cores buy something a bandwidth model says they should not, and the
 * event detector becomes affordable WITHOUT a smaller sensor mode -- which matters now that 320x200 is
 * confirmed broken in this pipeline.
 *
 * The sum is returned and printed so the optimiser cannot delete the loads. Reads only; nothing here
 * touches the recorder or the frame contract. */
typedef struct { const uint8_t *base; size_t off, len; uint64_t sum; } BenchSlice;

static void *bench_slice(void *arg)
{
    BenchSlice *s = (BenchSlice *)arg;
    const uint8_t *p = s->base + s->off;
    uint64_t acc = 0;
    size_t i;
    for (i = 0; i < s->len; i++) acc += p[i];   /* byte-wise: the WC penalty is per cache line, not per op */
    s->sum = acc;
    return NULL;
}

static double bench_read_ms(const uint8_t *base, size_t bytes, int nthreads, uint64_t *sum_out)
{
    pthread_t th[8];
    BenchSlice sl[8];
    struct timespec a, b;
    size_t chunk;
    int i;
    if (nthreads < 1) nthreads = 1;
    if (nthreads > 8) nthreads = 8;
    chunk = bytes / (size_t)nthreads;
    clock_gettime(CLOCK_MONOTONIC, &a);
    for (i = 0; i < nthreads; i++) {
        sl[i].base = base; sl[i].off = (size_t)i * chunk; sl[i].sum = 0;
        sl[i].len = (i == nthreads - 1) ? bytes - (size_t)i * chunk : chunk;
        if (nthreads == 1) bench_slice(&sl[i]);
        else pthread_create(&th[i], NULL, bench_slice, &sl[i]);
    }
    if (nthreads > 1) for (i = 0; i < nthreads; i++) pthread_join(th[i], NULL);
    clock_gettime(CLOCK_MONOTONIC, &b);
    for (i = 0; i < nthreads; i++) *sum_out += sl[i].sum;
    return (double)(b.tv_sec - a.tv_sec) * 1e3 + (double)(b.tv_nsec - a.tv_nsec) / 1e6;
}

int main(int argc, char **argv)
{
    const char *ini = NULL, *mode = NULL, *out_path = NULL, *trigger = NULL;
    long duration_s = 0;
    int read_bench = 0;          /* --read-bench N: WC full-frame read timing, N threads */
    long ring_seconds = -1, burst_frames = -1, burst_every = -1;
    char err[BCN_ERR_MAX];
    BcnConfig cfg;
    BcnRecorder *rec = NULL;
    FrameSource *src = NULL;
    int i, rc;
    uint64_t t0_us = 0, last_report_us = 0;
    uint32_t prev_seq = 0;
    int have_prev = 0;
    uint64_t gaps = 0;

    for (i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--config")       && i + 1 < argc) ini = argv[++i];
        else if (!strcmp(argv[i], "--mode")         && i + 1 < argc) mode = argv[++i];
        else if (!strcmp(argv[i], "--out")          && i + 1 < argc) out_path = argv[++i];
        else if (!strcmp(argv[i], "--trigger")      && i + 1 < argc) trigger = argv[++i];
        else if (!strcmp(argv[i], "--duration")     && i + 1 < argc) duration_s = atol(argv[++i]);
        else if (!strcmp(argv[i], "--read-bench")   && i + 1 < argc) read_bench = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ring-seconds") && i + 1 < argc) ring_seconds = atol(argv[++i]);
        else if (!strcmp(argv[i], "--burst-frames") && i + 1 < argc) burst_frames = atol(argv[++i]);
        else if (!strcmp(argv[i], "--burst-every")  && i + 1 < argc) burst_every = atol(argv[++i]);
        else { usage(); return 1; }
    }
    if (!ini) { usage(); return 1; }

    if (bcn_config_load(ini, &cfg, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_record: %s\n", err);
        return 1;                              /* config error names the key — Constitution VII */
    }
    /* CLI overrides land in the config copy BEFORE the hash... no: the hash must describe what RAN.
     * Overrides change the resolved config, so re-hash after applying them. */
    if (ring_seconds >= 0) cfg.ring_seconds = (uint32_t)ring_seconds;
    if (burst_frames >= 0) cfg.burst_frames = (uint32_t)burst_frames;
    if (burst_every  >= 0) cfg.burst_every  = (uint32_t)burst_every;
    if (trigger) snprintf(cfg.record_trigger, sizeof cfg.record_trigger, "%s", trigger);
    if (mode)    snprintf(cfg.record_mode,    sizeof cfg.record_mode,    "%s", mode);
    if (out_path) snprintf(cfg.record_path,   sizeof cfg.record_path,    "%s", out_path);
    cfg.config_hash = bcn_config_hash(&cfg);

    if (strcmp(cfg.record_trigger, "gpio") == 0) {
        fprintf(stderr, "beacon_record: --trigger gpio is not implemented on this bench (no GPIO trigger "
                        "wired); use manual (SIGUSR1)\n");
        return 1;
    }

    signal(SIGINT, on_stop);
    signal(SIGTERM, on_stop);
    signal(SIGUSR1, on_trigger);

    rt_hygiene();

    if (bcn_recorder_open(&rec, &cfg, NULL, NULL, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_record: %s\n", err);
        return 1;
    }
    if (bcn_libcamera_open(&src, &cfg, err, sizeof err) != 0) {
        fprintf(stderr, "beacon_record: %s\n", err);
        bcn_recorder_close(rec, err, sizeof err);
        return 3;
    }

    fprintf(stderr, "beacon_record: %s -> %s (mode %s, %s fps nominal %u)\n",
            cfg.camera_mode, cfg.record_path, cfg.record_mode,
            cfg.exposure_min_us ? "manual exposure" : "?", cfg.fps);

    rc = 0;
    {
    int bench_n = 0, bench_cnt[3] = {0,0,0};
    double bench_ms[3] = {0,0,0};
    uint64_t bench_sum = 0;
    for (;;) {
        FrameView fv;
        FrameStatus st;

        if (g_stop) break;
        if (g_trigger) {
            g_trigger = 0;
            if (bcn_recorder_trigger(rec, err, sizeof err) != 0) {
                fprintf(stderr, "beacon_record: %s\n", err);
                rc = 3; break;
            }
            fprintf(stderr, "beacon_record: ring dumped\n");
        }

        st = src->next(src, &fv);
        if (st == FRAME_END) break;
        if (st == FRAME_ERROR) { rc = 3; break; }

        if (read_bench && bench_n < 40) {
            /* Every frame for the first 40: sweep 1,2,4 threads round-robin so each count sees a
             * comparable mix of frames rather than a privileged warm-up. */
            static const int counts[3] = {1, 2, 4};
            int nt = counts[bench_n % 3];
            uint64_t sum = 0;
            double ms = bench_read_ms(fv.data, (size_t)fv.stride * fv.h, nt, &sum);
            bench_ms[bench_n % 3] += ms;
            bench_cnt[bench_n % 3]++;
            bench_sum += sum;
            bench_n++;
            if (bench_n == 40) {
                int k;
                fprintf(stderr, "beacon_record: WC full-frame read, %ux%u stride %u = %zu bytes\n",
                        fv.w, fv.h, fv.stride, (size_t)fv.stride * fv.h);
                for (k = 0; k < 3; k++)
                    if (bench_cnt[k])
                        fprintf(stderr, "beacon_record:   %d thread%s : %.3f ms  (%.0f MB/s)  n=%d\n",
                                counts[k], counts[k] == 1 ? " " : "s",
                                bench_ms[k] / bench_cnt[k],
                                ((double)fv.stride * fv.h / 1e6) / (bench_ms[k] / bench_cnt[k] / 1e3),
                                bench_cnt[k]);
                fprintf(stderr, "beacon_record:   frame period at %u fps = %.2f ms   (sink %llu)\n",
                        cfg.fps, 1000.0 / (double)cfg.fps, (unsigned long long)bench_sum);
            }
        }

        if (have_prev && fv.seq != prev_seq + 1u) gaps++;
        prev_seq = fv.seq; have_prev = 1;

        if (t0_us == 0) { t0_us = fv.t_us; last_report_us = fv.t_us; }
        if (bcn_recorder_push(rec, &fv, err, sizeof err) != 0) {
            fprintf(stderr, "beacon_record: %s\n", err);
            rc = 3; break;
        }

        if (fv.t_us - last_report_us >= 5000000ull) {     /* 5 s heartbeat, frame-clock timed */
            BcnRecorderStats s;
            bcn_recorder_stats(rec, &s);
            fprintf(stderr, "beacon_record: %llu offered, %llu written, %.1f MB, %llu seq gaps%s\n",
                    (unsigned long long)s.frames_offered, (unsigned long long)s.frames_written,
                    (double)s.bytes_written / 1e6, (unsigned long long)gaps,
                    s.o_direct ? "" : " [no O_DIRECT on this fs]");
            last_report_us = fv.t_us;
        }
        if (duration_s > 0 && fv.t_us - t0_us >= (uint64_t)duration_s * 1000000ull) break;
    }

    }
    src->close(src);
    {
        BcnRecorderStats s;
        bcn_recorder_stats(rec, &s);
        if (bcn_recorder_close(rec, err, sizeof err) != 0) {
            fprintf(stderr, "beacon_record: %s\n", err);
            if (rc == 0) rc = 3;
        }
        fprintf(stderr, "beacon_record: done — %llu frames written, %.1f MB, %llu sensor seq gaps\n",
                (unsigned long long)s.frames_written, (double)s.bytes_written / 1e6,
                (unsigned long long)gaps);
    }
    return rc;
}
