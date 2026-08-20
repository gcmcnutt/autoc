/* bench.c — T014, the kernel micro-bench harness (quickstart.md §4).
 *
 * Answers ONE question before any design leans on it: does research.md R5's "1–2 GMAC/s per A53 core"
 * assumption hold on the actual bench host? If the Pi comes back materially under, §10's compute budget
 * and the acquisition threading model both need revisiting — and week one is when you want to know.
 *
 * clock_gettime lives HERE and not in core/ (R3). tools/ is offline analysis; core/ takes its time from
 * the frame source or replay parity dies quietly.
 *
 * Kernels register themselves in the table below as they land (T028 NEON front end, T030 the measurement
 * of record). Today the table carries the integer-MAC baseline, which is the shape of the correlator's
 * inner loop and therefore the number R5 is actually about.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define N_PIX   (640 * 400)
#define REPEATS 32

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Integer MAC baseline: int16 x int16 -> int32 accumulate, the correlator's inner shape (R2 fixed-point).
 * Returned so the optimiser cannot delete the loop. */
static int32_t k_mac_i16(const int16_t *a, const int16_t *b, size_t n)
{
    int32_t acc = 0;
    size_t i;
    for (i = 0; i < n; i++) acc += (int32_t)a[i] * (int32_t)b[i];
    return acc;
}

/* Streaming 2x2 sum — memory-bound, the front end's actual character (R4). One MAC-equivalent per pixel. */
static uint32_t k_reduce2x2_ref(const uint8_t *src, size_t w, size_t h, uint32_t *dst)
{
    size_t x, y;
    uint32_t sink = 0;
    for (y = 0; y + 1 < h; y += 2) {
        const uint8_t *r0 = src + y * w, *r1 = r0 + w;
        uint32_t *o = dst + (y / 2) * (w / 2);
        for (x = 0; x + 1 < w; x += 2)
            o[x / 2] = (uint32_t)r0[x] + r0[x + 1] + r1[x] + r1[x + 1];
        sink += o[0];
    }
    return sink;
}

static void report(const char *name, uint64_t ns, double ops, size_t pixels)
{
    double s = (double)ns / 1e9;
    printf("  %-18s %8.2f ms   %6.2f ns/px   %6.3f G(MAC|op)/s\n",
           name, (double)ns / 1e6, (double)ns / (double)pixels, ops / s / 1e9);
}

static int bench_kernels(void)
{
    int16_t  *a = malloc(N_PIX * sizeof *a);
    int16_t  *b = malloc(N_PIX * sizeof *b);
    uint8_t  *img = malloc(N_PIX);
    uint32_t *out = malloc((size_t)(N_PIX / 4) * sizeof *out);
    uint64_t t0;
    int32_t sink32 = 0;
    uint32_t sinku = 0;
    int r;

    if (!a || !b || !img || !out) { fprintf(stderr, "bench: out of memory\n"); return 1; }
    for (r = 0; r < N_PIX; r++) { a[r] = (int16_t)(r & 0x7F); b[r] = (int16_t)((r * 7) & 0x7F); img[r] = (uint8_t)r; }

#ifndef __OPTIMIZE__
    /* An unoptimised bench measures the compiler, not the machine — and it under-reports by ~5x, which is
     * exactly the direction that would falsely condemn R5's assumption. Say so instead of printing a
     * number someone will quote later. */
    fprintf(stderr, "beacon_bench: WARNING — built WITHOUT optimisation. These numbers measure the\n"
                    "              compiler, not the machine, and must not be compared against R5.\n"
                    "              Reconfigure with -DCMAKE_BUILD_TYPE=RelWithDebInfo.\n\n");
#endif
    printf("beacon_bench --kernels   (%d px/frame, %d repeats)\n", N_PIX, REPEATS);
    printf("  R5 assumes 1-2 GMAC/s per Cortex-A53 core. Measure it here before anything depends on it.\n\n");

    t0 = now_ns();
    for (r = 0; r < REPEATS; r++) sink32 += k_mac_i16(a, b, N_PIX);
    report("mac_i16 (scalar)", now_ns() - t0, (double)N_PIX * REPEATS, (size_t)N_PIX * REPEATS);

    t0 = now_ns();
    for (r = 0; r < REPEATS; r++) sinku += k_reduce2x2_ref(img, 640, 400, out);
    report("reduce2x2 (scalar)", now_ns() - t0, (double)N_PIX * REPEATS, (size_t)N_PIX * REPEATS);

    printf("\n  (sinks %d %u — printed only so the optimiser cannot delete the loops)\n", sink32, sinku);
    printf("\n  NEON front end (T028) and its scalar-equivalence check (T029) register here as they land;\n"
           "  T030 records the measured GMAC/s against R5 in research.md.\n");

    free(a); free(b); free(img); free(out);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--kernels") == 0) return bench_kernels();
    fprintf(stderr, "usage: beacon_bench --kernels\n");
    return 1;
}
