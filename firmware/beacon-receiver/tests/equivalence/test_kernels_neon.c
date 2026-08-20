/* T029 — scalar-vs-NEON BIT-EXACT equivalence per kernel (R11: this is the verification strategy).
 * Random frames, awkward geometries (odd sizes, stride != w, tails the vector loop cannot cover),
 * deterministic xorshift seed so a failure reproduces. */
#include "bcn_test.h"
#include "reduce.h"
#include "hipass.h"
#include <stdlib.h>
#include <string.h>

static uint32_t rng = 0x1234ABCDu;
static uint32_t next(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

int main(void)
{
#if !defined(__aarch64__)
    printf("not aarch64 — NEON equivalence has nothing to test here\n");
    return 0;
#else
    /* geometry set: canonical planes + adversarial shapes for the tails */
    static const struct { uint16_t w, h, stride; } G[] = {
        { 640, 400, 640 },   /* the real frame                     */
        { 640, 200, 640 },   /* the patched mode                   */
        { 320, 200, 320 },   /* M2 plane through reduce2           */
        { 66,  10,  72 },    /* tail of 2 after the 32-wide vector */
        { 34,  8,   40 },    /* one vector iter + tail             */
        { 30,  6,   30 },    /* NO vector iterations at all        */
        { 128, 12,  131 },   /* unaligned stride                   */
    };
    int gi, rep;

    for (gi = 0; gi < (int)(sizeof G / sizeof G[0]); gi++) {
        uint16_t w = G[gi].w, h = G[gi].h, st = G[gi].stride;
        uint8_t  *src = malloc((size_t)st * h);
        uint16_t *a2 = malloc((size_t)(w / 2) * (h / 2) * 2), *b2 = malloc((size_t)(w / 2) * (h / 2) * 2);
        uint16_t *a4 = malloc((size_t)(w / 4) * (h / 4) * 2), *b4 = malloc((size_t)(w / 4) * (h / 4) * 2);
        int32_t  *ha = malloc((size_t)w * h * 4), *hb = malloc((size_t)w * h * 4);
        uint16_t *hs = malloc((size_t)w * h * 2);

        for (rep = 0; rep < 4; rep++) {
            size_t i;
            for (i = 0; i < (size_t)st * h; i++) src[i] = (uint8_t)next();
            /* poison outputs differently so "both untouched" cannot pass as "equal" */
            memset(a2, 0xAA, (size_t)(w / 2) * (h / 2) * 2);
            memset(b2, 0x55, (size_t)(w / 2) * (h / 2) * 2);
            reduce2_scalar(src, st, w, h, a2);
            reduce2_neon(src, st, w, h, b2);
            CHECK(memcmp(a2, b2, (size_t)(w / 2) * (h / 2) * 2) == 0,
                  "reduce2 %ux%u stride %u rep %d: NEON differs from scalar", w, h, st, rep);

            memset(a4, 0xAA, (size_t)(w / 4) * (h / 4) * 2);
            memset(b4, 0x55, (size_t)(w / 4) * (h / 4) * 2);
            reduce4_scalar(src, st, w, h, a4);
            reduce4_neon(src, st, w, h, b4);
            CHECK(memcmp(a4, b4, (size_t)(w / 4) * (h / 4) * 2) == 0,
                  "reduce4 %ux%u stride %u rep %d: NEON differs from scalar", w, h, st, rep);

            for (i = 0; i < (size_t)w * h; i++) hs[i] = (uint16_t)(next() & 0x0FFF);
            hipass_scalar(hs, w, h, ha);
            hipass_neon(hs, w, h, hb);
            CHECK(memcmp(ha, hb, (size_t)w * h * 4) == 0,
                  "hipass %ux%u rep %d: NEON differs from scalar", w, h, rep);
        }
        free(src); free(a2); free(b2); free(a4); free(b4); free(ha); free(hb); free(hs);
    }
    BCN_TEST_MAIN_END();
#endif
}
