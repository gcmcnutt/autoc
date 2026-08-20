/* reduce.c — T027 scalar reference + T028 NEON, in one file so drift between them is a diff away.
 *
 * The scalar functions are the PERMANENT ORACLE (R11): small, obviously correct, never optimised. Every
 * future SIMD change is verified against them bit-for-bit. Resist improving them.
 */
#include "reduce.h"

void reduce2_scalar(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst)
{
    uint16_t x, y;
    for (y = 0; y + 1 < h; y += 2) {
        const uint8_t *r0 = src + (uint32_t)y * stride;
        const uint8_t *r1 = r0 + stride;
        uint16_t *o = dst + (uint32_t)(y / 2) * (w / 2);
        for (x = 0; x + 1 < w; x += 2)
            o[x / 2] = (uint16_t)((uint16_t)r0[x] + r0[x + 1] + r1[x] + r1[x + 1]);
    }
}

void reduce4_scalar(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst)
{
    uint16_t x, y;
    for (y = 0; y + 3 < h; y += 4) {
        uint16_t *o = dst + (uint32_t)(y / 4) * (w / 4);
        for (x = 0; x + 3 < w; x += 4) {
            uint32_t s = 0;
            uint16_t dy, dx;
            for (dy = 0; dy < 4; dy++) {
                const uint8_t *r = src + (uint32_t)(y + dy) * stride + x;
                for (dx = 0; dx < 4; dx++) s += r[dx];
            }
            o[x / 4] = (uint16_t)s;
        }
    }
}

/* ---- NEON (T028). aarch64-only by design: both the Pi and the dev box are aarch64 (spec §16.2 — every
 * host in this project is), so there is no x86 fallback to rot. The WSL2 cross build targets aarch64 too. */
#if defined(__aarch64__)
#include <arm_neon.h>

void reduce2_neon(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst)
{
    uint16_t x, y;
    for (y = 0; y + 1 < h; y += 2) {
        const uint8_t *r0 = src + (uint32_t)y * stride;
        const uint8_t *r1 = r0 + stride;
        uint16_t *o = dst + (uint32_t)(y / 2) * (w / 2);
        x = 0;
        /* 32 source px -> 16 horizontal pair-sums per row (u16), add rows -> 16 outputs per iteration. */
        for (; x + 31 < w; x += 32) {
            uint8x16x2_t a0 = vld2q_u8(r0 + x);   /* de-interleave: even px / odd px                  */
            uint8x16x2_t a1 = vld2q_u8(r1 + x);
            uint16x8_t e0 = vaddl_u8(vget_low_u8(a0.val[0]), vget_low_u8(a0.val[1]));
            uint16x8_t e1 = vaddl_u8(vget_high_u8(a0.val[0]), vget_high_u8(a0.val[1]));
            uint16x8_t f0 = vaddl_u8(vget_low_u8(a1.val[0]), vget_low_u8(a1.val[1]));
            uint16x8_t f1 = vaddl_u8(vget_high_u8(a1.val[0]), vget_high_u8(a1.val[1]));
            vst1q_u16(o + x / 2,     vaddq_u16(e0, f0));
            vst1q_u16(o + x / 2 + 8, vaddq_u16(e1, f1));
        }
        for (; x + 1 < w; x += 2)                  /* scalar tail — same arithmetic, same bits        */
            o[x / 2] = (uint16_t)((uint16_t)r0[x] + r0[x + 1] + r1[x] + r1[x + 1]);
    }
}

void reduce4_neon(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst)
{
    uint16_t x, y;
    for (y = 0; y + 3 < h; y += 4) {
        const uint8_t *r0 = src + (uint32_t)y * stride;
        const uint8_t *r1 = r0 + stride;
        const uint8_t *r2 = r1 + stride;
        const uint8_t *r3 = r2 + stride;
        uint16_t *o = dst + (uint32_t)(y / 4) * (w / 4);
        x = 0;
        for (; x + 31 < w; x += 32) {
            /* Vertical first: four u8 rows -> u16 column sums (max 1020, no overflow). */
            uint8x16_t a = vld1q_u8(r0 + x), b = vld1q_u8(r1 + x);
            uint8x16_t c = vld1q_u8(r2 + x), d = vld1q_u8(r3 + x);
            uint16x8_t lo = vaddq_u16(vaddl_u8(vget_low_u8(a), vget_low_u8(b)),
                                      vaddl_u8(vget_low_u8(c), vget_low_u8(d)));
            uint16x8_t hi = vaddq_u16(vaddl_u8(vget_high_u8(a), vget_high_u8(b)),
                                      vaddl_u8(vget_high_u8(c), vget_high_u8(d)));
            /* Horizontal: pairwise twice folds 4 columns -> 1. 16 col-sums -> 4 outputs per q-reg. */
            uint16x8_t p = vpaddq_u16(lo, hi);            /* 16 -> 8  */
            uint8x16_t e = vld1q_u8(r0 + x + 16), f = vld1q_u8(r1 + x + 16);
            uint8x16_t g = vld1q_u8(r2 + x + 16), hh = vld1q_u8(r3 + x + 16);
            uint16x8_t lo2 = vaddq_u16(vaddl_u8(vget_low_u8(e), vget_low_u8(f)),
                                       vaddl_u8(vget_low_u8(g), vget_low_u8(hh)));
            uint16x8_t hi2 = vaddq_u16(vaddl_u8(vget_high_u8(e), vget_high_u8(f)),
                                       vaddl_u8(vget_high_u8(g), vget_high_u8(hh)));
            uint16x8_t p2 = vpaddq_u16(lo2, hi2);
            vst1q_u16(o + x / 4, vpaddq_u16(p, p2));      /* 8+8 -> 8 outputs                        */
        }
        for (; x + 3 < w; x += 4) {
            uint32_t s = 0;
            uint16_t dy, dx;
            for (dy = 0; dy < 4; dy++) {
                const uint8_t *r = src + (uint32_t)(y + dy) * stride + x;
                for (dx = 0; dx < 4; dx++) s += r[dx];
            }
            o[x / 4] = (uint16_t)s;
        }
    }
}
#endif /* __aarch64__ */
