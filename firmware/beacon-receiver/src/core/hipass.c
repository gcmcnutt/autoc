/* hipass.c — T027 scalar reference + T028 NEON. out = 9*centre − sum3x3, borders defined zero.
 * Scalar is the permanent oracle (R11) — do not optimise it. */
#include "hipass.h"
#include <string.h>

void hipass_scalar(const uint16_t *src, uint16_t w, uint16_t h, int32_t *dst)
{
    uint16_t x, y;
    memset(dst, 0, (size_t)w * h * sizeof *dst);   /* borders: defined zero */
    for (y = 1; y + 1 < h; y++) {
        for (x = 1; x + 1 < w; x++) {
            int32_t s = 0;
            int dy, dx;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++)
                    s += src[(uint32_t)(y + dy) * w + (uint32_t)(x + dx)];
            dst[(uint32_t)y * w + x] = 9 * (int32_t)src[(uint32_t)y * w + x] - s;
        }
    }
}

#if defined(__aarch64__)
#include <arm_neon.h>

void hipass_neon(const uint16_t *src, uint16_t w, uint16_t h, int32_t *dst)
{
    uint16_t x, y;
    memset(dst, 0, (size_t)w * h * sizeof *dst);
    for (y = 1; y + 1 < h; y++) {
        const uint16_t *r0 = src + (uint32_t)(y - 1) * w;
        const uint16_t *r1 = src + (uint32_t)y * w;
        const uint16_t *r2 = src + (uint32_t)(y + 1) * w;
        int32_t *o = dst + (uint32_t)y * w;
        x = 1;
        for (; x + 8 < w; x += 8) {                /* 8 outputs/iter; unaligned loads give the shifts */
            /* column sums c[-1..+1] as u32 (u16 x3 can exceed u16) */
            uint32x4_t cl_lo, cl_hi, cc_lo, cc_hi, cr_lo, cr_hi;
            {
                uint16x8_t a = vld1q_u16(r0 + x - 1), b = vld1q_u16(r1 + x - 1), c = vld1q_u16(r2 + x - 1);
                cl_lo = vaddw_u16(vaddl_u16(vget_low_u16(a), vget_low_u16(b)), vget_low_u16(c));
                cl_hi = vaddw_u16(vaddl_u16(vget_high_u16(a), vget_high_u16(b)), vget_high_u16(c));
            }
            {
                uint16x8_t a = vld1q_u16(r0 + x), b = vld1q_u16(r1 + x), c = vld1q_u16(r2 + x);
                cc_lo = vaddw_u16(vaddl_u16(vget_low_u16(a), vget_low_u16(b)), vget_low_u16(c));
                cc_hi = vaddw_u16(vaddl_u16(vget_high_u16(a), vget_high_u16(b)), vget_high_u16(c));
            }
            {
                uint16x8_t a = vld1q_u16(r0 + x + 1), b = vld1q_u16(r1 + x + 1), c = vld1q_u16(r2 + x + 1);
                cr_lo = vaddw_u16(vaddl_u16(vget_low_u16(a), vget_low_u16(b)), vget_low_u16(c));
                cr_hi = vaddw_u16(vaddl_u16(vget_high_u16(a), vget_high_u16(b)), vget_high_u16(c));
            }
            {
                uint32x4_t sum_lo = vaddq_u32(vaddq_u32(cl_lo, cc_lo), cr_lo);
                uint32x4_t sum_hi = vaddq_u32(vaddq_u32(cl_hi, cc_hi), cr_hi);
                uint16x8_t ctr = vld1q_u16(r1 + x);
                uint32x4_t c9_lo = vmull_n_u16(vget_low_u16(ctr), 9);
                uint32x4_t c9_hi = vmull_n_u16(vget_high_u16(ctr), 9);
                vst1q_s32(o + x,     vsubq_s32(vreinterpretq_s32_u32(c9_lo), vreinterpretq_s32_u32(sum_lo)));
                vst1q_s32(o + x + 4, vsubq_s32(vreinterpretq_s32_u32(c9_hi), vreinterpretq_s32_u32(sum_hi)));
            }
        }
        for (; x + 1 < w; x++) {                   /* scalar tail */
            int32_t s = 0;
            int dy, dx;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++)
                    s += src[(uint32_t)(y + dy) * w + (uint32_t)(x + dx)];
            o[x] = 9 * (int32_t)r1[x] - s;
        }
    }
}
#endif /* __aarch64__ */
