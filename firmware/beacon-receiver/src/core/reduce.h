/* reduce.h — software binning: the scale ladder's planes (spec §2.2, R4/R11).
 *
 * u8 frame -> u16 sums. SUMS, not averages: division discards the low bits that are the whole point of
 * binning (4×4 = 16× signal into one bin), and keeping integers exact is what makes scalar-vs-NEON
 * bit-exactness (R11) a meaningful claim instead of an epsilon test.
 */
#ifndef BEACON_REDUCE_H
#define BEACON_REDUCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 2×2 sum: (w,h) -> (w/2,h/2). Output max 4*255 = 1020, fits u16 with headroom. */
void reduce2_scalar(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst);
/* 4×4 sum: (w,h) -> (w/4,h/4). Max 16*255 = 4080. */
void reduce4_scalar(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst);

/* NEON (T028): fused per-tile pass — bit-exact vs the scalar reference (T029), or it does not ship. */
void reduce2_neon(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst);
void reduce4_neon(const uint8_t *src, uint16_t stride, uint16_t w, uint16_t h, uint16_t *dst);

#ifdef __cplusplus
}
#endif
#endif
