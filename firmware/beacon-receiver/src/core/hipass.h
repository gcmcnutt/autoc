/* hipass.h — per-frame spatial high-pass (R11).
 *
 * Purpose: make a point source stand out of smooth background before TEMPORAL work (blink detection in
 * acquisition; local-floor normalisation for lock_health). The beacon is ≤2 px; scene structure is
 * mostly low-frequency. Definition, chosen for exact integer reversibility and NEON friendliness:
 *
 *     out[y][x] = 9 * in[y][x] - sum3x3(in, x, y)         (int32 in, int32 out)
 *
 * i.e. 8×(centre − local mean) computed without division. Border pixels (no full 3×3 support) are ZERO —
 * defined, not "whatever the vector tail did", because bit-exactness includes the edges.
 * Operates on the u16 REDUCED planes (the ladder's working domain), not the raw u8 frame.
 */
#ifndef BEACON_HIPASS_H
#define BEACON_HIPASS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hipass_scalar(const uint16_t *src, uint16_t w, uint16_t h, int32_t *dst);
void hipass_neon(const uint16_t *src, uint16_t w, uint16_t h, int32_t *dst);

#ifdef __cplusplus
}
#endif
#endif
