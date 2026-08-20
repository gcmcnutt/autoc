/* corr.h — Gold-31 temporal correlation (T031/T032, R2 fixed-point).
 *
 * Everything here is integer. Bit-exact replay across A53/A76/Grace is a hard requirement, and float FMA
 * contraction differs between them — so float is banned, not discouraged.
 *
 * Domain: a WINDOW of W chip bins. Each bin holds the sum of the normalized frame samples that landed in
 * that chip interval (2–3 at 2.4 samples/chip) plus a count. corr works on per-bin means, mean-removed
 * over the window — that kills the DC pedestal exactly (a Gold code is near-balanced: 16 ones / 15 zeros).
 */
#ifndef BEACON_CORR_H
#define BEACON_CORR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CORR_N 31              /* code length in chips; config n_chips must equal it (loader range 1..32,
                                * the engine checks ==31 — the template table is Gold-31 by construction) */

/* code_bits: chip 0 in bit N-1, MSB-first — gold_codes.h convention. out[k] = ±1. */
void corr_template(uint32_t code_bits, int8_t out[CORR_N]);

/* Which chip (absolute index since epoch) does time t_us fall in, at chip_hz_q8?
 * chip = (t_us - epoch_us) * chip_hz / 1e6, in 64-bit integer arithmetic, floor. */
int64_t corr_chip_at(uint64_t t_us, uint64_t epoch_us, uint32_t chip_hz_q8);

typedef struct {
    int32_t corr;      /* peak correlation (signed; sign folds into polarity)                          */
    int32_t energy;    /* L1 energy of the mean-removed window — the q denominator                     */
    uint16_t q_q8;     /* 256 * |corr| / energy, clamped; scale-free quality                           */
    uint8_t phase;     /* best phase 0..30 (chip index of template start within the window)            */
    uint8_t code_id;   /* search only: 0 = A, 1 = B                                                    */
} CorrResult;

/* T032: the two entry points differ ~60x and the CHEAP one is the common path — keep them separate.
 *
 * corr_search: W bins (W >= CORR_N), both codes x 31 phases. ~2·31·W MACs. Cold acquisition / identity
 * re-verification. bins/counts: per-chip sums and sample counts; count 0 = missing chip (occluded or a
 * burst boundary) — it contributes nothing rather than a fake zero sample.
 * corr_track: ONE code, ONE phase: W MACs. The per-tick reaffirmation path. */
void corr_search(const int32_t *bins, const uint8_t *counts, uint16_t W,
                 const int8_t tmpl_a[CORR_N], const int8_t tmpl_b[CORR_N], CorrResult *out);
void corr_track(const int32_t *bins, const uint8_t *counts, uint16_t W,
                const int8_t tmpl[CORR_N], uint8_t phase, CorrResult *out);

#ifdef __cplusplus
}
#endif
#endif
