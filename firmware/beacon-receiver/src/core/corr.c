/* corr.c — T031/T032. Integer only (R2). */
#include "corr.h"

void corr_template(uint32_t code_bits, int8_t out[CORR_N])
{
    int k;
    for (k = 0; k < CORR_N; k++)
        out[k] = (code_bits >> (CORR_N - 1 - k)) & 1u ? 1 : -1;
}

int64_t corr_chip_at(uint64_t t_us, uint64_t epoch_us, uint32_t chip_hz_q8)
{
    /* (dt_us * chip_hz_q8) / (256 * 1e6). dt up to ~2^41 us (life of a run) * 2^16 fits int128-free if
     * we split: whole seconds and remainder — keeps everything in 64 bits exactly. */
    uint64_t dt = t_us - epoch_us;
    uint64_t sec = dt / 1000000u, rem = dt % 1000000u;
    /* chips = sec * hz + rem * hz / 1e6, all in q8 then >> 8; exact because sec*hz_q8 < 2^63 for any
     * realistic run length (hz_q8 ~ 5e4, sec < 2^38). */
    uint64_t whole_q8 = sec * chip_hz_q8;
    uint64_t frac_q8  = (rem * chip_hz_q8) / 1000000u;
    return (int64_t)((whole_q8 + frac_q8) >> 8);
}

/* Mean-removal setup shared by both paths: per-bin mean value in q8 (sum*256/count), window mean of
 * those, deviations. Missing bins (count 0) are excluded from both mean and MAC. */
/* 64-BIT THROUGHOUT, and that is the whole point (2026-08-29). These accumulators used to be int32 with
 * saturating clamps, and at the real operating point BOTH of them railed: measured on static_ir.bcnr,
 * corr railed on 73 % of slot-ticks and energy on 80 %, the SAME 73 % railing together. When both rail,
 * q = |corr|*256/energy = 2^31*256/2^31 = 256 exactly -- so 75 % of slot-ticks reported q = 1.00 and
 * 98 % of those readings were the double-saturation artifact rather than a measurement. q gates
 * promotion, HOLD entry and the scale ladder, so it was lying to three consumers at once, and it is why
 * q flipped 0.87 / 0.51 / 1.00 on consecutive ticks of a STATIC target.
 *
 * The magnitudes are structural, not a corner case. extract() forms (sum << 14) / (exposure_us*gain/256);
 * at 45 us / gain 1.0 that is a x364 multiplier, and the aperture SUM over ~1000 native px at ~2.4
 * frames per chip reaches ~1e9 per bin before devs() multiplies by another 256. There is no int32 that
 * holds it. Widening costs one extra scratch array (124 * 8 B) and nothing measurable in time.
 *
 * dev[] is int64 too: (bins[k] * 256) / counts[k] was itself computed in int64 and then TRUNCATED into an
 * int32 slot, which is the same overflow one step earlier. */
static void devs(const int32_t *bins, const uint8_t *counts, uint16_t W,
                 int64_t *dev /* caller scratch, W entries */, int64_t *energy_out, int64_t *level_out)
{
    int64_t total = 0;
    uint32_t present = 0;
    uint16_t k;
    int64_t mean_q8;
    int64_t energy = 0;

    for (k = 0; k < W; k++) {
        if (counts[k]) {
            dev[k] = ((int64_t)bins[k] * 256) / counts[k];              /* per-bin mean, q8 */
            total += dev[k];
            present++;
        }
    }
    mean_q8 = present ? total / (int64_t)present : 0;
    for (k = 0; k < W; k++) {
        if (counts[k]) {
            dev[k] -= mean_q8;
            energy += dev[k] < 0 ? -dev[k] : dev[k];
        } else {
            dev[k] = 0;    /* excluded: contributes nothing to any phase */
        }
    }
    *energy_out = energy | 1;                          /* |1: never /0. No clamp -- see above. */
    if (level_out) *level_out = mean_q8;
}

/* The window can exceed one code (W up to 124 = 4 words); template repeats with period 31. */
#define MAX_W 124

static int64_t mac_phase(const int64_t *dev, uint16_t W, const int8_t *tmpl, uint8_t phase)
{
    int64_t acc = 0;
    uint16_t k;
    uint8_t c = phase;
    for (k = 0; k < W; k++) {
        acc += tmpl[c] > 0 ? dev[k] : -dev[k];
        c = (uint8_t)(c + 1u == CORR_N ? 0u : c + 1u);
    }
    return acc;                    /* no clamp: |acc| <= W * max|dev|, which int64 holds comfortably */
}

/* q is |corr|/energy in q8 and is BOUNDED BY 1.0 by construction: |sum of +-dev| <= sum of |dev|. It is
 * clamped to 0xFFFF only against the degenerate all-excluded window, where energy is the |1 guard. */
static uint16_t quality_q8(int64_t corr, int64_t energy)
{
    int64_t q = ((corr < 0 ? -corr : corr) * 256) / energy;
    return q > 0xFFFF ? 0xFFFF : (uint16_t)q;
}

void corr_search(const int32_t *bins, const uint8_t *counts, uint16_t W,
                 const int8_t tmpl_a[CORR_N], const int8_t tmpl_b[CORR_N], CorrResult *out)
{
    int64_t dev[MAX_W];
    int64_t energy;
    int64_t level = 0;
    int64_t best = 0;
    uint8_t best_ph = 0, best_code = 0;
    uint8_t code, ph;

    if (W > MAX_W) W = MAX_W;
    devs(bins, counts, W, dev, &energy, &level);
    for (code = 0; code < 2; code++) {
        const int8_t *t = code ? tmpl_b : tmpl_a;
        for (ph = 0; ph < CORR_N; ph++) {
            int64_t c = mac_phase(dev, W, t, ph);
            int64_t a = c < 0 ? -c : c;
            if (a > (best < 0 ? -best : best)) { best = c; best_ph = ph; best_code = code; }
        }
    }
    out->corr = best;
    out->energy = energy;
    out->level = level;
    out->q_q8 = quality_q8(best, energy);
    out->phase = best_ph;
    out->code_id = best_code;
}

void corr_track(const int32_t *bins, const uint8_t *counts, uint16_t W,
                const int8_t tmpl[CORR_N], uint8_t phase, CorrResult *out)
{
    int64_t dev[MAX_W];
    int64_t energy;
    int64_t level = 0;

    if (W > MAX_W) W = MAX_W;
    devs(bins, counts, W, dev, &energy, &level);
    out->corr = mac_phase(dev, W, tmpl, phase);
    out->energy = energy;
    out->level = level;
    out->q_q8 = quality_q8(out->corr, energy);
    out->phase = phase;
    out->code_id = 0xFF;   /* track does not decide identity */
}
