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
static void devs(const int32_t *bins, const uint8_t *counts, uint16_t W,
                 int32_t *dev /* caller scratch, W entries */, int32_t *energy_out, int32_t *level_out)
{
    int64_t total = 0;
    uint32_t present = 0;
    uint16_t k;
    int32_t mean_q8;
    int64_t energy = 0;

    for (k = 0; k < W; k++) {
        if (counts[k]) {
            dev[k] = (int32_t)(((int64_t)bins[k] * 256) / counts[k]);   /* per-bin mean, q8 */
            total += dev[k];
            present++;
        }
    }
    mean_q8 = present ? (int32_t)(total / (int64_t)present) : 0;
    for (k = 0; k < W; k++) {
        if (counts[k]) {
            dev[k] -= mean_q8;
            energy += dev[k] < 0 ? -(int64_t)dev[k] : (int64_t)dev[k];
        } else {
            dev[k] = 0;    /* excluded: contributes nothing to any phase */
        }
    }
    *energy_out = energy > 0x7FFFFFFF ? 0x7FFFFFFF : (int32_t)(energy | 1);   /* |1: never /0 */
    if (level_out) *level_out = mean_q8;
}

/* The window can exceed one code (W up to 124 = 4 words); template repeats with period 31. */
#define MAX_W 124

static int32_t mac_phase(const int32_t *dev, uint16_t W, const int8_t *tmpl, uint8_t phase)
{
    int64_t acc = 0;
    uint16_t k;
    uint8_t c = phase;
    for (k = 0; k < W; k++) {
        acc += tmpl[c] > 0 ? (int64_t)dev[k] : -(int64_t)dev[k];
        c = (uint8_t)(c + 1u == CORR_N ? 0u : c + 1u);
    }
    return acc > 0x7FFFFFFF ? 0x7FFFFFFF : acc < -0x7FFFFFFF ? -0x7FFFFFFF : (int32_t)acc;
}

static uint16_t quality_q8(int32_t corr, int32_t energy)
{
    int64_t q = ((int64_t)(corr < 0 ? -corr : corr) * 256) / energy;
    return q > 0xFFFF ? 0xFFFF : (uint16_t)q;
}

void corr_search(const int32_t *bins, const uint8_t *counts, uint16_t W,
                 const int8_t tmpl_a[CORR_N], const int8_t tmpl_b[CORR_N], CorrResult *out)
{
    int32_t dev[MAX_W];
    int32_t energy;
    int32_t level = 0;
    int32_t best = 0;
    uint8_t best_ph = 0, best_code = 0;
    uint8_t code, ph;

    if (W > MAX_W) W = MAX_W;
    devs(bins, counts, W, dev, &energy, &level);
    for (code = 0; code < 2; code++) {
        const int8_t *t = code ? tmpl_b : tmpl_a;
        for (ph = 0; ph < CORR_N; ph++) {
            int32_t c = mac_phase(dev, W, t, ph);
            int32_t a = c < 0 ? -c : c;
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
    int32_t dev[MAX_W];
    int32_t energy;
    int32_t level = 0;

    if (W > MAX_W) W = MAX_W;
    devs(bins, counts, W, dev, &energy, &level);
    out->corr = mac_phase(dev, W, tmpl, phase);
    out->energy = energy;
    out->level = level;
    out->q_q8 = quality_q8(out->corr, energy);
    out->phase = phase;
    out->code_id = 0xFF;   /* track does not decide identity */
}
