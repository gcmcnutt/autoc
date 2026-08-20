/* agc.c — T038: the three coupled controllers (spec §4). Stage 1 wires the INTEGRATION-LENGTH loop (the
 * responsiveness lever) and computes desired exposure/gain for the app to actuate per-request; free-
 * running camera auto-anything is never acceptable (README §Measured facts, twice).
 */
#include "agc.h"

void agc_init(Agc *a, const BcnConfig *cfg)
{
    a->exposure_us = cfg->exposure_min_us;   /* bench operating point boots here                       */
    a->gain_q8 = cfg->gain_min_q8;
    a->settling_ticks = 0;
}

/* Integration length: SNR-driven (spec §4). Once the DPLL holds phase a fix does not need a full word —
 * near field runs short (low latency, low smear), far field runs long. Bounds from config; identity is
 * re-verified on a full word by the tick loop regardless (corr_search each tick). */
uint8_t agc_integration(const Agc *a, const BcnConfig *cfg, uint16_t q_q8, uint8_t cur)
{
    uint16_t hi = (uint16_t)(cfg->q_lock_q8 + ((256u - cfg->q_lock_q8) / 2u));
    uint8_t next = cur;
    (void)a;
    if (q_q8 > hi && cur > cfg->integration_min_chips) {
        next = (uint8_t)(cur / 2u);           /* strong: halve toward responsiveness                   */
        if (next < cfg->integration_min_chips) next = cfg->integration_min_chips;
    } else if (q_q8 < cfg->q_lock_q8 && cur < cfg->integration_max_chips) {
        next = (uint8_t)(cur * 2u);           /* weak: double toward processing gain                   */
        if (next > cfg->integration_max_chips) next = cfg->integration_max_chips;
    }
    return next;
}

/* Exposure/gain, ROI-driven (spec §4: a global statistic lets one sun glint starve every real track).
 * roi_peak = the brightest raw pixel across the TRACKED ROIs this tick. Doubling/halving with a dead
 * band — same policy the Python tracker's --agc used, now restart-free because src_libcamera applies it
 * per-request. Marks AGC_SETTLING for the flag bit. */
void agc_exposure(Agc *a, const BcnConfig *cfg, uint8_t roi_peak)
{
    uint32_t e = a->exposure_us;
    if (a->settling_ticks) { a->settling_ticks--; return; }
    if (roi_peak >= cfg->exposure_target_hi && e > cfg->exposure_min_us) {
        e /= 2u;
        if (e < cfg->exposure_min_us) e = cfg->exposure_min_us;
    } else if (roi_peak > 0u && roi_peak < cfg->exposure_target_lo && e < cfg->exposure_max_us) {
        e *= 2u;
        if (e > cfg->exposure_max_us) e = cfg->exposure_max_us;
    }
    if (e != a->exposure_us) {
        a->exposure_us = e;
        a->settling_ticks = 3;                /* integration windows spanning the change get flagged   */
    }
}
