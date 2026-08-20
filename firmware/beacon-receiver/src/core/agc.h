/* agc.h — exposure / gain / integration-length controllers (spec §4). */
#ifndef BEACON_AGC_H
#define BEACON_AGC_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t exposure_us;      /* desired — the app actuates it per-request                            */
    uint16_t gain_q8;
    uint8_t  settling_ticks;   /* nonzero -> AGC_SETTLING flag on emitted tracks                       */
} Agc;

void agc_init(Agc *a, const BcnConfig *cfg);
uint8_t agc_integration(const Agc *a, const BcnConfig *cfg, uint16_t q_q8, uint8_t cur);
void agc_exposure(Agc *a, const BcnConfig *cfg, uint8_t roi_peak);

#ifdef __cplusplus
}
#endif
#endif
