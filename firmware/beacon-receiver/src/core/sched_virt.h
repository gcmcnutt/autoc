/* sched_virt.h — acquisition budget virtualisation (T042, R3). See sched.c for why this exists. */
#ifndef BEACON_SCHED_VIRT_H
#define BEACON_SCHED_VIRT_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t cost_us, passes_max, frame_period_us;
    uint8_t  inflight;
    uint64_t complete_at;          /* frame index at which the in-flight pass's results APPLY          */
    uint32_t passes_run;
    uint32_t budget_violations;    /* live: compute overran the model — fix the config, not the model  */
} Sched;

void sched_init(Sched *s, const BcnConfig *cfg, uint32_t frame_period_us);
int  sched_try_begin(Sched *s, uint64_t frame_idx);    /* 1 = a pass may start now                    */
int  sched_completed(Sched *s, uint64_t frame_idx);    /* 1 = the pending pass's results apply NOW    */
void sched_reset_episode(Sched *s);

#ifdef __cplusplus
}
#endif
#endif
