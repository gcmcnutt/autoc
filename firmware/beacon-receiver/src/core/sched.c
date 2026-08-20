/* sched.c — T042: acquisition-budget virtualisation (R3, the non-obvious half of replay parity).
 *
 * The problem: live, a full-field acquisition pass takes real milliseconds on a loaded CPU, so its
 * results land ~N frames after the frames they were computed from. In replay, compute is instant. If
 * results were applied "when ready", replay would acquire EARLIER than live and the record streams
 * would diverge — parity dead, and worse, replay would flatter the tracker.
 *
 * The fix: a COST MODEL, not a clock. An acquisition pass requested at frame F completes at frame
 * F + ceil(cost_us / frame_period_us), by definition, in BOTH live and replay. Live compute that
 * finishes early WAITS for the model; live compute that finishes late is a budget violation that gets
 * counted (and must move the config cost, not the model). Determinism comes from the model being the
 * only authority on completion time.
 */
#include "sched_virt.h"

void sched_init(Sched *s, const BcnConfig *cfg, uint32_t frame_period_us)
{
    s->cost_us = cfg->acquire_cost_us_per_pass;
    s->passes_max = cfg->acquire_passes_max;
    s->frame_period_us = frame_period_us ? frame_period_us : 1u;
    s->inflight = 0u;
    s->complete_at = 0u;
    s->passes_run = 0u;
    s->budget_violations = 0u;
}

int sched_try_begin(Sched *s, uint64_t frame_idx)
{
    if (s->inflight) return 0;
    if (s->passes_run >= s->passes_max && s->passes_max) {
        /* passes_max bounds OUTSTANDING passes per acquisition episode; a completed episode resets via
         * sched_reset_episode. Without the bound, replay could queue unbounded virtual work. */
        return 0;
    }
    s->inflight = 1u;
    s->complete_at = frame_idx + (s->cost_us + s->frame_period_us - 1u) / s->frame_period_us;
    s->passes_run++;
    return 1;
}

int sched_completed(Sched *s, uint64_t frame_idx)
{
    if (!s->inflight || frame_idx < s->complete_at) return 0;
    s->inflight = 0u;
    return 1;
}

void sched_reset_episode(Sched *s) { s->passes_run = 0u; }
