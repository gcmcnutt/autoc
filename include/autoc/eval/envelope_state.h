#ifndef AUTOC_EVAL_ENVELOPE_STATE_H
#define AUTOC_EVAL_ENVELOPE_STATE_H

// 041 T037/T038 (FR-015, FR-016, FR-018b) — envelope occupancy, in ONE place.
//
// WHY A SHARED HEADER. `IN_ENVELOPE` / `ENVELOPE_SECS` are populated by three
// steppers that must not drift: the crrcsim worker (M1 + M2, production), the
// test-only `TrackerStepper` reference, and — at T075 — the xiao firmware. The
// ACCUMULATOR MECHANICS are identical in all of them; only the source of the
// per-tick flag differs:
//
//   M1  flag = stepScore >= FitStreakThreshold  — the objective's own
//       threshold decision, so the observation is EXACT, in sim and in flight.
//   M2  flag = perceivedInEnvelope(...)         — estimated from perception,
//       because the tracker cannot see the true geometry.
//
// ⚠️ The flag is the OBSERVABLE envelope condition, never the fitness
// machinery's internal streak counter (FR-015). They reset on the same
// condition, but one is an input and the other is reward state.
//
// ⚠️ MILLISECOND-BASED, deliberately (FR-016). Counting ticks would make
// ENVELOPE_SECS silently rescale when the control cadence changes — the same
// trap kNNHistoryLagsMsec exists to avoid. A cadence change must re-derive.

#include <algorithm>

#include "autoc/nn/evaluator.h"  // TrackerHistoryWindow
#include "autoc/types.h"

namespace autoc {
namespace eval {

// Carried per-scenario state. Owned by the caller (the stepper), like every
// other piece of carried tracker state — and reset at every scenario boundary
// for the same reason: unreset state leaks between scenarios and breaks the
// bitwise gate.
struct EnvelopeState {
    // Consecutive milliseconds at or above the envelope condition.
    double accum_msec = 0.0;
    // This tick's flag, as an input-ready 0/1.
    bool in_envelope = false;

    void reset() {
        accum_msec = 0.0;
        in_envelope = false;
    }

    // Advance from this tick's flag.
    //
    // ⚠️ Resets on ENVELOPE EXIT ONLY (research.md R2) — not on regime change,
    // not on visibility loss. This matches the reward's own reset condition
    // (`fitness_computer.cc:60-64`), which is what makes ENVELOPE_SECS a
    // faithful proxy for the streak multiplier rather than a lookalike.
    void advance(bool inside, double tick_msec) {
        in_envelope = inside;
        if (inside) {
            accum_msec += tick_msec;
        } else {
            accum_msec = 0.0;
        }
    }

    // min(seconds_in_envelope / rampSec, 1). LINEAR — no log, no tanh
    // (spec.md § Clarifications). 1.0 means "multiplier saturated", which is
    // the decision-relevant boundary, so squashing it would blur the one value
    // the controller most needs to read.
    //
    // A non-positive ramp returns 0 rather than dividing: there is no
    // defensible answer, and guessing one is the failure mode this header
    // exists to prevent (same stance as specific_force.h on gravity).
    gp_scalar normalizedSecs(double ramp_sec) const {
        if (!(ramp_sec > 0.0)) return static_cast<gp_scalar>(0);
        const double ramp_msec = ramp_sec * 1000.0;
        return static_cast<gp_scalar>(std::min(1.0, accum_msec / ramp_msec));
    }
};

// Thresholds for the M2 direct-perception estimator. Passed explicitly rather
// than reached for out of ConfigManager — the estimator runs inside the
// per-tick rule, which both steppers and (later) the firmware share, and none
// of those three has a ConfigManager. Constitution VII: no in-class defaults,
// so adding a field is a compile error at the call site.
struct EnvelopeEstimatorConfig {
    gp_scalar cep_visible_threshold;  // both beacons must be below this
    gp_scalar span_lo;                // rad, pair separation lower bound (far edge)
    gp_scalar span_hi;                // rad, pair separation upper bound (near edge)
    gp_scalar centroid_radius;        // rad, max pair-centroid offset from boresight
};

// 041 T038 (FR-018b) — the M2 direct-perception envelope estimator.
//
// M2 has no privileged access to the true along/lateral geometry M1 scores
// against, so "am I in the scoring envelope" has to be INFERRED from what the
// camera returns. The three conditions are a perception-side restatement of
// the same question:
//
//   both beacons visible   — a one-beacon fix has no span and no reliable
//                            centroid, so the estimate would be a guess
//   span within [lo, hi]   — span is the range proxy: too small = too far
//                            back, too large = too close / overshooting
//   centroid within radius — the pair sits near boresight, i.e. the target is
//                            ahead rather than off in the periphery
//
// ⚠️ Reads the "now" slot (index 5) only. This is deliberately instantaneous:
// the accumulator above is what carries time, and using a lagged slot here
// would put the flag and its own duration counter out of step.
//
// ⚠️ Estimator FIDELITY is characterised in Phase C/D, not assumed here. The
// slots are populated so the policy can learn what the estimate is worth; the
// M1 flag is exact, this one is not, and the difference is the measurement.
inline bool perceivedInEnvelope(const TrackerHistoryWindow& history,
                                const EnvelopeEstimatorConfig& cfg) {
    constexpr int kNow = 5;

    const bool both_visible =
        (history.left_cep[kNow]  < cfg.cep_visible_threshold) &&
        (history.right_cep[kNow] < cfg.cep_visible_threshold);
    if (!both_visible) return false;

    // Span is already CEP-gated at projection time (substituted with a neutral
    // 0.0 when either endpoint is untrusted), so the visibility test above and
    // the lower bound below agree by construction rather than by coincidence.
    const gp_scalar span = static_cast<gp_scalar>(history.span[kNow]);
    if (span < cfg.span_lo || span > cfg.span_hi) return false;

    const gp_scalar cx = static_cast<gp_scalar>(
        0.5f * (history.left_x[kNow] + history.right_x[kNow]));
    const gp_scalar cy = static_cast<gp_scalar>(
        0.5f * (history.left_y[kNow] + history.right_y[kNow]));
    const gp_scalar centroid_offset =
        static_cast<gp_scalar>(std::sqrt(static_cast<double>(cx * cx + cy * cy)));

    return centroid_offset <= cfg.centroid_radius;
}

}  // namespace eval
}  // namespace autoc

#endif  // AUTOC_EVAL_ENVELOPE_STATE_H
