#pragma once

#include <vector>
#include <cmath>          // 037: std::sqrt for the thr^2.5 energy curve
#include "autoc/types.h"
#include "autoc/eval/fitness_computer.h"
#include "autoc/rpc/crash_reason.h"
#include "autoc/eval/aircraft_state.h"   // AircraftState (computeSpanPredictionError)
#include "autoc/eval/arena.h"            // 041 P2-4 — ArenaEgressKind attribution
#include "autoc/rpc/protocol.h"          // CameraViewSample

// Forward declaration — full definition in rpc/protocol.h
struct EvalResults;

// Per-scenario fitness score (022 + 027 v4).
//
// All score fields use `gp_fitness` (= double) deliberately. Per-tick
// contributions are small (~-1) but accumulate across 245 scenarios ×
// hundreds of ticks; double precision avoids any sum-of-tiny-floats drift
// concerns, even when later summed into population-level aggregates.
//
// Three physics-grounded dimensions, all "lower = better":
// - `score`: accumulated tracking fitness (negated points). Streak multiplied.
// - `stability_score` (027 v4): time pitch/roll surfaces spend off-center.
//   Σ_t (|out_pt_t| - 1) + (|out_rl_t| - 1) over completed ticks. Each tick
//   contributes -2 (both surfaces centered, ideal) to 0 (both saturated, worst).
// - `energy_score` (035 FR-001b/R1, restored 041 P2-7): convex throttle-command
//   integral — the term that de-pegged throttle 0.93 -> 0.72 at 035.
//   Σ_t ((out_th_t + 1) / 2)² over completed ticks — out_th∈[-1,1] mapped to a
//   [0,1] throttle fraction, squared (super-linear). Each tick contributes
//   0 (idle) to 1 (full throttle). ≥0, lower = better. (Replaces the 027 v3
//   linear sign-wrong placeholder `Σ(out_th−1)/2`.)
//
// All three additive per tick AND per scenario — no normalization,
// no streak amplification on the physics terms. The "all-attitude smooth
// tracking" objective: be on the path with surfaces near center and
// throttle at minimum. Lexicase uses each dimension as its own test case
// per scenario; no weighting, no fine-tuning, multi-objective.
// 030 M11.wrap T088 — tracker-mode per-scenario diagnostics.
// Populated only when computeScenarioScores runs the tracker branch;
// pathgen scenarios leave this zero-initialized. All fields observation-only
// (no selection pressure).
struct TrackerDiag {
    // Streak-loss reason counters (T088 — GEOMETRIC streak-break taxonomy, polish
    // 2026-05-11). Incremented on each tick where streakCount transitions > 0 → 0;
    // exactly one bucket per event. NOTE: a previous version included
    // `loss_vis_fov` / `loss_vis_cone` for visibility-driven breaks, but the
    // streak counter is reset only via `stepPoints < streakThreshold` (see
    // FitnessComputer::applyStreak), so the visibility branch was unreachable
    // dead code. Visibility events are captured separately via `vis_frac` +
    // `max_lost_sight_run` below.
    int loss_geom_too_far = 0;     // computeStepScore < threshold, distTermSq dominates
    int loss_geom_angle = 0;       // computeStepScore < threshold, angleTermSq dominates
    int loss_geom_overshoot = 0;   // along > 0 (chase forward of rabbit, sharp ahead-ramp)
    int loss_hull = 0;             // hull strike fired this tick (scenario-terminating; max 1)

    // Visibility + ramp eligibility (per-scenario fractions, [0,1]).
    float vis_frac = 0.0f;         // ticks with ≥1 beacon visible / total ticks
    float in_fit_ramp_frac = 0.0f; // ticks computeStepScore ≥ threshold / total ticks

    // Range / closure / overrun stats (chase→target).
    float range_min = 0.0f;        // m, min chase→target distance over scenario
    float range_med = 0.0f;        // m, median
    float range_p95 = 0.0f;        // m, 95th percentile
    int closure_flips = 0;         // sign reversals of d(range)/dt (overshoot-recover count)
    float max_closure_rate = 0.0f; // m/s, peak signed closure rate (+= closing)

    // Forward-looking diagnostics (327-330 backlog entry).
    int max_lost_sight_run = 0;    // ticks, longest run of "both beacons sentinel"
    float spiral_ratio = 0.0f;     // mean per-tick |gyro| / max(|vel|, 1) — high = spiraling
    float thrash_rate_pt = 0.0f;   // pitch-axis transitions per second (|d(out_pt)| > 0.5)
    float thrash_rate_rl = 0.0f;   // roll-axis transitions per second
};

struct ScenarioScore {
    gp_fitness score;            // Tracking (negated accumulated points, lower = better)
    gp_fitness stability_score;  // Per-tick (|out_pt|-1) + (|out_rl|-1) summed; lower = better
    // Convex integral of commanded THROTTLE POWER — 035 FR-001b, restored at
    // 041 P2-7 after the Es-destroyed variant re-pegged throttle at 1.000.
    // Always >= 0; lower = better.
    //
    // ⚠️ It charges for power SPENT, not for energy lost. That distinction is
    // the whole point: Es-destroyed rewarded full throttle (throttle raises Es)
    // and charged manoeuvring instead. See the accumulation site for the
    // 034-vs-035 evidence.
    gp_fitness energy_score;
    // 038 US3 — aux span/closure-predictor error (tracker-only; 0 in pathgen and
    // when no CEP-visible (t, t+horizon) pairs exist). Mean |predicted_span −
    // realized_span| over the kSpanPredictHorizonsMsec lookaheads + the closure
    // rate; lower = better (a lexicase axis, gated on EnablePredictorHead). NOT
    // touched by applyCrashPenalty. Beats-persistence is a US3-gate check.
    gp_fitness prediction_score;
    bool crashed;
    CrashReason crashReason;

    // 041 P2-4 — WHICH BOUND was crossed, for M1 as well as M2.
    //
    // ⛔ `CrashReason::Eval` lumps floor, ceiling and wall into one number, so
    // the per-gen `#GenCrash eval=` count could not answer the question the 041
    // asymmetric band makes important: with only 10 m below the arm point, is a
    // run dying because the DECK is too tight, or because the policy is being
    // drawn down onto it by a target flying near it? Those call for opposite
    // responses — widen the band, or leave it and let the policy learn — and
    // widening in response to the second removes the lesson.
    //
    // Attributed parent-side from the terminal position against the RECORDED
    // arena, so it costs no worker change and no wire change. NONE when the
    // scenario did not end on egress.
    autoc::eval::ArenaEgressKind egress_kind;     // 030 M11.wrap diagnostics — full terminate reason mirror of `crashed` (which is just isCrash(crashReason))
    int steps_completed;
    int steps_total;

    // Streak diagnostics (tracking-only, for diagnostics not selection)
    int maxStreak;
    int totalStreakSteps;
    gp_fitness maxMultiplier;

    // 030 M11.wrap — tracker-mode diagnostics; zero in pathgen mode.
    TrackerDiag tracker_diag;

    ScenarioScore()
        : score(0.0), stability_score(0.0), energy_score(0.0),
          prediction_score(0.0),
          crashed(false), crashReason(CrashReason::None),
          egress_kind(autoc::eval::ArenaEgressKind::NONE),
          steps_completed(0), steps_total(0),
          maxStreak(0), totalStreakSteps(0), maxMultiplier(1.0) {}
};

// 035 FR-001b/R1 -- per-tick convex throttle energy. out_th in [-1,1] (tanh NN
// output) maps to a [0,1] throttle fraction. 037 (2026-06-09): curve is thr^2.5
// (was thr^2) -- real prop input power ~ throttle^2.5..3; 2.5 chosen as the
// closer-to-real "slightly different" curve. Still convex, >=0, lower=better.
// Pure + inline so the formula is unit-testable in isolation (energy_metric_tests).
inline gp_fitness throttleEnergyStep(gp_fitness out_th) {
    const gp_fitness thr = (out_th + static_cast<gp_fitness>(1.0)) * static_cast<gp_fitness>(0.5);
    return thr * thr * std::sqrt(thr);  // thr^2.5
}

// Compute per-scenario scores from EvalResults using point-accumulation fitness.
// variationScale: 0.0 (no variations) to 1.0 (full variations), used for streak threshold ramp.
// Returns one ScenarioScore per scenario (path×wind combination).
std::vector<ScenarioScore> computeScenarioScores(EvalResults& evalResults);

// 038 US3 — aux span/closure-prediction error, i.e. the `prediction_score`
// lexicase axis. Exposed for test (2026-08-02) because the TICK PAIRING between
// the two arrays is a SILENT invariant: `cams[j]` is the view from tick `j+1`,
// since the initial pre-tick state is pushed to `states` with no matching
// camera view. Getting it wrong scores every forecast one tick late and nothing
// else notices — which is exactly what happened from 038 US3 until 2026-08-02.
// See SpanPrediction.PerfectPredictorScoresExactlyZero.
// 041 T022 — takes ONE series. Previously two parallel vectors reconciled with
// a `- 1` offset inside; see the definition for what that cost.
gp_fitness computeSpanPredictionError(const std::vector<EvalTick>& ticks);

// Aggregate: sum of per-scenario scores (already negated, lower is better).
double aggregateRawFitness(const std::vector<ScenarioScore>& scores);
