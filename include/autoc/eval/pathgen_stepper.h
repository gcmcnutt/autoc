#pragma once

// 030 M6a — PathgenStepper (FR-019 mode dispatch).
//
// Pathgen-mode per-tick logic extracted from minisim's worker loop. Wraps
// the existing path-following body byte-identically — rebuild-perf.sh +
// autoc-eval bitwise on baseline gen9200.dmp must hold across the M6a
// extraction commit. No behavior change.

#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/derived_features.h"  // 033 §2.B — SmoothnessMotionMode
#include "autoc/eval/scenario_stepper.h"
#include "autoc/nn/evaluator.h"          // NNControllerBackend
#include "autoc/rpc/protocol.h"          // CrashReason, ScenarioMetadata, Path
#include "autoc/types.h"

namespace autoc::eval {

class PathgenStepper : public ScenarioStepper {
public:
    // 033 §2.B — smoothness floor + motion mode are PER-RUN config passed
    // at scenario construction (taken from AutocConfig / WorkerInit at the
    // caller site). No default values per Constitution III + M2-era policy
    // (no backward-compat fallback patterns); every caller passes them
    // explicitly.
    PathgenStepper(NNControllerBackend& nn,
                   AircraftState& state,
                   const std::vector<Path>& path,
                   const ScenarioMetadata& scenario_meta,
                   gp_scalar smoothness_floor,
                   SmoothnessMotionMode smoothness_mode);

    void initScenario() override;
    CrashReason stepOnce() override;
    unsigned long durationMsec() const override { return duration_msec_; }

private:
    NNControllerBackend& nn_;
    AircraftState& state_;
    const std::vector<Path>& path_;
    ScenarioMetadata scenario_meta_;
    gp_scalar rabbit_speed_ = 0.0f;
    unsigned long duration_msec_ = 0;

    // 033 §2.B — per-tick smoothness state.
    //
    // smoothness_floor_ / smoothness_mode_: configured at construction;
    // typed in their eval-pipeline domain (gp_scalar + enum).
    //
    // prev_out_*: previous-tick NN-saturated outputs (pitch/roll/throttle)
    // tracked across ticks so smoothness Δs can be computed. prev_out_valid_
    // is false at scenario start; first-tick smoothness factor = 1.0
    // (per spec contract: no "false-positive" penalty on scenario entry).
    gp_scalar smoothness_floor_ = static_cast<gp_scalar>(1.0);
    SmoothnessMotionMode smoothness_mode_ = SmoothnessMotionMode::Pythagorean;
    gp_scalar prev_out_pt_ = static_cast<gp_scalar>(0.0);
    gp_scalar prev_out_rl_ = static_cast<gp_scalar>(0.0);
    gp_scalar prev_out_th_ = static_cast<gp_scalar>(0.0);
    bool prev_out_valid_ = false;
};

}  // namespace autoc::eval
