#pragma once

// 030 M6a — PathgenStepper (FR-019 mode dispatch).
//
// Pathgen-mode per-tick logic extracted from minisim's worker loop. Wraps
// the existing path-following body byte-identically — rebuild-perf.sh +
// autoc-eval bitwise on baseline gen9200.dmp must hold across the M6a
// extraction commit. No behavior change.

#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/scenario_stepper.h"
#include "autoc/nn/evaluator.h"          // NNControllerBackend
#include "autoc/rpc/protocol.h"          // CrashReason, ScenarioMetadata, Path
#include "autoc/types.h"

namespace autoc::eval {

class PathgenStepper : public ScenarioStepper {
public:
    PathgenStepper(NNControllerBackend& nn,
                   AircraftState& state,
                   const std::vector<Path>& path,
                   const ScenarioMetadata& scenario_meta);

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
};

}  // namespace autoc::eval
