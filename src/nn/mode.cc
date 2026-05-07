#include "autoc/nn/mode.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/nn/topology.h"
#include "autoc/util/config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

// kTrackerMode gather signature-mismatch stub. The real gather_tracker_inputs
// (M6d, src/nn/evaluator.cc) takes `(AircraftState&, TrackerHistoryWindow&,
// gp_vec3 home, TrackerInputs&)` — a different signature than this bundle's
// `(PathProvider&, AircraftState&, NNInputs&)` because tracker mode reads
// caller-managed beacon history + tracker-shaped inputs, not pathgen path +
// pathgen-shaped inputs. The strategy-bundle abstraction designed in M2b
// no longer fits cleanly post-M6d; tracker-mode dispatch happens at the
// ScenarioStepper level (PathgenStepper vs TrackerStepper) instead. This
// bundle stays for the data.dat header walk + xiao firmware mode-name
// dispatch, but the gather function pointer is a guard rail (caller error
// if it ever fires, since the real call goes through ScenarioStepper).
void gather_tracker_inputs_signature_mismatch_guard(PathProvider&, AircraftState&, NNInputs&) {
    std::fprintf(stderr,
        "FATAL: kTrackerMode.gather_inputs invoked through the M2b strategy\n"
        "bundle, but tracker-mode dispatch moved to ScenarioStepper at M6d.\n"
        "Caller should be using TrackerStepper.stepOnce instead. This is a\n"
        "wiring bug — see src/eval/tracker_stepper.cc (M6d) for the right\n"
        "entry point.\n");
    std::abort();
}

}  // namespace

const ModeStrategy kPathgenMode = {
    &gather_pathgen_inputs,
    static_cast<int>(PathgenInput::COUNT),
    "pathgen",
    NN_TOPOLOGY,
    NN_RECURRENT,
    NN_NUM_LAYERS,
    NN_WEIGHT_COUNT,
    NN_HIDDEN_STATE_COUNT,
    NN_TOPOLOGY_STRING,
};

const ModeStrategy kTrackerMode = {
    &gather_tracker_inputs_signature_mismatch_guard,
    static_cast<int>(TrackerInput::COUNT),
    "tracker",
    TRACKER_NN_TOPOLOGY,
    TRACKER_NN_RECURRENT,
    TRACKER_NN_NUM_LAYERS,
    TRACKER_NN_WEIGHT_COUNT,
    TRACKER_NN_HIDDEN_STATE_COUNT,
    TRACKER_NN_TOPOLOGY_STRING,
};

const ModeStrategy& getActiveModeStrategy() {
    return getModeStrategyByName(ConfigManager::getConfig().mode.c_str());
}

const ModeStrategy& getModeStrategyByName(const char* name) {
    if (std::strcmp(name, "tracker") == 0) return kTrackerMode;
    return kPathgenMode;  // default for "pathgen" or any unrecognized name
}
