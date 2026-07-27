#include "autoc/nn/mode.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/nn/topology.h"
#include "autoc/rpc/protocol.h"   // 030 M11.preA — Mode enum

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

// 038 P0-D FR-P0H — gather_pathgen_inputs grew a FlightArena parameter
// (arena-awareness inputs), so it no longer matches this bundle's legacy
// (PathProvider&, AircraftState&, NNInputs&) function-pointer type. The
// pointer was already vestigial: real pathgen dispatch runs through
// NNControllerBackend::evaluate (which owns the config arena) and the xiao
// generated program (which bakes a compile-time arena literal), never this
// field. Keep a same-shaped guard so a stray call fails loud instead of
// silently gathering without an arena.
void gather_pathgen_inputs_signature_mismatch_guard(PathProvider&, AircraftState&, NNInputs&) {
    std::fprintf(stderr,
        "FATAL: kPathgenMode.gather_inputs invoked through the M2b strategy\n"
        "bundle, but gather_pathgen_inputs now needs a FlightArena and is\n"
        "dispatched via NNControllerBackend::evaluate (or the nn2cpp-generated\n"
        "program). This is a wiring bug — see src/nn/evaluator.cc.\n");
    std::abort();
}

}  // namespace

const ModeStrategy kPathgenMode = {
    &gather_pathgen_inputs_signature_mismatch_guard,
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

const ModeStrategy& getModeStrategy(Mode m) {
    return (m == Mode::TRACKER) ? kTrackerMode : kPathgenMode;
}

const ModeStrategy& getModeStrategyByName(const char* name) {
    // Route through the typed enum lookup so there's a single switch
    // point — keeps the string-compare confined to this one helper.
    return getModeStrategy(parseModeName(name ? name : ""));
}
