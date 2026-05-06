#include "autoc/nn/mode.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"

#include <cstdio>
#include <cstdlib>

namespace {

// kTrackerMode gather stub. Replaced with the real gather_tracker_inputs
// in M5 when the projection module + tracker NNInputs storage land. Until
// then, selecting tracker mode (e.g. `autoc -i autoc-tracker.ini`) is a
// setup bug — fail loud rather than silently produce garbage outputs.
void gather_tracker_inputs_stub(PathProvider&, AircraftState&, NNInputs&) {
    std::fprintf(stderr,
        "FATAL: tracker-mode sensor gather invoked but not yet implemented.\n"
        "Tracker-mode runtime support lands in 030 M5 (camera projection +\n"
        "TrackerInputs storage). Use `-i autoc.ini` for pathgen mode in the\n"
        "interim; see specs/030-tracker-mode/plan.md M5.\n");
    std::abort();
}

}  // namespace

const ModeStrategy kPathgenMode = {
    &gather_pathgen_inputs,
    static_cast<int>(PathgenInput::COUNT),
    "pathgen",
};

const ModeStrategy kTrackerMode = {
    &gather_tracker_inputs_stub,
    static_cast<int>(TrackerInput::COUNT),
    "tracker",
};
