#pragma once

// 030 M2b — Pluggable mode dispatch (FR-019).
//
// One autoc binary supports both pathgen (33-input) and tracker (48-input)
// training modes. The active mode is selected ONCE at autoc startup based
// on which config file was passed (autoc.ini → pathgen, autoc-tracker.ini
// → tracker — see FR-011 mutual exclusion). Per-mode operations are
// dispatched through a function-pointer bundle (no virtual class hierarchy,
// no scattered if-mode branches) — picked over GoF Strategy/virtual-class
// subclassing 2026-05-06 because pathgen and tracker are not an "is-a"
// hierarchy, just two implementations of the same operations.
//
// Hot path is one indirect call; xiao firmware can pin the bundle at
// compile time via -DAUTOC_MODE=PATHGEN to remove the indirection
// entirely (per FR-019 compile-time mode selection on embedded).

class PathProvider;
class AircraftState;
struct NNInputs;

struct ModeStrategy {
    // Gather mode-specific NN sensor inputs. Pathgen: 33 floats matching
    // NNInputs struct field order (see include/autoc/nn/nn_inputs.h).
    // Tracker: lands in M5 alongside projection module + tracker NNInputs
    // storage; for now kTrackerMode's gather is a stub that aborts loudly.
    void (*gather_inputs)(PathProvider& pathProvider,
                          AircraftState& aircraftState,
                          NNInputs& inputs);

    int input_count;     // PathgenInput::COUNT (33) or TrackerInput::COUNT (48)
    const char* name;    // "pathgen" | "tracker"
};

extern const ModeStrategy kPathgenMode;
extern const ModeStrategy kTrackerMode;
