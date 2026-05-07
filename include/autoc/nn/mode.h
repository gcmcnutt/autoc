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
    // Tracker: gather happens via gather_tracker_inputs(...) which has
    // a different signature (takes TrackerInputs + history + arena);
    // dispatched at the ScenarioStepper level (TrackerStepper.stepOnce),
    // not through this bundle. The function pointer below is a guard
    // rail that aborts loud if pathgen-style dispatch ever fires for
    // tracker mode (caller bug — should be using TrackerStepper).
    void (*gather_inputs)(PathProvider& pathProvider,
                          AircraftState& aircraftState,
                          NNInputs& inputs);

    int input_count;     // PathgenInput::COUNT (33) or TrackerInput::COUNT (45)
    const char* name;    // "pathgen" | "tracker"

    // 030 M7a — Topology metadata for runtime mode-select (FR-019). Both
    // bundles carry a pointer to the mode's compile-time topology array
    // + recurrent flag array + total weight count + hidden state count
    // + topology string. Consumed by autoc.cc population init,
    // minisim genome validation, and startup logging.
    const int* topology;             // {input, hidden1, hidden2, output}
    const bool* recurrent;           // per-layer recurrent flag
    int num_layers;                  // 4 in v1
    int weight_count;                // total feedforward + recurrent W_hh
    int hidden_state_count;          // total recurrent state floats
    const char* topology_string;     // "33,32,16r,3" or "45,32,16r,3"
};

extern const ModeStrategy kPathgenMode;
extern const ModeStrategy kTrackerMode;

// 030 M7a — Active mode lookup helper for autoc-side runtime dispatch.
// Reads ConfigManager and returns the matching strategy bundle. Called
// at autoc startup (population init + topology validation) — NOT in the
// hot path (per-tick gather goes through PathgenStepper / TrackerStepper
// directly).
const ModeStrategy& getActiveModeStrategy();
const ModeStrategy& getModeStrategyByName(const char* name);
