#ifndef NN_EVALUATOR_PORTABLE_H
#define NN_EVALUATOR_PORTABLE_H

#include <vector>
#include <cstdint>
#include "autoc/types.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/fitness_decomposition.h"

// Neural network genome — the fundamental unit of neuroevolution.
//
// Weight layout (spec 027):
//   For each layer transition l → l+1 (in order):
//     [W_l (out*in row-major), B_l (out)]
//   Then, appended at the end of the weight vector:
//     For each recurrent layer l (in order of layer index):
//       [W_hh_l (size*size row-major)]
//   Feedforward case (no recurrent layers) matches pre-027 layout.
struct NNGenome {
    std::vector<float> weights;       // Weights + biases + optional W_hh blocks (see above)
    std::vector<int> topology;        // Layer sizes, e.g., {33, 32, 16, 3}
    std::vector<uint8_t> recurrent;   // Per-layer recurrent flag (0/1), same size as topology
    double fitness;                   // Aggregated fitness from evaluation
    std::vector<ScenarioScore> scenario_scores;  // Per-scenario decomposed scores (015)
    uint32_t generation;              // Generation when created
    float mutation_sigma;             // Per-individual mutation step size (self-adaptive)
    float variation_scale;            // computeVariationScale() at save time — eval uses directly
    std::string source;               // Provenance: "bucket/key" from S3 extraction

    NNGenome() : fitness(0.0), generation(0), mutation_sigma(0.1f), variation_scale(1.0f) {}
};

// Compute total weight+bias count from topology (feedforward only).
// Callers that need recurrent-aware count pass the recurrent flag vector.
int nn_weight_count(const std::vector<int>& topology);
int nn_weight_count(const std::vector<int>& topology,
                    const std::vector<uint8_t>& recurrent);

// Total recurrent hidden-state float count (sum of size for each recurrent
// layer). Zero when `recurrent` is empty or all-false.
int nn_hidden_state_count(const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent);

// Forward pass: feedforward with tanh activation
// weights layout: row-major, layer-sequential [W1, B1, W2, B2, ...]
void nn_forward(const float* weights, const std::vector<int>& topology,
                const float* inputs, float* outputs);

// Signal 1 telemetry — see specs/028-deeper-rnn/data-model.md §2.
// Defined here (not telemetry.h) so that NNControllerBackend can hold one
// by value without a forward-decl/incomplete-type problem. All members are
// inline so this struct does not require linking telemetry.cc — important
// for crrcsim's mod_inputdev which compiles evaluator.cc directly without
// pulling in autoc_common.a.
struct RecurrentTelemetry {
    double xh_mag_sum = 0.0;
    double hh_mag_sum = 0.0;
    long long sample_count = 0;
    double activation_ratio() const {
        if (sample_count == 0) return 0.0;
        const double xh_mean = xh_mag_sum / static_cast<double>(sample_count);
        const double hh_mean = hh_mag_sum / static_cast<double>(sample_count);
        if (xh_mean < 1e-9) return 0.0;
        return hh_mean / xh_mean;
    }
    void reset() { xh_mag_sum = 0.0; hh_mag_sum = 0.0; sample_count = 0; }
};

// Recurrent forward pass: same as nn_forward but with a persistent hidden
// state. `hidden_state` is read-and-written — caller owns storage, zeros it
// on span start, and keeps it across ticks. Layout of hidden_state matches
// layer iteration order: for each recurrent layer, `topology[l]` floats are
// consumed/produced sequentially. W_hh blocks live at the tail of `weights`
// per the NNGenome layout above.
//
// If `telemetry` is non-null, the function accumulates per-recurrent-neuron
// magnitudes of |W_xh row · x_t + b| and |W_hh row · h_{t-1}| into the
// telemetry struct. Default null = no overhead, matches pre-028 behavior.
// See specs/028-deeper-rnn/data-model.md §2.
void nn_forward_recurrent(const float* weights,
                          const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent,
                          const float* inputs, float* outputs,
                          float* hidden_state,
                          RecurrentTelemetry* telemetry = nullptr);

// Fast tanh via 512-entry LUT with linear interpolation
gp_scalar fast_tanh(gp_scalar x);

// Xavier/Glorot weight initialization
void nn_xavier_init(NNGenome& genome);

// Gather pathgen-mode NN sensor inputs (PathgenInput::COUNT = 33 floats) from
// aircraft state (raw, no normalization). Renamed from nn_gather_inputs in
// 030 M2b — the typed function name is the FR-006 + FR-019 pluggable-mode
// dispatch contract (see autoc/nn/mode.h kPathgenMode strategy bundle).
// Body unchanged from the pre-rename nn_gather_inputs (FP-bit-exact).
void gather_pathgen_inputs(PathProvider& pathProvider, AircraftState& aircraftState,
                           NNInputs& inputs);

// 030 M6d — Tracker-mode NN sensor input gather (FR-006 + FR-016 + FR-019).
//
// Six-slot beacon-observation history per channel (oldest at index 0,
// "now" at index 5). Caller (TrackerStepper) owns + advances the history
// across ticks; this function is pure: read history + chase state →
// fill TrackerInputs.
//
// Replaces the M2b stub `gather_tracker_inputs_stub` in src/nn/mode.cc.
struct TrackerHistoryWindow {  // raw-ok: NN-byte-format buffer
    float left_x[6];      // raw-ok: NN-byte-format buffer
    float left_y[6];      // raw-ok: NN-byte-format buffer
    float left_cep[6];    // raw-ok: NN-byte-format buffer
    float right_x[6];     // raw-ok: NN-byte-format buffer
    float right_y[6];     // raw-ok: NN-byte-format buffer
    float right_cep[6];   // raw-ok: NN-byte-format buffer
};

void gather_tracker_inputs(const AircraftState& chase,
                           const TrackerHistoryWindow& history,
                           const gp_vec3& home_world,
                           TrackerInputs& out);

#include "autoc/eval/backend.h"

// NN controller backend — plugs into unified eval pipeline.
//
// Spec 027: when `genome.recurrent` has any true entry, the backend carries
// a persistent `hidden_state_` across evaluate() calls. Callers MUST:
//   1. Construct the backend ONCE per span (not per tick).
//   2. Call reset() on span start to zero hidden state.
//   3. Call evaluate() on NN eval ticks (~10 Hz), not on intermediate
//      outer-frame ticks — matches spec 027 clarify Q4.
//
// Feedforward-only genomes (all recurrent flags false / empty) behave
// identically to pre-027 — hidden_state_ is empty and unused.
class NNControllerBackend : public ControllerBackend {
public:
    explicit NNControllerBackend(const NNGenome& genome);

    void evaluate(AircraftState& aircraftState, PathProvider& pathProvider) override;
    const char* getName() const override { return "NeuralNet"; }

    // 030 M6d — Tracker-mode NN forward pass (FR-019). Parallel entry point
    // to evaluate(); caller pre-gathers TrackerInputs (48 floats) and passes
    // them in. Pathgen `evaluate` body is byte-identical pre-/post-M6d
    // (regression-tight invariant) — `evaluateTracker` is purely additive.
    void evaluateTracker(AircraftState& aircraftState, const TrackerInputs& inputs);

    // Zero the recurrent hidden state. Call on span/engage start.
    // No-op for feedforward networks.
    void reset();

    // 028 — telemetry capture for signal 1 (W_hh / W_xh activation ratio).
    // Off by default; enable only on best-of-gen evaluations to avoid
    // per-individual cost during selection. See specs/028-deeper-rnn/spec.md
    // §Clarifications and data-model.md §2.2.
    void enableTelemetryCapture();
    void disableTelemetryCapture();
    void resetTelemetry();
    double telemetryActivationRatio() const;
    long long telemetrySampleCount() const;

private:
    const NNGenome& genome_;
    std::vector<float> hidden_state_;  // Sized by nn_hidden_state_count(); empty for feedforward
    // 028: held by value; cheap, no allocation. Captures samples only when
    // telemetry_capture_enabled_ is true (see enableTelemetryCapture()).
    RecurrentTelemetry telemetry_;
    bool telemetry_capture_enabled_ = false;
};

// Expose LUT functions for testing
gp_scalar testFastTanh(gp_scalar x);

#endif
