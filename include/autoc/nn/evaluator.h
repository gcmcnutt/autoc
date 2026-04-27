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

// Recurrent forward pass: same as nn_forward but with a persistent hidden
// state. `hidden_state` is read-and-written — caller owns storage, zeros it
// on span start, and keeps it across ticks. Layout of hidden_state matches
// layer iteration order: for each recurrent layer, `topology[l]` floats are
// consumed/produced sequentially. W_hh blocks live at the tail of `weights`
// per the NNGenome layout above.
void nn_forward_recurrent(const float* weights,
                          const std::vector<int>& topology,
                          const std::vector<uint8_t>& recurrent,
                          const float* inputs, float* outputs,
                          float* hidden_state);

// Fast tanh via 512-entry LUT with linear interpolation
gp_scalar fast_tanh(gp_scalar x);

// Xavier/Glorot weight initialization
void nn_xavier_init(NNGenome& genome);

// Gather NN_INPUT_COUNT sensor inputs from aircraft state (raw, no normalization)
void nn_gather_inputs(PathProvider& pathProvider, AircraftState& aircraftState,
                      NNInputs& inputs);

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

    // Zero the recurrent hidden state. Call on span/engage start.
    // No-op for feedforward networks.
    void reset();

private:
    const NNGenome& genome_;
    std::vector<float> hidden_state_;  // Sized by nn_hidden_state_count(); empty for feedforward
};

// Expose LUT functions for testing
gp_scalar testFastTanh(gp_scalar x);

#endif
