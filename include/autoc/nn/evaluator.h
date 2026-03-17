#ifndef NN_EVALUATOR_PORTABLE_H
#define NN_EVALUATOR_PORTABLE_H

#include <vector>
#include <cstdint>
#include "autoc/types.h"
#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/fitness_decomposition.h"

// Neural network genome — the fundamental unit of neuroevolution
struct NNGenome {
    std::vector<float> weights;       // All weights + biases, layer-major order
    std::vector<int> topology;        // Layer sizes, e.g., {29, 16, 8, 3}
    double fitness;                   // Aggregated fitness from evaluation
    std::vector<ScenarioScore> scenario_scores;  // Per-scenario decomposed scores (015)
    uint32_t generation;              // Generation when created
    float mutation_sigma;             // Per-individual mutation step size (self-adaptive)
    std::string source;               // Provenance: "bucket/key" from S3 extraction

    NNGenome() : fitness(0.0), generation(0), mutation_sigma(0.1f) {}
};

// Compute total weight+bias count from topology
int nn_weight_count(const std::vector<int>& topology);

// Forward pass: feedforward with tanh activation
// weights layout: row-major, layer-sequential [W1, B1, W2, B2, ...]
void nn_forward(const float* weights, const std::vector<int>& topology,
                const float* inputs, float* outputs);

// Fast tanh via 512-entry LUT with linear interpolation
gp_scalar fast_tanh(gp_scalar x);

// Xavier/Glorot weight initialization
void nn_xavier_init(NNGenome& genome);

// Gather NN_INPUT_COUNT sensor inputs from aircraft state (raw, no normalization)
void nn_gather_inputs(PathProvider& pathProvider, AircraftState& aircraftState,
                      float* inputs);

#include "autoc/eval/backend.h"

// NN controller backend — plugs into unified eval pipeline
class NNControllerBackend : public ControllerBackend {
public:
    explicit NNControllerBackend(const NNGenome& genome);

    void evaluate(AircraftState& aircraftState, PathProvider& pathProvider) override;
    const char* getName() const override { return "NeuralNet"; }

private:
    const NNGenome& genome_;
};

// Expose LUT functions for testing
gp_scalar testFastTanh(gp_scalar x);

#endif
