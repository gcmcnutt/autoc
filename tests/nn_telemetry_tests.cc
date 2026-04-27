// 028 telemetry signal tests — see specs/028-deeper-rnn/contracts/evolution_log_columns.md
// and data-model.md.

#include <gtest/gtest.h>
#include "autoc/nn/evaluator.h"
#include "autoc/nn/telemetry.h"
#include "autoc/nn/topology.h"
#include <cmath>
#include <numeric>
#include <vector>

namespace {

// Build a 4-layer {3, 4, 4, 2} topology with layer-2 (4-wide) recurrent.
// Small enough to reason about analytically; same shape (1 recurrent layer)
// as the production 028 topology.
struct TinyTopo {
    std::vector<int> topology = {3, 4, 4, 2};
    std::vector<uint8_t> recurrent = {0, 0, 1, 0};
    int hidden_dim = 4;
    int input_dim = 3;
    int output_dim = 2;
};

// Compose a flat weight vector for TinyTopo with all blocks set to a constant.
// Layout: [W0(4*3), B0(4), W1(4*4), B1(4), W2(2*4), B2(2), W_hh(4*4)]
std::vector<float> tiny_weights_constant(float w0, float b0, float w1, float b1,
                                         float w2, float b2, float whh) {
    std::vector<float> w;
    // Layer 0: W (4*3=12), B (4)
    for (int i = 0; i < 12; i++) w.push_back(w0);
    for (int i = 0; i < 4; i++) w.push_back(b0);
    // Layer 1: W (4*4=16), B (4)
    for (int i = 0; i < 16; i++) w.push_back(w1);
    for (int i = 0; i < 4; i++) w.push_back(b1);
    // Layer 2: W (2*4=8), B (2)
    for (int i = 0; i < 8; i++) w.push_back(w2);
    for (int i = 0; i < 2; i++) w.push_back(b2);
    // W_hh for layer 2 (4*4=16)
    for (int i = 0; i < 16; i++) w.push_back(whh);
    return w;
}

// Make a synthetic NNGenome with TinyTopo and given uniform weight value.
NNGenome make_uniform_genome(const TinyTopo& tt, float value) {
    NNGenome g;
    g.topology = tt.topology;
    g.recurrent = tt.recurrent;
    g.weights = tiny_weights_constant(value, value, value, value, value, value, value);
    return g;
}

}  // namespace

// ============================================================
// T007 — WhhXhRatio_ZeroWhh_ReturnsZero
// ============================================================
//
// With W_hh hand-zeroed, hh_part is identically 0 across all (neuron, tick)
// samples, so activation_ratio() = hh_mean / xh_mean = 0 / nonzero = 0.
TEST(NNTelemetry, WhhXhRatio_ZeroWhh_ReturnsZero) {
    TinyTopo tt;
    auto weights = tiny_weights_constant(0.1f, 0.0f, 0.1f, 0.0f, 0.1f, 0.0f, /*whh=*/0.0f);
    std::vector<float> hidden_state(tt.hidden_dim, 0.5f);  // nonzero so any W_hh would matter
    std::vector<float> outputs(tt.output_dim);
    float inputs[3] = {0.5f, -0.3f, 0.7f};

    RecurrentTelemetry telemetry;
    // Run a few ticks so we accumulate samples — h_t evolves but W_hh=0 so hh_part stays 0.
    for (int tick = 0; tick < 5; tick++) {
        nn_forward_recurrent(weights.data(), tt.topology, tt.recurrent,
                             inputs, outputs.data(), hidden_state.data(), &telemetry);
    }
    EXPECT_EQ(telemetry.activation_ratio(), 0.0);
    EXPECT_GT(telemetry.sample_count, 0);  // We did capture samples
    EXPECT_EQ(telemetry.hh_mag_sum, 0.0);  // No W_hh contribution at all
    EXPECT_GT(telemetry.xh_mag_sum, 0.0);  // W_xh did contribute
}

// ============================================================
// T008 — WhhXhRatio_IdentityWhh_ReturnsExpectedMagnitude
// ============================================================
//
// With W_hh active and inputs/state at known scale, the ratio is positive
// and in a predictable order of magnitude. Records a calibration upper-bound
// for the plot threshold (data-model.md §2.3).
TEST(NNTelemetry, WhhXhRatio_ActiveWhh_ReturnsPositiveRatio) {
    TinyTopo tt;
    // W_xh = 0.1, biases = 0; W_hh = 0.1 (same scale as W_xh) so the ratio is
    // well-defined and bounded once h_t saturates near tanh's range.
    auto weights = tiny_weights_constant(0.1f, 0.0f, 0.1f, 0.0f, 0.1f, 0.0f, /*whh=*/0.1f);
    std::vector<float> hidden_state(tt.hidden_dim, 0.5f);
    std::vector<float> outputs(tt.output_dim);
    float inputs[3] = {0.5f, -0.3f, 0.7f};

    RecurrentTelemetry telemetry;
    for (int tick = 0; tick < 10; tick++) {
        nn_forward_recurrent(weights.data(), tt.topology, tt.recurrent,
                             inputs, outputs.data(), hidden_state.data(), &telemetry);
    }
    const double ratio = telemetry.activation_ratio();
    EXPECT_GT(ratio, 0.0) << "Ratio should be positive when W_hh is non-zero";
    // Order-of-magnitude calibration: with equal scales for W_xh and W_hh
    // and tanh-saturated h_t, expect ratio in roughly [0.1, 10].
    EXPECT_LT(ratio, 10.0) << "Ratio should not blow up with bounded weights";
    EXPECT_GT(ratio, 0.01) << "Ratio should not collapse with active W_hh";
}

// ============================================================
// T009 — WhhXhRatio_NoRecurrentLayer_Sentinel
// ============================================================
//
// All-feedforward topology → nn_forward_recurrent is still callable with
// recurrent={0,0,0,0} but produces no recurrent neurons, so sample_count
// stays 0 and activation_ratio() returns the 0.0 sentinel.
TEST(NNTelemetry, WhhXhRatio_NoRecurrentLayer_Sentinel) {
    std::vector<int> topology = {3, 4, 4, 2};
    std::vector<uint8_t> recurrent = {0, 0, 0, 0};  // ALL false — feedforward
    // Weight vector matches feedforward count (no W_hh tail).
    int ff_count = nn_weight_count(topology);
    std::vector<float> weights(static_cast<size_t>(ff_count), 0.1f);
    std::vector<float> hidden_state;  // empty — no recurrent layers
    std::vector<float> outputs(2);
    float inputs[3] = {0.5f, -0.3f, 0.7f};

    RecurrentTelemetry telemetry;
    nn_forward_recurrent(weights.data(), topology, recurrent,
                         inputs, outputs.data(),
                         hidden_state.empty() ? nullptr : hidden_state.data(),
                         &telemetry);
    EXPECT_EQ(telemetry.sample_count, 0);
    EXPECT_EQ(telemetry.activation_ratio(), 0.0);
}

// ============================================================
// T010 — WhhCv_IdenticalPopulation_Zero
// ============================================================
//
// All individuals have identical weights → block-mean(|w|) is identical
// across the population → stddev is 0 → CV is 0.
TEST(NNTelemetry, WhhCv_IdenticalPopulation_Zero) {
    TinyTopo tt;
    std::vector<NNGenome> pop;
    for (int i = 0; i < 5; i++) pop.push_back(make_uniform_genome(tt, 0.3f));

    PopulationBlockStats stats = compute_population_block_stats(pop);
    EXPECT_NEAR(stats.w_xh0_cv, 0.0, 1e-9);
    EXPECT_NEAR(stats.w_xh1_cv, 0.0, 1e-9);
    EXPECT_NEAR(stats.w_hh_cv, 0.0, 1e-9);
}

// ============================================================
// T011 — WhhCv_BimodalPopulation_AnalyticMatch
// ============================================================
//
// Half the population at +1.0, half at -1.0 (uniform across all blocks).
// Per-individual block-mean(|w|) = 1.0 for every individual → stddev = 0 → CV = 0.
//
// To get a non-zero CV we need *magnitude* variation across individuals,
// not sign variation. Build a 2-individual population with all-w=0.5 and all-w=1.5:
//   means = [0.5, 1.5]; pop mean = 1.0; pop stddev = 0.5; CV = 0.5.
TEST(NNTelemetry, WhhCv_TwoMagnitudes_AnalyticMatch) {
    TinyTopo tt;
    std::vector<NNGenome> pop;
    pop.push_back(make_uniform_genome(tt, 0.5f));
    pop.push_back(make_uniform_genome(tt, 1.5f));

    PopulationBlockStats stats = compute_population_block_stats(pop);
    // mean over pop of |w| = (0.5 + 1.5) / 2 = 1.0
    // stddev (pop) = sqrt(((0.5-1.0)^2 + (1.5-1.0)^2)/2) = sqrt(0.25) = 0.5
    // CV = 0.5 / 1.0 = 0.5
    EXPECT_NEAR(stats.w_xh0_cv, 0.5, 1e-6);
    EXPECT_NEAR(stats.w_xh1_cv, 0.5, 1e-6);
    EXPECT_NEAR(stats.w_hh_cv, 0.5, 1e-6);
}

// ============================================================
// T012 — WhhCv_NoRecurrentLayer_Sentinel
// ============================================================
//
// Population with all-feedforward genomes → w_hh_cv is NaN sentinel (block doesn't
// exist), but w_xh0_cv and w_xh1_cv compute normally.
TEST(NNTelemetry, WhhCv_NoRecurrentLayer_Sentinel) {
    NNGenome g;
    g.topology = {3, 4, 4, 2};
    g.recurrent = {0, 0, 0, 0};  // all-FF
    int ff_count = nn_weight_count(g.topology);
    g.weights.assign(static_cast<size_t>(ff_count), 0.5f);

    NNGenome g2 = g;
    g2.weights.assign(static_cast<size_t>(ff_count), 1.5f);

    std::vector<NNGenome> pop = {g, g2};
    PopulationBlockStats stats = compute_population_block_stats(pop);

    EXPECT_TRUE(std::isnan(stats.w_hh_cv)) << "w_hh_cv must be NaN sentinel when no recurrent layer";
    EXPECT_FALSE(std::isnan(stats.w_xh0_cv));
    EXPECT_FALSE(std::isnan(stats.w_xh1_cv));
    EXPECT_NEAR(stats.w_xh0_cv, 0.5, 1e-6);
    EXPECT_NEAR(stats.w_xh1_cv, 0.5, 1e-6);
}
