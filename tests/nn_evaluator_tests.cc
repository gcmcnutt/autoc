#include <gtest/gtest.h>
#include "autoc/nn/evaluator.h"
#include "autoc/nn/topology.h"
#include <cmath>
#include <vector>
#include <numeric>

// ============================================================
// T031: Weight count tests
// ============================================================

TEST(NNWeightCount, BasicTopology) {
    // Canonical topology should produce NN_WEIGHT_COUNT weights
    std::vector<int> topology(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    int count = nn_weight_count(topology);
    EXPECT_EQ(count, NN_WEIGHT_COUNT);
}

TEST(NNWeightCount, SingleLayer) {
    std::vector<int> topology = {4, 2};
    EXPECT_EQ(nn_weight_count(topology), 4*2 + 2);  // 10
}

TEST(NNWeightCount, LargeTopology) {
    std::vector<int> topology = {14, 32, 16, 3};
    EXPECT_EQ(nn_weight_count(topology), 14*32 + 32 + 32*16 + 16 + 16*3 + 3);  // 1011
}

TEST(NNWeightCount, MinimalTopology) {
    std::vector<int> topology = {1, 1};
    EXPECT_EQ(nn_weight_count(topology), 2);  // 1 weight + 1 bias
}

// ============================================================
// T025: Single-layer forward pass with identity-like weights
// ============================================================

TEST(NNForwardPass, SingleLayerIdentity) {
    // Topology: 2 inputs -> 2 outputs
    // Weights: identity matrix [1,0,0,1] + zero biases [0,0]
    std::vector<int> topology = {2, 2};
    float weights[] = {
        1.0f, 0.0f,  // W[0,0], W[0,1] (output 0 weights)
        0.0f, 1.0f,  // W[1,0], W[1,1] (output 1 weights)
        0.0f, 0.0f   // biases
    };
    float inputs[] = {0.5f, -0.3f};
    float outputs[2];

    nn_forward(weights, topology, inputs, outputs);

    // tanh(0.5) ≈ 0.4621, tanh(-0.3) ≈ -0.2913
    EXPECT_NEAR(outputs[0], std::tanh(0.5f), 1e-2);
    EXPECT_NEAR(outputs[1], std::tanh(-0.3f), 1e-2);
}

TEST(NNForwardPass, SingleLayerWithBias) {
    std::vector<int> topology = {1, 1};
    // Weight = 1.0, Bias = 0.5
    float weights[] = {1.0f, 0.5f};
    float inputs[] = {0.3f};
    float outputs[1];

    nn_forward(weights, topology, inputs, outputs);

    // tanh(0.3 + 0.5) = tanh(0.8) ≈ 0.6640
    EXPECT_NEAR(outputs[0], std::tanh(0.8f), 1e-2);
}

// ============================================================
// T026: Multi-layer forward pass with known weights
// ============================================================

TEST(NNForwardPass, TwoLayerKnown) {
    // Topology: 2 -> 2 -> 1
    std::vector<int> topology = {2, 2, 1};

    // Layer 1: 2->2 (4 weights + 2 biases)
    // Layer 2: 2->1 (2 weights + 1 bias)
    float weights[] = {
        // Layer 1 weights (row-major: output_j * fan_in + i)
        1.0f, 0.0f,   // hidden[0] = tanh(1*in[0] + 0*in[1] + bias[0])
        0.0f, 1.0f,   // hidden[1] = tanh(0*in[0] + 1*in[1] + bias[1])
        // Layer 1 biases
        0.0f, 0.0f,
        // Layer 2 weights
        1.0f, 1.0f,   // out[0] = tanh(1*h[0] + 1*h[1] + bias[0])
        // Layer 2 bias
        0.0f
    };
    float inputs[] = {0.5f, -0.3f};
    float outputs[1];

    nn_forward(weights, topology, inputs, outputs);

    // Hidden: h0 = tanh(0.5) ≈ 0.4621, h1 = tanh(-0.3) ≈ -0.2913
    float h0 = std::tanh(0.5f);
    float h1 = std::tanh(-0.3f);
    // Output: tanh(h0 + h1) = tanh(0.4621 - 0.2913) ≈ tanh(0.1708)
    float expected = std::tanh(h0 + h1);
    EXPECT_NEAR(outputs[0], expected, 1e-2);
}

TEST(NNForwardPass, FullTopology22_16_8_3) {
    // Verify the canonical topology works end-to-end
    std::vector<int> topology(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    int wc = nn_weight_count(topology);
    EXPECT_EQ(wc, NN_WEIGHT_COUNT);

    // All-zero weights: every layer produces tanh(0) = 0 (only biases matter, which are 0)
    std::vector<float> weights(wc, 0.0f);
    float inputs[NN_INPUT_COUNT];
    for (int i = 0; i < NN_INPUT_COUNT; i++) inputs[i] = 1.0f;
    float outputs[NN_OUTPUT_COUNT];

    nn_forward(weights.data(), topology, inputs, outputs);

    // With all-zero weights, all outputs should be tanh(0) = 0
    for (int i = 0; i < NN_OUTPUT_COUNT; i++) {
        EXPECT_NEAR(outputs[i], 0.0f, 1e-3);
    }
}

// ============================================================
// T027: Output range test — tanh guarantees [-1, 1]
// ============================================================

TEST(NNForwardPass, OutputRangeWithRandomWeights) {
    std::vector<int> topology(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    int wc = nn_weight_count(topology);
    std::vector<float> weights(wc);

    // Fill with "random" deterministic weights (large values to stress tanh)
    for (int i = 0; i < wc; i++) {
        weights[i] = static_cast<float>((i * 7 + 13) % 100 - 50) / 10.0f;  // range [-5, 5]
    }

    float inputs[NN_INPUT_COUNT];
    for (int i = 0; i < NN_INPUT_COUNT; i++) {
        inputs[i] = static_cast<float>(i - 11) / 11.0f;  // range [-1, 1]
    }
    float outputs[NN_OUTPUT_COUNT];

    nn_forward(weights.data(), topology, inputs, outputs);

    for (int i = 0; i < NN_OUTPUT_COUNT; i++) {
        EXPECT_GE(outputs[i], -1.0f);
        EXPECT_LE(outputs[i], 1.0f);
    }
}

// ============================================================
// T028: fast_tanh LUT accuracy test
// ============================================================

TEST(FastTanh, MatchesStdTanh) {
    // Test across domain [-5, 5] with fine resolution
    for (float x = -5.0f; x <= 5.0f; x += 0.01f) {
        gp_scalar result = testFastTanh(static_cast<gp_scalar>(x));
        gp_scalar expected = static_cast<gp_scalar>(std::tanh(x));
        EXPECT_NEAR(result, expected, 1e-2)
            << "fast_tanh(" << x << ") = " << result << " vs std::tanh = " << expected;
    }
}

TEST(FastTanh, SaturationBehavior) {
    // Beyond LUT domain, should clamp to ±1
    EXPECT_NEAR(testFastTanh(10.0f), 1.0f, 1e-3);
    EXPECT_NEAR(testFastTanh(-10.0f), -1.0f, 1e-3);
    EXPECT_NEAR(testFastTanh(100.0f), 1.0f, 1e-3);
    EXPECT_NEAR(testFastTanh(-100.0f), -1.0f, 1e-3);
}

TEST(FastTanh, ZeroIsZero) {
    EXPECT_NEAR(testFastTanh(0.0f), 0.0f, 1e-4);
}

TEST(FastTanh, Symmetry) {
    for (float x = 0.0f; x <= 5.0f; x += 0.1f) {
        gp_scalar pos = testFastTanh(x);
        gp_scalar neg = testFastTanh(-x);
        EXPECT_NEAR(pos, -neg, 1e-4) << "Asymmetry at x=" << x;
    }
}

// ============================================================
// T029: Xavier initialization test
// ============================================================

TEST(NNXavierInit, CorrectWeightCount) {
    NNGenome genome;
    genome.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    nn_xavier_init(genome);
    EXPECT_EQ(static_cast<int>(genome.weights.size()), nn_weight_count(genome.topology));
}

TEST(NNXavierInit, ZeroMean) {
    NNGenome genome;
    genome.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    nn_xavier_init(genome);

    double sum = 0;
    for (float w : genome.weights) {
        sum += w;
    }
    double mean = sum / genome.weights.size();
    // Mean should be near zero (statistical, so use generous bound)
    EXPECT_NEAR(mean, 0.0, 0.15);
}

TEST(NNXavierInit, VarianceMatchesFanIn) {
    NNGenome genome;
    genome.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    nn_xavier_init(genome);

    // Check first layer weights: variance should be ~1/NN_INPUT_COUNT
    int first_layer_weights = NN_INPUT_COUNT * NN_HIDDEN1_SIZE;
    double sum = 0, sum_sq = 0;
    for (int i = 0; i < first_layer_weights; i++) {
        sum += genome.weights[i];
        sum_sq += genome.weights[i] * genome.weights[i];
    }
    double mean = sum / first_layer_weights;
    double variance = sum_sq / first_layer_weights - mean * mean;
    double expected_variance = 1.0 / NN_INPUT_COUNT;

    // Statistical test — variance within reasonable range
    EXPECT_GT(variance, expected_variance * 0.2);
    EXPECT_LT(variance, expected_variance * 5.0);
}

TEST(NNXavierInit, AllFinite) {
    NNGenome genome;
    genome.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    nn_xavier_init(genome);

    for (float w : genome.weights) {
        EXPECT_TRUE(std::isfinite(w)) << "Non-finite weight: " << w;
    }
}

// ============================================================
// T030: Determinism test
// ============================================================

TEST(NNForwardPass, Deterministic) {
    std::vector<int> topology(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    int wc = nn_weight_count(topology);
    std::vector<float> weights(wc);
    for (int i = 0; i < wc; i++) {
        weights[i] = static_cast<float>((i * 3 + 7) % 50 - 25) / 25.0f;
    }

    float inputs[NN_INPUT_COUNT];
    for (int i = 0; i < NN_INPUT_COUNT; i++) {
        inputs[i] = static_cast<float>(i) / static_cast<float>(NN_INPUT_COUNT);
    }

    float outputs1[NN_OUTPUT_COUNT], outputs2[NN_OUTPUT_COUNT];

    nn_forward(weights.data(), topology, inputs, outputs1);
    nn_forward(weights.data(), topology, inputs, outputs2);

    // Bit-exact same outputs
    for (int i = 0; i < NN_OUTPUT_COUNT; i++) {
        EXPECT_EQ(outputs1[i], outputs2[i]) << "Non-deterministic at output " << i;
    }
}

// ============================================================
// T040: Input layout test (normalization removed — all inputs raw)
// ============================================================

// Input ordering (27 inputs — 021 redesign):
//  0- 5: dPhi  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  radians
//  6-11: dTheta same
// 12-17: dist  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  metres
//    18: dDist/dt closing rate (m/s)
// 19-22: quaternion (w,x,y,z)
//    23: airspeed (m/s)
// 24-26: gyro rates (p, q, r) in rad/s               standard aerospace RHR

TEST(NNInputLayout, InputCountMatchesTopology) {
    // NN_INPUT_COUNT must equal the first layer of NN_TOPOLOGY
    EXPECT_EQ(NN_INPUT_COUNT, NN_TOPOLOGY[0]);
    EXPECT_EQ(NN_INPUT_COUNT, 27);
}

// Note: Full nn_gather_inputs test requires AircraftState + PathProvider.
