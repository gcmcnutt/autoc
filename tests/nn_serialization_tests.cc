#include <gtest/gtest.h>
#include "autoc/nn/serialization.h"
#include "autoc/nn/topology.h"
#include <cstdio>
#include <cstring>

// ============================================================
// T043: Format detection tests
// ============================================================

TEST(NNFormatDetection, DetectsNNMagic) {
    uint8_t nn_data[] = {'N', 'N', '0', '1', 0, 0, 0, 0};
    EXPECT_TRUE(nn_detect_format(nn_data, sizeof(nn_data)));
}

TEST(NNFormatDetection, RejectsGPData) {
    uint8_t gp_data[] = {0x16, 0x00, 0x00, 0x00};
    EXPECT_FALSE(nn_detect_format(gp_data, sizeof(gp_data)));
}

TEST(NNFormatDetection, RejectsBytecodeData) {
    uint8_t bc_data[] = {0x16, 0x00, 0x04, 0x08, 0x04, 0x08};
    EXPECT_FALSE(nn_detect_format(bc_data, sizeof(bc_data)));
}

TEST(NNFormatDetection, RejectsTooShort) {
    uint8_t short_data[] = {'N', 'N', '0'};
    EXPECT_FALSE(nn_detect_format(short_data, sizeof(short_data)));
    EXPECT_FALSE(nn_detect_format(nullptr, 0));
}

TEST(NNFormatDetection, RejectsWrongMagic) {
    uint8_t wrong[] = {'N', 'N', '0', '2'};
    EXPECT_FALSE(nn_detect_format(wrong, sizeof(wrong)));
}

// ============================================================
// T042: Magic number test — serialized data starts with "NN01"
// ============================================================

TEST(NNSerialization, SerializedDataStartsWithMagic) {
    NNGenome genome;
    genome.topology = {2, 3, 1};
    genome.weights.resize(nn_weight_count(genome.topology), 0.5f);
    genome.fitness = 42.0;
    genome.generation = 7;
    genome.mutation_sigma = 0.1f;

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(genome, buf));
    ASSERT_GE(buf.size(), 4u);
    EXPECT_EQ(buf[0], 'N');
    EXPECT_EQ(buf[1], 'N');
    EXPECT_EQ(buf[2], '0');
    EXPECT_EQ(buf[3], '1');
}

// ============================================================
// T041: Round-trip test
// ============================================================

TEST(NNSerialization, RoundTripPreservesAllFields) {
    NNGenome original;
    original.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    original.weights.resize(nn_weight_count(original.topology));
    for (size_t i = 0; i < original.weights.size(); i++) {
        original.weights[i] = static_cast<float>(i) / 100.0f - 2.0f;
    }
    original.fitness = 123.456;
    original.generation = 42;
    original.mutation_sigma = 0.05f;

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(original, buf));

    NNGenome restored;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), restored));

    // Verify all fields match exactly
    EXPECT_EQ(restored.topology, original.topology);
    EXPECT_EQ(restored.weights.size(), original.weights.size());
    for (size_t i = 0; i < original.weights.size(); i++) {
        EXPECT_EQ(restored.weights[i], original.weights[i])
            << "Weight mismatch at index " << i;
    }
    EXPECT_DOUBLE_EQ(restored.fitness, original.fitness);
    EXPECT_EQ(restored.generation, original.generation);
    EXPECT_FLOAT_EQ(restored.mutation_sigma, original.mutation_sigma);
}

TEST(NNSerialization, RoundTripRecurrentCanonicalTopology) {
    // Spec 027: serialize with per-layer recurrent flags + W_hh weights;
    // deserialize recovers flags and full weight vector (1923 floats).
    NNGenome original;
    original.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    original.recurrent.resize(NN_NUM_LAYERS);
    for (int i = 0; i < NN_NUM_LAYERS; i++) original.recurrent[i] = NN_RECURRENT[i] ? 1 : 0;
    original.weights.resize(nn_weight_count(original.topology, original.recurrent));
    ASSERT_EQ(static_cast<int>(original.weights.size()), NN_WEIGHT_COUNT);
    for (size_t i = 0; i < original.weights.size(); i++) {
        original.weights[i] = static_cast<float>(i) / 200.0f - 4.0f;
    }
    original.fitness = -27.5;
    original.generation = 13;
    original.mutation_sigma = 0.2f;

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(original, buf));

    NNGenome restored;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), restored));

    EXPECT_EQ(restored.topology, original.topology);
    EXPECT_EQ(restored.recurrent, original.recurrent);
    ASSERT_EQ(restored.weights.size(), original.weights.size());
    for (size_t i = 0; i < original.weights.size(); i++) {
        EXPECT_EQ(restored.weights[i], original.weights[i])
            << "Weight mismatch at index " << i;
    }
    EXPECT_DOUBLE_EQ(restored.fitness, original.fitness);
    EXPECT_EQ(restored.generation, original.generation);
    EXPECT_FLOAT_EQ(restored.mutation_sigma, original.mutation_sigma);
}

TEST(NNSerialization, RoundTripSmallNetwork) {
    NNGenome original;
    original.topology = {1, 1};
    original.weights = {0.5f, -0.3f};
    original.fitness = -999.0;
    original.generation = 0;
    original.mutation_sigma = 1.0f;

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(original, buf));

    NNGenome restored;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), restored));

    EXPECT_EQ(restored.topology, original.topology);
    EXPECT_EQ(restored.weights, original.weights);
    EXPECT_DOUBLE_EQ(restored.fitness, original.fitness);
}

TEST(NNSerialization, DetectFormatOnSerializedData) {
    NNGenome genome;
    genome.topology = {2, 2};
    genome.weights.resize(nn_weight_count(genome.topology), 0.0f);

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(genome, buf));
    EXPECT_TRUE(nn_detect_format(buf.data(), buf.size()));
}

// ============================================================
// T044: Corrupt data test
// ============================================================

TEST(NNSerialization, RejectsTruncatedData) {
    NNGenome genome;
    genome.topology = std::vector<int>(NN_TOPOLOGY, NN_TOPOLOGY + NN_NUM_LAYERS);
    genome.weights.resize(nn_weight_count(genome.topology), 1.0f);

    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(genome, buf));

    // Truncate to half
    NNGenome restored;
    EXPECT_FALSE(nn_deserialize(buf.data(), buf.size() / 2, restored));
}

TEST(NNSerialization, RejectsGarbageData) {
    uint8_t garbage[] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA};
    NNGenome restored;
    EXPECT_FALSE(nn_deserialize(garbage, sizeof(garbage), restored));
}

TEST(NNSerialization, RejectsEmptyData) {
    NNGenome restored;
    EXPECT_FALSE(nn_deserialize(nullptr, 0, restored));
}

// ============================================================
// 044 FR-016 (c) — NN01 v2 carries the input-layout signature; genome tools refuse
// a genome from another layout (incl. a same-count reorder) through ONE check.
// ============================================================

namespace {
NNGenome pathgenGenome() {
    NNGenome g;
    g.topology = {NN_INPUT_COUNT, 4, NN_OUTPUT_COUNT};
    g.recurrent = {0, 0, 0};
    g.weights.resize(nn_weight_count(g.topology), 0.25f);
    return g;
}
}  // namespace

TEST(NNSerializationV2, FormatVersionIsTwo_044) {
    EXPECT_EQ(NN_FORMAT_VERSION, 2u);
}

TEST(NNSerializationV2, RoundTripCarriesTheLayoutSignature_044) {
    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(pathgenGenome(), buf));
    NNGenome back;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), back));
    EXPECT_EQ(back.layout_signature, kNNInputLayoutSignature);
    EXPECT_EQ(nn_check_genome_layout(back), "");
}

TEST(NNSerializationV2, VersionOneFileIsRefused_044) {
    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(pathgenGenome(), buf));
    const uint32_t v1 = 1;
    std::memcpy(buf.data() + 4, &v1, sizeof(v1));   // [4..7] format version
    NNGenome back;
    EXPECT_FALSE(nn_deserialize(buf.data(), buf.size(), back));
}

TEST(NNSerializationV2, CheckRefusesForeignSignatureNamingBoth_044) {
    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(pathgenGenome(), buf));
    NNGenome back;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), back));
    back.layout_signature ^= 0x1ull;                  // same count, different layout
    const std::string err = nn_check_genome_layout(back);
    ASSERT_FALSE(err.empty());
    char want[32];
    std::snprintf(want, sizeof(want), "%016llx", static_cast<unsigned long long>(kNNInputLayoutSignature));
    EXPECT_NE(err.find(want), std::string::npos) << err;
    std::snprintf(want, sizeof(want), "%016llx", static_cast<unsigned long long>(back.layout_signature));
    EXPECT_NE(err.find(want), std::string::npos) << err;
}

TEST(NNSerializationV2, CheckRefusesWrongInputCount_044) {
    NNGenome g = pathgenGenome();
    g.topology[0] = NN_INPUT_COUNT + 1;
    g.layout_signature = kNNInputLayoutSignature;
    EXPECT_FALSE(nn_check_genome_layout(g).empty());
}

TEST(NNSerializationV2, TrackerGenomeChecksAgainstTheTrackerTable_044) {
    NNGenome g;
    g.topology = {TRACKER_NN_INPUT_COUNT, 4, 7};
    g.recurrent = {0, 0, 0};
    g.weights.resize(nn_weight_count(g.topology), 0.1f);
    std::vector<uint8_t> buf;
    ASSERT_TRUE(nn_serialize(g, buf));
    NNGenome back;
    ASSERT_TRUE(nn_deserialize(buf.data(), buf.size(), back));
    EXPECT_EQ(back.layout_signature, kTrackerInputLayoutSignature);
    EXPECT_EQ(nn_check_genome_layout(back), "");
}
