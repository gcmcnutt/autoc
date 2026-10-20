#ifndef NN_SERIALIZATION_H
#define NN_SERIALIZATION_H

#include <vector>
#include <cstdint>
#include <string>
#include "autoc/nn/evaluator.h"

// Magic bytes for NN binary format
constexpr char NN_MAGIC[4] = {'N', 'N', '0', '1'};
// v2 (044 FR-016): a uint64 input-layout signature at [8..15], right after the
// version — the genome says which layout it was trained on. v1 files are refused
// (no migration: a 45-input genome cannot be evaluated by a 53-input build; read
// 043 genomes with tag 043-final).
constexpr uint32_t NN_FORMAT_VERSION = 2;

// Serialize NNGenome to binary format
bool nn_serialize(const NNGenome& genome, std::vector<uint8_t>& out);

// Deserialize NNGenome from binary format
bool nn_deserialize(const uint8_t* data, size_t size, NNGenome& genome);

// Check if data starts with NN magic bytes
bool nn_detect_format(const uint8_t* data, size_t size);

// 044 FR-016 — THIS build's layout signature for a genome of `inputCount`
// inputs: kNNInputLayoutSignature for the pathgen (M1) width,
// kTrackerInputLayoutSignature for the tracker (M2) width, 0 for any other
// width (synthetic test topologies — no NN input layout at all).
// nn_serialize stamps it.
uint64_t nn_layout_signature_for_inputs(int inputCount);

// 044 FR-016 — the ONE check every genome consumer runs (nnextractor, nn2cpp,
// eval mode): "" if the genome was trained on this build's layout for its input
// width; otherwise a message naming both signatures (or the width problem).
std::string nn_check_genome_layout(const NNGenome& genome);

#endif
