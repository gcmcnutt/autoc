#include "autoc/nn/serialization.h"
#include "autoc/nn/topology.h"
#include <cstdio>
#include <cstring>

// Binary format layout (spec 027 bumped layout to carry recurrent flags):
// [0..3]   char[4]   magic "NN01"
// [4..7]   uint32_t  format version (2)
// [8..15]  uint64_t  input-layout signature (044 FR-016; v2+)
// [16..19] uint32_t  num_layers
// [12..]   uint32_t[num_layers]  topology
//          uint8_t[num_layers]   recurrent flags (0/1, one byte per layer)
//          uint32_t  num_weights
//          float[num_weights]    weights (ff pairs then W_hh blocks per 027)
//          double    fitness
//          uint32_t  generation
//          float     mutation_sigma
//          float     variation_scale
//          uint32_t  source_len
//          char[source_len] source

namespace {

template<typename T>
void write_val(std::vector<uint8_t>& buf, const T& val) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&val);
    buf.insert(buf.end(), p, p + sizeof(T));
}

template<typename T>
bool read_val(const uint8_t*& ptr, size_t& remaining, T& val) {
    if (remaining < sizeof(T)) return false;
    std::memcpy(&val, ptr, sizeof(T));
    ptr += sizeof(T);
    remaining -= sizeof(T);
    return true;
}

} // namespace

bool nn_serialize(const NNGenome& genome, std::vector<uint8_t>& out) {
    out.clear();

    // Magic
    out.push_back('N');
    out.push_back('N');
    out.push_back('0');
    out.push_back('1');

    // Version
    write_val(out, NN_FORMAT_VERSION);

    // 044 FR-016 — the layout this genome belongs to, stamped from THIS build
    // for its input width (a genome is only ever serialized by a build that can
    // evaluate it; foreign genomes are refused by nn_check_genome_layout first).
    const uint64_t sig = genome.topology.empty()
        ? 0ull : nn_layout_signature_for_inputs(genome.topology.front());
    write_val(out, sig);

    // Topology
    uint32_t num_layers = static_cast<uint32_t>(genome.topology.size());
    write_val(out, num_layers);
    for (int layer_size : genome.topology) {
        uint32_t ls = static_cast<uint32_t>(layer_size);
        write_val(out, ls);
    }

    // Recurrent flags (spec 027). One byte per layer; if the genome's
    // recurrent vector is empty or shorter, remaining layers are
    // serialized as 0 (feedforward).
    for (uint32_t i = 0; i < num_layers; i++) {
        uint8_t flag = (i < genome.recurrent.size()) ? genome.recurrent[i] : 0;
        out.push_back(flag);
    }

    // Weights
    uint32_t num_weights = static_cast<uint32_t>(genome.weights.size());
    write_val(out, num_weights);
    for (float w : genome.weights) {
        write_val(out, w);
    }

    // Metadata
    write_val(out, genome.fitness);
    write_val(out, genome.generation);
    write_val(out, genome.mutation_sigma);
    write_val(out, genome.variation_scale);

    // Provenance string
    uint32_t source_len = static_cast<uint32_t>(genome.source.size());
    write_val(out, source_len);
    for (char c : genome.source) {
        out.push_back(static_cast<uint8_t>(c));
    }

    return true;
}

bool nn_deserialize(const uint8_t* data, size_t size, NNGenome& genome) {
    if (!data || size < 4) return false;

    const uint8_t* ptr = data;
    size_t remaining = size;

    // Check magic
    if (ptr[0] != 'N' || ptr[1] != 'N' || ptr[2] != '0' || ptr[3] != '1') {
        return false;
    }
    ptr += 4;
    remaining -= 4;

    // Version
    uint32_t version;
    if (!read_val(ptr, remaining, version)) return false;
    if (version != NN_FORMAT_VERSION) return false;

    // 044 FR-016 — the layout signature as the FILE states it (checked by
    // nn_check_genome_layout, not here, so the caller can name both values).
    if (!read_val(ptr, remaining, genome.layout_signature)) return false;

    // Topology
    uint32_t num_layers;
    if (!read_val(ptr, remaining, num_layers)) return false;
    if (num_layers == 0 || num_layers > 100) return false;  // sanity check

    genome.topology.resize(num_layers);
    for (uint32_t i = 0; i < num_layers; i++) {
        uint32_t ls;
        if (!read_val(ptr, remaining, ls)) return false;
        genome.topology[i] = static_cast<int>(ls);
    }

    // Recurrent flags (spec 027)
    genome.recurrent.resize(num_layers);
    for (uint32_t i = 0; i < num_layers; i++) {
        uint8_t flag;
        if (!read_val(ptr, remaining, flag)) return false;
        genome.recurrent[i] = flag;
    }

    // Weights
    uint32_t num_weights;
    if (!read_val(ptr, remaining, num_weights)) return false;
    if (remaining < num_weights * sizeof(float)) return false;  // check before allocating

    genome.weights.resize(num_weights);
    for (uint32_t i = 0; i < num_weights; i++) {
        if (!read_val(ptr, remaining, genome.weights[i])) return false;
    }

    // Metadata
    if (!read_val(ptr, remaining, genome.fitness)) return false;
    if (!read_val(ptr, remaining, genome.generation)) return false;
    if (!read_val(ptr, remaining, genome.mutation_sigma)) return false;
    if (!read_val(ptr, remaining, genome.variation_scale)) return false;

    // Provenance string (optional — graceful if not present)
    genome.source.clear();
    if (remaining > sizeof(uint32_t)) {
        uint32_t source_len;
        if (read_val(ptr, remaining, source_len) && source_len > 0 && remaining >= source_len) {
            genome.source.assign(reinterpret_cast<const char*>(ptr), source_len);
            ptr += source_len;
            remaining -= source_len;
        }
    }

    return true;
}

uint64_t nn_layout_signature_for_inputs(int inputCount) {
    if (inputCount == NN_INPUT_COUNT) return kNNInputLayoutSignature;
    if (inputCount == TRACKER_NN_INPUT_COUNT) return kTrackerInputLayoutSignature;
    return 0ull;
}

std::string nn_check_genome_layout(const NNGenome& genome) {
    if (genome.topology.empty()) return "genome has no topology";
    const int width = genome.topology.front();
    const uint64_t want = nn_layout_signature_for_inputs(width);
    char buf[320];
    if (want == 0ull) {
        std::snprintf(buf, sizeof(buf),
                      "genome input width %d matches no input layout of this build "
                      "(pathgen %d, tracker %d) -- it was trained on another layout",
                      width, NN_INPUT_COUNT, TRACKER_NN_INPUT_COUNT);
        return buf;
    }
    if (genome.layout_signature != want) {
        std::snprintf(buf, sizeof(buf),
                      "genome input layout signature %016llx != this build's %016llx "
                      "(%d inputs) -- same count, different slot order/names/version; "
                      "use the build that trained it (043 genomes: tag 043-final)",
                      static_cast<unsigned long long>(genome.layout_signature),
                      static_cast<unsigned long long>(want), width);
        return buf;
    }
    return "";
}

bool nn_detect_format(const uint8_t* data, size_t size) {
    if (!data || size < 4) return false;
    return data[0] == 'N' && data[1] == 'N' && data[2] == '0' && data[3] == '1';
}
