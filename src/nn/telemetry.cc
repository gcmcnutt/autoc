// 028 telemetry — signal 2 population CV per weight block + synthetic
// activation-ratio capture for signal 1. RecurrentTelemetry::activation_ratio()
// is inlined in evaluator.h so this TU is not required by the recurrent
// forward pass — only by callers of compute_population_block_stats /
// compute_synthetic_activation_ratio (autoc.cc, tests).
//
// See specs/028-deeper-rnn/data-model.md.

#include "autoc/nn/telemetry.h"
#include "autoc/nn/evaluator.h"

#include <cmath>
#include <cstddef>

namespace {
constexpr double kEps = 1e-9;

// For a single genome, compute mean(|w|) over a contiguous slice of its
// flat weight vector — that's the per-individual aggregate magnitude
// for one weight block (see data-model §3.1 rationale).
double block_mean_abs(const NNGenome& g, std::size_t offset, std::size_t length) {
    if (length == 0 || offset + length > g.weights.size()) {
        return std::nan("");
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < length; i++) {
        sum += std::fabs(static_cast<double>(g.weights[offset + i]));
    }
    return sum / static_cast<double>(length);
}

// Population-level CV across the per-individual block-mean values.
double pop_cv(const std::vector<double>& per_individual_means) {
    if (per_individual_means.empty()) return std::nan("");
    double mean = 0.0;
    for (double v : per_individual_means) {
        if (std::isnan(v)) return std::nan("");
        mean += v;
    }
    mean /= static_cast<double>(per_individual_means.size());
    double var = 0.0;
    for (double v : per_individual_means) {
        const double d = v - mean;
        var += d * d;
    }
    var /= static_cast<double>(per_individual_means.size());
    const double stddev = std::sqrt(var);
    if (mean < kEps) return 0.0;  // Degenerate near-zero magnitude → CV is 0 not NaN
    return stddev / mean;
}

// Compute the offset+length for each weight block in a genome. Layout per
// NNGenome doc-comment in evaluator.h:
//   For each layer transition l → l+1: [W_l (out·in), B_l (out)]
//   Then: for each recurrent layer l: [W_hh_l (size·size)]
//
// Returns:
//   ff_blocks[i] = offset/length for layer-i feedforward block (W+B together)
//   whh_blocks[i] = offset/length for the i-th W_hh block, in layer-index order
struct BlockLayout {
    struct Range { std::size_t offset; std::size_t length; };
    std::vector<Range> ff_blocks;     // One per transition l → l+1
    std::vector<Range> whh_blocks;    // One per recurrent layer (in topology order)
    std::vector<int> recurrent_layer_indices;  // Which layer index each whh_block belongs to
};

BlockLayout compute_block_layout(const NNGenome& g) {
    BlockLayout out;
    std::size_t offset = 0;
    for (std::size_t l = 0; l + 1 < g.topology.size(); l++) {
        const std::size_t in_size = static_cast<std::size_t>(g.topology[l]);
        const std::size_t out_size = static_cast<std::size_t>(g.topology[l + 1]);
        const std::size_t length = in_size * out_size + out_size;  // W + B
        out.ff_blocks.push_back({offset, length});
        offset += length;
    }
    // W_hh blocks for each recurrent layer (l where recurrent[l]==1)
    for (std::size_t l = 0; l < g.recurrent.size() && l < g.topology.size(); l++) {
        if (g.recurrent[l]) {
            const std::size_t size = static_cast<std::size_t>(g.topology[l]);
            const std::size_t length = size * size;
            out.whh_blocks.push_back({offset, length});
            out.recurrent_layer_indices.push_back(static_cast<int>(l));
            offset += length;
        }
    }
    return out;
}

}  // namespace

PopulationBlockStats compute_population_block_stats(
    const std::vector<NNGenome>& population) {
    PopulationBlockStats stats;
    if (population.empty()) return stats;

    // Layout from the first individual; we assume homogeneous topology
    // across the population (canonical autoc-NN evolution invariant).
    const BlockLayout layout = compute_block_layout(population.front());

    // Layer-0 W_xh+B = ff_blocks[0]
    if (layout.ff_blocks.size() >= 1) {
        std::vector<double> means;
        means.reserve(population.size());
        for (const auto& g : population) {
            means.push_back(block_mean_abs(g, layout.ff_blocks[0].offset,
                                           layout.ff_blocks[0].length));
        }
        stats.w_xh0_cv = pop_cv(means);
    }

    // Layer-1 W_xh+B = ff_blocks[1]
    if (layout.ff_blocks.size() >= 2) {
        std::vector<double> means;
        means.reserve(population.size());
        for (const auto& g : population) {
            means.push_back(block_mean_abs(g, layout.ff_blocks[1].offset,
                                           layout.ff_blocks[1].length));
        }
        stats.w_xh1_cv = pop_cv(means);
    }

    // W_hh — first recurrent block (matches 028's NN_RECURRENT[2]=true layout
    // where layer 2 (16-wide) is the only recurrent layer).
    if (!layout.whh_blocks.empty()) {
        std::vector<double> means;
        means.reserve(population.size());
        for (const auto& g : population) {
            means.push_back(block_mean_abs(g, layout.whh_blocks[0].offset,
                                           layout.whh_blocks[0].length));
        }
        stats.w_hh_cv = pop_cv(means);
    }
    // else: stats.w_hh_cv remains NaN sentinel

    return stats;
}

double compute_synthetic_activation_ratio(const NNGenome& genome, int num_ticks) {
    const int hs = nn_hidden_state_count(genome.topology, genome.recurrent);
    if (hs <= 0) return 0.0;  // Feedforward sentinel

    std::vector<float> hidden_state(static_cast<std::size_t>(hs), 0.0f);
    std::vector<float> inputs(static_cast<std::size_t>(genome.topology.front()), 0.0f);
    std::vector<float> outputs(static_cast<std::size_t>(genome.topology.back()), 0.0f);

    RecurrentTelemetry tlm;
    for (int tick = 0; tick < num_ticks; tick++) {
        for (std::size_t i = 0; i < inputs.size(); i++) {
            inputs[i] = ((i + static_cast<std::size_t>(tick)) % 2 == 0) ? 0.5f : -0.5f;
        }
        nn_forward_recurrent(genome.weights.data(), genome.topology,
                             genome.recurrent,
                             inputs.data(), outputs.data(), hidden_state.data(),
                             &tlm);
    }
    return tlm.activation_ratio();
}
