#ifndef AUTOC_NN_TELEMETRY_H
#define AUTOC_NN_TELEMETRY_H

#include <cmath>
#include <vector>
#include "autoc/nn/evaluator.h"  // RecurrentTelemetry + NNGenome

// Signal 2 — population-level coefficient of variation per weight block.
// CV per block = stddev_over_pop( mean(|w| for w in block) )
//              / max( mean_over_pop( mean(|w| for w in block) ), eps )
//
// Definition rationale (load-bearing for hypothesis-1 discrimination):
// per-individual aggregate magnitude, NOT per-weight position-wise stddev.
// See specs/028-deeper-rnn/data-model.md §3.1.
//
// Returns NaN for any block that doesn't exist in the population's topology
// (e.g., w_hh_cv is NaN when no layer has the recurrent flag set).
struct PopulationBlockStats {
    double w_xh0_cv = std::nan("");
    double w_xh1_cv = std::nan("");
    double w_hh_cv  = std::nan("");
};

PopulationBlockStats compute_population_block_stats(
    const std::vector<NNGenome>& population);

// Compute a structural fingerprint of signal 1 (W_hh / W_xh activation ratio)
// via a deterministic synthetic multi-tick replay against a zero-init hidden
// state. Inputs alternate ±0.5 to give the recurrent layer something to
// respond to without depending on RNG state.
//
// Returns 0.0 if the genome is feedforward (no recurrent layer) — sentinel
// per data-model.md §1.2. Captures the structural property "is W_hh
// contribution comparable to W_xh contribution" rather than the per-scenario
// real-eval ratio (which would require plumbing through the eval RPC).
//
// See specs/028-deeper-rnn/data-model.md §2.
double compute_synthetic_activation_ratio(const NNGenome& genome,
                                          int num_ticks = 10);

#endif  // AUTOC_NN_TELEMETRY_H
