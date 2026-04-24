#ifndef NN_POPULATION_H
#define NN_POPULATION_H

#include <vector>
#include <cstdint>
#include <functional>
#include "autoc/nn/evaluator.h"

// Collection of NNGenome individuals with shared topology
struct NNPopulation {
    std::vector<NNGenome> individuals;
    std::vector<int> topology;        // Shared topology (all same shape)
    std::vector<uint8_t> recurrent;   // Per-layer recurrent flag, same size as topology (spec 027)
    int population_size;
    uint32_t generation;
    double best_fitness;
    int best_index;

    NNPopulation() : population_size(0), generation(0), best_fitness(0.0), best_index(0) {}
};

// Initialize population with Xavier-initialized individuals. `recurrent`
// is a per-layer flag vector parallel to `topology` (spec 027); pass an
// empty vector for fully feedforward networks.
void nn_init_population(NNPopulation& pop,
                        const std::vector<int>& topology,
                        const std::vector<uint8_t>& recurrent,
                        int size);

// Feedforward-only overload (legacy). Equivalent to passing an empty
// recurrent vector. Kept so tests and other callers don't need to thread
// the flag vector through when the network is pure feedforward.
inline void nn_init_population(NNPopulation& pop,
                               const std::vector<int>& topology,
                               int size) {
    nn_init_population(pop, topology, std::vector<uint8_t>{}, size);
}

// Arithmetic crossover (BLX-alpha blend)
NNGenome nn_arithmetic_crossover(const NNGenome& parent1, const NNGenome& parent2, float alpha);

// Gaussian mutation with self-adaptive sigma
void nn_gaussian_mutation(NNGenome& genome);

// Sigma floor: minimum mutation sigma (0 = disabled). Set from config.
extern float nn_sigma_floor;

// Tournament selection
int nn_tournament_select(const NNPopulation& pop, int tournament_size);

// Selection function type: given population, return index of selected parent
using SelectionFn = std::function<int(const NNPopulation&)>;

// Evolution parameters (shared with GP config where noted)
struct NNEvolveParams {
    int tournament_size;         // GP & NN: TournamentSize
    double crossover_prob;       // GP & NN: CrossoverProbability (0-100)
    double creation_prob;        // GP & NN: CreationProbability (0-100)
    double mutation_prob;        // GP & NN: SwapMutationProbability (0-100)
    float crossover_alpha;       // NN only: NNCrossoverAlpha (-1 = random blend)
    int elitism_count;           // GP & NN: AddBestToNewPopulation
    SelectionFn select;          // Selection function (default: tournament on scalar fitness)
};

// Evolve one generation using config-driven parameters
void nn_evolve_generation(NNPopulation& pop, const NNEvolveParams& params);

#endif
