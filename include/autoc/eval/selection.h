#pragma once

#include <vector>
#include <string>
#include "autoc/eval/fitness_decomposition.h"

enum class SelectionMode {
    SUM,        // Legacy: tournament on scalar fitness (lower is better)
    MINIMAX,    // Tournament on worst-scenario fitness (lower is better)
    LEXICASE    // Epsilon-lexicase: single dimension per scenario (score, lower is better)
};

SelectionMode parseSelectionMode(const std::string& str);
const char* selectionModeToString(SelectionMode mode);

// Lexicase selection (022; 027/035 multi-axis). Each selection event shuffles
// the (scenario × field) test-case pool and, per test case, keeps candidates
// within epsilon of the best. Returns an index into the population.
//
// Epsilon mode (035 FR-003):
//   use_mad_epsilon == false (constant): eps = max(0.5 floor, |best|·`epsilon`)
//                                        — bit-reproduces prior runs (SC-003).
//   use_mad_epsilon == true  (mad):      eps = per-axis MAD (median abs deviation)
//                                        of the field over the surviving candidates.
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, bool use_mad_epsilon, double epsilon = 0.05);

// Minimax fitness: worst-case scenario drives selection.
double minimax_fitness(const std::vector<ScenarioScore>& scores);
