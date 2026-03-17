#pragma once

#include <vector>
#include <string>
#include "autoc/eval/fitness_decomposition.h"

enum class SelectionMode {
    SUM,        // Legacy: tournament on scalar fitness (lower is better)
    MINIMAX,    // Tournament on worst-scenario fitness (lower is better)
    LEXICASE    // Epsilon-lexicase: scenarios as test cases, completion-first
};

SelectionMode parseSelectionMode(const std::string& str);
const char* selectionModeToString(SelectionMode mode);

// Lexicase selection: pick one parent from population.
// Each scenario is a test case. For each selection event:
//   1. Shuffle scenario order
//   2. Filter to individuals within epsilon of best on completion_fraction
//   3. Among survivors, filter on distance_rmse
//   4. If tie, pick randomly among survivors
// Returns index into population.
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, double epsilon = 0.05);

// Minimax fitness: worst-case scenario drives selection.
// Returns the maximum per-scenario legacy fitness (higher = worse).
double minimax_fitness(const std::vector<ScenarioScore>& scores);
