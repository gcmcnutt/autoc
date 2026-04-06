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

// Lexicase selection (022): single dimension per scenario = score (lower = better).
// For each selection event: shuffle scenarios, filter candidates within epsilon of
// best score on each scenario. Returns index into population.
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, double epsilon = 0.05);

// Minimax fitness: worst-case scenario drives selection.
double minimax_fitness(const std::vector<ScenarioScore>& scores);
