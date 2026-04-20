#include "autoc/eval/selection.h"
#include "autoc/util/rng.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>

// Selection is called serially from the main autoc thread only (see
// population.cc::breed_population — single for-loop, no threading).
// Use the main seeded PRNG so training is bitwise reproducible given
// autoc.ini's Seed. Prior implementation used a thread_local mt19937
// seeded from std::random_device, which bypassed the user's Seed and
// made gen 2+ populations drift between reruns of the same config.

SelectionMode parseSelectionMode(const std::string& str) {
    if (str == "minimax") return SelectionMode::MINIMAX;
    if (str == "lexicase") return SelectionMode::LEXICASE;
    return SelectionMode::SUM;
}

const char* selectionModeToString(SelectionMode mode) {
    switch (mode) {
        case SelectionMode::MINIMAX: return "minimax";
        case SelectionMode::LEXICASE: return "lexicase";
        default: return "sum";
    }
}

// Epsilon-lexicase selection (022): single dimension per scenario = score (lower is better).
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, double epsilon) {
    if (pop_size <= 0) return 0;
    int num_scenarios = all_scores.empty() ? 0 : static_cast<int>(all_scores[0].size());
    if (num_scenarios == 0) {
        return std::uniform_int_distribution<int>(0, pop_size - 1)(rng::engine());
    }

    // Start with all candidates
    std::vector<int> candidates(pop_size);
    std::iota(candidates.begin(), candidates.end(), 0);

    // Shuffled scenario order
    std::vector<int> scenario_order(num_scenarios);
    std::iota(scenario_order.begin(), scenario_order.end(), 0);
    std::shuffle(scenario_order.begin(), scenario_order.end(), rng::engine());

    // For each scenario, filter on score (lower = better)
    for (int si : scenario_order) {
        if (candidates.size() <= 1) break;

        // Find best (lowest) score among candidates
        double best_score = 1e30;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                si < static_cast<int>(all_scores[idx].size())) {
                best_score = std::min(best_score, all_scores[idx][si].score);
            }
        }

        // Keep candidates within epsilon of best
        double score_epsilon = std::max(0.5, std::abs(best_score) * epsilon);
        std::vector<int> survivors;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                si < static_cast<int>(all_scores[idx].size())) {
                if (all_scores[idx][si].score <= best_score + score_epsilon) {
                    survivors.push_back(idx);
                }
            }
        }

        if (!survivors.empty()) {
            candidates = survivors;
        }
    }

    // Pick randomly among remaining candidates
    if (candidates.empty()) return 0;
    return candidates[std::uniform_int_distribution<int>(0, static_cast<int>(candidates.size()) - 1)(rng::engine())];
}

// Minimax: worst-case scenario score (most positive = worst for negated scores)
double minimax_fitness(const std::vector<ScenarioScore>& scores) {
    double worst = -1e30;
    for (const auto& s : scores) {
        worst = std::max(worst, s.score);
    }
    return worst;
}
