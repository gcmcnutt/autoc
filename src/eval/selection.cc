#include "autoc/eval/selection.h"
#include <algorithm>
#include <random>
#include <numeric>
#include <cmath>
#include <cassert>

// Thread-local RNG for selection (avoid contention)
static thread_local std::mt19937 tl_rng(std::random_device{}());

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

// Epsilon-lexicase selection.
// Priority per scenario: completion_fraction → distance_rmse.
// NOTE: smoothness (Σ|Δu|) was tried as a dimension but rewards saturation — a pegged
// output has Δ=0 and looks perfectly smooth, reinforcing the spiral exploit. Revisit
// once sensor expansion (dDist/dt) gives the NN closing-rate information to self-regulate.
// Path-relative smoothness (normalised by path curvature) is the right formulation but
// requires per-step curvature instrumentation — deferred to Phase 5.
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, double epsilon) {
    if (pop_size <= 0) return 0;
    int num_scenarios = all_scores.empty() ? 0 : static_cast<int>(all_scores[0].size());
    if (num_scenarios == 0) {
        // No scenario scores — fall back to random
        return std::uniform_int_distribution<int>(0, pop_size - 1)(tl_rng);
    }

    // Start with all candidates
    std::vector<int> candidates(pop_size);
    std::iota(candidates.begin(), candidates.end(), 0);

    // Build shuffled scenario order
    std::vector<int> scenario_order(num_scenarios);
    std::iota(scenario_order.begin(), scenario_order.end(), 0);
    std::shuffle(scenario_order.begin(), scenario_order.end(), tl_rng);

    // For each scenario in random order, filter candidates
    for (int si : scenario_order) {
        if (candidates.size() <= 1) break;

        // Phase 1: Filter on completion_fraction (higher is better)
        double best_completion = -1.0;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                si < static_cast<int>(all_scores[idx].size())) {
                best_completion = std::max(best_completion, all_scores[idx][si].completion_fraction);
            }
        }

        // Keep candidates within epsilon of best completion
        std::vector<int> survivors;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                si < static_cast<int>(all_scores[idx].size())) {
                if (all_scores[idx][si].completion_fraction >= best_completion - epsilon) {
                    survivors.push_back(idx);
                }
            }
        }

        if (survivors.empty()) break;  // Safety: keep current candidates
        candidates = survivors;
        if (candidates.size() <= 1) break;

        // Phase 2: Among completion survivors, filter on distance_rmse (lower is better)
        double best_dist = 1e30;
        for (int idx : candidates) {
            best_dist = std::min(best_dist, all_scores[idx][si].distance_rmse);
        }

        survivors.clear();
        double dist_epsilon = std::max(0.5, best_dist * epsilon);
        for (int idx : candidates) {
            if (all_scores[idx][si].distance_rmse <= best_dist + dist_epsilon) {
                survivors.push_back(idx);
            }
        }

        if (!survivors.empty()) {
            candidates = survivors;
        }
    }

    // Pick randomly among remaining candidates
    if (candidates.empty()) return 0;
    return candidates[std::uniform_int_distribution<int>(0, static_cast<int>(candidates.size()) - 1)(tl_rng)];
}

// Minimax: worst-case scenario legacy fitness
double minimax_fitness(const std::vector<ScenarioScore>& scores) {
    double worst = 0.0;
    for (const auto& s : scores) {
        double local = s.legacy_distance_sum + s.legacy_attitude_sum * s.legacy_attitude_scale;
        if (s.crashed) {
            local = s.legacy_crash_penalty + local;
        }
        worst = std::max(worst, local);
    }
    return worst;
}
