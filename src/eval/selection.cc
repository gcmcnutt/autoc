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

// Epsilon-lexicase selection.
//
// 022: single dimension per scenario = tracking score (lower is better).
// 027 C2: added a second dimension per scenario = smoothness score. The
// pool doubles — for each scenario both test cases are available; the
// shuffle mixes them so a jittery-but-well-tracking individual may
// survive the tracking round but be filtered in the smoothness round
// (or vice versa). Smoothness epsilon is calibrated to its own scale
// (0.05 relative, floor 0.02 absolute) so it filters tighter than
// tracking's 0.5-absolute floor.
//
// Score extractor: member pointer lets us pick which ScenarioScore field
// each test case scores against.
namespace {
    using ScoreField = double ScenarioScore::*;
}

int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, double epsilon) {
    if (pop_size <= 0) return 0;
    int num_scenarios = all_scores.empty() ? 0 : static_cast<int>(all_scores[0].size());
    if (num_scenarios == 0) {
        return std::uniform_int_distribution<int>(0, pop_size - 1)(rng::engine());
    }

    // Build the combined test-case pool: (scenario_idx, field, abs_epsilon_floor).
    // tracking: floor 0.5 absolute (legacy); smoothness: floor 0.02 absolute
    // since smoothness scores are O(1) in magnitude for healthy training.
    struct TestCase {
        int scenario;
        ScoreField field;
        double epsilon_floor;
    };
    std::vector<TestCase> pool;
    pool.reserve(num_scenarios * 2);
    for (int s = 0; s < num_scenarios; s++) {
        pool.push_back({s, &ScenarioScore::score,            0.5});
        pool.push_back({s, &ScenarioScore::smoothness_score, 0.02});
    }

    // Start with all candidates
    std::vector<int> candidates(pop_size);
    std::iota(candidates.begin(), candidates.end(), 0);

    // Shuffled test-case order
    std::shuffle(pool.begin(), pool.end(), rng::engine());

    for (const auto& tc : pool) {
        if (candidates.size() <= 1) break;

        // Find best (lowest) value among candidates on this test case
        double best_score = 1e30;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                tc.scenario < static_cast<int>(all_scores[idx].size())) {
                best_score = std::min(best_score, all_scores[idx][tc.scenario].*(tc.field));
            }
        }

        // Keep candidates within epsilon of best (relative * |best| floored at absolute)
        double score_epsilon = std::max(tc.epsilon_floor, std::abs(best_score) * epsilon);
        std::vector<int> survivors;
        for (int idx : candidates) {
            if (idx < static_cast<int>(all_scores.size()) &&
                tc.scenario < static_cast<int>(all_scores[idx].size())) {
                if (all_scores[idx][tc.scenario].*(tc.field) <= best_score + score_epsilon) {
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
