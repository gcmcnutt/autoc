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
// 027 C2 v4: three dimensions per scenario — tracking, stability, energy.
// All three are per-tick-additive across the scenario with comparable
// magnitudes (tracking: -100s to -10000s; stability: 0 to -2*N_ticks;
// energy: 0 to -N_ticks). Same 0.5 absolute epsilon floor applies.
// Pool size = 3 × num_scenarios. Random shuffle mixes all three
// dimensions, so an individual must be balanced across all three to
// survive many rounds — pure tracking-saturated, pure-do-nothing, or
// pinned-deflection strategies each fail one of the dimensions.
//
// Score extractor: member pointer lets us pick which ScenarioScore field
// each test case scores against.
namespace {
    using ScoreField = gp_fitness ScenarioScore::*;

    // Median of fitness values (sorts in place). Empty → 0.
    gp_fitness median_inplace(std::vector<gp_fitness>& v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const size_t n = v.size();
        return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }
    // 035 FR-003/R2 — MAD = median(|x − median(x)|), the per-axis epsilon in mad mode.
    gp_fitness computeMAD(std::vector<gp_fitness> vals) {
        const gp_fitness med = median_inplace(vals);
        for (gp_fitness& x : vals) x = std::abs(x - med);
        return median_inplace(vals);
    }
}

int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, bool use_mad_epsilon, double epsilon,
                    bool include_prediction_axis) {
    if (pop_size <= 0) return 0;
    int num_scenarios = all_scores.empty() ? 0 : static_cast<int>(all_scores[0].size());
    if (num_scenarios == 0) {
        return std::uniform_int_distribution<int>(0, pop_size - 1)(rng::engine());
    }

    // Build the combined test-case pool: (scenario_idx, field, abs_epsilon_floor).
    // All three physics-grounded dimensions use the same 0.5 absolute floor.
    struct TestCase {
        int scenario;
        ScoreField field;
        double epsilon_floor;
    };
    // 038 US3 — the aux span/closure-prediction axis is added only when the
    // caller passes include_prediction_axis (EnablePredictorHead ablation gate,
    // threaded from the training loop; kept out of this pure function so unit
    // tests need no ConfigManager). Separate lexicase axis — NOT scalar-
    // composited (sidesteps the 033 collapse), per FR-003.
    std::vector<TestCase> pool;
    pool.reserve(static_cast<size_t>(num_scenarios) * (include_prediction_axis ? 3 : 2));
    for (int s = 0; s < num_scenarios; s++) {
        pool.push_back({s, &ScenarioScore::score,           0.5});
        // 035 FR-008: stability_score axis stays OFF — it's a control-amplitude
        // whip; smoothness must EMERGE from energy via induced drag (FR-009),
        // and it would confound the energy verdict.
        // pool.push_back({s, &ScenarioScore::stability_score, 0.5});
        // 035 FR-001: energy axis ON (gen-0 unconditional) — per-scenario convex
        // throttle energy (FR-001b). Throttle is decoupled from pitch/roll, so
        // adding this axis trivially satisfies the FR-002 axis-independence ask.
        pool.push_back({s, &ScenarioScore::energy_score,    0.5});
        if (include_prediction_axis) {
            pool.push_back({s, &ScenarioScore::prediction_score, 0.5});
        }
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

        // Keep candidates within epsilon of best. Constant mode: |best|·relative
        // floored at the absolute 0.5 (bit-identical to prior runs). MAD mode
        // (035 FR-003): per-axis MAD of the field over the alive candidate set,
        // recomputed each test case.
        gp_fitness score_epsilon;
        if (use_mad_epsilon) {
            std::vector<gp_fitness> vals;
            vals.reserve(candidates.size());
            for (int idx : candidates) {
                if (idx < static_cast<int>(all_scores.size()) &&
                    tc.scenario < static_cast<int>(all_scores[idx].size())) {
                    vals.push_back(all_scores[idx][tc.scenario].*(tc.field));
                }
            }
            score_epsilon = computeMAD(std::move(vals));
        } else {
            score_epsilon = std::max(tc.epsilon_floor, std::abs(best_score) * epsilon);
        }
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
