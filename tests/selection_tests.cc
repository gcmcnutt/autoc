#include <gtest/gtest.h>
#include <vector>
#include <map>
#include "autoc/eval/selection.h"

// Helper: build scenario scores for N individuals with M scenarios
static std::vector<std::vector<ScenarioScore>> makeScores(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& completions,  // [individual][scenario]
    const std::vector<std::vector<double>>& distances     // [individual][scenario]
) {
    std::vector<std::vector<ScenarioScore>> all_scores(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all_scores[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all_scores[i][s].completion_fraction = completions[i][s];
            all_scores[i][s].distance_rmse = distances[i][s];
            all_scores[i][s].crashed = (completions[i][s] < 1.0);
        }
    }
    return all_scores;
}

// Helper: set smoothness[3] for all scenarios of one individual
static void setSmoothnessAll(std::vector<std::vector<ScenarioScore>>& scores,
                              int individual, double pitch, double roll, double throttle) {
    for (auto& s : scores[individual]) {
        s.smoothness[0] = pitch;
        s.smoothness[1] = roll;
        s.smoothness[2] = throttle;
    }
}

// T025: lexicase selects individual with best completion
TEST(Selection, LexicaseCompletionFirst) {
    // 3 individuals, 2 scenarios
    // Individual 0: completes both (1.0, 1.0), bad distance (50, 50)
    // Individual 1: crashes on scenario 0 (0.5, 1.0), good distance (5, 5)
    // Individual 2: crashes on both (0.3, 0.3), great distance (1, 1)
    auto scores = makeScores(3, 2,
        {{1.0, 1.0}, {0.5, 1.0}, {0.3, 0.3}},
        {{50.0, 50.0}, {5.0, 5.0}, {1.0, 1.0}}
    );

    // Over many selections, individual 0 should be selected most often
    // because it has best completion on both scenarios
    std::map<int, int> counts;
    for (int i = 0; i < 1000; i++) {
        int selected = lexicase_select(scores, 3);
        counts[selected]++;
    }

    // Individual 0 (best completion) should dominate
    EXPECT_GT(counts[0], counts[2]);
    // Individual 2 (worst completion) should be rare
    EXPECT_LT(counts[2], 200);
}

// T026: epsilon-lexicase with continuous completion uses epsilon
TEST(Selection, EpsilonLexicaseEpsilon) {
    // 2 individuals, 1 scenario
    // Both complete fully, but ind 0 has better distance
    auto scores = makeScores(2, 1,
        {{1.0}, {1.0}},
        {{5.0}, {20.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 1000; i++) {
        int selected = lexicase_select(scores, 2);
        counts[selected]++;
    }

    // Both survive completion filter (both 1.0), then ind 0 wins on distance
    EXPECT_GT(counts[0], counts[1]);
}

// T027: minimax returns worst-scenario fitness
TEST(Selection, MinimaxWorstScenario) {
    std::vector<ScenarioScore> scores(3);
    // Scenario 0: good (low fitness)
    scores[0].legacy_distance_sum = 10.0;
    scores[0].legacy_attitude_sum = 5.0;
    scores[0].legacy_attitude_scale = 1.0;
    scores[0].crashed = false;
    // Scenario 1: bad (high fitness)
    scores[1].legacy_distance_sum = 100.0;
    scores[1].legacy_attitude_sum = 50.0;
    scores[1].legacy_attitude_scale = 1.0;
    scores[1].crashed = false;
    // Scenario 2: medium
    scores[2].legacy_distance_sum = 30.0;
    scores[2].legacy_attitude_sum = 20.0;
    scores[2].legacy_attitude_scale = 1.0;
    scores[2].crashed = false;

    double mm = minimax_fitness(scores);
    // Should return scenario 1's fitness: 100 + 50*1 = 150
    EXPECT_DOUBLE_EQ(mm, 150.0);
}

// T027b: minimax with crash scenario
TEST(Selection, MinimaxWithCrash) {
    std::vector<ScenarioScore> scores(2);
    scores[0].legacy_distance_sum = 10.0;
    scores[0].legacy_attitude_sum = 5.0;
    scores[0].legacy_attitude_scale = 1.0;
    scores[0].crashed = false;
    scores[0].legacy_crash_penalty = 0.0;

    scores[1].legacy_distance_sum = 10.0;
    scores[1].legacy_attitude_sum = 5.0;
    scores[1].legacy_attitude_scale = 1.0;
    scores[1].crashed = true;
    scores[1].legacy_crash_penalty = 500000.0;

    double mm = minimax_fitness(scores);
    // Crash scenario dominates: 500000 + 10 + 5*1 = 500015
    EXPECT_DOUBLE_EQ(mm, 500015.0);
}

// T028: parseSelectionMode
TEST(Selection, ParseMode) {
    EXPECT_EQ(parseSelectionMode("sum"), SelectionMode::SUM);
    EXPECT_EQ(parseSelectionMode("minimax"), SelectionMode::MINIMAX);
    EXPECT_EQ(parseSelectionMode("lexicase"), SelectionMode::LEXICASE);
    EXPECT_EQ(parseSelectionMode("unknown"), SelectionMode::SUM);  // default
}

// T028b: selectionModeToString
TEST(Selection, ModeToString) {
    EXPECT_STREQ(selectionModeToString(SelectionMode::SUM), "sum");
    EXPECT_STREQ(selectionModeToString(SelectionMode::MINIMAX), "minimax");
    EXPECT_STREQ(selectionModeToString(SelectionMode::LEXICASE), "lexicase");
}
