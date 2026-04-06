#include <gtest/gtest.h>
#include <vector>
#include <map>
#include "autoc/eval/selection.h"

// Helper: build scenario scores for N individuals with M scenarios
// scores[individual][scenario] = negated score (lower = better)
static std::vector<std::vector<ScenarioScore>> makeScores(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& score_values  // [individual][scenario], already negated
) {
    std::vector<std::vector<ScenarioScore>> all_scores(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all_scores[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all_scores[i][s].score = score_values[i][s];
        }
    }
    return all_scores;
}

// T018: lexicase selects individual with best (most negative) score
TEST(Selection022, LexicaseBestScore) {
    // 3 individuals, 2 scenarios
    // Individual 0: score (-100, -100) — best
    // Individual 1: score (-50, -50) — medium
    // Individual 2: score (-10, -10) — worst
    auto scores = makeScores(3, 2,
        {{-100.0, -100.0}, {-50.0, -50.0}, {-10.0, -10.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 1000; i++) {
        int selected = lexicase_select(scores, 3);
        counts[selected]++;
    }

    // Individual 0 (best score) should dominate
    EXPECT_GT(counts[0], counts[1]);
    EXPECT_GT(counts[0], counts[2]);
}

// T018: tie-breaking — equal scores should select roughly uniformly
TEST(Selection022, LexicaseTieBreaking) {
    // 2 individuals with identical scores
    auto scores = makeScores(2, 2,
        {{-50.0, -50.0}, {-50.0, -50.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 1000; i++) {
        int selected = lexicase_select(scores, 2);
        counts[selected]++;
    }

    // Both should be selected roughly equally (within margin)
    EXPECT_GT(counts[0], 300);
    EXPECT_GT(counts[1], 300);
}

// T018: per-scenario differentiation — individual excels on different scenarios
TEST(Selection022, LexicasePerScenarioDifferentiation) {
    // Individual 0: great on scenario 0, bad on scenario 1
    // Individual 1: bad on scenario 0, great on scenario 1
    auto scores = makeScores(2, 2,
        {{-100.0, -10.0}, {-10.0, -100.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 1000; i++) {
        int selected = lexicase_select(scores, 2);
        counts[selected]++;
    }

    // Both should be selected roughly equally (each wins on one scenario)
    EXPECT_GT(counts[0], 300);
    EXPECT_GT(counts[1], 300);
}

// Minimax: worst-case scenario (most positive score = worst)
TEST(Selection022, MinimaxWorstScenario) {
    std::vector<ScenarioScore> scores(3);
    scores[0].score = -100.0;  // good
    scores[1].score = -10.0;   // bad (closest to zero = worst)
    scores[2].score = -50.0;   // medium

    double mm = minimax_fitness(scores);
    // Should return the most positive (worst) score: -10
    EXPECT_DOUBLE_EQ(mm, -10.0);
}

// parseSelectionMode
TEST(Selection022, ParseMode) {
    EXPECT_EQ(parseSelectionMode("sum"), SelectionMode::SUM);
    EXPECT_EQ(parseSelectionMode("minimax"), SelectionMode::MINIMAX);
    EXPECT_EQ(parseSelectionMode("lexicase"), SelectionMode::LEXICASE);
    EXPECT_EQ(parseSelectionMode("unknown"), SelectionMode::SUM);
}

// selectionModeToString
TEST(Selection022, ModeToString) {
    EXPECT_STREQ(selectionModeToString(SelectionMode::SUM), "sum");
    EXPECT_STREQ(selectionModeToString(SelectionMode::MINIMAX), "minimax");
    EXPECT_STREQ(selectionModeToString(SelectionMode::LEXICASE), "lexicase");
}
