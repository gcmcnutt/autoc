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

// ============================================================
// T043 (spec 027 C2): smoothness test cases in lexicase pool
// ============================================================

// Helper: build scores with tracking + smoothness per individual/scenario.
static std::vector<std::vector<ScenarioScore>> makeScoresWithSmoothness(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& tracking,
    const std::vector<std::vector<double>>& smoothness
) {
    std::vector<std::vector<ScenarioScore>> all(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all[i][s].score = tracking[i][s];
            all[i][s].smoothness_score = smoothness[i][s];
        }
    }
    return all;
}

// Equal tracking, different smoothness: smoother must win in aggregate.
TEST(Selection027, SmoothnessBreaksTrackingTie) {
    // Two individuals, same tracking on 2 scenarios; individual 0 is smooth.
    auto scores = makeScoresWithSmoothness(2, 2,
        /*tracking*/   {{-50.0, -50.0}, {-50.0, -50.0}},
        /*smoothness*/ {{ 0.10,  0.10}, { 1.50,  1.50}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 2);
        counts[selected]++;
    }
    // The smooth individual should dominate — every smoothness round
    // filters the jittery one out.
    EXPECT_GT(counts[0], counts[1] * 3)
        << "smooth=" << counts[0] << " jittery=" << counts[1];
}

// Tradeoff: A tracks better but jitters; B tracks worse but smooth.
// Lexicase with shuffled test-case order should let BOTH survive across
// many selections (each wins on a different dimension).
TEST(Selection027, TradeoffBothSurvive) {
    auto scores = makeScoresWithSmoothness(2, 2,
        /*tracking*/   {{-100.0, -100.0}, { -30.0,  -30.0}},
        /*smoothness*/ {{   1.5,    1.5}, {   0.1,    0.1}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 2);
        counts[selected]++;
    }
    // Both should get meaningful wins. Not symmetric (tracking has larger
    // absolute scale), but each must land above a reasonable floor.
    EXPECT_GT(counts[0], 200) << "A (tracks-better) wins = " << counts[0];
    EXPECT_GT(counts[1], 200) << "B (smoother) wins = " << counts[1];
}

// Smoothness equal across all: tracking still decides winner.
TEST(Selection027, EqualSmoothnessFallsThroughToTracking) {
    auto scores = makeScoresWithSmoothness(3, 2,
        /*tracking*/   {{-100.0, -100.0}, {-50.0, -50.0}, {-10.0, -10.0}},
        /*smoothness*/ {{   0.5,    0.5}, {  0.5,   0.5}, {  0.5,   0.5}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 3);
        counts[selected]++;
    }
    // Best-tracking (individual 0) should dominate — smoothness ties pass
    // all candidates through.
    EXPECT_GT(counts[0], counts[1]);
    EXPECT_GT(counts[0], counts[2]);
}
