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
        int selected = lexicase_select(scores, 3, false);
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
        int selected = lexicase_select(scores, 2, false);
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
        int selected = lexicase_select(scores, 2, false);
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
// T043 (spec 027 C2 v3): energy test cases in lexicase pool
// energy_score = Σ (out_th - 1) / 2 per tick across scenario;
// values are in [-N, 0] with lower (more negative) = less energy used.
// ============================================================

// Helper: build scores with tracking + energy (and optional stability)
// per individual/scenario. Existing tests keep stability=0 (neutral).
static std::vector<std::vector<ScenarioScore>> makeScoresWithEnergy(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& tracking,
    const std::vector<std::vector<double>>& energy
) {
    std::vector<std::vector<ScenarioScore>> all(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all[i][s].score = tracking[i][s];
            all[i][s].energy_score = energy[i][s];
            // stability_score defaults to 0 — equal across individuals,
            // so the stability lexicase round will be a pass-through.
        }
    }
    return all;
}

// Helper: build scores with all three dimensions per individual/scenario.
static std::vector<std::vector<ScenarioScore>> makeScoresThreeDim(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& tracking,
    const std::vector<std::vector<double>>& stability,
    const std::vector<std::vector<double>>& energy
) {
    std::vector<std::vector<ScenarioScore>> all(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all[i][s].score = tracking[i][s];
            all[i][s].stability_score = stability[i][s];
            all[i][s].energy_score = energy[i][s];
        }
    }
    return all;
}

// 035 US1: the energy lexicase axis is ON (FR-001), so the energy tests below
// are re-enabled. Stability stays OFF (FR-008), so the two stability tests now
// assert that stability is NOT a selection axis (rather than a stability
// tiebreak). Re-introduce the tiebreak assertions if the stability axis lands.

// Equal tracking, different energy: lower-energy must win in aggregate.
TEST(Selection027, EnergyBreaksTrackingTie) {
    // Two individuals, same tracking on 2 scenarios; individual 0 is efficient
    // (energy ≈ -90, almost-no-throttle); individual 1 wasteful (≈ -10, full
    // throttle).
    auto scores = makeScoresWithEnergy(2, 2,
        /*tracking*/ {{-50.0, -50.0}, {-50.0, -50.0}},
        /*energy*/   {{-90.0, -90.0}, {-10.0, -10.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 2, false);
        counts[selected]++;
    }
    // The efficient individual should dominate — every energy round
    // filters the wasteful one out.
    EXPECT_GT(counts[0], counts[1] * 3)
        << "efficient=" << counts[0] << " wasteful=" << counts[1];
}

// Tradeoff: A tracks better but burns energy; B tracks worse but efficient.
// Lexicase with shuffled test-case order should let BOTH survive across
// many selections (each wins on a different dimension).
TEST(Selection027, TradeoffBothSurvive) {
    auto scores = makeScoresWithEnergy(2, 2,
        /*tracking*/ {{-100.0, -100.0}, { -30.0,  -30.0}},
        /*energy*/   {{ -10.0,  -10.0}, { -90.0,  -90.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 2, false);
        counts[selected]++;
    }
    // Both should get meaningful wins.
    EXPECT_GT(counts[0], 200) << "A (tracks-better) wins = " << counts[0];
    EXPECT_GT(counts[1], 200) << "B (efficient) wins = " << counts[1];
}

// Energy equal across all: tracking still decides winner.
TEST(Selection027, EqualEnergyFallsThroughToTracking) {
    auto scores = makeScoresWithEnergy(3, 2,
        /*tracking*/ {{-100.0, -100.0}, {-50.0, -50.0}, {-10.0, -10.0}},
        /*energy*/   {{ -50.0,  -50.0}, {-50.0, -50.0}, {-50.0, -50.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 3, false);
        counts[selected]++;
    }
    // Best-tracking (individual 0) should dominate — energy ties pass
    // all candidates through.
    EXPECT_GT(counts[0], counts[1]);
    EXPECT_GT(counts[0], counts[2]);
}

// ============================================================
// 027 v4: stability test cases (third lexicase dimension)
// ============================================================

// FR-008: stability axis OFF — a stability-only difference must NOT drive
// selection. Equal tracking + equal energy → ~uniform regardless of stability.
TEST(Selection027v4, StabilityOffDoesNotBreakTie) {
    auto scores = makeScoresThreeDim(2, 2,
        /*tracking*/  {{-50.0, -50.0}, {-50.0, -50.0}},
        /*stability*/ {{-90.0, -90.0}, {-10.0, -10.0}},   // differ — but axis is OFF
        /*energy*/    {{-50.0, -50.0}, {-50.0, -50.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) {
        int selected = lexicase_select(scores, 2, false);
        counts[selected]++;
    }
    // Neither dominates — stability is not a selection axis (FR-008).
    EXPECT_GT(counts[0], 600) << "ind0=" << counts[0];
    EXPECT_GT(counts[1], 600) << "ind1=" << counts[1];
}

// FR-008: with stability OFF the three-way (tracking/stability/energy) tradeoff
// collapses to tracking-vs-energy. The stability-only specialist (ind1) is best
// at nothing and is suppressed; ind0 (best tracking) and ind2 (best energy) win.
TEST(Selection027v4, EnergyTrackingTradeoffSurvivesStabilityOff) {
    auto scores = makeScoresThreeDim(3, 2,
        /*tracking*/  {{-100.0, -100.0}, { -30.0,  -30.0}, { -30.0,  -30.0}},
        /*stability*/ {{ -10.0,  -10.0}, { -90.0,  -90.0}, { -10.0,  -10.0}},  // OFF
        /*energy*/    {{ -10.0,  -10.0}, { -10.0,  -10.0}, { -90.0,  -90.0}}
    );

    std::map<int, int> counts;
    for (int i = 0; i < 3000; i++) {
        int selected = lexicase_select(scores, 3, false);
        counts[selected]++;
    }
    // Best-tracker and best-energy survive; the stability-only specialist does not.
    EXPECT_GT(counts[0], 200) << "best-tracker wins = " << counts[0];
    EXPECT_GT(counts[2], 200) << "best-energy wins = " << counts[2];
    EXPECT_LT(counts[1], counts[0]) << "stability-only specialist suppressed = " << counts[1];
    EXPECT_LT(counts[1], counts[2]) << "stability-only specialist suppressed = " << counts[1];
}

// ============================================================
// 035 FR-003 — MAD-relative epsilon (T031)
// ============================================================

// Same scores, different survivor sets per mode. Constant epsilon is the
// RELATIVE term max(0.5, |best|·0.05); at |best|=100 that's 5 (wide). MAD over
// a tight cluster is much smaller, so MAD filters where constant does not.
// (energy_score defaults 0 → energy axis is a universal tie; tracking drives it.)
TEST(Selection035MadEpsilon, MadNarrowsVsConstantRelative) {
    auto scores = makeScores(3, 1, {{-100.0}, {-98.0}, {-97.0}});

    // Constant: eps=max(0.5,|−100|·0.05)=5 → all within best+5=-95 → ~uniform.
    std::map<int, int> c;
    for (int i = 0; i < 3000; i++) c[lexicase_select(scores, 3, /*mad=*/false)]++;
    EXPECT_GT(c[0], 300); EXPECT_GT(c[1], 300); EXPECT_GT(c[2], 300)
        << "constant relative eps (5) keeps all three";

    // MAD: values {-100,-98,-97}, median -98, |dev| {2,0,1}, MAD=1 → survivors
    // ≤ best+1 = -99 → ONLY ind0.
    std::map<int, int> m;
    for (int i = 0; i < 3000; i++) m[lexicase_select(scores, 3, /*mad=*/true)]++;
    EXPECT_GT(m[0], 2700) << "mad: tight MAD=1 keeps only the best";
    EXPECT_EQ(m[1], 0); EXPECT_EQ(m[2], 0);
}

// Constant mode pins the historical fixed-0.5 floor (SC-003: bit-reproducible
// path; the rebuild-perf replay gate is the full guarantee). Small magnitudes so
// |best|·0.05 < 0.5 and the floor governs.
TEST(Selection035MadEpsilon, ConstantFloorIsHalf) {
    auto scores = makeScores(3, 1, {{-5.0}, {-4.6}, {-4.0}});
    // best=-5, eps=max(0.5, 0.25)=0.5 → survivors ≤ -4.5: ind0,ind1 in; ind2 out.
    std::map<int, int> c;
    for (int i = 0; i < 3000; i++) c[lexicase_select(scores, 3, /*mad=*/false)]++;
    EXPECT_GT(c[0], 100) << "ind0 best";
    EXPECT_GT(c[1], 100) << "ind1(-4.6) within the 0.5 floor";
    EXPECT_EQ(c[2], 0) << "ind2(-4.0) outside the 0.5 floor";
}

// ---------------------------------------------------------------------------
// 043 T102 — excess-rotation axis (EnableExcessRotationAxis). Gated: OFF must
// leave the pool exactly as before; ON adds a co-equal per-scenario case.
static std::vector<std::vector<ScenarioScore>> makeScoresWithRotation(
    int pop_size, int num_scenarios,
    const std::vector<std::vector<double>>& tracking,
    const std::vector<std::vector<double>>& excess_rotation) {
    std::vector<std::vector<ScenarioScore>> all(pop_size);
    for (int i = 0; i < pop_size; i++) {
        all[i].resize(num_scenarios);
        for (int s = 0; s < num_scenarios; s++) {
            all[i][s].score = tracking[i][s];
            all[i][s].energy_score = 0.0;              // equal → pass-through
            all[i][s].excess_rotation = excess_rotation[i][s];
        }
    }
    return all;
}

TEST(Selection043, RotationAxisOffIsIgnored) {
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-50.0, -50.0}, {-50.0, -50.0}},
        /*excess*/   {{ 10.0,  10.0}, { 90.0,  90.0}});
    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) counts[lexicase_select(scores, 2, false, 0.05, false, ExcessRotationAxisMode::Off)]++;
    // Identical on every ACTIVE case → ties resolve randomly, roughly even.
    EXPECT_GT(counts[1], 600) << "smooth=" << counts[0] << " wandering=" << counts[1];
}

TEST(Selection043, RotationAxisOnPrefersLessExcess) {
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-50.0, -50.0}, {-50.0, -50.0}},
        /*excess*/   {{ 10.0,  10.0}, { 90.0,  90.0}});
    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) counts[lexicase_select(scores, 2, false, 0.05, false, ExcessRotationAxisMode::Independent)]++;
    EXPECT_GT(counts[0], counts[1] * 3) << "smooth=" << counts[0] << " wandering=" << counts[1];
}

TEST(Selection043, RotationTradeoffBothSurvive) {
    // A tracks better but wanders; B tracks worse but rotates only as the path
    // demands. Lexicase must keep both alive — this is the "not dull" property.
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-100.0, -100.0}, {-30.0, -30.0}},
        /*excess*/   {{  90.0,   90.0}, { 10.0,  10.0}});
    std::map<int, int> counts;
    for (int i = 0; i < 2000; i++) counts[lexicase_select(scores, 2, false, 0.05, false, ExcessRotationAxisMode::Independent)]++;
    EXPECT_GT(counts[0], 200) << "A (tracks-better) wins = " << counts[0];
    EXPECT_GT(counts[1], 200) << "B (rotates-less) wins = " << counts[1];
}

// ---------------------------------------------------------------------------
// 043 t5 — ExcessRotationAxisMode::Conditional: the rotation case is a tie-break
// INSIDE the score case. Pinned, in order: (1) the t4 failure mode is gone — a
// passive flyer can never be selected on rotation; (2) among genomes that track
// equally, less excess rotation wins; (3) tracking is never traded away — the
// Independent-mode "both survive" trade (test above) does NOT hold here, by
// design; (4) the mode string parses strictly.
#include <stdexcept>

TEST(Selection043Conditional, PassiveFlyerCannotWinOnRotation) {
    // A tracks (−1000/scenario) and rotates 50 rad; B never follows the path
    // (−10) and rotates 1 rad — the genome t4's pool kept selecting. MAD epsilon
    // as in the bakes.
    auto scores = makeScoresWithRotation(2, 4,
        /*tracking*/ {{-1000.0, -1000.0, -1000.0, -1000.0}, {-10.0, -10.0, -10.0, -10.0}},
        /*excess*/   {{   50.0,    50.0,    50.0,    50.0}, {  1.0,   1.0,   1.0,   1.0}});
    std::map<int, int> ind, cond;
    for (int i = 0; i < 2000; i++) ind[lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Independent)]++;
    for (int i = 0; i < 2000; i++) cond[lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Conditional)]++;
    // Independent: whichever non-energy case shuffles first decides → B wins ~half.
    EXPECT_GT(ind[1], 500) << "independent: the passive flyer should win a large share (t4 dilution) — got " << ind[1];
    // Conditional: every score case removes B before rotation is ever consulted.
    EXPECT_EQ(cond[1], 0) << "conditional: the passive flyer must never be selected — got " << cond[1];
}

TEST(Selection043Conditional, EqualTrackersBreakOnLessExcess) {
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-50.0, -50.0}, {-50.0, -50.0}},
        /*excess*/   {{ 10.0,  10.0}, { 90.0,  90.0}});
    std::map<int, int> off, cond;
    for (int i = 0; i < 2000; i++) off[lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Off)]++;
    for (int i = 0; i < 2000; i++) cond[lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Conditional)]++;
    EXPECT_GT(off[1], 600) << "off: identical on every active case → roughly even — got " << off[1];
    EXPECT_EQ(cond[0], 2000) << "conditional: equal trackers → the smoother one every time — got " << cond[0];
}

TEST(Selection043Conditional, NeverTradesTrackingForSmoothness) {
    // The Independent-mode trade (RotationTradeoffBothSurvive): A tracks better
    // but wanders, B tracks worse but is smooth. Conditional keeps A every time —
    // B's smoothness buys nothing while it is outside epsilon on tracking.
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-100.0, -100.0}, {-30.0, -30.0}},
        /*excess*/   {{  90.0,   90.0}, { 10.0,  10.0}});
    std::map<int, int> cond;
    for (int i = 0; i < 2000; i++) cond[lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Conditional)]++;
    EXPECT_EQ(cond[0], 2000) << "A (tracks-better) must win every time — got " << cond[0];
}

TEST(Selection043Conditional, TelemetryCountsTiebreakBites) {
    auto scores = makeScoresWithRotation(2, 2,
        /*tracking*/ {{-50.0, -50.0}, {-50.0, -50.0}},
        /*excess*/   {{ 10.0,  10.0}, { 90.0,  90.0}});
    (void)drainLexicaseStats();
    for (int i = 0; i < 100; i++) (void)lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Off);
    LexicaseStats off = drainLexicaseStats();
    EXPECT_EQ(off.selections, 100);
    EXPECT_EQ(off.tiebreaks_applied, 0) << "Off must never run a tie-break";
    for (int i = 0; i < 100; i++) (void)lexicase_select(scores, 2, /*mad=*/true, 0.05, false, ExcessRotationAxisMode::Conditional);
    LexicaseStats cond = drainLexicaseStats();
    EXPECT_EQ(cond.selections, 100);
    // Equal trackers: the first score case (whenever it shuffles up) tie-breaks 2 → 1,
    // after which the selection is decided; so exactly one tie-break per selection, all biting.
    EXPECT_EQ(cond.tiebreaks_applied, 100);
    EXPECT_EQ(cond.tiebreaks_bit, 100);
    EXPECT_EQ(cond.tiebreak_in, 200);
    EXPECT_EQ(cond.tiebreak_out, 100);
    EXPECT_EQ(drainLexicaseStats().selections, 0) << "drain must reset";
}

TEST(Selection043Conditional, ModeParsesStrictly) {
    EXPECT_EQ(parseExcessRotationAxisMode("conditional"), ExcessRotationAxisMode::Conditional);
    EXPECT_EQ(parseExcessRotationAxisMode("independent"), ExcessRotationAxisMode::Independent);
    EXPECT_EQ(parseExcessRotationAxisMode(" conditional   # t5 "), ExcessRotationAxisMode::Conditional);  // inih leaves '#'
    EXPECT_THROW(parseExcessRotationAxisMode(""), std::runtime_error);        // no default when the axis is on
    EXPECT_THROW(parseExcessRotationAxisMode("1"), std::runtime_error);
    EXPECT_THROW(parseExcessRotationAxisMode("Conditional"), std::runtime_error);
}
