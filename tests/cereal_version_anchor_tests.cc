// Cereal class-version anchor — Constitution Principle V (Versioned Persistence Artifacts).
//
// Anchors persistence-format versions at known points so accidental drift fails loud.
// 030 M8a (2026-05-06) bumped EvalResults v=1 → v=2 (cameraViewList +
// targetTrajectoryList + arenaEgressCount + hullStrikeCount fields per FR-015) and
// AircraftState v=1 → v=2 (gyroRates_ field added + NN data always-on per the
// honest-recording audit, per memory:feedback_honest_dmp_recording). Per spec
// FR-015a + Session 2026-05-04 Q5: versioning gates are at milestone boundaries,
// NOT arbitrary commits — intra-development churn does not bump sub-versions;
// freezes happen at milestone boundaries.

#include <gtest/gtest.h>
#include <cstdint>

#include "autoc/rpc/protocol.h"
#include "autoc/eval/aircraft_state.h"

// 041 P2-4 (2026-08-18) — re-anchored 3 → 4 at the second FR-005 contract
// break of this feature: the input vector changed shape in BOTH modes
// (42→45, 63→66), the tracker vector was REORDERED so the shared craft block
// could become one sub-struct, and AircraftState gained Es / boundary-closure /
// score-gradient. Milestone-boundary, which is the only kind Constitution V
// permits — not intra-development churn.
//
// (041 T044, 2026-08-11, was the 2 → 3 bump: the grouped per-tick record, the
// five US4 slots, and the US4 knobs on RecordedRunConfig.)
//
// The anchor did its job: it failed the moment the bump landed and named the
// edit to make. That is the whole reason it exists, so it is re-anchored here
// rather than loosened.
TEST(CerealVersionAnchor, EvalResultsAtVersion4) {
    EXPECT_EQ(cereal::detail::Version<EvalResults>::version, 4u)
        << "EvalResults schema version drifted from 4 (the 041 P2-4 contract-break "
           "bump). If this is intentional, update this anchor test alongside the "
           "CEREAL_CLASS_VERSION(EvalResults, ...) edit in protocol.h AND "
           "EvalResults::kSchemaVersion. Do NOT bump intra-development per "
           "Constitution V + Session 2026-05-04 Q5.";
}

// 041 T044 — the reader-side constant and the cereal registration must agree.
// protocol.h already static_asserts this; asserting it here too means a
// one-sided edit fails in the test suite even if someone edits the assert.
TEST(CerealVersionAnchor, EvalResultsReaderConstantMatchesCerealRegistration) {
    EXPECT_EQ(cereal::detail::Version<EvalResults>::version,
              EvalResults::kSchemaVersion)
        << "EvalResults::kSchemaVersion (used by the fail-loud check in "
           "serialize) disagrees with CEREAL_CLASS_VERSION. Every read would "
           "throw, or none would.";
}

TEST(CerealVersionAnchor, AircraftStateAtVersion2) {
    EXPECT_EQ(cereal::detail::Version<AircraftState>::version, 2u)
        << "AircraftState schema version drifted from 2. M8a (2026-05-06) bumped "
           "1 → 2 for the honest-recording audit (gyroRates_ added, NN data "
           "always-on). M9.preA (2026-05-07) added trackerInputs_ in-place at "
           "v=2 per user 'while inside M2 we don't need backward compat — old "
           "ones will be obsolete'. v=1 read path unchanged (gen9200.dmp gate).";
}

TEST(CerealVersionAnchor, ScenarioMetadataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<ScenarioMetadata>::version, 1u);
}

TEST(CerealVersionAnchor, EvalDataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<EvalData>::version, 1u);
}
