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

TEST(CerealVersionAnchor, EvalResultsAtVersion2) {
    EXPECT_EQ(cereal::detail::Version<EvalResults>::version, 2u)
        << "EvalResults schema version drifted from 2 (the M8a tracker-mode bump). "
           "If this is intentional (e.g., a future v=3 schema), update this anchor "
           "test alongside the CEREAL_CLASS_VERSION(EvalResults, ...) edit in "
           "protocol.h. Do NOT bump intra-development per Constitution V + Session "
           "2026-05-04 Q5.";
}

TEST(CerealVersionAnchor, AircraftStateAtVersion2) {
    EXPECT_EQ(cereal::detail::Version<AircraftState>::version, 2u)
        << "AircraftState schema version drifted from 2 (the M8a honest-recording "
           "bump — gyroRates_ added, NN data always-on). If this is intentional, "
           "update this anchor alongside the CEREAL_CLASS_VERSION edit.";
}

TEST(CerealVersionAnchor, ScenarioMetadataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<ScenarioMetadata>::version, 1u);
}

TEST(CerealVersionAnchor, EvalDataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<EvalData>::version, 1u);
}
