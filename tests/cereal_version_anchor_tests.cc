// Cereal class-version anchor — Constitution Principle V (Versioned Persistence Artifacts).
//
// Anchors the M1 (pathgen-mode dmp) frozen schema at version 1. Fires loud if the
// CEREAL_CLASS_VERSION constant in protocol.h drifts without an explicit task to bump
// it (T048 in tasks.md performs the M1 → M2 boundary bump to version 2 at the M8
// freeze). Per spec FR-015a + Session 2026-05-04 Q5: versioning gates are at milestone
// boundaries, NOT arbitrary commits — intra-development churn during M5–M7 does not
// bump sub-versions; the freeze happens once at M8.

#include <gtest/gtest.h>
#include <cstdint>

#include "autoc/rpc/protocol.h"

TEST(CerealVersionAnchor, EvalResultsAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<EvalResults>::version, 1u)
        << "EvalResults schema version drifted from 1. If this is intentional "
           "(e.g., M8 freeze bumping to version 2), update this anchor test alongside "
           "the CEREAL_CLASS_VERSION(EvalResults, ...) edit in protocol.h. Do NOT bump "
           "intra-development per Constitution V + Session 2026-05-04 Q5.";
}

TEST(CerealVersionAnchor, ScenarioMetadataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<ScenarioMetadata>::version, 1u);
}

TEST(CerealVersionAnchor, EvalDataAtVersion1) {
    EXPECT_EQ(cereal::detail::Version<EvalData>::version, 1u);
}
