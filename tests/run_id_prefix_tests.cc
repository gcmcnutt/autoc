// 034 FR-015 (T034) — run-id prefix routing by mode.
//
// tracker-mode run-ids must carry the "tracker-" prefix; pathgen (and any
// non-tracker mode) carry "autoc-", so M1 and M2 dmps coexist in one S3
// bucket distinguishably. Mirrors the call site in autoc.cc that feeds
// generate_iso8601_timestamp().

#include <gtest/gtest.h>

#include "autoc/util/run_id.h"

TEST(RunIdPrefix, TrackerModeGetsTrackerPrefix) {
  EXPECT_EQ(autoc::runIdPrefixForMode("tracker"), "tracker-");
}

TEST(RunIdPrefix, PathgenModeGetsAutocPrefix) {
  EXPECT_EQ(autoc::runIdPrefixForMode("pathgen"), "autoc-");
}

TEST(RunIdPrefix, UnknownModeDefaultsToAutoc) {
  // Any non-"tracker" mode string routes to the M1 "autoc-" prefix.
  EXPECT_EQ(autoc::runIdPrefixForMode(""), "autoc-");
  EXPECT_EQ(autoc::runIdPrefixForMode("something-else"), "autoc-");
}

TEST(RunIdPrefix, MatchOnlyExactTrackerToken) {
  // Guard against accidental substring/prefix matching — only the exact
  // "tracker" mode token triggers the tracker prefix.
  EXPECT_EQ(autoc::runIdPrefixForMode("Tracker"), "autoc-");
  EXPECT_EQ(autoc::runIdPrefixForMode("tracker-mode"), "autoc-");
}
