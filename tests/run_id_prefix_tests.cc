// 034 FR-015 → 035 FR-P07b — run-id prefix is now UNIFORM across modes.
//
// Per the 035 per-mode-bucket contract the bucket is the mode discriminator,
// so every mode's run-id uses the "autoc-" prefix (the old "tracker-" branch
// is retired). The selector's SetPrefix("autoc-") then matches all modes.

#include <gtest/gtest.h>

#include "autoc/util/run_id.h"

TEST(RunIdPrefix, AllModesGetUniformAutocPrefix) {
  // FR-P07b: tracker no longer gets a distinct prefix — bucket discriminates.
  EXPECT_EQ(autoc::runIdPrefixForMode("tracker"), "autoc-");
  EXPECT_EQ(autoc::runIdPrefixForMode("pathgen"), "autoc-");
  EXPECT_EQ(autoc::runIdPrefixForMode(""), "autoc-");
  EXPECT_EQ(autoc::runIdPrefixForMode("something-else"), "autoc-");
}
