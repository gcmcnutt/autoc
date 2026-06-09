// 035 FR-P07 / FR-P07b — shared S3 run-selector contract tests.
// Pure-logic only (extractGenNumber + the synthetic-list pickers); the AWS
// wrappers (findLatestRun/findLatestGenKey) are integration, not unit-tested here.
#include "autoc/util/s3_run_selector.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using autoc::extractGenNumber;
using autoc::pickLatestRunPrefix;
using autoc::pickLatestGenKey;

// --- extractGenNumber: inverts the 10000-N encoding, .zst-aware -------------

TEST(S3Selector, ExtractGenInvertsEncoding) {
    // gen9410.dmp ⇒ actual gen 590 (10000-9410). This is the renderer bug fix:
    // renderer previously returned the raw 9410.
    EXPECT_EQ(extractGenNumber("autoc-123-2026.../gen9410.dmp"), 590);
    EXPECT_EQ(extractGenNumber("autoc-123-2026.../gen9200.dmp"), 800);   // final gen
    EXPECT_EQ(extractGenNumber("autoc-123-2026.../gen9999.dmp"), 1);     // gen 1
    EXPECT_EQ(extractGenNumber("autoc-123-2026.../gen9999.dmp.zst"), 1); // .zst variant
    EXPECT_EQ(extractGenNumber("autoc-123-2026.../gen9410.dmp.zst"), 590);
}

TEST(S3Selector, ExtractGenRejectsMalformed) {
    EXPECT_EQ(extractGenNumber("autoc-123/notagen.dmp"), -1);
    EXPECT_EQ(extractGenNumber("autoc-123/gen9410.txt"), -1);
    EXPECT_EQ(extractGenNumber(""), -1);
    EXPECT_EQ(extractGenNumber("autoc-123/"), -1);
}

// --- pickLatestRunPrefix: greatest autoc- string = newest, tracker- ignored -

TEST(S3Selector, PickLatestRunNewestByReverseTime) {
    // Reverse-time encoding (id number = INT64_MAX - ms): the run created LATER
    // (Jun 4) has the SMALLER number/string, and is the "latest run". So the
    // picker returns the lexicographically-SMALLEST autoc- id.
    std::vector<std::string> prefixes = {
        "autoc-9223370256441628515-2026-06-02T15:12:27Z/",  // Jun 2, larger number (older)
        "autoc-9223370256301596645-2026-06-04T06:06:19Z/",  // Jun 4, smaller number (NEWEST)
    };
    EXPECT_EQ(pickLatestRunPrefix(prefixes),
              "autoc-9223370256301596645-2026-06-04T06:06:19Z/");
}

TEST(S3Selector, PickLatestRunIgnoresNonAutocPrefixes) {
    std::vector<std::string> prefixes = {
        "tracker-9223370256301596645-2026-06-04T06:06:19Z/",  // legacy, retired — ignored
        "autoc-9223370256441628515-2026-06-02T15:12:27Z/",    // only autoc- candidate
        "misc-folder/",
    };
    EXPECT_EQ(pickLatestRunPrefix(prefixes),
              "autoc-9223370256441628515-2026-06-02T15:12:27Z/");
}

TEST(S3Selector, PickLatestRunEmptyWhenNoAutoc) {
    std::vector<std::string> prefixes = {"tracker-abc/", "other/"};
    EXPECT_EQ(pickLatestRunPrefix(prefixes), "");
}

// --- pickLatestGenKey: max actualGen --------------------------------------

TEST(S3Selector, PickLatestGenHighestActualGen) {
    std::vector<std::string> keys = {
        "autoc-123/gen9999.dmp",      // gen 1
        "autoc-123/gen9410.dmp.zst",  // gen 590  ← latest
        "autoc-123/gen9800.dmp",      // gen 200
    };
    EXPECT_EQ(pickLatestGenKey(keys), "autoc-123/gen9410.dmp.zst");
}

TEST(S3Selector, PickLatestGenEmptyWhenNoDmp) {
    std::vector<std::string> keys = {"autoc-123/", "autoc-123/notes.txt"};
    EXPECT_EQ(pickLatestGenKey(keys), "");
}
