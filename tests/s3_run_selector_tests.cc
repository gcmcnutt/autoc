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

// --- 045 F1 / US1 T005 — local dmp cache, read-through (contracts/dmp-cache.md)
// Pure file-system logic with an injected fetch, so no AWS is needed here.
#include "autoc/util/zstd_io.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <unistd.h>

namespace {
namespace fs = std::filesystem;

std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}
void spit(const fs::path& p, const std::string& bytes) {
    fs::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary);
    out << bytes;
}

class DmpCache : public ::testing::Test {
protected:
    void SetUp() override {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        root = fs::temp_directory_path() /
               ("autoc_dmp_cache_" + std::to_string(::getpid()) + "_" + info->name());
        fs::remove_all(root);
        fs::create_directories(root);
        cfg.dir = (root / "cache").string();
        raw = autoc::zstdCompress(blob);
    }
    void TearDown() override { fs::remove_all(root); }

    std::string fetchCounting() { ++calls; return raw; }
    std::function<std::string()> fetcher() { return [this] { return fetchCounting(); }; }

    fs::path root;
    autoc::DmpCacheConfig cfg;
    const std::string bucket = "autoc-m1";
    const std::string key = "autoc-9223370244339188361-2026-10-20T16:59:47.446Z/gen9200.dmp.zst";
    const std::string blob = std::string("cereal-blob-bytes\0with-a-nul", 28);
    std::string raw;
    int calls = 0;
};
}  // namespace

TEST(DmpCacheConfig, EnvValueUnsetOrEmptyIsDisabled) {
    EXPECT_FALSE(autoc::dmpCacheConfigFromEnvValue(nullptr).enabled());
    EXPECT_FALSE(autoc::dmpCacheConfigFromEnvValue("").enabled());
    const auto c = autoc::dmpCacheConfigFromEnvValue("/data/dmp-cache");
    EXPECT_TRUE(c.enabled());
    EXPECT_EQ(c.dir, "/data/dmp-cache");
}

TEST_F(DmpCache, PathIsDirBucketKey) {
    EXPECT_EQ(autoc::dmpCachePath(cfg, bucket, key), cfg.dir + "/" + bucket + "/" + key);
}

TEST_F(DmpCache, MissFetchesOnceFillsThenHits) {
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher()), blob);
    EXPECT_EQ(calls, 1);
    const fs::path p = autoc::dmpCachePath(cfg, bucket, key);
    ASSERT_TRUE(fs::exists(p));
    EXPECT_EQ(slurp(p), raw);  // the cache holds the COMPRESSED bytes, exactly as in S3
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher()), blob);
    EXPECT_EQ(calls, 1);  // second read is local
}

TEST_F(DmpCache, NoTmpFileLeftAfterFill) {
    autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher());
    const fs::path dir = fs::path(autoc::dmpCachePath(cfg, bucket, key)).parent_path();
    int entries = 0;
    for (const auto& e : fs::directory_iterator(dir)) {
        ++entries;
        EXPECT_EQ(e.path().filename().string().find(".tmp."), std::string::npos) << e.path();
    }
    EXPECT_EQ(entries, 1);
}

TEST_F(DmpCache, StrayPartialTmpIsNeverRead) {
    const fs::path p = autoc::dmpCachePath(cfg, bucket, key);
    spit(p.string() + ".tmp.999", "partial-garbage");
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher()), blob);
    EXPECT_EQ(calls, 1);  // a miss: the tmp file is not the entry
}

TEST_F(DmpCache, CorruptEntryThrowsNamingPathWithoutRefetch) {
    const fs::path p = autoc::dmpCachePath(cfg, bucket, key);
    spit(p, "not-a-zstd-frame");
    try {
        autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher());
        FAIL() << "expected a throw on a corrupt cache entry";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(p.string()), std::string::npos) << e.what();
    }
    EXPECT_EQ(calls, 0);  // no silent fall-back to S3 (Constitution VII)
}

TEST_F(DmpCache, UnwritableCacheThrowsNamingPath) {
    spit(root / "a-regular-file", "x");
    cfg.dir = (root / "a-regular-file" / "cache").string();  // a dir under a file: cannot be created
    try {
        autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher());
        FAIL() << "expected a throw when the cache cannot be written";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(cfg.dir), std::string::npos) << e.what();
    }
}

TEST_F(DmpCache, DisabledAlwaysFetchesAndWritesNothing) {
    const autoc::DmpCacheConfig off{};
    EXPECT_EQ(autoc::dmpFetchBlobCached(off, bucket, key, fetcher()), blob);
    EXPECT_EQ(autoc::dmpFetchBlobCached(off, bucket, key, fetcher()), blob);
    EXPECT_EQ(calls, 2);
    EXPECT_FALSE(fs::exists(cfg.dir));
}

TEST_F(DmpCache, LegacyUncompressedKeyIsStoredAsIs) {
    const std::string legacyKey = "autoc-1/gen9990.dmp";
    raw = blob;  // a legacy .dmp is stored and served uncompressed
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, legacyKey, fetcher()), blob);
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, legacyKey, fetcher()), blob);
    EXPECT_EQ(calls, 1);
}

TEST_F(DmpCache, CorruptS3ObjectIsNotCached) {
    raw = "not-a-zstd-frame";  // a bad object in the archive must not poison the cache
    EXPECT_THROW(autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher()), std::runtime_error);
    EXPECT_FALSE(fs::exists(autoc::dmpCachePath(cfg, bucket, key)));
}

// --- 045 F1 / US1 T010 — write-through (dmpCacheStore, used by s3PutDmpBlob) --

TEST_F(DmpCache, StoreWritesEntryAndFetchThenHits) {
    autoc::dmpCacheStore(cfg, bucket, key, raw);
    const fs::path p = autoc::dmpCachePath(cfg, bucket, key);
    ASSERT_TRUE(fs::exists(p));
    EXPECT_EQ(slurp(p), raw);
    EXPECT_EQ(autoc::dmpFetchBlobCached(cfg, bucket, key, fetcher()), blob);
    EXPECT_EQ(calls, 0);  // a run's own dmps never leave the archive again
    int entries = 0;
    for (const auto& e : fs::directory_iterator(p.parent_path())) { ++entries; (void)e; }
    EXPECT_EQ(entries, 1);  // tmp + rename: no tmp file left behind
}

TEST_F(DmpCache, StoreUnwritableThrowsNamingPath) {
    spit(root / "a-regular-file", "x");
    cfg.dir = (root / "a-regular-file" / "cache").string();
    try {
        autoc::dmpCacheStore(cfg, bucket, key, raw);
        FAIL() << "expected a throw when the cache cannot be written";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find(cfg.dir), std::string::npos) << e.what();
    }
}

TEST_F(DmpCache, StoreDisabledWritesNothing) {
    autoc::dmpCacheStore(autoc::DmpCacheConfig{}, bucket, key, raw);
    EXPECT_FALSE(fs::exists(cfg.dir));
}
