// 030 M8a — Tracker-mode dmp v=2 schema contract tests (T047).
//
// Three contract guarantees per FR-015 + FR-015a + Constitution V:
//   1. v=2 round-trip identity — write EvalResults with the new tracker
//      fields populated, read back, every field bitwise-equal.
//   2. v=1 (pathgen historical) back-compat — dmps written with v=1
//      schema deserialize cleanly into a v=2-aware reader; new fields
//      come back empty (cereal version-aware serialize).
//   3. Future v=3 loud-fail — a synthesized future-version dmp throws
//      when read by the v=2-only reader. Constitution V: never silently
//      truncate, always force an explicit operator decision.
//
// M2-dmp self-containedness (FR-015): the v=2 dmp embeds
// targetTrajectoryList as a copy of the source-scenario data the run
// consumed. Renderer (M9) reads M2 dmp directly and never reaches back
// into the M1 source dmp. This test asserts the copy is verbatim.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>

#include <cereal/archives/binary.hpp>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/camera_projection.h"  // BeaconObservation
#include "autoc/rpc/protocol.h"

namespace {

// Build a v=2 EvalResults populated with known per-scenario per-tick
// values so round-trip identity is verifiable on every field.
EvalResults makeSyntheticV2(int numScenarios, int ticksPer) {
    EvalResults r;
    r.gp.assign({0x4e, 0x4e, 0x30, 0x31});  // "NN01" magic-byte placeholder
    r.gpHash = 0xdeadbeefcafebabeULL;
    r.workerId = 7;
    r.workerPid = 12345;
    r.workerEvalCounter = 42;

    r.crashReasonList.assign(numScenarios, CrashReason::TimeLimit);
    // 041 T020/T028 — one grouped series per scenario, replacing the three
    // parallel lists this suite used to populate independently.
    r.tickList.resize(numScenarios);
    r.pathList.resize(numScenarios);
    r.scenarioList.resize(numScenarios);
    r.arenaEgressCount.assign(numScenarios, 0);
    r.hullStrikeCount.assign(numScenarios, 0);

    for (int i = 0; i < numScenarios; ++i) {
        r.scenarioList[i].pathVariantIndex = i;
        r.scenarioList[i].windVariantIndex = i + 100;
        r.scenarioList[i].scenarioSequence = static_cast<uint64_t>(i + 1000);

        for (int t = 0; t < ticksPer; ++t) {
            // Camera view sample with distinguishable values per scenario × tick.
            CameraViewSample cv;
            cv.camera_pose_world_pos =
                gp_vec3(static_cast<gp_scalar>(i), static_cast<gp_scalar>(t),
                        static_cast<gp_scalar>(i * 10 + t));
            cv.camera_pose_world_orient = gp_quat::Identity();
            cv.camera_fov_h_deg = 120.0f;
            cv.camera_fov_v_deg = 90.0f;
            cv.beacon_left.bearing_x_rad = 0.1f * static_cast<float>(t);
            cv.beacon_left.bearing_y_rad = 0.2f * static_cast<float>(t);
            cv.beacon_left.cep = 0.3f;
            cv.beacon_left.raw_px_x = static_cast<int16_t>(t);
            cv.beacon_left.raw_px_y = static_cast<int16_t>(t * 2);
            cv.beacon_left.raw_cep_int8 = 38;
            // 040 T090 (FR-029) — the new diagnostic fields. Values chosen to
            // vary per tick and to include a NEGATIVE SNR, because a naive
            // unsigned round-trip would survive positive-only data.
            cv.beacon_left.raw_margin = -4.5f + 0.25f * static_cast<float>(t);
            cv.beacon_left.lock_state =
                static_cast<int8_t>(t % 4);  // sweeps all four LockState values
            cv.beacon_right.bearing_x_rad = -0.1f * static_cast<float>(t);
            cv.beacon_right.bearing_y_rad = -0.2f * static_cast<float>(t);
            cv.beacon_right.cep = 0.3f;
            cv.beacon_right.raw_px_x = static_cast<int16_t>(-t);
            cv.beacon_right.raw_px_y = static_cast<int16_t>(-t * 2);
            cv.beacon_right.raw_cep_int8 = 38;
            cv.beacon_right.raw_margin = 12.75f - 0.5f * static_cast<float>(t);
            cv.beacon_right.lock_state = static_cast<int8_t>((t + 2) % 4);
            EvalTick tick;
            tick.state.setSimTimeMsec(static_cast<unsigned long>(t) * 50UL);
            tick.cameraView = cv;

            CopiedTargetSample tg;
            tg.position = gp_vec3(static_cast<gp_scalar>(i + 1),
                                   static_cast<gp_scalar>(t + 1),
                                   static_cast<gp_scalar>(-10));
            tg.orientation = gp_quat::Identity();
            tg.velocity = gp_vec3(static_cast<gp_scalar>(15), 0, 0);
            tg.trail_rabbit_position = tg.position - gp_vec3(static_cast<gp_scalar>(3.048), 0, 0);
            tg.inside_crash_hull = (t == ticksPer - 1);  // last tick crashes (M7 telemetry)
            tick.targetSample = tg;
            r.tickList[i].ticks.push_back(tick);
        }
        r.arenaEgressCount[i] = i;
        r.hullStrikeCount[i] = (i % 2);
    }

    return r;
}

// Serialize-then-deserialize via in-memory stringstream. Returns the
// deserialized result.
EvalResults roundTripBinary(const EvalResults& src) {
    std::stringstream buf(std::ios::in | std::ios::out | std::ios::binary);
    {
        cereal::BinaryOutputArchive oa(buf);
        oa(src);
    }
    EvalResults dst;
    {
        cereal::BinaryInputArchive ia(buf);
        ia(dst);
    }
    return dst;
}

}  // namespace

// ---------------------------------------------------------------------------
// (1) v=2 round-trip identity.
// ---------------------------------------------------------------------------

TEST(TrackerDmpRoundtrip, V2RoundTripIdentity) {
    const int numScenarios = 3;
    const int ticksPer = 5;
    EvalResults src = makeSyntheticV2(numScenarios, ticksPer);
    EvalResults dst = roundTripBinary(src);

    // Top-level metadata
    EXPECT_EQ(dst.gpHash, src.gpHash);
    EXPECT_EQ(dst.workerId, src.workerId);
    EXPECT_EQ(dst.workerPid, src.workerPid);
    EXPECT_EQ(dst.workerEvalCounter, src.workerEvalCounter);
    ASSERT_EQ(dst.crashReasonList.size(), static_cast<size_t>(numScenarios));
    ASSERT_EQ(dst.tickList.size(), static_cast<size_t>(numScenarios));
    // 041 T028 — the ScenarioMetadata dual-lifetime guard (index-coupling
    // inventory class D1). `scenarioList` is the PERSISTED per-scenario table
    // and must agree with the tick series; `scenario` (singular) is the per-eval
    // RPC field and is NOT a persistence consumer's business. Disagreement here
    // is the shape that cost a launch on 2026-08-02.
    ASSERT_EQ(dst.scenarioList.size(), dst.tickList.size())
        << "scenarioList and tickList must describe the same scenario set";
    ASSERT_EQ(dst.arenaEgressCount.size(), static_cast<size_t>(numScenarios));
    ASSERT_EQ(dst.hullStrikeCount.size(), static_cast<size_t>(numScenarios));

    for (int i = 0; i < numScenarios; ++i) {
        ASSERT_EQ(dst.tickList[i].ticks.size(), static_cast<size_t>(ticksPer)) << i;
        // 041 T028 — the initial state survives WITHOUT being confused for tick 0.
        EXPECT_EQ(dst.tickList[i].initialState.getSimTimeMsec(),
                  src.tickList[i].initialState.getSimTimeMsec()) << i;
        EXPECT_EQ(dst.arenaEgressCount[i], src.arenaEgressCount[i]);
        EXPECT_EQ(dst.hullStrikeCount[i], src.hullStrikeCount[i]);

        for (int t = 0; t < ticksPer; ++t) {
            ASSERT_TRUE(dst.tickList[i].ticks[t].cameraView.has_value()) << i << "," << t;
            ASSERT_TRUE(dst.tickList[i].ticks[t].targetSample.has_value()) << i << "," << t;
            const CameraViewSample& sCv = *src.tickList[i].ticks[t].cameraView;
            const CameraViewSample& dCv = *dst.tickList[i].ticks[t].cameraView;
            EXPECT_FLOAT_EQ(dCv.camera_pose_world_pos.x(), sCv.camera_pose_world_pos.x());
            EXPECT_FLOAT_EQ(dCv.camera_pose_world_pos.y(), sCv.camera_pose_world_pos.y());
            EXPECT_FLOAT_EQ(dCv.camera_pose_world_pos.z(), sCv.camera_pose_world_pos.z());
            EXPECT_FLOAT_EQ(dCv.camera_fov_h_deg, sCv.camera_fov_h_deg);
            EXPECT_FLOAT_EQ(dCv.beacon_left.bearing_x_rad, sCv.beacon_left.bearing_x_rad);
            EXPECT_EQ(dCv.beacon_left.raw_px_x, sCv.beacon_left.raw_px_x);
            EXPECT_FLOAT_EQ(dCv.beacon_right.bearing_y_rad, sCv.beacon_right.bearing_y_rad);
            EXPECT_EQ(dCv.beacon_right.raw_cep_int8, sCv.beacon_right.raw_cep_int8);
            // 040 T090 — without these the renderer cannot draw the HOLD coast
            // or the q bar, which is the one thing the Phase 6 playback review
            // most needs to see.
            EXPECT_FLOAT_EQ(dCv.beacon_left.raw_margin, sCv.beacon_left.raw_margin);
            EXPECT_EQ(dCv.beacon_left.lock_state, sCv.beacon_left.lock_state);
            EXPECT_FLOAT_EQ(dCv.beacon_right.raw_margin, sCv.beacon_right.raw_margin);
            EXPECT_EQ(dCv.beacon_right.lock_state, sCv.beacon_right.lock_state);

            const CopiedTargetSample& sTg = *src.tickList[i].ticks[t].targetSample;
            const CopiedTargetSample& dTg = *dst.tickList[i].ticks[t].targetSample;
            EXPECT_FLOAT_EQ(dTg.position.x(), sTg.position.x());
            EXPECT_FLOAT_EQ(dTg.position.y(), sTg.position.y());
            EXPECT_FLOAT_EQ(dTg.trail_rabbit_position.x(), sTg.trail_rabbit_position.x());
            EXPECT_EQ(dTg.inside_crash_hull, sTg.inside_crash_hull);
        }
    }
}

// ---------------------------------------------------------------------------
// (2) v=1 back-compat — pathgen historical dmps still parse cleanly.
// New tracker fields come back empty.
//
// We synthesize a v=1 dmp by manually writing the v=1 field set with the
// version field forced to 1. The simplest robust path is to use cereal's
// own machinery via a dedicated test struct mirroring the v=1 layout —
// but that's heavyweight. Instead, exercise the "empty new fields"
// behavior directly: construct an EvalResults with only v=1 fields
// populated, serialize at the current version (v=2 — new fields are
// just empty), deserialize, and assert both that pathgen-style fields
// round-trip AND new fields stay empty. Future v=3 → v=2 read is the
// loud-fail case in test (3) below.
// ---------------------------------------------------------------------------

TEST(TrackerDmpRoundtrip, PathgenStyleDmpHasEmptyTrackerFields) {
    EvalResults src;
    // 041 T020 — a PATHGEN scenario: ticks carry a state and NOTHING else. The
    // tracker members are absent (std::optional), not zero-filled, so a reader
    // cannot mistake "this run had no camera" for "the camera saw nothing".
    src.workerId = 1;
    src.workerEvalCounter = 5;
    src.gpHash = 0xabcd;
    src.crashReasonList.push_back(CrashReason::RabbitComplete);
    src.tickList.resize(1);
    src.tickList[0].ticks.resize(3);   // three stepped ticks, no tracker members
    src.pathList.resize(1);
    src.scenarioList.resize(1);

    EvalResults dst = roundTripBinary(src);

    EXPECT_EQ(dst.workerId, 1);
    EXPECT_EQ(dst.workerEvalCounter, 5);
    EXPECT_EQ(dst.crashReasonList.size(), 1u);
    // 041 T020 — the tracker members round-trip as ABSENT, which is the whole
    // point of making them optional rather than default-constructed.
    ASSERT_EQ(dst.tickList.size(), 1u);
    ASSERT_EQ(dst.tickList[0].ticks.size(), 3u);
    for (const auto& tk : dst.tickList[0].ticks) {
        EXPECT_FALSE(tk.cameraView.has_value())
            << "a pathgen tick must carry NO camera view, not a zeroed one";
        EXPECT_FALSE(tk.targetSample.has_value());
    }
    EXPECT_TRUE(dst.arenaEgressCount.empty());
    EXPECT_TRUE(dst.hullStrikeCount.empty());
}

// ---------------------------------------------------------------------------
// (3) Future v=3 loud-fail per Constitution V.
//
// Cereal embeds the class version in the binary stream. If the file's
// version exceeds CEREAL_CLASS_VERSION(EvalResults, 2), cereal's
// version-aware read either throws (recommended cereal behavior) or
// silently skips — the principle requires the former. We verify by
// hand-crafting a stream where the version byte is 99, then attempting
// to deserialize.
//
// The exact byte layout cereal uses for the version field is internal,
// so a robust integration test runs through cereal's own writer and
// then tampers with the version field. For v1 of this contract test,
// we just document the expectation; a follow-up could synthesize the
// exact byte tampering once cereal's stream layout is locked.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 041 T044 — schema-mismatch loud fail, now HERMETIC.
//
// This replaces a SUCCEED() placeholder that asserted nothing and deferred the
// real check to "operator-driven verification". It could be written now only
// because T044 added an explicit version check to EvalResults::serialize:
// before that there was no defined behaviour to test, because cereal does not
// reject a version mismatch — it forwards the number and lets the body read a
// payload that is not there.
//
// What that produced, and why this test exists: a mismatched read walked into
// the `version >= 2` branch, pulled a garbage container length, and the process
// died inside the allocator as `vector::_M_default_append`. That trace names
// neither the artifact nor the schema, and it points at memory corruption —
// a diagnosis that costs hours and goes to the wrong place entirely.
// ---------------------------------------------------------------------------

// Rewrite the class-version field cereal writes at the head of the archive.
// Asserted rather than assumed: if cereal's layout ever moves, the assertion
// below fails loudly instead of silently tampering with an unrelated byte and
// leaving a test that passes for the wrong reason.
std::string withSchemaVersion(const std::string& blob, std::uint32_t version) {
    EXPECT_GE(blob.size(), sizeof(std::uint32_t));
    std::uint32_t stored = 0;
    std::memcpy(&stored, blob.data(), sizeof(stored));
    EXPECT_EQ(stored, EvalResults::kSchemaVersion)
        << "expected the archive to open with the EvalResults class version; "
           "cereal's layout may have changed, in which case this helper is "
           "tampering with the wrong bytes";

    std::string out = blob;
    std::memcpy(out.data(), &version, sizeof(version));
    return out;
}

std::string serializeCurrent() {
    EvalResults r = makeSyntheticV2(2, 3);
    std::ostringstream os(std::ios::binary);
    {
        cereal::BinaryOutputArchive oa(os);
        oa(r);
    }
    return os.str();
}

TEST(TrackerDmpRoundtrip, PriorVersionArtifactFailsLoudNamingBothVersions_T044) {
    const std::string tampered = withSchemaVersion(serializeCurrent(), 2u);

    EvalResults out;
    std::istringstream is(tampered, std::ios::binary);
    cereal::BinaryInputArchive ia(is);

    try {
        ia(out);
        FAIL() << "reading a v2 artifact with a v3 reader must throw, not "
                  "succeed — a silent partial read is the failure Constitution V "
                  "exists to prevent";
    } catch (const std::exception& e) {
        const std::string msg = e.what();
        // BOTH numbers, per the task: one alone leaves the reader guessing
        // which side is stale.
        EXPECT_NE(msg.find("v2"), std::string::npos) << msg;
        EXPECT_NE(msg.find("v4"), std::string::npos) << msg;
        // And it must say which direction, since the remedies are opposite.
        EXPECT_NE(msg.find("predates"), std::string::npos) << msg;
    }
}

TEST(TrackerDmpRoundtrip, NewerVersionArtifactFailsLoudAndSaysRebuild_T044) {
    // The other direction: a stale BINARY against a newer artifact. Same class
    // of bug, opposite remedy, so the message must distinguish them.
    const std::string tampered = withSchemaVersion(serializeCurrent(), 99u);

    EvalResults out;
    std::istringstream is(tampered, std::ios::binary);
    cereal::BinaryInputArchive ia(is);

    try {
        ia(out);
        FAIL() << "reading a v99 artifact with a v3 reader must throw";
    } catch (const std::exception& e) {
        const std::string msg = e.what();
        EXPECT_NE(msg.find("v99"), std::string::npos) << msg;
        EXPECT_NE(msg.find("v4"), std::string::npos) << msg;
        EXPECT_NE(msg.find("NEWER"), std::string::npos) << msg;
    }
}

TEST(TrackerDmpRoundtrip, CurrentVersionStillRoundTrips_T044) {
    // The guard must reject mismatches WITHOUT rejecting the current schema —
    // a check that throws on everything would pass both tests above and break
    // every real read.
    const std::string blob = serializeCurrent();
    EvalResults out;
    std::istringstream is(blob, std::ios::binary);
    cereal::BinaryInputArchive ia(is);
    EXPECT_NO_THROW(ia(out));
    EXPECT_EQ(out.tickList.size(), 2u);
}
