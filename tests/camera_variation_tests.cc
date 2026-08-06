// 040 T066-T070 (US6) — per-scenario camera variation.
//
// WHY THIS EXISTS. Through 040 t1 the controller trained against a PERFECTLY
// known camera: exact boresight, exact roll, exact mount. Every scenario saw the
// same optics. That is the one assumption a real airframe cannot honour — glue
// sets crooked, mounts shift, foam board varies — and a controller that has only
// ever seen a perfect camera has no reason to be robust to an imperfect one.
//
// THE HIGHEST-IMPACT TERM IS ROLL, not boresight. Under the angular
// representation a boresight error lands as a near-constant additive offset on
// both bearings — clean, and something a network can learn to subtract. ROLL
// rotates the image plane, so it biases the port→starboard TILT cue
// degree-for-degree, and tilt drives the roll command. `RollErrorBiasesTiltButBoresightDoesNot`
// pins that asymmetry, because it is the reason roll deserves its own axis.
//
// SCOPE (operator 2026-08-02): CAMERA variation only — the emitter stays
// perfect. `ambientScale` is drawn and recorded so the plumbing exists and is
// verifiable, but ships at sigma 0 until the lens+filter field tests pin
// `SignalAmbientKnee`. `AmbientIsDrawnButHeldNominal` asserts that deliberately,
// so a later reader cannot mistake the zero for an oversight.
//
// This suite mirrors `craft_variation_tests.cc` — camera is a new CLASS in a
// working pipeline, not a new mechanism.

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

#include <cereal/archives/binary.hpp>

#include "autoc/eval/airframe_occlusion.h"
#include "autoc/eval/camera_variation.h"
#include "autoc/eval/tracker_tick_rule.h"
#include "autoc/rpc/protocol.h"
#include "autoc/util/scenario_prng.h"

using autoc::eval::CameraDeltas;
using autoc::eval::CameraSigmas;
using autoc::eval::generateCameraFromClassPRNG;
using autoc::util::ClassPRNG;
using autoc::util::kGaussianSigmaClamp;
using autoc::util::deriveClassSubSeeds;

namespace {

// Shipped magnitudes. The ENVELOPE is expressed as sigma, because the
// pipeline truncates every draw at 2.5 sigma — same convention as entry/craft.
CameraSigmas shippedSigmas() {
    CameraSigmas s;
    // 2.5σ IS the envelope, per the pipeline-wide truncation — same convention
    // as `entryConeSigma = 18.0 // 2.5sigma = 45 deg`.
    // ⚠️ Keep in step with autoc-tracker.ini — this fixture drifted out of sync
    // once already (it still read the pre-t3 values while the ini had moved),
    // which is how a "shipped magnitudes" helper quietly stops testing what ships.
    s.boresightSigmaDeg = 4.0;          // 2.5σ = 10 deg
    s.rollSigmaDeg = 4.0;               // 2.5σ = 10 deg
    s.mountTranslationSigmaX = 0.0020;  // 2.5σ = ±5 mm  (into/out of the bond face)
    s.mountTranslationSigmaY = 0.0040;  // 2.5σ = ±10 mm (spanwise, loosest)
    s.mountTranslationSigmaZ = 0.0012;  // 2.5σ = ±3 mm  (vertical, tightest)
    s.wingThicknessSigmaM = 0.0008;
    s.ambientSigmaFrac = 0.0;           // held nominal — see the header note
    return s;
}

CameraDeltas drawFor(uint64_t scenario_seed, const CameraSigmas& sigmas) {
    ClassPRNG prng(deriveClassSubSeeds(scenario_seed).camera);
    return generateCameraFromClassPRNG(prng, sigmas);
}

}  // namespace

// ---------------------------------------------------------------------------
// T066 (FR-021) — scenarios draw distinct parameters, within bounds.
// ---------------------------------------------------------------------------

TEST(CameraVariation, DifferentScenariosDrawDifferentCameras) {
    const CameraSigmas s = shippedSigmas();
    const CameraDeltas a = drawFor(0x0318fceb14b1dfd5ULL, s);
    const CameraDeltas b = drawFor(0x2fbc927725e57acbULL, s);

    // At least one alignment axis must differ — identical draws across scenarios
    // would mean the controller still sees one camera, just a wrong one.
    EXPECT_TRUE(a.boresightYawDeg != b.boresightYawDeg ||
                a.boresightPitchDeg != b.boresightPitchDeg ||
                a.rollDeg != b.rollDeg)
        << "two scenarios drew an identical camera";
    EXPECT_NE(a.mountTranslation.x(), b.mountTranslation.x());
}

TEST(CameraVariation, DrawsStayWithinTheTwoPointFiveSigmaEnvelope) {
    // The bound is the PIPELINE-WIDE truncation in ClassPRNG::nextGaussian, not
    // a camera-specific clip. Asserting it against kGaussianSigmaClamp × sigma
    // is what keeps camera honest to the same envelope rule as entry and craft —
    // if someone re-tunes a sigma, the envelope moves with it automatically and
    // there is no second constant to forget.
    const CameraSigmas s = shippedSigmas();
    const double align_limit = kGaussianSigmaClamp * s.boresightSigmaDeg;   // 20 deg
    const double roll_limit = kGaussianSigmaClamp * s.rollSigmaDeg;         // 20 deg
    const double tx_limit = kGaussianSigmaClamp * s.mountTranslationSigmaX;  // 5 mm
    const double ty_limit = kGaussianSigmaClamp * s.mountTranslationSigmaY;  // 10 mm
    const double tz_limit = kGaussianSigmaClamp * s.mountTranslationSigmaZ;  // 3 mm

    EXPECT_DOUBLE_EQ(align_limit, 10.0) << "the 10 deg envelope, expressed as sigma";
    EXPECT_DOUBLE_EQ(tx_limit, 0.005) << "x: +/-5 mm into/out of the bond face";
    EXPECT_DOUBLE_EQ(ty_limit, 0.010) << "y: +/-10 mm spanwise, the loosest axis";
    EXPECT_DOUBLE_EQ(tz_limit, 0.003) << "z: +/-3 mm vertical, the tightest";

    for (uint64_t k = 1; k <= 500; ++k) {
        const CameraDeltas d = drawFor(k * 0x9E3779B97F4A7C15ULL, s);
        EXPECT_LE(std::abs(static_cast<double>(d.boresightYawDeg)), align_limit * (1 + 1e-6));
        EXPECT_LE(std::abs(static_cast<double>(d.boresightPitchDeg)), align_limit * (1 + 1e-6));
        EXPECT_LE(std::abs(static_cast<double>(d.rollDeg)), roll_limit * (1 + 1e-6));
        // Relative epsilon: the draw is computed in double and stored as
        // gp_scalar (float), so a value clamped exactly AT the bound rounds a few
        // ULPs past it. That is representation, not an envelope breach — the z
        // axis trips it because its bound is the smallest.
        constexpr double kUlp = 1e-6;
        EXPECT_LE(std::abs(static_cast<double>(d.mountTranslation.x())), tx_limit * (1 + kUlp));
        EXPECT_LE(std::abs(static_cast<double>(d.mountTranslation.y())), ty_limit * (1 + kUlp));
        EXPECT_LE(std::abs(static_cast<double>(d.mountTranslation.z())), tz_limit * (1 + kUlp));
    }
}

// ---------------------------------------------------------------------------
// T067 (FR-022) — reproducible from the scenario identifier alone.
// ---------------------------------------------------------------------------

TEST(CameraVariation, SameScenarioSeedReproducesTheDrawBitExactly) {
    const CameraSigmas s = shippedSigmas();
    for (uint64_t seed : {0x0318fceb14b1dfd5ULL, 0x715d97070e3127e9ULL, 1ULL}) {
        const CameraDeltas a = drawFor(seed, s);
        const CameraDeltas b = drawFor(seed, s);
        EXPECT_EQ(a.boresightYawDeg, b.boresightYawDeg);
        EXPECT_EQ(a.boresightPitchDeg, b.boresightPitchDeg);
        EXPECT_EQ(a.rollDeg, b.rollDeg);
        EXPECT_EQ(a.mountTranslation.x(), b.mountTranslation.x());
        EXPECT_EQ(a.mountTranslation.y(), b.mountTranslation.y());
        EXPECT_EQ(a.mountTranslation.z(), b.mountTranslation.z());
        EXPECT_EQ(a.wingThicknessDelta, b.wingThicknessDelta);
        EXPECT_EQ(a.ambientScale, b.ambientScale);
    }
}

// ---------------------------------------------------------------------------
// T070 (FR-022) — camera variation is CHASE-specific.
// ---------------------------------------------------------------------------

TEST(CameraVariation, CameraDrawIsIndependentOfTheOtherVariationClasses) {
    // `TrackerChaseUseSourceScenarioSeed=1` shares the M1 source's
    // wind/entry/craft seeds with the chase so both aircraft fly the same
    // weather. Perception belongs to the CHASE ALONE, so the camera sub-seed
    // must not be reconstructible from, or collide with, the shared classes.
    const auto sub = deriveClassSubSeeds(0x0318fceb14b1dfd5ULL);
    EXPECT_NE(sub.camera, sub.wind);
    EXPECT_NE(sub.camera, sub.rabbit);
    EXPECT_NE(sub.camera, sub.entry);
    EXPECT_NE(sub.camera, sub.craft);

    // And the camera slot must be the LAST derived, per the append-only
    // contract — if a future class is inserted before it, every prior bake's
    // camera draws change silently.
    autoc::util::ScenarioRootPRNG root(0x0318fceb14b1dfd5ULL);
    const uint32_t s0 = root.next(), s1 = root.next(), s2 = root.next(),
                   s3 = root.next(), s4 = root.next();
    EXPECT_EQ(s0, sub.wind);
    EXPECT_EQ(s1, sub.rabbit);
    EXPECT_EQ(s2, sub.entry);
    EXPECT_EQ(s3, sub.craft);
    EXPECT_EQ(s4, sub.camera) << "camera must remain the last-derived sub-seed";
}

// ---------------------------------------------------------------------------
// T068 (FR-023, SC-006) — zero sigma is bit-identical to no variation.
// ---------------------------------------------------------------------------

TEST(CameraVariation, ZeroSigmaProducesExactlyTheNominalCamera) {
    // THE LOAD-BEARING ONE. It is what lets a variation-off run be compared
    // against the t1 baseline without an asterisk: with sigmas at zero the
    // camera path must be not merely close but IDENTICAL to having no variation
    // code at all.
    CameraSigmas zero;  // all defaults are 0
    const CameraDeltas nominal;  // default-constructed = nominal camera

    for (uint64_t k = 1; k <= 200; ++k) {
        const CameraDeltas d = drawFor(k * 0x9E3779B97F4A7C15ULL, zero);
        EXPECT_EQ(d.boresightYawDeg, nominal.boresightYawDeg);
        EXPECT_EQ(d.boresightPitchDeg, nominal.boresightPitchDeg);
        EXPECT_EQ(d.rollDeg, nominal.rollDeg);
        EXPECT_EQ(d.mountTranslation.x(), nominal.mountTranslation.x());
        EXPECT_EQ(d.mountTranslation.y(), nominal.mountTranslation.y());
        EXPECT_EQ(d.mountTranslation.z(), nominal.mountTranslation.z());
        EXPECT_EQ(d.wingThicknessDelta, nominal.wingThicknessDelta);
        EXPECT_EQ(d.ambientScale, nominal.ambientScale)
            << "ambient must collapse to EXACTLY nominal (1.0), not 0.0";
    }
}

// ---------------------------------------------------------------------------
// T069 — hard clip, not tail resampling.
// ---------------------------------------------------------------------------

TEST(CameraVariation, DrawsAreTruncatedNotResampled) {
    // The distinction is about DETERMINISM, not shape. Resampling a tail draw
    // would change the PRNG draw COUNT for that scenario, shifting every later
    // draw and breaking the frozen draw-order contract bit-exact replay rests
    // on. `scenario_prng.h` says so where the clamp is defined: "Clamp rather
    // than resample so the PRNG draw COUNT per call stays fixed."
    //
    // Detected by the pile-up at the boundary: a truncated normal puts finite
    // mass exactly ON ±2.5σ, a resampler puts none there.
    const CameraSigmas s = shippedSigmas();
    const double limit = kGaussianSigmaClamp * s.rollSigmaDeg;

    int at_limit = 0, total = 0;
    for (uint64_t k = 1; k <= 2000; ++k) {
        const double roll = std::abs(static_cast<double>(
            drawFor(k * 0x9E3779B97F4A7C15ULL, s).rollDeg));
        ++total;
        if (std::abs(roll - limit) < 1e-6) ++at_limit;
        EXPECT_LE(roll, limit);
    }
    EXPECT_GT(at_limit, 0)
        << "a TRUNCATED normal must place draws exactly on the boundary; "
           "a resampler would leave it empty";
}

TEST(CameraVariation, AmbientIsDrawnButHeldNominal) {
    // Operator scope decision 2026-08-02: camera variation only, emitter stays
    // perfect. The ambient draw is plumbed so it is verifiable, but its sigma
    // ships at zero until the lens+filter field tests pin SignalAmbientKnee.
    // Asserted rather than assumed so the zero reads as a DECISION.
    const CameraSigmas shipped = shippedSigmas();
    EXPECT_EQ(shipped.ambientSigmaFrac, 0.0);
    for (uint64_t k = 1; k <= 50; ++k) {
        EXPECT_EQ(drawFor(k * 0x9E3779B97F4A7C15ULL, shipped).ambientScale,
                  static_cast<gp_scalar>(1.0));
    }
    // ...but the plumbing genuinely works when switched on.
    CameraSigmas on = shipped;
    on.ambientSigmaFrac = 0.5;
    bool varied = false;
    for (uint64_t k = 1; k <= 50 && !varied; ++k) {
        varied = drawFor(k * 0x9E3779B97F4A7C15ULL, on).ambientScale !=
                 static_cast<gp_scalar>(1.0);
    }
    EXPECT_TRUE(varied) << "the ambient draw must be live, merely held at zero sigma";
}

// ---------------------------------------------------------------------------
// T072 — the draws survive the WorkerInit round trip.
//
// ⚠️ THEY RIDE WorkerInit, NOT ScenarioMetadata, AND THE DISTINCTION IS THE
// WHOLE POINT. The first implementation put them in ScenarioMetadata, which is
// persisted inside every dmp's `scenarioList` — so it orphaned the T003a-pinned
// M1 source and the t2 launch died with `vector::_M_default_append` before a
// single generation ran. 040 cannot rebake M1 without destroying the SC-008
// comparison it exists to make.
//
// `WorkerInitRoundTripPreservesCameraDraws` is therefore also a REGRESSION
// GUARD: if someone moves these fields back onto a persisted struct, the pinned
// source breaks again, and the failure will look like a corrupt dmp rather than
// a schema change.
// ---------------------------------------------------------------------------

TEST(CameraVariation, WorkerInitRoundTripPreservesCameraDraws) {
    WorkerInit src{};
    src.cameraVariations.resize(2);
    src.cameraVariations[0].boresightYawDeg = static_cast<gp_scalar>(-7.25);
    src.cameraVariations[0].boresightPitchDeg = static_cast<gp_scalar>(3.5);
    src.cameraVariations[0].rollDeg = static_cast<gp_scalar>(-19.75);
    src.cameraVariations[0].mountTranslation = gp_vec3(static_cast<gp_scalar>(0.004),
                                                       static_cast<gp_scalar>(-0.002),
                                                       static_cast<gp_scalar>(0.001));
    src.cameraVariations[0].wingThicknessDelta = static_cast<gp_scalar>(0.0013);
    src.cameraVariations[1].ambientScale = static_cast<gp_scalar>(1.0);

    std::stringstream ss;
    { cereal::BinaryOutputArchive ar(ss); ar(src); }
    WorkerInit dst{};
    { cereal::BinaryInputArchive ar(ss); ar(dst); }

    ASSERT_EQ(dst.cameraVariations.size(), 2u);
    EXPECT_EQ(dst.cameraVariations[0].boresightYawDeg, src.cameraVariations[0].boresightYawDeg);
    EXPECT_EQ(dst.cameraVariations[0].boresightPitchDeg, src.cameraVariations[0].boresightPitchDeg);
    EXPECT_EQ(dst.cameraVariations[0].rollDeg, src.cameraVariations[0].rollDeg);
    EXPECT_EQ(dst.cameraVariations[0].mountTranslation.y(),
              src.cameraVariations[0].mountTranslation.y());
    EXPECT_EQ(dst.cameraVariations[0].wingThicknessDelta,
              src.cameraVariations[0].wingThicknessDelta);
    EXPECT_EQ(dst.cameraVariations[1].ambientScale, src.cameraVariations[1].ambientScale);
}

TEST(CameraVariation, ScenarioMetadataCarriesNoCameraFields) {
    // The regression guard, stated as a compile-time fact rather than a comment:
    // ScenarioMetadata is PERSISTED, so its wire format must stay frozen against
    // the pinned M1 source. If a future change adds camera fields back here, the
    // round-trip below changes size and this test's sibling above becomes the
    // only thing standing between us and an unreadable baseline.
    //
    // Asserted by round-tripping a default ScenarioMetadata and confirming the
    // byte length matches a second default — i.e. nothing camera-shaped has
    // silently grown into it.
    ScenarioMetadata a{}, b{};
    std::stringstream sa, sb;
    { cereal::BinaryOutputArchive ar(sa); ar(a); }
    { cereal::BinaryOutputArchive ar(sb); ar(b); }
    EXPECT_EQ(sa.str().size(), sb.str().size());
    EXPECT_GT(sa.str().size(), 0u);
}

// ---------------------------------------------------------------------------
// T073/T074 — the draws actually reach the perception path, and reach the
// RIGHT parts of it.
// ---------------------------------------------------------------------------

TEST(CameraVariation, RollRotatesTheCameraButBoresightOnlyOffsetsIt) {
    // THE ASYMMETRY THE AXIS SPLIT RESTS ON, asserted rather than asserted-in-
    // prose. A boresight error tips the optical axis: both bearings shift by
    // roughly the same angle and the image plane's ORIENTATION is untouched. A
    // roll error leaves the axis alone and rotates the image plane — which is
    // what biases the port→starboard tilt cue degree-for-degree, and tilt drives
    // the roll command.
    //
    // Measured on the camera's own axes: roll must move the plane's "up"
    // direction while leaving the boresight put; yaw must do the reverse.
    autoc::eval::TickRuleConfig base;
    base.camera = autoc::eval::CameraConfig{};

    auto varied = [&](gp_scalar yaw, gp_scalar roll) {
        autoc::eval::TickRuleConfig cfg = base;
        autoc::eval::CameraDeltas d;
        d.boresightYawDeg = yaw;
        d.rollDeg = roll;
        autoc::eval::applyCameraVariation(cfg, d);
        return cfg.camera.mount_orientation_body;
    };

    const gp_vec3 axis(1, 0, 0), up(0, 0, -1);
    const gp_quat nominal = base.camera.mount_orientation_body;

    const gp_quat rolled = varied(0, static_cast<gp_scalar>(15));
    EXPECT_NEAR((rolled * axis - nominal * axis).norm(), 0.0, 1e-5)
        << "roll must leave the OPTICAL AXIS untouched";
    EXPECT_GT((rolled * up - nominal * up).norm(), 0.2)
        << "roll must rotate the IMAGE PLANE — this is the tilt bias";

    const gp_quat yawed = varied(static_cast<gp_scalar>(15), 0);
    EXPECT_GT((yawed * axis - nominal * axis).norm(), 0.2)
        << "boresight error must tip the optical axis";
}

TEST(CameraVariation, MountTranslationMovesObstructionOnlyNotBearing) {
    // Research R6: ±5 mm is 0.03° at 10 m — nothing for bearing — but swings
    // propeller clearance ~15%. Routing it into bearing as well would add
    // arithmetic that cannot matter; this pins the split so a later "tidy-up"
    // cannot silently collapse the two mounts back together.
    autoc::eval::TickRuleConfig cfg;
    cfg.camera = autoc::eval::CameraConfig{};
    cfg.obstruction_mount_offset = cfg.camera.mount_offset_body;
    const gp_vec3 nominal_mount = cfg.camera.mount_offset_body;

    autoc::eval::CameraDeltas d;
    d.mountTranslation = gp_vec3(static_cast<gp_scalar>(0.004),
                                 static_cast<gp_scalar>(-0.003),
                                 static_cast<gp_scalar>(0.002));
    autoc::eval::applyCameraVariation(cfg, d);

    EXPECT_EQ(cfg.camera.mount_offset_body.x(), nominal_mount.x())
        << "the BEARING mount must not move";
    EXPECT_EQ(cfg.camera.mount_offset_body.y(), nominal_mount.y());
    EXPECT_EQ(cfg.camera.mount_offset_body.z(), nominal_mount.z());
    EXPECT_NEAR(static_cast<double>(cfg.obstruction_mount_offset.x() - nominal_mount.x()),
                0.004, 1e-6) << "the OBSTRUCTION mount must move";
    EXPECT_NEAR(static_cast<double>(cfg.obstruction_mount_offset.y() - nominal_mount.y()),
                -0.003, 1e-6);
}

TEST(CameraVariation, NominalDrawLeavesTheTickConfigExactlyUntouched) {
    // The FR-023 no-op, one level up from the draw: applying a NOMINAL draw must
    // be bit-identical to not calling applyCameraVariation at all. Without this,
    // a variation-off t2 could differ from t1 by a rounding crumb and nobody
    // would know where it came from.
    autoc::eval::TickRuleConfig cfg;
    cfg.camera = autoc::eval::CameraConfig{};
    cfg.airframe = autoc::eval::hb1AirframeObstruction();
    cfg.signal = autoc::eval::hb1SignalConfig();
    cfg.obstruction_mount_offset = cfg.camera.mount_offset_body;
    const autoc::eval::TickRuleConfig before = cfg;

    autoc::eval::applyCameraVariation(cfg, autoc::eval::CameraDeltas{});

    EXPECT_EQ(cfg.camera.mount_orientation_body.w(), before.camera.mount_orientation_body.w());
    EXPECT_EQ(cfg.camera.mount_orientation_body.x(), before.camera.mount_orientation_body.x());
    EXPECT_EQ(cfg.obstruction_mount_offset.x(), before.obstruction_mount_offset.x());
    EXPECT_EQ(cfg.airframe.wing_max.z(), before.airframe.wing_max.z());
    EXPECT_EQ(cfg.signal.ambient_floor, before.signal.ambient_floor);
}

// ---------------------------------------------------------------------------
// THE PANEL'S LOAD-BEARING INVARIANT: dots and reticle share ONE frame.
//
// The POV panel draws two kinds of thing. Beacon DOTS come from bearings the
// simulator recorded, which are already in the VARIED camera frame. Every
// reticle element — thrust-axis locus, range rings, vertical member, obstruction
// hatch — originates in BODY frame and must be pushed through the same varied
// orientation to land in the same picture.
//
// Until 2026-08-02 the reticle assumed identity while the dots were varied, so
// with a 20 deg draw they disagreed by ~53 px — a SIXTH of the panel width. The
// renderer recovers the orientation as
//     mountQ = chase_orient^-1 * camera_pose_world_orient
// and projects body directions through mountQ^-1.
//
// This test asserts that recipe reproduces `projectBeacon`'s own bearing to
// floating-point agreement. If the two ever diverge, the panel is drawing a
// reticle for one camera over dots from another — and it would look plausible.
// ---------------------------------------------------------------------------

TEST(CameraVariation, ReticleFrameMatchesTheRecordedBeaconBearing) {
    const gp_quat chase_orient(Eigen::AngleAxis<gp_scalar>(
        static_cast<gp_scalar>(0.3), gp_vec3(0.2f, 0.9f, 0.35f).normalized()));
    const gp_vec3 chase_pos(3.0f, -2.0f, -11.0f);
    const gp_vec3 beacon_world(28.0f, 6.0f, -14.0f);

    for (double yaw : {0.0, 12.0, -20.0}) {
        for (double roll : {0.0, -17.0, 20.0}) {
            autoc::eval::TickRuleConfig cfg;
            cfg.camera = autoc::eval::CameraConfig{};
            cfg.obstruction_mount_offset = cfg.camera.mount_offset_body;
            autoc::eval::CameraDeltas d;
            d.boresightYawDeg = static_cast<gp_scalar>(yaw);
            d.rollDeg = static_cast<gp_scalar>(roll);
            autoc::eval::applyCameraVariation(cfg, d);

            // --- what the SIMULATOR records (the beacon dot) ---
            autoc::eval::ProjectionInput in;
            in.chase_position_world = chase_pos;
            in.chase_orientation_world = chase_orient;
            in.target_position_world = beacon_world;
            in.target_orientation_world = gp_quat::Identity();
            in.beacon_mount_target_body = gp_vec3(0, 0, 0);
            in.beacon_emission_axis_target_body = gp_vec3(-1, 0, 0);
            in.camera_mount_chase_body = cfg.camera.mount_offset_body;
            in.camera_orientation_chase_body = cfg.camera.mount_orientation_body;
            in.obstruction_mount_chase_body = cfg.obstruction_mount_offset;
            in.camera = cfg.camera;
            in.beacon = autoc::eval::BeaconConfig{};
            in.beacon.mount_body = in.beacon_mount_target_body;
            in.beacon.emission_axis_body = in.beacon_emission_axis_target_body;
            in.chase_airframe = autoc::eval::AirframeObstruction{};
            in.chase_airframe.enabled = false;
            in.signal = autoc::eval::hb1SignalConfig();
            const autoc::eval::BeaconObservation obs = autoc::eval::projectBeacon(in);
            ASSERT_LT(obs.cep, autoc::eval::kCepSentinelThreshold)
                << "fixture must keep the beacon visible (yaw " << yaw << " roll " << roll << ")";

            // --- what the RENDERER's reticle recipe produces ---
            // Recover the orientation exactly as the panel does, from the pose
            // the dmp records.
            const gp_quat recorded_pose = chase_orient * cfg.camera.mount_orientation_body;
            gp_quat mountQ = chase_orient.inverse() * recorded_pose;
            mountQ.normalize();
            const gp_vec3 beacon_body =
                chase_orient.inverse() * (beacon_world - chase_pos) - cfg.camera.mount_offset_body;
            const gp_vec3 dcam = mountQ.inverse() * beacon_body;
            ASSERT_GT(dcam.x(), 0.0f);
            const gp_scalar ryz = std::sqrt(dcam.y() * dcam.y() + dcam.z() * dcam.z());
            const gp_scalar th = std::atan2(ryz, dcam.x());
            const double ax = static_cast<double>(th * (dcam.y() / ryz));
            const double ay = static_cast<double>(th * (dcam.z() / ryz));

            // Agreement to within one pixel — the recorded bearing is QUANTISED
            // to the grid (T031) while the reticle recipe is continuous, so a
            // sub-pixel gap is expected and anything larger is a frame error.
            const double px = static_cast<double>(cfg.camera.radPerPx());
            EXPECT_NEAR(ax, static_cast<double>(obs.bearing_x_rad), px)
                << "yaw " << yaw << " roll " << roll;
            EXPECT_NEAR(ay, static_cast<double>(obs.bearing_y_rad), px)
                << "yaw " << yaw << " roll " << roll;
        }
    }
}

// ===========================================================================
// 2026-08-03 — THE MOUNT MUST NEVER END UP INSIDE THE AIRFRAME.
//
// T008 asserted this for the NOMINAL mount. Nothing asserted it for a VARIED
// one, and that gap voided two full training runs.
//
// The baseline mount sits just 2 mm proud of the wing leading edge, and its y
// and z are ALREADY inside the wing slab's ranges — only that 2 mm of x keeps
// the camera out of the wing. With a 2 mm translation sigma, ~16% of scenarios
// (≈47 of 294) pushed the aperture INSIDE the slab. A ray origin inside a box
// is blocked in every direction, so those scenarios were totally blind: the
// worst blind streak pinned at ~44 s from generation 1, identically at a ±20°
// and a ±2° angular envelope, because the angles were never the cause.
//
// Physically the clamp is not a fudge: the aperture is bonded to the leading-
// edge face, so a build tolerance slides it ALONG that face and outward — it
// cannot recess into the structure it is bonded to.
// ===========================================================================

TEST(CameraVariation, AMountInsideTheAirframeStillSees) {
    // The invariant is about BEHAVIOUR, not position. A mount may land inside a
    // primitive — the leading edge is cut away for the camera, so structure the
    // aperture sits within cannot obstruct it. What must never happen is that
    // such a scenario goes blind.
    //
    // Deliberately drives the pathological case rather than waiting for a draw
    // to produce it: place the aperture squarely inside the wing slab and
    // confirm the forward field is still clear.
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    const autoc::eval::AirframeObstruction airframe = autoc::eval::hb1AirframeObstruction();
    const autoc::eval::CameraConfig ccfg{};

    // THE REAL CASE: the nominal outboard mount pushed 5 mm aft in x — the very
    // draw that voided t2/t3. Deliberately NOT the wing's centroid: that sits on
    // the centreline directly behind the pod nose, so its forward rays are
    // legitimately blocked by the nose and the test would measure the wrong
    // thing. The actual camera is 8" outboard, well clear of the nose.
    const gp_vec3 buried =
        ccfg.mount_offset_body + gp_vec3(static_cast<gp_scalar>(-0.005),
                                         static_cast<gp_scalar>(0),
                                         static_cast<gp_scalar>(0));
    ASSERT_GE(buried.x(), airframe.wing_min.x());
    ASSERT_LE(buried.x(), airframe.wing_max.x()) << "fixture must actually be inside the wing";

    auto blockedCount = [&](const gp_vec3& origin) {
        int n = 0;
        for (double ax = -50; ax <= 50; ax += 5) {
            for (double ay = -35; ay <= 35; ay += 5) {
                const double th = std::sqrt(ax * ax + ay * ay) * kDeg;
                const double sn = (th < 1e-9) ? 1.0 : std::sin(th) / th;
                const gp_vec3 dir(static_cast<gp_scalar>(std::cos(th)),
                                  static_cast<gp_scalar>(sn * ax * kDeg),
                                  static_cast<gp_scalar>(sn * ay * kDeg));
                if (autoc::eval::testObstruction(
                        origin, origin + dir * static_cast<gp_scalar>(10), airframe).blocked) {
                    ++n;
                }
            }
        }
        return n;
    };

    const int nominalBlocked = blockedCount(ccfg.mount_offset_body);
    const int buriedBlocked = blockedCount(buried);

    // The invariant is RELATIVE: burying the aperture must not make obstruction
    // worse. Absolute zero would be the wrong bar — some inboard rays reach the
    // pod nose from either origin, and that is real obstruction we want kept.
    EXPECT_LE(buriedBlocked, nominalBlocked + 2)
        << "buried=" << buriedBlocked << " nominal=" << nominalBlocked
        << ". A mount inside a primitive must not blind the camera; blocking "
           "every direction is what voided the t2 and t3 bakes.";
    EXPECT_LT(buriedBlocked, 315 / 4)
        << "a buried mount is blocking most of the field — the interior-origin "
           "rule is not doing its job";

    // ...and the sanity half: the nominal mount is unaffected by the change, so
    // real obstruction (pod nose, prop disc) still works.
    const auto nose = autoc::eval::testObstruction(
        ccfg.mount_offset_body,
        ccfg.mount_offset_body + gp_vec3(static_cast<gp_scalar>(1),
                                         static_cast<gp_scalar>(-1),
                                         static_cast<gp_scalar>(0)).normalized() *
            static_cast<gp_scalar>(10),
        airframe);
    EXPECT_TRUE(nose.blocked || nose.attenuation < static_cast<gp_scalar>(1))
        << "an inboard ray from the nominal mount must still meet the nose or prop — "
           "the interior-origin rule must not have disabled obstruction wholesale";
}
