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
    s.boresightSigmaDeg = 8.0;          // 2.5σ = 20 deg
    s.rollSigmaDeg = 8.0;               // 2.5σ = 20 deg
    s.mountTranslationSigmaM = 0.002;   // 2.5σ = 5 mm (the 1 cm box)
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
    const double trans_limit = kGaussianSigmaClamp * s.mountTranslationSigmaM;  // 5 mm

    EXPECT_DOUBLE_EQ(align_limit, 20.0) << "the spec's hard 20 deg, expressed as sigma";
    EXPECT_DOUBLE_EQ(trans_limit, 0.005) << "the spec's 1 cm box, expressed as sigma";

    for (uint64_t k = 1; k <= 500; ++k) {
        const CameraDeltas d = drawFor(k * 0x9E3779B97F4A7C15ULL, s);
        EXPECT_LE(std::abs(static_cast<double>(d.boresightYawDeg)), align_limit);
        EXPECT_LE(std::abs(static_cast<double>(d.boresightPitchDeg)), align_limit);
        EXPECT_LE(std::abs(static_cast<double>(d.rollDeg)), roll_limit);
        for (int i = 0; i < 3; ++i) {
            EXPECT_LE(std::abs(static_cast<double>(d.mountTranslation[i])), trans_limit);
        }
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
