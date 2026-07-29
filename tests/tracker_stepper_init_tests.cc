// 030 Session 2026-05-07 — TrackerStepper init-geometry contract tests.
//
// Locks the chase initialization geometry chosen during the M9.preA →
// post-pre-roll simplification (chat 2026-05-07):
//
//   - chase init position = (trail_distance, 0, 0) when source non-empty
//     ("10ft north of source's tick-0 position" = behind source's velocity
//     heading; source positions stay raw in dmp / log / renderer)
//   - chase init velocity = source.samples.front().velocity (world-frame
//     copy) — eliminates the SIM_INITIAL_VELOCITY=20 → 13 spike that
//     dominated tick-0 NN inputs
//   - chase init relVel scalar = velocity.norm() — keeps the worker's
//     dRelVel-from-throttle math consistent with the velocity vector
//   - empty-source fallback (defensive — worker guards against this
//     ahead of construction): legacy origin + SIM_INITIAL_VELOCITY × forward
//   - chase orientation = M1-style 180° yaw about Z (faces world -x)
//
// Test harness builds a minimal NNGenome (empty weights/topology) so
// nn.reset() is a no-op; the test exercises only initScenario, never
// stepOnce. Source trajectory is synthesized in-memory (no S3, no fixture).

#include <gtest/gtest.h>

#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/eval/beacon_config.h"
#include "autoc/eval/camera_config.h"
#include "autoc/eval/camera_projection.h"
#include "autoc/eval/crash_hull.h"
#include "autoc/eval/source_trajectory.h"
#include "autoc/eval/tracker_stepper.h"
#include "autoc/nn/evaluator.h"
#include "autoc/types.h"

using autoc::eval::AirframeProxy;
using autoc::eval::BeaconConfig;
using autoc::eval::CameraConfig;
using autoc::eval::CrashHull;
using autoc::eval::FlightArena;
using autoc::eval::TrackerStepper;
using autoc::eval::defaultAirframeProxyHB1;

namespace {

NNGenome makeMinimalGenome() {
    NNGenome g;
    // Empty topology + recurrent ⇒ nn_hidden_state_count returns 0 ⇒
    // NNControllerBackend's hidden_state_ vector stays empty ⇒ reset() is
    // a no-op. We never call evaluate/evaluateTracker in these tests.
    return g;
}

SourceTickSample makeSourceTickSample(double simTimeMsec,
                                      const gp_vec3& pos,
                                      const gp_vec3& vel,
                                      const gp_quat& orient = gp_quat::Identity()) {
    SourceTickSample s;
    s.simTimeMsec = simTimeMsec;
    s.position = pos;
    s.orientation = orient;
    s.velocity = vel;
    return s;
}

SourceScenarioTrajectory makeSyntheticSource(int tick_count,
                                             const gp_vec3& tick0_vel) {
    SourceScenarioTrajectory traj;
    traj.sourceScenarioIndex = 0;
    traj.variation = ScenarioMetadata{};
    traj.samples.reserve(tick_count);
    gp_vec3 pos(0.0f, 0.0f, 0.0f);
    for (int t = 0; t < tick_count; ++t) {
        traj.samples.push_back(makeSourceTickSample(
            t * static_cast<double>(SIM_TIME_STEP_MSEC),  // compiled NN cadence
            pos,
            tick0_vel,
            gp_quat::Identity()));
        // Advance position by velocity × one tick for downstream realism;
        // initScenario only reads samples[0] so later ticks don't matter
        // for these tests. (037: spacing MUST equal SIM_TIME_STEP_MSEC —
        // initScenario fails loud on mismatched source libraries.)
        pos += tick0_vel * (SIM_TIME_STEP_MSEC / 1000.0f);
    }
    return traj;
}

// Construct a TrackerStepper with sensible defaults and the caller-
// supplied source trajectory. nn + state are owned by the test fixture.
TrackerStepper makeStepperWithSource(NNControllerBackend& nn,
                                     AircraftState& state,
                                     const SourceScenarioTrajectory& source) {
    return TrackerStepper(nn, state, source, ScenarioMetadata{},
                          CameraConfig{},
                          BeaconConfig{},
                          BeaconConfig{},
                          defaultAirframeProxyHB1(),
                          FlightArena{},
                          CrashHull{},
                          /*p_crash_this_gen=*/0.0f,
                          /*prng_seed=*/0,
                          /*trail_distance=*/3.048f,
                          /*cep_gate_threshold=*/1.25f);
}

}  // namespace

// ---------------------------------------------------------------------------
// Position: chase starts 1.5 × trail_distance "north" (= world +x) of source's
// tick-0 position when source is non-empty. The 1.5× multiplier (rather than
// 1.0×) places chase 0.5 × trail_distance BEHIND trail rabbit on the
// safe-cone side — gentle distScaleBehind=7m gradient, tick-0 score ≈ 0.955,
// real signal not the AT-rabbit knife-edge. Pre-2026-05-09 was 1.0×.
// ---------------------------------------------------------------------------

TEST(TrackerStepperInit, ChasePositionIsNorthOfSourceByTrailDistance) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;

    // Source velocity (-12.77, -3.14, -1.23) m/s — matches pastonly3 tick 0.
    SourceScenarioTrajectory source =
        makeSyntheticSource(/*tick_count=*/30, gp_vec3(-12.77f, -3.14f, -1.23f));

    TrackerStepper stepper = makeStepperWithSource(nn, state, source);
    stepper.initScenario();

    // Chase at (+1.5 × trail_distance, 0, 0). With trail_distance = 3.048 (10ft):
    // 1.5 × 3.048 = 4.572 m.
    EXPECT_NEAR(state.getPosition().x(), 4.572f, 1e-5f);
    EXPECT_NEAR(state.getPosition().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.getPosition().z(), 0.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// Velocity: chase init copies source.samples.front().velocity verbatim.
// ---------------------------------------------------------------------------

TEST(TrackerStepperInit, ChaseVelocityCopiedFromSourceTick0) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;

    const gp_vec3 src_vel(-12.77f, -3.14f, -1.23f);
    SourceScenarioTrajectory source = makeSyntheticSource(30, src_vel);

    TrackerStepper stepper = makeStepperWithSource(nn, state, source);
    stepper.initScenario();

    EXPECT_FLOAT_EQ(state.getVelocity().x(), src_vel.x());
    EXPECT_FLOAT_EQ(state.getVelocity().y(), src_vel.y());
    EXPECT_FLOAT_EQ(state.getVelocity().z(), src_vel.z());
}

// ---------------------------------------------------------------------------
// relVel scalar: matches velocity vector magnitude (not SIM_INITIAL_VELOCITY).
// ---------------------------------------------------------------------------

TEST(TrackerStepperInit, ChaseRelVelMatchesVelocityMagnitude) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;

    const gp_vec3 src_vel(-12.77f, -3.14f, -1.23f);
    SourceScenarioTrajectory source = makeSyntheticSource(30, src_vel);

    TrackerStepper stepper = makeStepperWithSource(nn, state, source);
    stepper.initScenario();

    const gp_scalar expected_mag = src_vel.norm();
    EXPECT_NEAR(state.getRelVel(), expected_mag, 1e-5f);
    // And it must NOT be the legacy SIM_INITIAL_VELOCITY = 20.0f spike.
    EXPECT_LT(state.getRelVel(), 20.0f);
    EXPECT_GT(state.getRelVel(), 10.0f);  // pastonly3 tick 0 magnitude ~13
}

// ---------------------------------------------------------------------------
// Orientation: M1-style 180° yaw about Z. Identity rotation 'aligned with
// world -x' isn't directly checkable, but quat.z near sin(π/2) = 1 and
// quat.w near cos(π/2) = 0 confirms the 180° about-Z rotation.
// ---------------------------------------------------------------------------

TEST(TrackerStepperInit, ChaseOrientationIs180YawAboutZ) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;

    SourceScenarioTrajectory source =
        makeSyntheticSource(30, gp_vec3(-13.0f, 0.0f, 0.0f));

    TrackerStepper stepper = makeStepperWithSource(nn, state, source);
    stepper.initScenario();

    const gp_quat& q = state.getOrientation();
    // 180° about Z ⇒ q ≈ (0, 0, 0, 1) in (w, x, y, z) ordering. Eigen's
    // AngleAxis builds this with possible sign flip — check magnitudes.
    EXPECT_NEAR(std::abs(q.w()), 0.0f, 1e-5f);
    EXPECT_NEAR(std::abs(q.x()), 0.0f, 1e-5f);
    EXPECT_NEAR(std::abs(q.y()), 0.0f, 1e-5f);
    EXPECT_NEAR(std::abs(q.z()), 1.0f, 1e-5f);

    // Sanity check: chase forward (body +x, rotated by q) should be world
    // -x. q * UnitX() ⇒ -UnitX() under 180° about Z.
    const gp_vec3 forward_world = q * gp_vec3::UnitX();
    EXPECT_NEAR(forward_world.x(), -1.0f, 1e-5f);
    EXPECT_NEAR(forward_world.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(forward_world.z(), 0.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// Empty-source fallback: legacy origin + SIM_INITIAL_VELOCITY × world-(-x).
// Defensive path — worker won't construct TrackerStepper with empty source
// in production (it falls back to Eval-crash) — but the fallback ensures
// initScenario is well-defined for unit/integration tests that bypass that
// guard.
// ---------------------------------------------------------------------------

TEST(TrackerStepperInit, EmptySourceFallsBackToLegacyInit) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;

    SourceScenarioTrajectory empty_source;  // samples vector default-empty
    TrackerStepper stepper = makeStepperWithSource(nn, state, empty_source);
    stepper.initScenario();

    EXPECT_NEAR(state.getPosition().x(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.getPosition().y(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.getPosition().z(), 0.0f, 1e-5f);
    EXPECT_NEAR(state.getRelVel(), SIM_INITIAL_VELOCITY, 1e-5f);
    // Velocity = orient * (SIM_INITIAL_VELOCITY, 0, 0) = -SIM_INITIAL_VELOCITY in x
    // (180° yaw flips x). Magnitude = SIM_INITIAL_VELOCITY.
    EXPECT_NEAR(state.getVelocity().x(), -SIM_INITIAL_VELOCITY, 1e-3f);
    EXPECT_NEAR(state.getVelocity().norm(), SIM_INITIAL_VELOCITY, 1e-3f);
}

// ---------------------------------------------------------------------------
// 040 T007 (FR-020a) — per-scenario perception-state reset.
//
// These guard the trap the constitution and the existing situational-awareness
// code both warn about: carried perception state that is NOT reset at a
// scenario boundary leaks into the next scenario and breaks the bitwise gate.
// The leak is invisible — no crash, no failing assertion, just a different
// training trajectory nobody can attribute months later.
//
// 040 routes every reset through resetPerceptionState() so the acquisition
// state machine (Stage E) is picked up by BOTH execution paths without either
// being edited. These tests pin that contract at the observable level:
// re-initialising a scenario must reproduce first-tick perception exactly,
// regardless of what ran before it.
// ---------------------------------------------------------------------------

namespace {

// Capture the NN-facing perception slots after a given number of ticks.
std::vector<float> perceptionAfterTicks(TrackerStepper& stepper, int ticks) {  // raw-ok: NN-byte-format buffer
    stepper.initScenario();
    for (int i = 0; i < ticks; ++i) stepper.stepOnce();
    const CameraViewSample& v = stepper.lastCameraView();
    return {v.beacon_left.screen_x,  v.beacon_left.screen_y,  v.beacon_left.cep,
            v.beacon_right.screen_x, v.beacon_right.screen_y, v.beacon_right.cep};
}

}  // namespace

TEST(TrackerStepperReset, ReinitReproducesFirstTickPerception) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;
    SourceScenarioTrajectory source =
        makeSyntheticSource(/*tick_count=*/40, gp_vec3(-13.0f, 0.0f, 0.0f));
    TrackerStepper stepper = makeStepperWithSource(nn, state, source);

    const std::vector<float> first = perceptionAfterTicks(stepper, 1);  // raw-ok: NN-byte-format buffer
    const std::vector<float> again = perceptionAfterTicks(stepper, 1);  // raw-ok: NN-byte-format buffer

    ASSERT_EQ(first.size(), again.size());
    for (size_t i = 0; i < first.size(); ++i) {
        // BIT-exact, not near: determinism is the contract (FR-020).
        EXPECT_EQ(first[i], again[i])
            << "perception slot " << i << " differs after re-init — carried "
               "state leaked across the scenario boundary (FR-020a)";
    }
}

TEST(TrackerStepperReset, DeepRunDoesNotContaminateNextScenario) {
    NNGenome genome = makeMinimalGenome();
    NNControllerBackend nn(genome, autoc::eval::FlightArena{});
    AircraftState state;
    SourceScenarioTrajectory source =
        makeSyntheticSource(/*tick_count=*/40, gp_vec3(-13.0f, 0.0f, 0.0f));
    TrackerStepper stepper = makeStepperWithSource(nn, state, source);

    // Baseline: a fresh scenario's first tick.
    const std::vector<float> baseline = perceptionAfterTicks(stepper, 1);  // raw-ok: NN-byte-format buffer

    // Run deep enough to fill the observation ring and advance any carried
    // state well past its initial value, then re-init and re-measure.
    perceptionAfterTicks(stepper, TrackerObservationRing::kDepth + 5);
    const std::vector<float> after_deep_run = perceptionAfterTicks(stepper, 1);  // raw-ok: NN-byte-format buffer

    ASSERT_EQ(baseline.size(), after_deep_run.size());
    for (size_t i = 0; i < baseline.size(); ++i) {
        EXPECT_EQ(baseline[i], after_deep_run[i])
            << "perception slot " << i << " differs after a deep prior run — "
               "ring or acquisition state survived initScenario() (FR-020a)";
    }
}
