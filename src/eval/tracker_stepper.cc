// 030 M6d — TrackerStepper implementation.
//
// Drives chase-craft per-tick simulation off recorded source target
// trajectory. Symmetric with PathgenStepper but uses M5 beacon projection
// + M6d gather_tracker_inputs + NNControllerBackend::evaluateTracker
// instead of pathgen's path-following body.

#include "autoc/eval/tracker_stepper.h"

#include <algorithm>
#include <cmath>

#include "autoc/eval/trail_rabbit.h"  // M7b — real trail-rabbit math

namespace autoc::eval {

TrackerStepper::TrackerStepper(NNControllerBackend& nn,
                               AircraftState& state,
                               const SourceScenarioTrajectory& source,
                               const ScenarioMetadata& scenario_meta,
                               const CameraConfig& camera,
                               const BeaconConfig& beacon_left,
                               const BeaconConfig& beacon_right,
                               const AirframeProxy& airframe,
                               const FlightArena& arena,
                               int pre_roll_ticks)
    : nn_(nn),
      state_(state),
      source_(source),
      scenario_meta_(scenario_meta),
      camera_(camera),
      beacon_left_(beacon_left),
      beacon_right_(beacon_right),
      airframe_(airframe),
      arena_(arena),
      history_{},
      cursor_(0),
      duration_msec_(0),
      pre_roll_ticks_(pre_roll_ticks) {}

void TrackerStepper::initScenario() {
    nn_.reset();  // zero recurrent state at scenario start (no-op for feedforward)

    // Initial chase pose. M6d: same orientation/position convention as
    // pathgen (virtual origin, identity orientation modulo 180° about Z to
    // face the target). M6e may revisit when source-scenario entry params
    // (joint-PRNG variation) wire through.
    gp_quat aircraft_orientation =
        gp_quat(Eigen::AngleAxis<gp_scalar>(static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitY())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitX()));
    gp_vec3 initialPosition(0.0f, 0.0f, 0.0f);
    gp_vec3 initial_velocity =
        aircraft_orientation * gp_vec3(SIM_INITIAL_VELOCITY, 0.0f, 0.0f);

    state_ = AircraftState{0,
                           SIM_INITIAL_VELOCITY,
                           initial_velocity,
                           aircraft_orientation,
                           initialPosition,
                           0.0f,
                           0.0f,
                           SIM_INITIAL_THROTTLE,
                           0};

    duration_msec_ = 0;

    // 030 M8b geometry fix — source pre-roll. Source craft plays forward
    // pre_roll_ticks_ samples while chase sits at its M1-style entry
    // pose, so source has a head start and is in chase's forward FOV
    // when chase starts evolving. NN's 6-slot history is populated from
    // the last 6 source ticks of the pre-roll window.
    //
    // History layout after warm-start (pre_roll_ticks_ = N >= 6):
    //   slot 0 (oldest, t-0.5s) = source[N-6]  vs chase[init]
    //   slot 1 (t-0.4s)         = source[N-5]  vs chase[init]
    //   ...
    //   slot 5 (now, t-0.0s)    = source[N-1]  vs chase[init]
    //
    // For N < 6, the older slots replicate source[0]'s projection. For
    // N == 0, all 6 slots replicate source[0] (legacy behavior).
    cursor_ = 0;  // projectAndShiftHistory uses chase[init]; cursor advances
                  // through source samples for the warm-start projections.
    if (!source_.samples.empty()) {
        const int pre_roll = std::max(0, pre_roll_ticks_);
        const int kHistory = 6;
        // Replicate source[0] for the front-fill when pre_roll < 6 (older
        // slots get duplicated source[0] projection).
        const int replicate_count = (pre_roll < kHistory) ? (kHistory - pre_roll) : 0;
        const int real_start =
            (pre_roll >= kHistory) ? (pre_roll - kHistory) : 0;

        for (int r = 0; r < replicate_count; ++r) {
            projectAndShiftHistory(source_.samples[0]);
        }
        for (int k = real_start; k < pre_roll; ++k) {
            const size_t idx = static_cast<size_t>(k);
            if (idx < source_.samples.size()) {
                projectAndShiftHistory(source_.samples[idx]);
            }
        }
        // If pre_roll == 0, no real samples projected above — replicate
        // source[0] across all 6 slots so the NN sees a non-zero history.
        if (pre_roll == 0) {
            // The replicate_count loop already ran 6 times for pre_roll==0
            // (kHistory - 0 = 6), so no extra work. Validated by the math:
            // replicate_count=6, real_start=0, real loop body skipped.
        }
    }

    // Cursor starts at pre_roll_ticks so stepOnce #1 consumes
    // source[pre_roll_ticks] (the first non-pre-roll sample).
    cursor_ = static_cast<size_t>(std::max(0, pre_roll_ticks_));
}

void TrackerStepper::projectAndShiftHistory(const SourceTickSample& target) {
    // Shift slots [1..5] → [0..4]; new "now" lands at index 5.
    for (int i = 0; i < 5; ++i) {
        history_.left_x[i] = history_.left_x[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_y[i] = history_.left_y[i + 1];      // raw-ok: NN-byte-format primitive
        history_.left_cep[i] = history_.left_cep[i + 1];  // raw-ok: NN-byte-format primitive
        history_.right_x[i] = history_.right_x[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_y[i] = history_.right_y[i + 1];    // raw-ok: NN-byte-format primitive
        history_.right_cep[i] = history_.right_cep[i + 1];// raw-ok: NN-byte-format primitive
    }

    // Build projection inputs for both beacons. Camera mount + orientation
    // come from CameraConfig (chase body frame); beacons mount at
    // BeaconConfig::mount_body in target body frame.
    ProjectionInput proj;
    proj.chase_position_world = state_.getPosition();
    proj.chase_orientation_world = state_.getOrientation();
    proj.target_position_world = target.position;
    proj.target_orientation_world = target.orientation;
    proj.camera_mount_chase_body = camera_.mount_offset_body;
    proj.camera_orientation_chase_body = camera_.mount_orientation_body;
    proj.camera = camera_;
    proj.chase_airframe = airframe_;

    // Left beacon.
    proj.beacon_mount_target_body = beacon_left_.mount_body;
    proj.beacon_emission_axis_target_body = beacon_left_.emission_axis_body;
    proj.beacon = beacon_left_;
    BeaconObservation left = projectBeacon(proj);

    // Right beacon.
    proj.beacon_mount_target_body = beacon_right_.mount_body;
    proj.beacon_emission_axis_target_body = beacon_right_.emission_axis_body;
    proj.beacon = beacon_right_;
    BeaconObservation right = projectBeacon(proj);

    history_.left_x[5] = left.screen_x;       // raw-ok: NN-byte-format primitive
    history_.left_y[5] = left.screen_y;       // raw-ok: NN-byte-format primitive
    history_.left_cep[5] = left.cep;          // raw-ok: NN-byte-format primitive
    history_.right_x[5] = right.screen_x;     // raw-ok: NN-byte-format primitive
    history_.right_y[5] = right.screen_y;     // raw-ok: NN-byte-format primitive
    history_.right_cep[5] = right.cep;        // raw-ok: NN-byte-format primitive

    // 030 M8b — Per-tick recording for v=2 dmp output (FR-015).
    // Camera world-pose: chase_position + chase_orient * camera_mount;
    // chase_orient * camera_orient gives the camera frame in world coords.
    last_camera_view_.camera_pose_world_pos =
        state_.getPosition() + state_.getOrientation() * camera_.mount_offset_body;
    last_camera_view_.camera_pose_world_orient =
        state_.getOrientation() * camera_.mount_orientation_body;
    last_camera_view_.camera_fov_h_deg = static_cast<float>(camera_.fov_h_deg);   // raw-ok: cereal byte-format member (CameraViewSample fp32 contract)
    last_camera_view_.camera_fov_v_deg = static_cast<float>(camera_.fov_v_deg);   // raw-ok: cereal byte-format member (CameraViewSample fp32 contract)
    last_camera_view_.beacon_left = left;
    last_camera_view_.beacon_right = right;

    // Target sample copied verbatim from the source SourceTickSample.
    // 030 M7b: trail_rabbit_position now uses the real
    // computeTrailRabbit() function (FR-008 simplified shape per
    // Session 2026-05-07 Q1: target_pos − velocity_unit × trail_distance,
    // degenerate fallback rabbit ≡ target_pos when |velocity| < 1e-3).
    // inside_crash_hull defaults to false until M7c lands the hull check.
    // Trail distance currently uses the kDefaultTrailDistance compile-time
    // constant; M7d's FitnessComputer wire-up plumbs the configurable
    // TrailDistance from autoc-tracker.ini through EvalData.
    last_target_sample_.position = target.position;
    last_target_sample_.orientation = target.orientation;
    last_target_sample_.velocity = target.velocity;
    last_target_sample_.trail_rabbit_position = computeTrailRabbit(target);
    last_target_sample_.inside_crash_hull = false;
}

CrashReason TrackerStepper::stepOnce() {
    // Termination: source trajectory exhausted.
    if (cursor_ >= source_.samples.size()) {
        return CrashReason::TimeLimit;
    }

    CrashReason crash = CrashReason::None;

    // Current source tick's target state.
    const SourceTickSample& target = source_.samples[cursor_];

    // Step 1: project beacons + shift history.
    projectAndShiftHistory(target);

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(state_, history_, arena_, inputs);

    // Step 3: NN forward pass → control commands.
    nn_.evaluateTracker(state_, inputs);

    // Step 4: advance chase physics. M6d simplification: one
    // SIM_TIME_STEP_MSEC step per source tick (assumes source nominal 100ms
    // tick interval, which matches pastonly3 source dmps). Variable-rate
    // source handling lands at FR-018 / M6f timing_model_tests.
    state_.minisimAdvanceState(SIM_TIME_STEP_MSEC);
    duration_msec_ += SIM_TIME_STEP_MSEC;
    state_.setSimTimeMsec(duration_msec_);

    // 030 M7a — arena out-of-bounds via FR-016 config-driven arena.h.
    // Same FlightArena that fed `gather_tracker_inputs` slot 44 — single
    // source of truth between NN input AND egress termination per
    // Session 2026-05-07 Q1. Replaces the M8b shim's checkAircraftOOB
    // for tracker mode (pathgen keeps M1 hardcoded path). Crash hull
    // (FR-008b) lands at M7c.
    {
        const ArenaEgressKind egress = checkArenaBounds(state_, arena_);
        if (egress != ArenaEgressKind::NONE) {
            crash = arenaEgressToCrashReason(egress);
        }
    }

    ++cursor_;
    // Source exhaustion (TimeLimit) overrides Eval — last-write-wins
    // matches PathgenStepper's RabbitComplete > TimeLimit > Eval pattern.
    if (cursor_ >= source_.samples.size()) {
        crash = CrashReason::TimeLimit;
    }
    return crash;
}

}  // namespace autoc::eval
