// 030 M6d — TrackerStepper implementation.
//
// Drives chase-craft per-tick simulation off recorded source target
// trajectory. Symmetric with PathgenStepper but uses M5 beacon projection
// + M6d gather_tracker_inputs + NNControllerBackend::evaluateTracker
// instead of pathgen's path-following body.

#include "autoc/eval/tracker_stepper.h"

#include <cmath>

namespace autoc::eval {

TrackerStepper::TrackerStepper(NNControllerBackend& nn,
                               AircraftState& state,
                               const SourceScenarioTrajectory& source,
                               const ScenarioMetadata& scenario_meta,
                               const CameraConfig& camera,
                               const BeaconConfig& beacon_left,
                               const BeaconConfig& beacon_right,
                               const AirframeProxy& airframe,
                               const gp_vec3& home_world)
    : nn_(nn),
      state_(state),
      source_(source),
      scenario_meta_(scenario_meta),
      camera_(camera),
      beacon_left_(beacon_left),
      beacon_right_(beacon_right),
      airframe_(airframe),
      home_world_(home_world),
      history_{},
      cursor_(0),
      duration_msec_(0) {}

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

    // Pre-fill beacon history with the first source tick's projection so
    // the NN sees a consistent 6-slot window at the first NN evaluation
    // (parallels pathgen's resetHistory).
    cursor_ = 0;
    duration_msec_ = 0;
    if (!source_.samples.empty()) {
        const SourceTickSample& first = source_.samples.front();
        projectAndShiftHistory(first);
        // After the first projection, all 6 slots hold the same observation
        // (shift-and-overwrite was called once above; copy the "now" value
        // backward across the older slots for warm-start).
        for (int i = 0; i < 5; ++i) {
            history_.left_x[i] = history_.left_x[5];      // raw-ok: NN-byte-format primitive
            history_.left_y[i] = history_.left_y[5];      // raw-ok: NN-byte-format primitive
            history_.left_cep[i] = history_.left_cep[5];  // raw-ok: NN-byte-format primitive
            history_.right_x[i] = history_.right_x[5];    // raw-ok: NN-byte-format primitive
            history_.right_y[i] = history_.right_y[5];    // raw-ok: NN-byte-format primitive
            history_.right_cep[i] = history_.right_cep[5];// raw-ok: NN-byte-format primitive
        }
    }
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
}

CrashReason TrackerStepper::stepOnce() {
    // Termination: source trajectory exhausted.
    if (cursor_ >= source_.samples.size()) {
        return CrashReason::TimeLimit;
    }

    // Current source tick's target state.
    const SourceTickSample& target = source_.samples[cursor_];

    // Step 1: project beacons + shift history.
    projectAndShiftHistory(target);

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(state_, history_, home_world_, inputs);

    // Step 3: NN forward pass → control commands.
    nn_.evaluateTracker(state_, inputs);

    // Step 4: advance chase physics. M6d simplification: one
    // SIM_TIME_STEP_MSEC step per source tick (assumes source nominal 100ms
    // tick interval, which matches pastonly3 source dmps). Variable-rate
    // source handling lands at FR-018 / M6f timing_model_tests.
    state_.minisimAdvanceState(SIM_TIME_STEP_MSEC);
    duration_msec_ += SIM_TIME_STEP_MSEC;
    state_.setSimTimeMsec(duration_msec_);

    // M6d does NOT yet wire arena egress (FR-016) or crash hull (FR-008b).
    // Both land at M7 (tracker-mode FitnessComputer integration). For now
    // the only termination is source exhaustion (TimeLimit) above.

    ++cursor_;
    if (cursor_ >= source_.samples.size()) {
        return CrashReason::TimeLimit;
    }
    return CrashReason::None;
}

}  // namespace autoc::eval
