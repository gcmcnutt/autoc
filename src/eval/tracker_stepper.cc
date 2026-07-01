// 030 M6d — TrackerStepper implementation.
//
// Drives chase-craft per-tick simulation off recorded source target
// trajectory. Symmetric with PathgenStepper but uses M5 beacon projection
// + M6d gather_tracker_inputs + NNControllerBackend::evaluateTracker
// instead of pathgen's path-following body.

#include "autoc/eval/tracker_stepper.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "autoc/eval/camera_projection.h"  // kCepSentinelThreshold (032 default gate)
#include "autoc/eval/crash_hull.h"     // M7c — geometric inside-hull telemetry
#include "autoc/eval/derived_features.h"  // 032 phase 1 — compute_pair_span
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
                               const CrashHull& crash_hull,
                               gp_scalar p_crash_this_gen,
                               uint32_t prng_seed,
                               gp_scalar trail_distance,
                               gp_scalar cep_gate_threshold)
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
      crash_hull_(crash_hull),
      p_crash_this_gen_(p_crash_this_gen),
      // Park-Miller LCG requires non-zero seed in [1, 2^31-2]. Map seed=0
      // (and any seed at/above the modulus) deterministically into the
      // valid range; OR with 1 to ensure non-zero.
      prng_state_(((prng_seed == 0 ? 0xC0FFEEu
                                   : prng_seed % 0x7FFFFFFFu)) | 1u),
      hull_fired_count_(0),
      trail_distance_(trail_distance),
      cep_gate_threshold_(cep_gate_threshold) {}

void TrackerStepper::initScenario() {
    nn_.reset();  // zero recurrent state at scenario start (no-op for feedforward)

    // Chase orientation: M1-style 180° yaw about Z so chase faces world -x
    // (= "south" in NED). Source dmps were recorded in this convention;
    // chase here matches.
    gp_quat aircraft_orientation =
        gp_quat(Eigen::AngleAxis<gp_scalar>(static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitY())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitX()));

    // 030 V1.5 (2026-05-09 chase-init-geometry fix) — chase initializes
    // 1.5 × trail_distance NORTH of source's tick-0 position. Source dmp
    // positions stay raw (rabbit/source coordinates are dmp-as-recorded;
    // the only thing offset is chase). At tick 0:
    //   - source effectively at (0,0,0) (raw dmp tick 0, untouched)
    //   - trail rabbit  = source - velocity_unit × trail_distance
    //                   = (0,0,0) - (-1,0,0)×3.048 = (+3.048, 0, 0)
    //   - chase init    = (+1.5 × trail_distance, 0, 0) = (+4.572, 0, 0)
    //   - chase is +0.5 × trail_distance BEHIND rabbit on the safe-cone
    //     side (along<0, distScaleBehind=7m gentle decay) → tick-0 score
    //     ≈ 0.955, real signal not the knife-edge of AT-rabbit
    //   - chase velocity = source[0].velocity — eliminates the
    //     SIM_INITIAL_VELOCITY=20 spike that drops to ~13 by tick 1
    // Pre-fix used 1.0 × trail_distance (chase AT rabbit, knife-edge).
    // For empty-source fallback (defensive — the worker guards against this
    // ahead of TrackerStepper construction): legacy M1 init at virtual
    // origin + 20 m/s spike.
    const bool source_has_samples = !source_.samples.empty();
    gp_vec3 initialPosition = source_has_samples
        ? gp_vec3(static_cast<gp_scalar>(1.5) * trail_distance_, 0.0f, 0.0f)
        : gp_vec3(0.0f, 0.0f, 0.0f);
    gp_vec3 initial_velocity = source_has_samples
        ? source_.samples.front().velocity
        : aircraft_orientation * gp_vec3(SIM_INITIAL_VELOCITY, 0.0f, 0.0f);
    gp_scalar initial_rel_vel = source_has_samples
        ? static_cast<gp_scalar>(initial_velocity.norm())
        : SIM_INITIAL_VELOCITY;

    state_ = AircraftState{0,
                           initial_rel_vel,
                           initial_velocity,
                           aircraft_orientation,
                           initialPosition,
                           0.0f,
                           0.0f,
                           SIM_INITIAL_THROTTLE,
                           0};

    duration_msec_ = 0;

    // 037 T022 — fail loud on a source library whose tick spacing does not
    // match the compiled cadence. stepOnce advances chase physics one
    // SIM_TIME_STEP_MSEC per SOURCE tick, so a 100 ms-recorded library run
    // at a 50 ms cadence would silently play the target at 2× speed.
    // 038 P0-D-1: STRICT single-gap check restored — simTimeMsec is now
    // round()-stamped (exact 50 ms gaps), so the first gap is a faithful cadence
    // probe again. (The 2026-06-15 average-gap workaround tolerated the old
    // truncation jitter of 49/50/51 ms, now fixed at the stamp.) Mirrors
    // crrcsim_tracker_helper.cpp.
    if (source_.samples.size() >= 2) {
        const auto& s = source_.samples;
        const long firstGapMsec =
            std::lround(s[1].simTimeMsec - s[0].simTimeMsec);
        if (firstGapMsec != SIM_TIME_STEP_MSEC) {
            throw std::runtime_error(
                "TrackerStepper: source trajectory tick spacing " +
                std::to_string(firstGapMsec) + " ms != compiled SIM_TIME_STEP_MSEC " +
                std::to_string(SIM_TIME_STEP_MSEC) +
                " ms — rebake the M2 source library at the current cadence.");
        }
    }

    // History pre-fill (M8b shape, post-PreRollSec-removal 2026-05-11). The
    // pre-roll feature was retired by the 2026-05-07 chase-init geometry
    // simplification — production has always run with pre_roll = 0. Behavior
    // preserved: replicate source[0]'s projection across the WHOLE
    // observation ring (037: depth grew with the R5 lag window) so the NN
    // sees a populated, non-zero history at tick 1.
    cursor_ = 0;
    obs_ring_.reset();

    // 038 P0-D FR-P0H (A) — reset situational-awareness state per scenario
    // (FR-030 determinism: un-reset blind-tick / exit-bearing state would leak
    // across scenarios and break the bitwise gate). The state is advanced only
    // on real ticks (stepOnce), NOT during the history pre-fill below, so it
    // starts each scenario at "visible-now / neutral bearing".
    sa_state_.reset();

    if (!source_.samples.empty()) {
        for (int r = 0; r < TrackerObservationRing::kDepth; ++r) {
            projectAndShiftHistory(source_.samples[0]);
        }
    }
}

void TrackerStepper::projectAndShiftHistory(const SourceTickSample& target) {
    // 037 T022 — observations land in the deep ring; the 6-slot gather view
    // (history_) is materialized from the ring at the R5 lag offsets at the
    // end of this function. (Pre-037: 6-slot shift-left register.)

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

    TrackerObservationRing::Record rec;
    rec.left_x = left.screen_x;       // raw-ok: NN-byte-format primitive
    rec.left_y = left.screen_y;       // raw-ok: NN-byte-format primitive
    rec.left_cep = left.cep;          // raw-ok: NN-byte-format primitive
    rec.right_x = right.screen_x;     // raw-ok: NN-byte-format primitive
    rec.right_y = right.screen_y;     // raw-ok: NN-byte-format primitive
    rec.right_cep = right.cep;        // raw-ok: NN-byte-format primitive

    // 032 PHASE 1 — Cache beacon-pair span at the current tick. CEP-gated:
    // if EITHER beacon's CEP exceeds the configured threshold, substitute
    // neutral 0.0 (= "no closing-distance signal"). Per spec Q4 + R2.
    const float cep_gate = static_cast<float>(cep_gate_threshold_);  // raw-ok: NN-byte-format comparison boundary
    const bool cep_gated =
        left.cep >= cep_gate ||
        right.cep >= cep_gate;
    if (cep_gated) {
        rec.span = 0.0f;
    } else {
        rec.span = static_cast<float>(  // raw-ok: NN-byte-format slot write
            autoc::eval::compute_pair_span(
                static_cast<gp_scalar>(left.screen_x),
                static_cast<gp_scalar>(left.screen_y),
                static_cast<gp_scalar>(right.screen_x),
                static_cast<gp_scalar>(right.screen_y)));
    }
    obs_ring_.push(rec);
    obs_ring_.materialize(history_);

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
    // 030 M7c: inside_crash_hull is the GEOMETRIC inside-hull check
    // (sphere intersection at default 1m radius). Probabilistic firing
    // + scenario termination are M7d's responsibility — this flag is
    // per-tick telemetry for renderer / inspect / M11c analytics.
    // Both trail_distance + hull_radius currently use compile-time
    // defaults; M7d plumbs the configurable values from autoc-tracker.ini
    // through EvalData.
    last_target_sample_.position = target.position;
    last_target_sample_.orientation = target.orientation;
    last_target_sample_.velocity = target.velocity;
    last_target_sample_.trail_rabbit_position =
        computeTrailRabbit(target, trail_distance_);
    last_target_sample_.inside_crash_hull =
        isInsideHull(crash_hull_, state_.getPosition(), target.position);
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

    // Step 1b (038 P0-D FR-P0H): advance the situational-awareness state from
    // the freshly-projected "now" beacon observation. Visibility uses the
    // sentinel threshold (matches fitness_decomposition.cc). Single-sourced
    // update rule mirrored in CrrcsimTrackerHelper::tick.
    sa_state_.update(history_.left_x[5], history_.left_y[5], history_.left_cep[5],
                     history_.right_x[5], history_.right_y[5], history_.right_cep[5],
                     autoc::eval::kCepSentinelThreshold);

    // Step 2: gather tracker NN inputs.
    TrackerInputs inputs = {};
    gather_tracker_inputs(state_, history_, arena_,
                          static_cast<float>(cep_gate_threshold_), sa_state_, inputs);

    // Step 3: NN forward pass → control commands.
    nn_.evaluateTracker(state_, inputs);

    // Step 4: advance chase physics. M6d simplification: one
    // SIM_TIME_STEP_MSEC step per source tick (assumes source nominal 100ms
    // tick interval, which matches pastonly3 source dmps). Variable-rate
    // source handling lands at FR-018 / M6f timing_model_tests.
    state_.advanceState(SIM_TIME_STEP_MSEC);
    duration_msec_ += SIM_TIME_STEP_MSEC;
    state_.setSimTimeMsec(duration_msec_);

    // 030 M7a — arena out-of-bounds via FR-016 config-driven arena.h.
    // Same FlightArena that fed `gather_tracker_inputs` slot 44 — single
    // source of truth between NN input AND egress termination per
    // Session 2026-05-07 Q1. Replaces the M8b shim's checkAircraftOOB
    // for tracker mode (pathgen keeps M1 hardcoded path).
    {
        const ArenaEgressKind egress = checkArenaBounds(state_, arena_);
        if (egress != ArenaEgressKind::NONE) {
            crash = arenaEgressToCrashReason(egress);
        }
    }

    // 030 M11.preA.3 (2026-05-10) — Crash-hull RE-ENABLED with deterministic
    // fixed-probability Bernoulli (no curriculum ramp). Seed comes from
    // the rabbit-class sub-PRNG (033 cleanup; pre-cleanup used windSeed —
    // see contracts/scenario_prng_chain.md crash-hull as rabbit-class
    // consumer). Stable across train/elite-reeval. prng_state_ is
    // consumed only when chase is INSIDE the hull AND p_crash > 0, so cost
    // for late-pop NNs that don't enter the hull is zero.
    if (crash == CrashReason::None) {
        if (didCrashFire(crash_hull_, state_.getPosition(), target.position,
                         p_crash_this_gen_, prng_state_)) {
            crash = CrashReason::HullStrike;
            ++hull_fired_count_;
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
