// 030 M6a — PathgenStepper implementation.
//
// Body extracted byte-identically from the prior tools/minisim.cc per-tick
// loop (lines 143-265 in the pre-M6a tree). Math + tolerance constants +
// branch order preserved exactly so the regression-tight gate
// (rebuild-perf.sh + autoc-eval bitwise on baseline gen9200.dmp) holds
// across the M6a extraction commit.

#include "autoc/eval/pathgen_stepper.h"

#include <cmath>

#include "autoc/eval/sensor_math.h"
#include "autoc/nn/nn_input_computation.h"

namespace autoc::eval {

PathgenStepper::PathgenStepper(NNControllerBackend& nn,
                               AircraftState& state,
                               const std::vector<Path>& path,
                               const ScenarioMetadata& scenario_meta,
                               gp_scalar smoothness_floor,
                               SmoothnessMotionMode smoothness_mode)
    : nn_(nn),
      state_(state),
      path_(path),
      scenario_meta_(scenario_meta),
      smoothness_floor_(smoothness_floor),
      smoothness_mode_(smoothness_mode) {}

void PathgenStepper::initScenario() {
    nn_.reset();  // zero recurrent state at span start (no-op for feedforward)

    // Fixed initial orientation and position for deterministic evaluation.
    // Virtual coordinates: start at origin (0,0,0). Path is also at virtual origin.
    gp_quat aircraft_orientation =
        gp_quat(Eigen::AngleAxis<gp_scalar>(static_cast<gp_scalar>(M_PI), gp_vec3::UnitZ())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitY())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitX()));
    gp_vec3 initialPosition(0.0f, 0.0f, 0.0f);  // virtual origin
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

    {
        gp_vec3 tangent;
        if (path_.size() > 1)
            tangent = path_[1].start - path_[0].start;
        else
            tangent = gp_vec3::UnitX();
        double tn = tangent.norm();  // raw-ok: Eigen .norm() returns underlying scalar; tolerance literal
        if (tn > 1e-6) tangent = tangent / tn;
        else tangent = gp_vec3::UnitX();
        state_.resetHistory(path_[0].start, tangent);
    }
    state_.setRabbitOdometer(0.0f);

    // Get rabbit speed from scenario metadata.
    rabbit_speed_ = SIM_INITIAL_VELOCITY;
    if (scenario_meta_.rabbitSpeed > 0.0f) {  // raw-ok: ScenarioMetadata cereal byte-format member
        rabbit_speed_ = static_cast<gp_scalar>(scenario_meta_.rabbitSpeed);
    }
    state_.setRabbitSpeed(rabbit_speed_);

    // Set initial rabbit position (odometer=0 = first path point).
    {
        VectorPathProvider pathProvider(path_, 0);
        gp_vec3 rabbitPos =
            getInterpolatedTargetPosition(pathProvider, 0.0f, 0.0f);
        state_.setRabbitPosition(rabbitPos);
    }

    duration_msec_ = 0;

    // 033 §2.B — reset per-tick smoothness tracking. First tick of next
    // scenario gets factor = 1.0 (no "false-positive" penalty on scenario
    // entry). Per contracts/smoothness_factor.md "First tick of scenario".
    prev_out_valid_ = false;
    prev_out_pt_ = static_cast<gp_scalar>(0.0);
    prev_out_rl_ = static_cast<gp_scalar>(0.0);
    prev_out_th_ = static_cast<gp_scalar>(0.0);
}

CrashReason PathgenStepper::stepOnce() {
    // Per-tick body extracted byte-for-byte from the prior minisim loop
    // body. Crash-reason determination is layered exactly as before: OOB
    // is set first, but path-index / rabbit-position updates still run on
    // the OOB tick (they affect the recorded post-tick state); TimeLimit
    // and RabbitComplete checks fire afterward and may override only when
    // the prior crashReason is still None.
    CrashReason crash = CrashReason::None;

    // Capture temporal history before NN evaluation.
    {
        VectorPathProvider pathProvider(path_, state_.getThisPathIndex());
        gp_scalar rabbitOdo = state_.getRabbitOdometer();
        gp_vec3 targetPos =
            getInterpolatedTargetPosition(pathProvider, rabbitOdo, 0.0f);
        gp_vec3 craftToTarget = targetPos - state_.getPosition();
        gp_vec3 target_local =
            state_.getOrientation().inverse() * craftToTarget;
        float distance = static_cast<float>(target_local.norm());  // raw-ok: NN-byte-format primitive (NNInputs.dist[] is float[])

        // Path tangent for singularity fallback.
        gp_vec3 posAhead =
            getInterpolatedTargetPosition(pathProvider, rabbitOdo, 0.5f);
        gp_vec3 tangent = posAhead - targetPos;
        double tn = tangent.norm();  // raw-ok: Eigen .norm() returns underlying scalar; tolerance literal
        gp_vec3 tangent_body = (tn > 1e-6)
            ? state_.getOrientation().inverse() * (tangent / tn)
            : gp_vec3::UnitX();

        gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);
        state_.setRabbitPosition(targetPos);
        state_.recordErrorHistory(dir, distance, duration_msec_);
    }

    // Run NN controller (per-span instance owned by caller).
    {
        VectorPathProvider pathProvider(path_, state_.getThisPathIndex());
        nn_.evaluate(state_, pathProvider);
    }

    // 033 §2.B — compute per-tick smoothness factor from NN-output Δs vs
    // previous tick, then stash on AircraftState for downstream fitness +
    // data.dat consumption (single source of truth). First-tick path:
    // prev_out_valid_ = false → Δs all zero → factor = 1.0 (no penalty).
    {
        const gp_scalar curr_pt = state_.getPitchCommand();
        const gp_scalar curr_rl = state_.getRollCommand();
        const gp_scalar curr_th = state_.getThrottleCommand();
        gp_scalar dpt = static_cast<gp_scalar>(0.0);
        gp_scalar drl = static_cast<gp_scalar>(0.0);
        gp_scalar dth = static_cast<gp_scalar>(0.0);
        if (prev_out_valid_) {
            dpt = curr_pt - prev_out_pt_;
            drl = curr_rl - prev_out_rl_;
            dth = curr_th - prev_out_th_;
        }
        const gp_scalar factor = compute_smoothness_factor(
            dpt, drl, dth, smoothness_floor_, smoothness_mode_);
        state_.setSmoothnessFactor(factor);
        // Update prev_out for next tick (post-saturation; matches contract).
        prev_out_pt_ = curr_pt;
        prev_out_rl_ = curr_rl;
        prev_out_th_ = curr_th;
        prev_out_valid_ = true;
    }

    // Advance aircraft state.
    state_.minisimAdvanceState(SIM_TIME_STEP_MSEC);
    duration_msec_ += SIM_TIME_STEP_MSEC;
    state_.setSimTimeMsec(duration_msec_);

    // Advance rabbit odometer.
    gp_scalar dtSec = static_cast<gp_scalar>(SIM_TIME_STEP_MSEC) / 1000.0f;
    state_.setRabbitOdometer(state_.getRabbitOdometer() + rabbit_speed_ * dtSec);

    // Arena out-of-bounds check (030 M8b extraction). Body byte-identical
    // to the prior inline block via inline expansion of checkAircraftOOB
    // at -O3 (regression-tight invariant). M2 tracker mode now uses the
    // same shared check — see scenario_stepper.h.
    crash = checkAircraftOOB(state_);

    // Advance path index by scanning distanceFromStart against rabbit odometer.
    while (state_.getThisPathIndex() < static_cast<int>(path_.size()) - 2 &&
           path_.at(state_.getThisPathIndex()).distanceFromStart <
               state_.getRabbitOdometer()) {
        state_.setThisPathIndex(state_.getThisPathIndex() + 1);
    }

    // Update rabbit position to match advanced odometer (for renderer error bars).
    {
        VectorPathProvider pathProvider(path_, state_.getThisPathIndex());
        gp_vec3 rabbitPos = getInterpolatedTargetPosition(
            pathProvider, state_.getRabbitOdometer(), 0.0f);
        state_.setRabbitPosition(rabbitPos);
    }

    // Termination assignments are unconditional (last-write-wins) to match
    // the prior loop's behavior: Eval first, then TimeLimit overrides if
    // duration >= max, then RabbitComplete overrides if path advanced to
    // end. Priority on a coincident tick: RabbitComplete > TimeLimit > Eval.
    if (duration_msec_ >= SIM_TOTAL_TIME_MSEC) {
        crash = CrashReason::TimeLimit;
    }
    if (state_.getThisPathIndex() >= static_cast<int>(path_.size()) - 2) {
        crash = CrashReason::RabbitComplete;
    }
    return crash;
}

}  // namespace autoc::eval
