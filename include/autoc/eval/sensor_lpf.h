// 043 §6/§9 — the NN's own sensor path, as the FLIGHT ARTICLE actually delivers it.
// 044 FR-008 — WIRED IN, at the FDM sub-step (option (a) below).
//
// ⛔ WHY THIS EXISTS. Measured 2026-09-05 (docs/inav-acro-path-audit.md §6): the
// real aircraft filters the gyro and accelerometer BEFORE they reach the policy,
// and the sim did not model any of it — it handed the NN raw FDM state.
//
//   channel | real (what the NN gets)                  | sim (044)
//   --------|------------------------------------------|---------------------------
//   gyro    | gyro.gyroADCf: gyro_main_lpf_hz = 25 PT1 | FcSensorFilterChain, 5 ms
//   accel   | acc.accADCf:   acc_lpf_hz = 15 BIQUAD    | FcSensorFilterChain, 5 ms
//
// Low-frequency group delay on the policy's OWN INPUTS: gyro PT1 rc =
// 1/(2*pi*25) = 6.4 ms; accel biquad (Q 0.707) 2*zeta/wn = sqrt2/(2*pi*15) =
// 15.0 ms (14.7 ms at the 200 Hz sub-step). ⚠️ An earlier note said ~21 ms for
// the accel: that is 2/(2*pi*fc), the CRITICALLY-damped figure, not this
// Butterworth's (044, tick_rescale_tests FcSensorFilters.GroupDelaysAtTheSubstep).
//
// ⛔ WHY IT MUST RUN AT THE SUB-STEP. An attempt to apply these at the 20 Hz
// sensor-gather point was BACKED OUT 2026-09-07: a 25 Hz or 15 Hz filter run at
// 20 Hz sits far above the 10 Hz Nyquist and is not a model of anything
// (measured k = 0.887 gyro / 0.825 accel, near pass-through). The real chain is
// filter at the FC rate, THEN sample at 20 Hz over MSP; its effect on the
// SAMPLED value is a group delay. 044 does exactly that: crrcsim steps one
// FcSensorFilterChain per scenario after every 5 ms FDM sub-step
// (autoc_sensor_substep, called from fdm_larcsim) and the 20 Hz gather reads
// its output. ⚠️ Do NOT run these filters on a 20 Hz path.
//
// ⓘ NOT modelled, deliberately:
//   - the dynamic gyro notch (2D, Q 250, >= 30 Hz): above the band of interest
//     and its phase contribution below 10 Hz is negligible.
//   - the gyro Kalman: it was TURNED OFF on the article 2026-09-05 precisely
//     because an input-adaptive filter with state-dependent group delay does not
//     belong in front of a trained policy. Modelling it would re-introduce what
//     the config change removed.
//   - INAV's own rate loop reads the same filtered gyro on the aircraft; the sim's
//     Cntrl_InavFwRate still reads the raw FDM rate (FR-008 scopes the POLICY's
//     signals; see BACKLOG if the loop's 6 ms ever matters).
#ifndef AUTOC_SENSOR_LPF_H
#define AUTOC_SENSOR_LPF_H

#include <cmath>
#include "autoc/types.h"

namespace autoc { namespace eval {

// First-order low-pass, INAV's PT1 form. Group delay ~ 1/(2*pi*fc).
class Pt1Filter {
 public:
  void configure(double cutoffHz, double dtSec) {
    const double rc = 1.0 / (2.0 * M_PI * cutoffHz);
    k_ = dtSec / (rc + dtSec);
    ready_ = true;
  }
  // ⚠️ First sample seeds the state rather than ramping from zero: a scenario
  // starts with the aircraft already in motion, so ramping would inject a
  // transient the real sensor never has.
  double apply(double x) {
    if (!seeded_) { y_ = x; seeded_ = true; return y_; }
    y_ += k_ * (x - y_);
    return y_;
  }
  void reset() { seeded_ = false; y_ = 0.0; }
  bool ready() const { return ready_; }
 private:
  double k_ = 0.0, y_ = 0.0;
  bool ready_ = false, seeded_ = false;
};

// Second-order Butterworth biquad, INAV's BIQUAD_LPF form (direct form 1).
// Group delay ~ 2/(2*pi*fc) in the passband.
class BiquadFilter {
 public:
  void configure(double cutoffHz, double dtSec) {
    const double omega = 2.0 * M_PI * cutoffHz * dtSec;
    const double sn = std::sin(omega), cs = std::cos(omega);
    const double alpha = sn / (2.0 * 0.7071067811865475);  // Q = 1/sqrt(2)
    const double b0 = (1.0 - cs) * 0.5, b1 = 1.0 - cs, b2 = (1.0 - cs) * 0.5;
    const double a0 = 1.0 + alpha, a1 = -2.0 * cs, a2 = 1.0 - alpha;
    b0_ = b0 / a0; b1_ = b1 / a0; b2_ = b2 / a0; a1_ = a1 / a0; a2_ = a2 / a0;
    ready_ = true;
  }
  double apply(double x) {
    if (!seeded_) { x1_ = x2_ = y1_ = y2_ = x; seeded_ = true; return x; }
    const double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = x; y2_ = y1_; y1_ = y;
    return y;
  }
  void reset() { seeded_ = false; x1_ = x2_ = y1_ = y2_ = 0.0; }
  bool ready() const { return ready_; }
 private:
  double b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;
  double x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
  bool ready_ = false, seeded_ = false;
};

// The article's measured constants (xiao/inav-hb1.cfg).
constexpr double kGyroLpfHz = 25.0;   // gyro_main_lpf_hz, PT1
constexpr double kAccelLpfHz = 15.0;  // acc_lpf_hz, BIQUAD

// 044 FR-008 — the three gyro axes and three accel axes as the FC filters them,
// stepped once per FDM sub-step (dt) and read at the decision tick. Per-scenario
// state: reset() at scenario start, and the first step() seeds every filter from
// its input (no start transient). dt has no default (Constitution VII).
class FcSensorFilterChain {
 public:
  explicit FcSensorFilterChain(double dtSec) {  // raw-ok: filter config in the double-based Pt1Filter/BiquadFilter above
    for (int a = 0; a < 3; ++a) {
      gyro_[a].configure(kGyroLpfHz, dtSec);
      accel_[a].configure(kAccelLpfHz, dtSec);
    }
  }
  void reset() {
    for (int a = 0; a < 3; ++a) { gyro_[a].reset(); accel_[a].reset(); }
    seeded_ = false;
  }
  // One FC-rate sample: body rates (rad/s) and body specific force (g).
  void step(const gp_vec3& gyroRadS, const gp_vec3& specificForceG) {
    for (int a = 0; a < 3; ++a) {
      gyroOut_[a] = static_cast<gp_scalar>(gyro_[a].apply(static_cast<double>(gyroRadS[a])));        // raw-ok: filter state is double (Pt1Filter); gp_scalar at the boundary
      accelOut_[a] = static_cast<gp_scalar>(accel_[a].apply(static_cast<double>(specificForceG[a])));  // raw-ok: filter state is double (BiquadFilter); gp_scalar at the boundary
    }
    seeded_ = true;
  }
  bool seeded() const { return seeded_; }
  gp_vec3 gyro() const { return gyroOut_; }
  gp_vec3 accel() const { return accelOut_; }
 private:
  Pt1Filter gyro_[3];
  BiquadFilter accel_[3];
  gp_vec3 gyroOut_ = gp_vec3::Zero();
  gp_vec3 accelOut_ = gp_vec3::Zero();
  bool seeded_ = false;
};

}}  // namespace autoc::eval
#endif
