// 043 §6/§9 — the NN's own sensor path, as the FLIGHT ARTICLE actually delivers it.
//
// ⛔ WHY THIS EXISTS. Measured 2026-09-05 (docs/inav-acro-path-audit.md §6): the
// real aircraft filters the gyro and accelerometer BEFORE they reach the policy,
// and the sim did not model any of it — it handed the NN raw FDM state.
//
//   channel | real (what the NN gets)                  | sim (before this)
//   --------|------------------------------------------|-------------------
//   gyro    | gyro.gyroADCf: gyro_main_lpf_hz = 25 PT1 | raw getOmegaBody()
//   accel   | acc.accADCf:   acc_lpf_hz = 15 BIQUAD    | raw specific force
//
// Un-modelled group delay on the policy's OWN INPUTS: ~6.4 ms gyro, ~21.2 ms
// accel. 21 ms is 42% of a 20 Hz tick, on the channel the whole 041 P5-1 accel
// work depends on. The policy trained on sensors that respond instantly and flew
// sensors that lag.
//
// ⓘ NOT modelled, deliberately:
//   - the dynamic gyro notch (2D, Q 250, >= 30 Hz): above the band of interest
//     and its phase contribution below 10 Hz is negligible.
//   - the gyro Kalman: it was TURNED OFF on the article 2026-09-05 precisely
//     because an input-adaptive filter with state-dependent group delay does not
//     belong in front of a trained policy. Modelling it would re-introduce what
//     the config change removed.
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

}}  // namespace autoc::eval
#endif
