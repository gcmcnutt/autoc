#pragma once
// 043 US2 — the modelled INAV fixed-wing ACRO rate loop, as a PURE,
// dependency-free per-axis core so it is unit-testable (tests/inav_fw_rate_tests.cc)
// independently of crrcsim's FDM. The crrcsim adapter
// (crrcsim/src/mod_cntrl/cntrl_inavfwrate/) wires fdm->getPQR() and the stick
// command into step() and maps the ±pidSumLimit output onto the surface.
//
// ⛔ THIS IS ACRO — RATE control. There is NO attitude term anywhere. An
// implementation that reads attitude has built ANGLE (spec FR-019a). See
// contracts/inav-fw-rate-loop.md (source of truth ~/inav src/main/flight/pid.c)
// and research.md R1: the loop is FEED-FORWARD DOMINANT (kFF >> kP) with P and D
// attenuated by a GAUSSIAN IN THE SETPOINT, plus an I-term lock.
//
// Units: rates in deg/s (matches the XML maxRate); the adapter converts the
// FDM's rad/s gyro to deg/s at the boundary. dt in seconds.
//
// Type-domain (Constitution VI): this is FDM-native physics math that runs
// inside crrcsim's double-precision FDM substep, not GP-determinism state, so
// every scalar here is `double` and annotated `// raw-ok` as a block.
//
// FILTERS MODELLED (T038): only `gyro_main_lpf_hz` (25 Hz PT1) sits inside the
// loop — it filters the P path here. `dterm_lpf_hz` (10 Hz PT2) filters D.
// ⛔ `acc_lpf_hz` is EXCLUDED: it is the accelerometer OBSERVATION path and
// contributes NO phase to the ACRO rate loop (FR-013, corrected 2026-08-25).
//
// DELIBERATELY NOT MODELLED (T039), each because it is off/identity on the
// flying config (contracts/inav-fw-rate-loop.md · research.md addendum C):
//   • TPA                       — tpa_rate=0 ⇒ tpaFactor 1.0 (a mechanism that is off).
//   • D-boost                   — d_boost_min=max=1.0 ⇒ identity.
//   • setpoint accel limit      — rate_accel_limit_roll_pitch=0 ⇒ off (FR-019b).
//                                 yaw's 10000 is irrelevant: no rudder.
//   • anti-alias LPF            — 1.15° at 5 Hz, negligible in-band.
//   • dynamic gyro notch        — effective Q=2.5, ≥30 Hz floor ⇒ <4° at 5 Hz;
//                                 measured absent per T006 (research.md addendum D).
//   • pidLevel / self-levelling — ⛔ ANGLE-only; including it is the FR-019a defect.
//   • yaw output                — computed by INAV, reaches no surface (FR-018).

#include <algorithm>  // std::clamp, std::min, std::max
#include <cmath>      // std::exp, M_PI

namespace autoc::control {

// INAV's gyro saturation guard (GYRO_SATURATION_LIMIT, deg/s). maxRate
// (360/120) is far below this, so it never bites for our commands — modelled
// for fidelity to the source, not because it is reachable.
inline constexpr double kGyroSaturationLimitDps = 1998.0;  // raw-ok: FDM-native

// Per-axis constants, loaded from the model XML (FR-014). Defaults are the
// flying-config values in contracts/inav-fw-rate-loop.md for the ROLL axis; the
// adapter overwrites every field from XML (Constitution VII — no silent
// fallback: the adapter must supply all of them).
struct InavFwRateGains {                       // raw-ok: FDM-native config block
    double kP = 0.484;
    double kI = 0.750;
    double kD = 0.003675;
    double kFF = 1.613;
    double maxRate = 360.0;                    // deg/s
    double gyroLpfHz = 25.0;                    // P path PT1 (gyro_main_lpf_hz) — T038
    double dtermLpfHz = 10.0;                   // D path PT2 (dterm_lpf_hz)
    double itermLockRateThresholdPct = 40.0;    // sets the Gaussian width σ
    double engageThresholdPct = 10.0;           // I-lock engage on |rateError|
    double lockTimeMaxMs = 500.0;               // I-lock memory window
    double itermLimitPct = 33.0;                // I clamp as % of pidSumLimit
    double pidSumLimit = 500.0;                 // output clamp, fixed-wing
};

// Per-axis mutable state, reset at scenario init (determinism). ⛔ No attitude
// state of ANY kind (FR-019a, data-model.md §4).
struct InavFwRateState {                       // raw-ok: FDM-native state block
    double integrator = 0.0;
    double prevGyroRate = 0.0;
    double ptermPt1 = 0.0;                       // gyro_main_lpf PT1 state (P path)
    double dtermPt2a = 0.0;                      // dterm PT2 = two cascaded PT1
    double dtermPt2b = 0.0;
    double msSinceTargetHigh = 1e9;             // ms since |rateTarget| last > 0.2·maxRate
    bool   primed = false;                       // first call seeds prevGyroRate (no D spike)

    void reset() { *this = InavFwRateState{}; }
};

// One-pole (PT1) gain for cutoff fc at timestep dt: g = dt/(dt+RC), RC=1/(2πfc).
inline double pt1Gain(double dt, double fcHz) {          // raw-ok: FDM-native
    if (fcHz <= 0.0) return 1.0;  // 0 = disabled → pass-through
    const double rc = 1.0 / (2.0 * M_PI * fcHz);
    return dt / (dt + rc);
}

// Gaussian setpoint-attenuation width. The well's FWHM is
// itermLockRateThreshold% of maxRate; ÷2.35482 converts FWHM→σ. deg/s.
// contracts/inav-fw-rate-loop.md: σ = 61.2 (roll, maxRate 360) / 20.4 (pitch, 120).
inline double inavFwRateSigma(const InavFwRateGains& g) {   // raw-ok: FDM-native
    return (g.maxRate * g.itermLockRateThresholdPct / 100.0) / 2.35482;
}

// Setpoint-attenuation multiplier aP=aD at a given commanded rate (deg/s).
// exp(-rateTarget²/2σ²): 1.0 at zero rate, falling as |rateTarget| rises — loop
// gain is HIGHEST at zero commanded rate (research.md R1).
inline double inavFwRateDamping(const InavFwRateGains& g, double rateTarget) {
    const double sigma = inavFwRateSigma(g);              // raw-ok: FDM-native
    if (sigma <= 0.0) return 1.0;
    return std::exp(-(rateTarget * rateTarget) / (2.0 * sigma * sigma));
}

// Advance one substep. `command` ∈ [-1,+1] (the stick/NN output for this axis);
// `gyroRateDps` is the measured body rate on this axis in deg/s. Returns the
// axis PID output in [-pidSumLimit, +pidSumLimit] — what INAV feeds the mixer,
// identical to the MANUAL rcCommand path (contracts/action-space.md).
inline double inavFwRateStep(const InavFwRateGains& g, InavFwRateState& s,
                             double dt, double command, double gyroRateDps) {
    if (!s.primed) { s.prevGyroRate = gyroRateDps; s.primed = true; }

    double rateTarget = command * g.maxRate;                              // deg/s
    rateTarget = std::clamp(rateTarget, -kGyroSaturationLimitDps, kGyroSaturationLimitDps);
    const double rateError = rateTarget - gyroRateDps;

    // Gaussian setpoint attenuation: aP=aD=damping (highest at zero rate).
    const double damping = inavFwRateDamping(g, rateTarget);
    const double aP = damping;
    const double aD = damping;

    // I-term lock: engage while |rateError| is large AND |rateTarget| exceeded
    // 0.2·maxRate within the last lockTimeMaxMs (a rapid-maneuver anti-windup).
    if (std::abs(rateTarget) > 0.2 * g.maxRate) s.msSinceTargetHigh = 0.0;
    else s.msSinceTargetHigh += dt * 1000.0;
    const bool iLockActive =
        (std::abs(rateError) > g.maxRate * g.engageThresholdPct / 100.0) &&
        (s.msSinceTargetHigh <= g.lockTimeMaxMs);
    const double aI = std::min(damping, iLockActive ? 0.0 : 1.0);

    // P — filtered by the 25 Hz gyro main LPF (PT1), then attenuated (T038).
    const double pRaw = rateError * g.kP;
    const double pg = pt1Gain(dt, g.gyroLpfHz);
    s.ptermPt1 += (pRaw - s.ptermPt1) * pg;
    const double P = s.ptermPt1 * aP;

    // D — on MEASUREMENT (prevGyro − gyro), PT2-filtered at dterm_lpf_hz, ×kD/dt.
    const double dRaw = (s.prevGyroRate - gyroRateDps) * g.kD / dt;
    const double dg = pt1Gain(dt, g.dtermLpfHz);
    s.dtermPt2a += (dRaw - s.dtermPt2a) * dg;      // PT2 = two cascaded PT1
    s.dtermPt2b += (s.dtermPt2a - s.dtermPt2b) * dg;
    const double D = s.dtermPt2b * aD;

    // FF — unfiltered, the dominant term (kFF >> kP).
    const double FF = rateTarget * g.kFF;

    // I — accumulate (attenuated + lockable), clamped to ±pidSumLimit·itermLimit%.
    const double iLimit = g.pidSumLimit * g.itermLimitPct / 100.0;
    s.integrator = std::clamp(s.integrator + rateError * g.kI * dt * aI, -iLimit, iLimit);
    const double I = s.integrator;

    s.prevGyroRate = gyroRateDps;
    return std::clamp(P + FF + I + D, -g.pidSumLimit, g.pidSumLimit);
}

// Pitch-axis gains per contracts/inav-fw-rate-loop.md (roll defaults live in the
// struct). A convenience for tests + the adapter's default before XML load.
inline InavFwRateGains pitchGainsDefault() {             // raw-ok: FDM-native
    InavFwRateGains g;
    g.kP = 0.484; g.kI = 1.250; g.kD = 0.002625; g.kFF = 2.258;
    g.maxRate = 120.0;
    return g;
}

}  // namespace autoc::control
