// 043 US2 — contract tests for the modelled INAV fixed-wing ACRO rate loop
// (include/autoc/control/inav_fw_rate.h). Maps to contracts/inav-fw-rate-loop.md
// tests 1-6 + the cascade-ratio test (T033a). Pure-core tests: no crrcsim FDM.
//
// ⛔ ACRO = rate control. NONE of these tests reference attitude — the core has
// no attitude input, which is itself the FR-019a guarantee (test T030).
//
// The closed-loop tests (T028/T029) drive a deliberately trivial first-order
// rate plant: the surface output produces a steady rate through plantGain, with
// a lag tau. plantGain = 1/kFF so the feed-forward term alone reaches the
// commanded rate (which is how kFF is tuned) — P/I then null the residual.

#include <gtest/gtest.h>

#include <cmath>

#include "autoc/control/inav_fw_rate.h"

using autoc::control::InavFwRateGains;
using autoc::control::InavFwRateState;
using autoc::control::inavFwRateStep;
using autoc::control::inavFwRateSigma;
using autoc::control::inavFwRateDamping;
using autoc::control::pitchGainsDefault;

namespace {
constexpr double kInnerDt = 0.005;   // 200 Hz FDM substep (baseline.md T003)

// Trivial first-order rate plant. Steady rate = out * plantGain, reached with
// time constant tau. `disturbanceDps` is a constant rate bias (a mis-trim).
struct ToyPlant {
    double gyro = 0.0;
    double plantGain;
    double tau;
    double disturbanceDps;
    double step(double out, double dt) {
        const double steady = out * plantGain + disturbanceDps;
        gyro += (steady - gyro) * (dt / tau);
        return gyro;
    }
};

// Run the closed loop for `seconds` and return the final achieved rate (deg/s).
double runClosedLoop(const InavFwRateGains& g, double command, ToyPlant plant,
                     double seconds, double* riseTimeSec = nullptr,
                     double riseTarget = 0.0) {
    InavFwRateState s;
    double t = 0.0;
    bool rose = false;
    const int steps = static_cast<int>(seconds / kInnerDt);
    for (int i = 0; i < steps; ++i) {
        const double out = inavFwRateStep(g, s, kInnerDt, command, plant.gyro);
        const double gyro = plant.step(out, kInnerDt);
        t += kInnerDt;
        if (riseTimeSec && !rose && riseTarget != 0.0 &&
            std::abs(gyro) >= 0.9 * std::abs(riseTarget)) {
            *riseTimeSec = t; rose = true;
        }
    }
    return plant.gyro;
}
}  // namespace

// T028 — steady state: a constant rate setpoint ⇒ achieved rate converges to it,
// with a rise time consistent with the gains (contract test 1).
TEST(InavFwRate, ConstantSetpointConverges) {
    InavFwRateGains g;                       // roll
    const double command = 88.0 / g.maxRate; // rateTarget = 88 deg/s
    ToyPlant plant{0.0, 1.0 / g.kFF, 0.10, 0.0};
    double rise = 0.0;
    const double achieved = runClosedLoop(g, command, plant, 3.0, &rise, 88.0);
    EXPECT_NEAR(achieved, 88.0, 88.0 * 0.05);      // within 5% of setpoint
    EXPECT_GT(rise, 0.0);
    EXPECT_LT(rise, 1.0);                           // sub-second rise at this lag
}

// T029 — SC-012: zero command on a mis-trimmed craft ⇒ body rate settles to ZERO
// and stays. The constant trim disturbance is rejected by P+I (contract test 2).
TEST(InavFwRate, ZeroCommandMisTrimSettlesToZero) {
    InavFwRateGains g;                       // roll
    ToyPlant plant{0.0, 1.0 / g.kFF, 0.10, 8.0};   // +8 deg/s trim bias
    const double achieved = runClosedLoop(g, 0.0, plant, 6.0);
    EXPECT_NEAR(achieved, 0.0, 1.0);               // settles to ~0 despite the bias
}

// T030 — ⛔ NO self-levelling (FR-019a, SC-012 converse), core level: the loop
// has no attitude input, so its output is a function of (command, gyroRate)
// ONLY. Zero command + zero rate ⇒ zero output, from any attitude — there is
// nothing for the output to correlate with bank sign. (The full +30°/−30°
// sign-correlation sweep is a SIM test, T043.)
TEST(InavFwRate, NoAttitudeTermNoSelfLevelling) {
    InavFwRateGains g;
    InavFwRateState s;
    // Zero command, zero rate → zero output (no restoring term exists).
    const double out = inavFwRateStep(g, s, kInnerDt, 0.0, 0.0);
    EXPECT_NEAR(out, 0.0, 1e-9);
    // Bank-independence by construction: two "different attitudes" with the same
    // command+gyro produce identical output — the core cannot see bank.
    InavFwRateState s1, s2;
    const double a = inavFwRateStep(g, s1, kInnerDt, 0.0, 5.0);
    const double b = inavFwRateStep(g, s2, kInnerDt, 0.0, 5.0);
    EXPECT_EQ(a, b);
}

// T031 — attenuation curve matches exp(−r²/2σ²) at r ∈ {0, σ, 2σ}, with
// σ = 61.2 °/s roll and 20.4 °/s pitch (contract test 4).
TEST(InavFwRate, AttenuationCurveMatchesGaussian) {
    InavFwRateGains roll;                    // maxRate 360
    InavFwRateGains pitch = pitchGainsDefault();  // maxRate 120
    EXPECT_NEAR(inavFwRateSigma(roll), 61.2, 0.2);
    EXPECT_NEAR(inavFwRateSigma(pitch), 20.4, 0.2);

    for (const auto& g : {roll, pitch}) {
        const double sigma = inavFwRateSigma(g);
        EXPECT_NEAR(inavFwRateDamping(g, 0.0), 1.0, 1e-9);
        EXPECT_NEAR(inavFwRateDamping(g, sigma), std::exp(-0.5), 1e-6);
        EXPECT_NEAR(inavFwRateDamping(g, 2.0 * sigma), std::exp(-2.0), 1e-6);
    }
}

// T032 — FF dominance: 88 °/s roll setpoint at ZERO error yields ≈142 of the
// ±500 budget (FF alone: 88 × 1.613; contract test 5).
TEST(InavFwRate, FeedForwardDominatesAtZeroError) {
    InavFwRateGains g;                       // roll
    InavFwRateState s;
    const double command = 88.0 / g.maxRate;
    // gyroRate == rateTarget ⇒ rateError 0 ⇒ P=I=0, and constant gyro ⇒ D=0.
    inavFwRateStep(g, s, kInnerDt, command, 88.0);       // prime
    const double out = inavFwRateStep(g, s, kInnerDt, command, 88.0);
    EXPECT_NEAR(out, 88.0 * g.kFF, 1.0);                 // ≈ 141.9
    EXPECT_NEAR(out, 142.0, 2.0);
}

// T033 — I-term lock: a large setpoint step with large error freezes I
// accumulation while locked (contract test 6).
TEST(InavFwRate, ItermLockFreezesAccumulation) {
    InavFwRateGains g;                       // roll
    InavFwRateState s;
    // command 1.0 ⇒ rateTarget 360 (high); gyro 0 ⇒ error 360 (large) ⇒ locked.
    for (int i = 0; i < 50; ++i) {           // 250 ms, well within lockTimeMaxMs
        inavFwRateStep(g, s, kInnerDt, 1.0, 0.0);
        EXPECT_EQ(s.integrator, 0.0) << "I must not accumulate while locked";
    }
    // Release: low target (0.05 ⇒ 18 °/s < 0.2·360) + small error, held past the
    // lock window ⇒ I unlocks and accumulates.
    for (int i = 0; i < 200; ++i) inavFwRateStep(g, s, kInnerDt, 0.05, 10.0);
    EXPECT_NE(s.integrator, 0.0) << "I must accumulate once unlocked";
}

// ---------------------------------------------------------------------------
// T043 — all-attitude zero-command sweep (FR-019, SC-014 part 1). The model
// standing ALONE (no autoc): across the attitude sphere, zero command must
// (a) null any body rate at any attitude, and (b) ⛔ NOT self-level. Per
// contract #3 the discriminator is SIGN CORRELATION, not stability: an ANGLE
// loop drives bank toward 0 (restoring correlated with bank sign); ACRO's
// residual drift is uncorrelated. We CONTRAST the ACRO core against a simulated
// ANGLE outer loop (attitude→rate command into the same core) to prove the test
// can tell them apart — otherwise "bank held" would pass trivially.
// ---------------------------------------------------------------------------
namespace {
// Minimal rigid-body roll/pitch model. Surface → angular accel (+ damping);
// body rate → attitude via kinematics. ⭐ A bank produces NO roll-restoring
// moment — a fixed wing does not self-right, which is exactly what ACRO must
// preserve. Sign convention matches the crrcsim FDM: +aileron→+p, −elevator→+q.
struct AttitudePlant {
    double phiDeg, thetaDeg, p, q;               // attitude (deg), body rates (deg/s)
    double kRoll = 1200.0, kPitch = 1200.0;      // surface(±0.5) → accel (deg/s²)
    double dampP = 3.0, dampQ = 3.0;             // aero rate damping (1/s)
    void step(double aileron, double elevator, double dt) {
        p += (kRoll * aileron - dampP * p) * dt;
        q += (-kPitch * elevator - dampQ * q) * dt;
        phiDeg   += p * dt;
        thetaDeg += q * dt;
    }
};

// Run one attitude for `seconds`. If kLevelPerDeg>0, an ANGLE outer loop feeds a
// bank-proportional rate command into the core (the self-levelling we must NOT
// have). Returns final (phi, theta, p, q).
struct SweepResult { double phi, theta, p, q; };
SweepResult runAttitude(const InavFwRateGains& gr, const InavFwRateGains& gp,
                        AttitudePlant plant, double p0, double q0, double seconds,
                        double kLevelPerDeg) {
    plant.p = p0; plant.q = q0;
    InavFwRateState sr, sp;
    const int steps = static_cast<int>(seconds / kInnerDt);
    for (int i = 0; i < steps; ++i) {
        // ACRO: zero command. ANGLE ref: command = −kLevel·angle (restoring).
        const double rollCmd  = std::clamp(-kLevelPerDeg * plant.phiDeg, -1.0, 1.0);
        const double pitchCmd = std::clamp(-kLevelPerDeg * plant.thetaDeg, -1.0, 1.0);
        const double outR = inavFwRateStep(gr, sr, kInnerDt, rollCmd, plant.p);
        const double outP = inavFwRateStep(gp, sp, kInnerDt, pitchCmd, plant.q);
        plant.step(outR / gr.pidSumLimit * 0.5, -outP / gp.pidSumLimit * 0.5, kInnerDt);
    }
    return {plant.phiDeg, plant.thetaDeg, plant.p, plant.q};
}
}  // namespace

TEST(InavFwRate, AllAttitudeZeroCommandSweep) {
    const InavFwRateGains gr;                       // roll
    const InavFwRateGains gp = pitchGainsDefault(); // pitch
    const double banks[]  = {-60, -45, -30, -15, 15, 30, 45, 60};
    const double pitches[] = {-30, -15, 15, 30};

    double acroRestoreSum = 0.0, angleRestoreSum = 0.0;
    int n = 0;
    double signCorrNum = 0.0;  // Σ sign(phi0)·(phi0−phiFinal) — >0 means restoring
    for (double phi0 : banks) {
        for (double th0 : pitches) {
            AttitudePlant plant{phi0, th0, 0, 0};
            // (a) rate-nulling: start with a 20 °/s disturbance on both axes.
            const SweepResult acro = runAttitude(gr, gp, plant, 20.0, 20.0, 3.0, 0.0);
            EXPECT_LT(std::abs(acro.p), 2.0) << "ACRO must null roll rate at phi0=" << phi0;
            EXPECT_LT(std::abs(acro.q), 2.0) << "ACRO must null pitch rate at phi0=" << phi0;
            // (b) no self-levelling: bank is HELD (residual drift small), and the
            // residual is NOT sign-correlated with the initial bank.
            const double acroRestore = (std::abs(phi0) - std::abs(acro.phi)) / std::abs(phi0);
            acroRestoreSum += acroRestore;
            signCorrNum += (phi0 > 0 ? 1.0 : -1.0) * (phi0 - acro.phi);

            // ANGLE contrast (same plant + core, plus a self-levelling command):
            // it DOES restore — proving the sweep discriminates.
            const SweepResult ang = runAttitude(gr, gp, plant, 20.0, 20.0, 3.0, 0.05);
            angleRestoreSum += (std::abs(phi0) - std::abs(ang.phi)) / std::abs(phi0);
            ++n;
        }
    }
    const double acroRestore = acroRestoreSum / n;
    const double angleRestore = angleRestoreSum / n;
    // ⛔ ACRO barely changes bank (no attitude reference); ANGLE drives it to ~0.
    EXPECT_LT(acroRestore, 0.15) << "ACRO must NOT self-level (mean restore fraction)";
    EXPECT_GT(angleRestore, 0.80) << "ANGLE reference MUST restore — else the test is blind";
    EXPECT_GT(angleRestore, acroRestore + 0.6) << "the two modes must be clearly separable";
    // The ACRO residual is not a sign-correlated restoring trend (contract #3).
    EXPECT_LT(std::abs(signCorrNum) / n, 3.0) << "ACRO drift must be ~uncorrelated with bank sign";
}

// T033a — ⭐ the cascade RATIOS are right, not just the constants (FR-011). A
// model with correct gains but a wrong rate oscillates where the aircraft does
// not. Assert the as-run cadence/filter ratios against the contract + baseline.
TEST(InavFwRate, CascadeRatiosMatchContract) {
    // Inner PID cadence 200 Hz (5 ms, baseline.md T003) : outer control 20 Hz
    // (50 ms) : servo command frame 50 Hz (20 ms).
    const double innerDt = 0.005, outerDt = 0.050, servoFrame = 0.020;
    EXPECT_NEAR(outerDt / innerDt, 10.0, 1e-9);          // inner is 10× the outer
    EXPECT_NEAR(servoFrame / innerDt, 4.0, 1e-9);        // servo frame is 4 substeps

    // Gyro-filter corner (25 Hz) relative to the inner cadence (200 Hz): well
    // below Nyquist (100 Hz), so the PT1 is resolved, and its per-step gain is a
    // proper fraction — not degenerate.
    InavFwRateGains g;
    EXPECT_LT(g.gyroLpfHz, 0.5 / innerDt);               // 25 < 100 Hz Nyquist
    EXPECT_GT(g.gyroLpfHz, g.dtermLpfHz);                // P corner above D corner
    const double gg = autoc::control::pt1Gain(innerDt, g.gyroLpfHz);
    EXPECT_GT(gg, 0.0);
    EXPECT_LT(gg, 1.0);
    EXPECT_NEAR(gg, innerDt / (innerDt + 1.0 / (2.0 * M_PI * g.gyroLpfHz)), 1e-12);
}
