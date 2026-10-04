// 043 t4 — per-scenario WIND ENVELOPE variation (speed, turbulence intensity,
// gust length scale, thermal strength/count).
//
// Why this exists: every t3 training scenario flew a single fixed wind speed
// (autoc_config.xml velocity="12" ft/s = 3.66 m/s) because crrcsim reads one
// global T_Wind value. Direction, gust noise and thermal placement were already
// per-scenario; speed — and therefore Dryden gust sigma, which is 0.1·V — was
// not. The 2026-09-13 flight at 4–7 m/s showed a one-sided ~25 m downwind
// tracking bias (flight-analysis-20260913.md §2, ADDENDUM A2/A3).
//
// Design rules (tasks.md T087 / T090):
//  * RPC-only. `WindVariationConfig` travels in WorkerInit, never in
//    ScenarioMetadata, so no dmp is orphaned (040 US6 cameraVariations precedent).
//  * Draw-and-discard. `drawWindVariation` ALWAYS consumes the same four draws
//    from the scenario's wind-class PRNG, AFTER the existing drawnWindSeed, so
//    toggling any key leaves the thermal/gust seed stream untouched.
//  * OFF by default. Every range defaults to "no change"; with all keys at
//    their defaults `realizeWindVariation` returns the base values exactly,
//    which is what keeps t3's tier0 bitwise-reproducible under the new binary.
//  * Curriculum-ramped. Each realized value is base + variationScale·(target −
//    base), the same ramp every other variation class uses.
//
// The pure functions here are shared by the crrcsim worker (which applies the
// realization) and the unit tests (which pin the contract).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "autoc/util/scenario_prng.h"

namespace autoc {
namespace eval {

struct WindVariationConfig {
  // Wind speed, metres per second. min < 0 ⇒ OFF (keep the sim's base speed).
  double windSpeedMinMps = -1.0;
  double windSpeedMaxMps = -1.0;
  // Dryden turbulence intensity multiplier on T_Wind::turbulence. 1/1 ⇒ OFF.
  double turbIntensityMin = 1.0;
  double turbIntensityMax = 1.0;
  // Multiplier on the Dryden length scales L_u/L_v/L_w. 1/1 ⇒ OFF. Smaller =
  // gusts that change faster within a scenario (0.25 ⇒ ~0.04 Hz corner at 13 m/s).
  double gustLengthScaleMin = 1.0;
  double gustLengthScaleMax = 1.0;
  // Multiplier on arena-thermal strength_mean/strength_sigma. 1/1 ⇒ OFF.
  double thermalStrengthMin = 1.0;
  double thermalStrengthMax = 1.0;
  // Override for arena-thermal count_max (0 ⇒ keep the XML value).
  int thermalCountMax = 0;

  bool speedEnabled() const { return windSpeedMinMps >= 0.0 && windSpeedMaxMps >= windSpeedMinMps; }
  bool turbEnabled() const { return turbIntensityMin != 1.0 || turbIntensityMax != 1.0; }
  bool gustScaleEnabled() const { return gustLengthScaleMin != 1.0 || gustLengthScaleMax != 1.0; }
  bool thermalStrengthEnabled() const { return thermalStrengthMin != 1.0 || thermalStrengthMax != 1.0; }
  bool thermalCountEnabled() const { return thermalCountMax > 0; }
  bool anyEnabled() const {
    return speedEnabled() || turbEnabled() || gustScaleEnabled() ||
           thermalStrengthEnabled() || thermalCountEnabled();
  }

  template <class Archive>
  void serialize(Archive& ar) {
    ar(windSpeedMinMps, windSpeedMaxMps,
       turbIntensityMin, turbIntensityMax,
       gustLengthScaleMin, gustLengthScaleMax,
       thermalStrengthMin, thermalStrengthMax,
       thermalCountMax);
  }
};

// The four uniform [0,1) draws, in a fixed order. Always drawn.
struct WindDraws {
  double uSpeed = 0.0;
  double uTurb = 0.0;
  double uGust = 0.0;
  double uThermal = 0.0;
};

inline WindDraws drawWindVariation(autoc::util::ClassPRNG& prng) {
  WindDraws d;
  d.uSpeed = prng.nextDouble();
  d.uTurb = prng.nextDouble();
  d.uGust = prng.nextDouble();
  d.uThermal = prng.nextDouble();
  return d;
}

struct WindRealization {
  double windSpeedMps = 0.0;        // absolute, to be written back in ft/s by the caller
  double turbIntensity = 1.0;       // absolute T_Wind::turbulence value
  double gustLengthScale = 1.0;     // multiplier applied in calculate_gust
  double thermalStrengthScale = 1.0;// multiplier applied at thermal spawn
  int thermalCountMaxTarget = 0;    // 0 ⇒ keep XML; ramp applied where the XML value is known
  double rampScale = 1.0;           // the variationScale used, for the count ramp
};

inline double lerp(double a, double b, double u) { return a + (b - a) * u; }

// base* are the sim's unmodified values (captured once at worker start).
// variationScale is the per-eval curriculum ramp in [0,1]. enabled is the
// master EnableWindVariations gate: when false everything returns base.
inline WindRealization realizeWindVariation(const WindVariationConfig& cfg,
                                            const WindDraws& d,
                                            double baseWindSpeedMps,
                                            double baseTurbIntensity,
                                            double variationScale,
                                            bool enabled) {
  WindRealization r;
  r.windSpeedMps = baseWindSpeedMps;
  r.turbIntensity = baseTurbIntensity;
  const double vs = std::clamp(variationScale, 0.0, 1.0);
  r.rampScale = vs;
  if (!enabled) return r;

  if (cfg.speedEnabled()) {
    const double target = lerp(cfg.windSpeedMinMps, cfg.windSpeedMaxMps, d.uSpeed);
    r.windSpeedMps = std::max(0.0, lerp(baseWindSpeedMps, target, vs));
  }
  if (cfg.turbEnabled()) {
    const double target = baseTurbIntensity * lerp(cfg.turbIntensityMin, cfg.turbIntensityMax, d.uTurb);
    r.turbIntensity = std::max(0.0, lerp(baseTurbIntensity, target, vs));
  }
  if (cfg.gustScaleEnabled()) {
    const double target = lerp(cfg.gustLengthScaleMin, cfg.gustLengthScaleMax, d.uGust);
    r.gustLengthScale = std::max(0.05, lerp(1.0, target, vs));
  }
  if (cfg.thermalStrengthEnabled()) {
    const double target = lerp(cfg.thermalStrengthMin, cfg.thermalStrengthMax, d.uThermal);
    r.thermalStrengthScale = std::max(0.0, lerp(1.0, target, vs));
  }
  if (cfg.thermalCountEnabled()) {
    r.thermalCountMaxTarget = cfg.thermalCountMax;
  }
  return r;
}

// Ramp the thermal count where the XML base is known (crrcsim side).
inline int rampedThermalCountMax(int xmlCountMax, int target, double rampScale) {
  if (target <= 0) return xmlCountMax;
  const double v = lerp(static_cast<double>(xmlCountMax), static_cast<double>(target),
                        std::clamp(rampScale, 0.0, 1.0));
  return static_cast<int>(std::lround(v));
}

}  // namespace eval
}  // namespace autoc
