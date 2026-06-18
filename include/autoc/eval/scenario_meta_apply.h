#pragma once

// 030 V1.5 priming (2026-05-08) — pure helper that applies the
// variation-ramp scale factor to a ScenarioMetadata's entry offsets in
// place. Extracted from autoc.cc::populateVariationOffsets so workers can
// run the same scaling locally on cached base metas (ScenarioMetadata
// pre-populated at full scale, sent once via WorkerInit).
//
// Callers MUST already have meta.entry*Offset / meta.entrySpeedFactor /
// meta.windDirectionOffset / meta.entry{North,East,Alt}Offset populated
// at FULL SCALE (i.e. as drawn from the joint-PRNG variation table at
// startup with scale=1.0). This function does the per-eval scaling +
// position clamp; identical formula to the legacy populateVariationOffsets
// so byte-equivalent training fitness is preserved.

#include <cmath>

#include "autoc/autoc.h"  // ENTRY_SAFE_RADIUS, ENTRY_SAFE_ALT_MIN/MAX
#include "autoc/rpc/scenario_metadata.h"
#include "autoc/types.h"  // gp_scalar

namespace autoc::eval {

inline void applyVariationScale(ScenarioMetadata& meta, gp_scalar scale) {
    // Angular offsets: scale toward 0.
    meta.entryHeadingOffset *= scale;
    meta.entryRollOffset *= scale;
    meta.entryPitchOffset *= scale;
    meta.windDirectionOffset *= scale;

    // Speed factor: interpolate from 1.0 (nominal) toward the base value.
    // At scale=0 → 1.0; at scale=1 → base value.
    meta.entrySpeedFactor = 1.0 + scale * (meta.entrySpeedFactor - 1.0);

    // Position offsets: scale and clamp to safe arena bounds.
    double northScaled = meta.entryNorthOffset * scale;
    double eastScaled = meta.entryEastOffset * scale;
    double altScaled = meta.entryAltOffset * scale;

    // Horizontal radius clamp.
    double horizDist = std::sqrt(northScaled * northScaled + eastScaled * eastScaled);
    if (horizDist > ENTRY_SAFE_RADIUS) {
        double clampFactor = ENTRY_SAFE_RADIUS / horizDist;
        northScaled *= clampFactor;
        eastScaled *= clampFactor;
    }
    // Altitude clamp (NED: Down is +, ENTRY_SAFE_ALT_MAX < ENTRY_SAFE_ALT_MIN).
    if (altScaled < ENTRY_SAFE_ALT_MAX) altScaled = ENTRY_SAFE_ALT_MAX;
    if (altScaled > ENTRY_SAFE_ALT_MIN) altScaled = ENTRY_SAFE_ALT_MIN;

    meta.entryNorthOffset = northScaled;
    meta.entryEastOffset = eastScaled;
    meta.entryAltOffset = altScaled;

    // 037 (operator 2026-06-10): craft variations are NOT ramped -- only the
    // ENV variations above (entry pose, wind, position) ramp as a difficulty
    // curriculum. Rationale: the ramp exists to keep a fresh random population
    // from being killed by UNFLYABLE difficulty before it learns the core task,
    // and that risk is environmental (extreme entry poses, big gusts), not the
    // airframe. A varied craft -- CG/drag/trim/thrust/control-effectiveness plus
    // the 037 actuator dynamics (servoSlew/thrustTau) -- is still
    // perfectly flyable; it is DIVERSITY, not difficulty. Ramping it would only
    // delay fleet-robustness training to the hard end of the env curriculum and
    // (for the dynamics) means physically-odd interpolation of a time constant.
    // So craft is present at its FULL drawn magnitude from gen 0: the base meta
    // already holds the full draws (generateCraftFromClassPRNG), so we leave the
    // craft fields UNTOUCHED here -- `scale` affects ONLY the env fields above.
    //
    // NOTE: future CAMERA variations (tracker mode) are the same category --
    // sensor/airframe diversity, not difficulty -- and should likewise NOT be
    // ramped when they land.
}

}  // namespace autoc::eval
