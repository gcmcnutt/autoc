#ifndef AUTOC_EVAL_CONE_CONSTANTS_H
#define AUTOC_EVAL_CONE_CONSTANTS_H

// 041 P5-3 — the tracking-cone constants, as a value the firmware can carry.
//
// WHY THIS EXISTS. SCORE_GRAD_* is an exact closed form over the virtual
// target's geometry, so the xiao can compute it in flight — but only if it
// knows the cone the objective was shaped with. On the desktop those six
// numbers come from the .ini through ConfigManager; the firmware has no
// ConfigManager, so nn2cpp bakes them beside the arena template (the same
// mechanism, for the same reason, as 039 D5).
//
// ⛔ NO DEFAULT MEMBER INITIALIZERS, DELIBERATELY. The arena learned this the
// hard way (see BakedArena in tools/nn2cpp.cc): a struct that carries its own
// copy of the numbers becomes a THIRD source of truth alongside the .ini keys
// and the Config defaults, and it stays plausible-looking long after the real
// values move. Every construction site must state where its values came from —
// codegen derives them from Config, tests state them explicitly.
//
// ⚠️ These must match the run that produced the genome. A policy evolved
// against one cone and flown against another sees a gradient pointing somewhere
// its training never rewarded — silent, and only visible as bad tracking.

namespace autoc {
namespace eval {

struct ConeConstants {
    double distScaleBehind;      // m, distance half-decay when BEHIND the rabbit
    double distScaleAhead;       // m, ditto ahead (small => sharp ahead penalty)
    double coneAngleDeg;         // deg, angular half-decay from "directly behind"
    double streakThreshold;      // min stepPoints to hold the streak
    double streakRampSec;        // seconds to reach the max multiplier
    double streakMultiplierMax;  // max streak multiplier
};

}  // namespace eval
}  // namespace autoc

#endif  // AUTOC_EVAL_CONE_CONSTANTS_H
