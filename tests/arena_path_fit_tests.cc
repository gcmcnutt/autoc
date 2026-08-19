// 041 P2-3 — DO THE TRAINING TARGETS FIT IN THE ARENA, WITH ROOM TO SPARE?
//
// Operator 2026-08-18: *"Also make sure the pathgen paths range fit in the
// arena with room to spare. I think we are ok but a max extent cross check is a
// good idea."*
//
// ⭐ WE WERE NOT OK, and this file is why it is now checked rather than
// believed. At the first 041 geometry the `aeroStandard` SpiralClimb rabbit
// topped out **3 cm** under the ceiling — a coincidence, not a clearance, and
// worse than it sounds: the CHASE has to fly above the rabbit to track it, so
// the aircraft would have been egressing on the most demanding path in the set.
// SpiralClimb was shrunk 50 → 35 m as a result (operator 2026-08-18), and the
// random path's vertical draw was split out as SIM_PATH_HEIGHT_BOUNDS — because
// once SpiralClimb stopped being the binding path, the SEEDED one became it, at
// 3 m of margin. Neither of those was visible without measuring.
//
// ⚠️ WHY A TEST AND NOT A MEASUREMENT. The extents are a property of the
// GENERATOR, not of a number someone recorded once. Cubic interpolation means
// the realized path does not stop where its control points do (SpiralClimb's
// literal is a −50 m control input; the realized apex is −49.97 m), and
// `SeededRandomB` re-rolls its geometry from `gPathSeed` every generation. A
// transcribed constant would go stale the first time a generator, a seed range,
// or an arena bound moved — silently, and only visible as crashes in a bake.
// So this links the real `generateSmoothPaths` and measures.
//
// ⚠️ WHAT THIS DOES **NOT** CHECK: that the CHASE fits — and it does not,
// knowingly. Measured on 131,127 t1 ticks the chase reached entry +66.2 m,
// against a half-band of K = 48. Roughly 1% of t1-like ticks would now egress at
// the ceiling.
//
// That is accepted rather than designed around, for two reasons: the chase's
// extent is a property of the POLICY (not of the generator, so it cannot be
// asserted here), and a binding ceiling is PRESSURE AGAINST THE ZOOM that the
// new Ps energy axis charges for anyway. ⛔ But it is a real behavioural change
// and it is the first thing to look at in the P3-4 smoke: watch
// `ArenaEgressKind::CEILING` counts. If the run bleeds scenarios there, the
// half-band is the knob, not the objective.

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/eval/pathgen.h"

namespace {

using autoc::eval::FlightArena;

struct Extents {
    double radius_max = 0.0;
    double agl_min = std::numeric_limits<double>::max();
    double agl_max = std::numeric_limits<double>::lowest();
    unsigned worst_seed = 0;
};

// Sweep the generator over a range of seeds and return the worst-case extents.
//
// The seed sweep is not decoration: `gPathSeed` changes every generation, so a
// single-seed check would only ever prove that ONE generation's geometry fits.
Extents measure(const std::string& method, int numPaths, unsigned seedCount) {
    const double entry_agl = -static_cast<double>(SIM_INITIAL_ALTITUDE);
    Extents e;
    for (unsigned seed = 1; seed <= seedCount; ++seed) {
        // ⚠️ radius and HEIGHT are separate bounds (041 P2-3) — passing
        // SIM_PATH_BOUNDS for both, as this once did, generates a taller rabbit
        // than production and would have this test measuring a path set no run
        // actually flies. Must mirror src/autoc.cc's call exactly.
        const auto paths = generateSmoothPaths(method, numPaths, SIM_PATH_BOUNDS,
                                               SIM_PATH_HEIGHT_BOUNDS, seed);
        for (const auto& path : paths) {
            for (const auto& seg : path) {
                const double x = seg.start[0], y = seg.start[1], z = seg.start[2];
                const double r = std::sqrt(x * x + y * y);
                const double agl = entry_agl - z;   // virtual z is NED down-positive
                if (r > e.radius_max) { e.radius_max = r; e.worst_seed = seed; }
                e.agl_min = std::min(e.agl_min, agl);
                e.agl_max = std::max(e.agl_max, agl);
            }
        }
    }
    return e;
}

// "Room to spare", made a number rather than a feeling: **one median turn
// radius**, 11.9 m measured on path 5, rounded to 12. That is the clearance the
// chase needs to notice a bound and turn away from it — as opposed to merely
// not touching it while tracking perfectly, which is what a 3 cm margin bought.
constexpr double kRequiredMarginM = 12.0;

// ⚠️ THE FLOOR GETS A SMALLER BAR, and the reason is a design choice rather than
// a concession. The 041 P2-3 band is asymmetric — +60 up, −10 down — because the
// rabbit climbs 34.98 m above the arm point and descends only 2.74 m below it.
// A 12 m bar under a 10 m extent is arithmetically unsatisfiable, so demanding
// it here would not be a stricter test, just a broken one.
//
// 5 m is what the rabbit actually needs: it leaves 7.26 m of clearance, i.e. the
// deck is never a factor for the TARGET.
//
// ⛔ It IS a factor for the chase, which measured **17.9 m below the arm point**
// on t1 — and was floor-limited even then, so it wanted more. Under a 10 m
// down-extent the chase will hit the deck. That is the operator's chosen
// geometry and it is watched at the P3-4 smoke as `ArenaEgressKind::FLOOR`,
// not asserted here: the chase's extent is a property of the policy, not of the
// generator.
constexpr double kRequiredFloorMarginM = 5.0;

void expectFits(const std::string& method, const Extents& e) {
    FlightArena arena;
    const double entry_agl = -static_cast<double>(SIM_INITIAL_ALTITUDE);

    EXPECT_LT(e.radius_max, arena.radius_m - kRequiredMarginM)
        << method << ": rabbit reaches " << e.radius_max << " m radius against a "
        << arena.radius_m << " m wall — only "
        << (arena.radius_m - e.radius_max) << " m of margin (worst seed "
        << e.worst_seed << "). The CHASE flies outside the rabbit's envelope, "
           "so a margin here is the chase's room, not the rabbit's.";

    EXPECT_LT(e.agl_max, arena.ceiling_agl_m - kRequiredMarginM)
        << method << ": rabbit reaches " << e.agl_max << " m AGL against a "
        << arena.ceiling_agl_m << " m ceiling — only "
        << (arena.ceiling_agl_m - e.agl_max) << " m of margin. ⚠️ This is the "
           "check that caught the 3 cm clearance at the first 041 geometry.";

    EXPECT_GT(e.agl_min, arena.floor_agl_m + kRequiredFloorMarginM)
        << method << ": rabbit descends to " << e.agl_min << " m AGL against a "
        << arena.floor_agl_m << " m hard deck — only "
        << (e.agl_min - arena.floor_agl_m) << " m of margin under a 10 m "
           "down-extent.";

    // Print the realized envelope even when everything passes. A green test
    // that prints nothing tells the next reader how much room there is only if
    // they go and re-measure it, which is how the 3 cm clearance survived.
    std::cout << "    [envelope] " << method
              << ": radius_max=" << e.radius_max
              << "  agl=[" << e.agl_min << ", " << e.agl_max << "]"
              << "  entry=" << entry_agl
              << "  margins: radial=" << (arena.radius_m - e.radius_max)
              << " ceiling=" << (arena.ceiling_agl_m - e.agl_max)
              << " floor=" << (e.agl_min - arena.floor_agl_m) << " m\n";
}

}  // namespace

// The generator every production M1 run uses (`autoc.ini`:
// PathGeneratorMethod = aeroStandard, SimNumPathsPerGeneration = 6).
TEST(ArenaPathFit, AeroStandardFitsWithRoomToSpare) {
    const Extents e = measure("aeroStandard", 6, 200);
    // Sanity: the sweep must actually have produced a climbing path, or the
    // ceiling assertion below is vacuous.
    ASSERT_GT(e.agl_max, -static_cast<double>(SIM_INITIAL_ALTITUDE) + 10.0)
        << "no path in the sweep climbed — the ceiling check would prove nothing";
    expectFits("aeroStandard", e);
}

// The generator the P3-4 smoke uses (`autoc-basic-m1.ini`: longSequential, 1 path).
TEST(ArenaPathFit, LongSequentialFitsWithRoomToSpare) {
    expectFits("longSequential", measure("longSequential", 1, 200));
}

// `random` is not selected by any shipped .ini today, but it is a live
// generator and is the one whose geometry is purely seed-driven — so it is the
// one most likely to surprise if it were selected.
TEST(ArenaPathFit, RandomFitsWithRoomToSpare) {
    expectFits("random", measure("random", 6, 200));
}

// ⛔ SOME LEGACY GENERATORS DO NOT FIT, AND THAT IS RECORDED RATHER THAN FIXED.
//
// Measured against 70 / 25 / 95: `computedPaths` reaches 96.99 m AGL (over the
// ceiling) and `line` reaches 76.16 m RADIUS — outside the 70 m wall entirely.
// They predate the arena and no shipped .ini selects them, so they are not worth
// re-designing; what IS worth having is this test failing the moment one of them
// is selected, instead of a bake crashing out and the cause being inferred from
// egress counts.
//
// ⚠️ `classic` was on this list and now FITS (r=40.00, AGL 35.01..90.00) — the
// asymmetric band happens to suit it. Removed rather than left asserted-broken,
// because a list that lies about which generators are safe is worse than no list.
TEST(ArenaPathFit, NoShippedConfigSelectsAnOutOfEnvelopeGenerator) {
    // The generators known not to fit the 041 arena.
    const std::vector<std::string> out_of_envelope = {"computedPaths", "line"};

    for (const std::string& m : out_of_envelope) {
        const Extents e = measure(m, 6, 20);
        FlightArena arena;
        const bool fits = e.radius_max < arena.radius_m &&
                          e.agl_max < arena.ceiling_agl_m &&
                          e.agl_min > arena.floor_agl_m;
        EXPECT_FALSE(fits)
            << m << " now FITS the arena. That is good news, not a failure — "
                 "delete it from this list and give it a positive test instead. "
                 "The list exists to stop a silent selection, not to enshrine "
                 "the breakage.";
    }
}

// ===========================================================================
// ⭐ THE ANALYTIC BOUND — operator ask 2026-08-18: *"double check the COMPUTED
// max extent of random paths — e.g. how large can they go — not just the
// geometric ones."*
// ===========================================================================
//
// The seed sweeps above are SAMPLES. For `random` / `SeededRandomB` the geometry
// is drawn fresh from `gPathSeed` every generation, so no finite sweep proves a
// bound — it only reports the largest thing seen so far. Over 400 seeds the
// sweep found radius 47.43 m; the true bound is 50.00 m, and a bake runs far
// more than 400 generations.
//
// THE DERIVATION. Control points come from `localRandomPointInCylinder(rng,
// radius=SIM_PATH_BOUNDS, height=SIM_PATH_BOUNDS, base=0)`:
//
//     r = radius * cbrt(u)  ∈ [0, 40]        z = -u * height ∈ [-40, 0]
//
// so every CONTROL POINT is inside a 40 m-radius, 40 m-tall cylinder sitting on
// the entry altitude. The realized path is NOT the control polygon, though: it
// is a uniform Catmull-Rom spline (`cubicInterpolate`, tension 0.5), and
// Catmull-Rom OVERSHOOTS its own control hull. With
//
//     b0 = ½(−t + 2t² − t³)      b1 = ½(2 − 5t² + 3t³)
//     b2 = ½( t + 4t² − 3t³)     b3 = ½(  −t² +  t³)
//
// the basis is a partition of unity (Σbᵢ = 1) but is NOT non-negative, and both
// extremes land at t = ½:
//
//     max Σ|bᵢ|            = 1.250   ⇒ radius  ≤ 40 × 1.250 = 50.00 m   (a NORM)
//     max Σ(positive bᵢ)   = 1.125   ⇒ AGL_max ≤ entry + 30 × 1.125 = entry + 33.75 m  (a COMPONENT)
//     max Σ|negative bᵢ|   = 0.125   ⇒ AGL_min ≥ entry − 30 × 0.125 = entry −  3.75 m
//
// (radius uses SIM_PATH_BOUNDS = 40, height uses SIM_PATH_HEIGHT_BOUNDS = 30 —
//  separate bounds since 041 P2-3.)
//
// i.e. exactly **one eighth** of overshoot, which is the well-known uniform
// Catmull-Rom figure. ⚠️ The −5 m is worth noticing on its own: the RANDOM
// generator is the only one whose rabbit can go BELOW the entry altitude at all.
// The 6 analytic aeroStandard paths never do.
namespace {
// Exact, from the basis above. Not measured — computed, so no sweep can miss it.
//
// ⚠️ TWO DIFFERENT CONSTANTS, and conflating them is a real error this test
// caught while being written (a claimed 45 m radial bound against a measured
// 47.43 m):
//   * a NORM over all three components is amplified by Σ|bᵢ| = 1.250, because
//     each component can independently swing to its own extreme;
//   * a SINGLE component drawn from a one-sided interval [−H, 0] reaches only
//     Σ(positive bᵢ) = 1.125 past the far end, and Σ|negative bᵢ| = 0.125 past
//     the near end.
// Radius is a norm. Altitude is a single component. They do not share a factor.
constexpr double kCatmullRomOvershoot = 0.125;                        // 1/8, at t = ½
constexpr double kSumAbsBasis         = 1.0 + 2 * kCatmullRomOvershoot;  // Σ|bᵢ| = 1.250
constexpr double kOneSidedOvershoot   = 1.0 + kCatmullRomOvershoot;      // 1.125
}  // namespace

TEST(ArenaPathFit, RandomPathsAnalyticWorstCaseFitsTheArena) {
    FlightArena arena;
    const double entry_agl = -static_cast<double>(SIM_INITIAL_ALTITUDE);
    const double cp_radius = static_cast<double>(SIM_PATH_BOUNDS);        // control-point cylinder
    const double cp_height = static_cast<double>(SIM_PATH_HEIGHT_BOUNDS);  // 041 P2-3: separate bound

    const double worst_radius  = cp_radius  * kSumAbsBasis;                     // 50.00
    const double worst_agl_max = entry_agl + cp_height * kOneSidedOvershoot;    // entry + 33.75
    const double worst_agl_min = entry_agl - cp_height * kCatmullRomOvershoot;  // entry −  3.75

    EXPECT_NEAR(worst_radius, 50.0, 1e-9);                 // 40 x 1.250 (a NORM)
    EXPECT_NEAR(worst_agl_max - entry_agl, 33.75, 1e-9);   // 30 x 1.125 (a COMPONENT)
    EXPECT_NEAR(entry_agl - worst_agl_min, 3.75, 1e-9);    // 30 x 0.125

    // ⛔ The bound, not a sample, against the arena.
    EXPECT_LT(worst_radius, arena.radius_m - kRequiredMarginM)
        << "a randomly-seeded rabbit can reach " << worst_radius
        << " m radius (40 m control cylinder × 1.125 Catmull-Rom overshoot) "
           "against a " << arena.radius_m << " m wall";
    EXPECT_LT(worst_agl_max, arena.ceiling_agl_m - kRequiredMarginM)
        << "a randomly-seeded rabbit can reach " << worst_agl_max
        << " m AGL against a " << arena.ceiling_agl_m << " m ceiling";
    EXPECT_GT(worst_agl_min, arena.floor_agl_m + kRequiredFloorMarginM)
        << "a randomly-seeded rabbit can descend to " << worst_agl_min
        << " m AGL — BELOW the entry altitude, which only this generator does — "
           "against a " << arena.floor_agl_m << " m hard deck";

    // And the sweep must never have exceeded the bound. If it does, the
    // derivation above is wrong and everything resting on it is too.
    //
    // ⚠️ `random` ONLY. `aeroStandard` is five closed-form aerobatic figures
    // plus one seeded path, and the analytic figures are NOT bounded by the
    // control-point cylinder — HighPerchSplitS reaches 57.14 m radius by
    // construction. Applying this bound to aeroStandard would assert something
    // false about paths it does not describe.
    const Extents e = measure("random", 6, 200);
    EXPECT_LE(e.radius_max, worst_radius + 1e-3)
        << "random exceeded the analytic radial bound — the Catmull-Rom "
           "derivation in this file is wrong";
    EXPECT_GE(e.agl_min, worst_agl_min - 1e-3)
        << "random went below the analytic floor bound";
    EXPECT_LE(e.agl_max, worst_agl_max + 1e-3)
        << "random exceeded the analytic ceiling bound";
}

// ⚠️ aeroStandard's binding constraint is NOT the random path. Five of its six
// paths are closed-form aerobatic figures whose climbs are literals, and after
// the 041 P2-3 shrinks the tallest is HighPerchSplitS at 15 + 15 m — still above
// the seeded path's analytic entry + 33.75 m. So the ceiling is sized by an
// ANALYTIC path, and stating that here stops the next reader sizing the arena
// off the random bound alone and coming up short.
TEST(ArenaPathFit, AnAnalyticPathNotTheRandomOneIsWhatSizesTheCeiling) {
    const double entry_agl = -static_cast<double>(SIM_INITIAL_ALTITUDE);
    const Extents aero = measure("aeroStandard", 6, 200);  // HighPerchSplitS is the tallest
    const double random_bound_agl =
        entry_agl + static_cast<double>(SIM_PATH_HEIGHT_BOUNDS) * kOneSidedOvershoot;

    EXPECT_GT(aero.agl_max, random_bound_agl)
        << "aeroStandard no longer climbs past the random-path analytic bound; "
           "if the analytic paths changed, re-derive which one sizes the ceiling";

    FlightArena arena;
    EXPECT_LT(aero.agl_max, arena.ceiling_agl_m - kRequiredMarginM)
        << "SpiralClimb reaches " << aero.agl_max << " m AGL against a "
        << arena.ceiling_agl_m << " m ceiling — "
        << (arena.ceiling_agl_m - aero.agl_max) << " m of margin. ⚠️ This is the "
           "assertion that stood at 0.03 m before the max-extent cross-check.";
}
