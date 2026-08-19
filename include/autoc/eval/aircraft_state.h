/* aircraft state and path types — shared between desktop and embedded */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <string>
#include "autoc/types.h"
#include "autoc/nn/topology.h"
#include "autoc/nn/nn_inputs.h"  // 037 R5 — kNNHistoryLagsMsec / layout version
#include "autoc/nn/nn_input_computation.h"

#ifndef ARDUINO
#include <cereal/cereal.hpp>
#endif
#include <vector>
// Portable math macros — work on desktop (std::) and embedded (Arduino)
#define CLAMP_DEF(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define ATAN2_DEF(y, x) std::atan2f(y, x)
#define ABS_DEF(v) std::fabs(v)
#define SQRT_DEF(v) std::sqrt(v)
#define MIN_DEF(a, b) ((a) < (b) ? (a) : (b))
#define MAX_DEF(a, b) ((a) > (b) ? (a) : (b))

#define SIM_MAX_ROLL_RATE_RADSEC (static_cast<gp_scalar>(M_PI))
#define SIM_MAX_PITCH_RATE_RADSEC (static_cast<gp_scalar>(M_PI))

#define SIM_INITIAL_VELOCITY static_cast<gp_scalar>(20.0f)
#define SIM_THROTTLE_SCALE static_cast<gp_scalar>(10.0f)
#define SIM_CRASH_PENALTY static_cast<gp_scalar>(300.0f)  // Large penalty to ensure path completion
// 041 P2-3 (operator 2026-08-18) — the virtual origin is where the craft
// ENTERS and where the xiao ARMS, and it sits at the EXACT VERTICAL CENTRE of
// the arena cylinder: floor 25 m AGL, ceiling 105 m AGL ⇒ arm 55 m AGL
// (up-extent +50, down-extent −30).
//
// ⚠️ Settled at 35 through three corrections, all from measurement:
//   * the max-extent cross-check (operator ask: *"make sure the pathgen paths
//     range fit in the arena with room to spare"*) found the SpiralClimb rabbit
//     topping out **3 cm** under the then-ceiling — a coincidence, not a
//     clearance, and the chase has to fly ABOVE the rabbit to track it;
//   * then sizing against the **site's 400 ft AGL working envelope** (121.92 m),
//     which got SpiralClimb shrunk 50 → 35 m (see SIM_PATH_HEIGHT_BOUNDS and
//     SpiralClimb's climbAmount);
//   * ⭐ then the band stopped being SYMMETRIC at all. Measured from the cylinder
//     centre the M1 rabbit climbs **34.98 m** and descends **2.74 m** — so a ±K
//     band spent half its height on airspace nothing ever enters while being
//     tight at the top. Operator: *"Maybe we should go back to non uniform
//     vertical extent for arena. 60m up and 10 down?"* The arm point is
//     therefore NO LONGER the vertical centre.
//   * ⛔ then the t2 SMOKE MEASURED that 10 m below the arm is not survivable:
//     **16 of 16 scenarios terminated on the deck** (terminal AGL 25.01–25.51
//     against a 25 m deck), mean survival 4.9 s of a 272-step path, and the
//     elite sat frozen at −161.468690 for 11 straight generations because
//     nothing lived long enough to generate gradient. Meanwhile the ceiling was
//     never approached — 48.07 m reached of a 95 m ceiling, 13 m of 60 used.
//     The band had been sized to the RABBIT (which flies flat and never
//     descends) while the CHASE is the thing being contained. Settled at
//     **+50 / −30** (operator 2026-08-18).
//
// ⛔ **400 ft IS NOT AN ARENA CONSTRAINT, AND IS NOT ENCODED AS ONE.** Operator
// 2026-08-18: *"if operator arms a path at 100m then it is on them to not go
// above 400ft… Not the models problem. Is operator."* The arena is a purely
// RELATIVE band — radius R about the arm origin, ceiling and floor at ±K — and
// where that band lands in the real world is chosen at ARM TIME. Staying under
// 400 ft and staying above terrain are both operator responsibilities, on the
// same footing. The AGL numbers below are simply where the relative band sits
// IN SIM, where the ground is known; nothing in the code enforces an absolute
// altitude and nothing should.
// arena_path_fit_tests measures all of this from the generator itself, every
// build, instead of trusting numbers someone once wrote down.
//
// Was −25. The move exists because "entry at the centre of the cylinder" and
// "the hard deck is above the ground" were not BOTH satisfiable at 25 m: the M1
// targets climb to 74.98 m AGL (measured), and a band centred on 25 m that
// reaches 75 m needs its floor 25 m underground.
//
// ⚠️ THIS IS NOT THE RETRACTED "ORIGIN → HARD DECK" PROPOSAL. z = 0 is still the
// entry point, so nothing about what a recorded z MEANS has changed and no dmp
// reader's frame shifts. The paths are still generated about the origin and the
// craft still starts on it — the whole scene simply sits 35 m higher over the
// terrain. Operator: *"Pilot responsible for that being way above terrain so
// that bottom of cyl is above ground. So really sim should do identical."*
//
// ⛔ MUST STAY CONSISTENT WITH THE ARENA. −SIM_INITIAL_ALTITUDE is where the arm
// point sits inside the FlightArena band, and `resolveEngageArena` derives the
// up/down extents from exactly that. If this and the arena bounds move
// independently, the sim trains in one band and the aircraft flies another. It
// is a runtime value, so this cannot be a static_assert — arena_tests.cc pins
// it instead, and that test is the thing that fails if either moves alone.
//
// ⛔ MUST STAY CONSISTENT WITH crrcsim/autoc_config.xml. `<launch altitude>` is
// in FEET and measured to the aircraft's LOWEST POINT above the terrain, so it
// is 80 m / 0.3048 − zLow(0.125 ft) + ground(0.1 ft) = 262.442 ft. See
// specs/041-m2-depth/toolchain-datum-validation.md for the measured chain.
#define SIM_INITIAL_ALTITUDE static_cast<gp_scalar>(-55.0f)
#define SIM_INITIAL_THROTTLE static_cast<gp_scalar>(0.0f)
#define SIM_INITIAL_LOCATION_DITHER static_cast<gp_scalar>(30.0f)
#define SIM_PATH_BOUNDS static_cast<gp_scalar>(40.0f)
// 041 P2-3 — the VERTICAL half of the path-generation bound, split out from
// SIM_PATH_BOUNDS because the arena is not a cube: the half-band K and the
// radius R are independent, and only the vertical was tight.
//
// ⚠️ IT IS NOT THE REALIZED EXTENT. `localRandomPointInCylinder` places CONTROL
// POINTS inside this bound, but the path is a uniform Catmull-Rom spline
// (`cubicInterpolate`) which OVERSHOOTS its own control hull by exactly 1/8 at
// t = ½. So the realized envelope is 1.125x this: 30 m of draw becomes
// **33.75 m** above entry, and 40 m of radius becomes **45 m**. Sizing an arena
// off the draw rather than off the realized bound is short by 12.5% — see the
// derivation in tests/arena_path_fit_tests.cc.
//
// 30 m chosen so the random path clears the ceiling by 16.25 m (arm + 33.75
// against the +50 up-extent), comfortably behind the analytic paths.
// Was implicitly SIM_PATH_BOUNDS (40 ⇒ 45 m realized ⇒ only 3 m of margin),
// which would have made the SEEDED path the binding one the moment SpiralClimb
// stopped being.
#define SIM_PATH_HEIGHT_BOUNDS static_cast<gp_scalar>(30.0f)
#define SIM_PATH_RADIUS_LIMIT static_cast<gp_scalar>(70.0f)
// 041 P2-3 — SIM_MIN_ELEVATION / SIM_MAX_ELEVATION are GONE, deliberately and
// without a compatibility shim, so that every site that used them has to be
// visited rather than silently keeping the old envelope.
//
// They encoded a THIRD arena. M1 terminated on 70 / 7 / 120 (these macros) while
// telling its own network about `FlightArena`'s 80 / 5 / 100 — an input cylinder
// 10 m wider and 20 m shorter than the one that actually killed it. TA01 ranks
// DIST_TO_BOUNDARY the third most important input in the vector, so the policy
// was leaning hard on a boundary signal wrong in both directions.
//
// Both modes now terminate on `autoc::eval::checkArenaBounds` against the one
// `FlightArena` the gather also reads. What the network is told and what ends
// the scenario are the same cylinder, by construction.
//
// SIM_PATH_RADIUS_LIMIT survives because it is a different quantity — a
// PATH-GENERATION bound (how far out a rabbit may be placed), not a containment
// envelope. It is not the arena and must not be conflated with it again.

#define SIM_TOTAL_TIME_MSEC (100 * 1000)
// 037 OUTCOME — cadence returned 50 → 100 ms (back to 10 Hz; operator
// 2026-06-11, see specs/037-20hz-control-loop/outcome.md). The 20 Hz A/B
// (t6/t7) showed the converged bang-bang is objective-optimal at the depths
// THEN reached. t10 (2026-06-13) revisits 20 Hz on a NEW basis: the t9
// datasheet-honest servo (82.5 ms full-span transit) sits between the 50 ms
// and 100 ms tick — at 20 Hz the actuator can't complete a reversal in one
// tick, so it band-limits the relay physically, and the close-in tracking
// regime (LOS rate ∝ 1/range) is where the one-tick transport delay finally
// binds. Paired with the 0.8 s history window (kNNHistoryLagsMsec) whose
// 50 ms finest slot only lands on integer ticks at >=20 Hz. This is the
// compile-time cadence MASTER; ControlIntervalMsec in every .ini must equal
// it (config.cc fails loud otherwise) and crrcsim derives
// gEvalUpdateIntervalMsec from it via WorkerInit. The FDM config
// (crrcsim/autoc_config.xml dt=0.005, fps=20) satisfies the cadence triple
// at 50 ms: cycleLength 50 ms, framesPerEval 1, FDM oversample 10×.
// NOTE: reverting to 10 Hz now also requires reverting the 0.8 s lag set
// (50 ms slot ∤ 100 ms tick → historyLagsIntegral static_assert fires —
// intended fail-loud, no longer a pure 2-knob flip).
#define SIM_TIME_STEP_MSEC (50)
#define SIM_MAX_INTERVAL_MSEC (SIM_TIME_STEP_MSEC * 5)

// 037 R5/T021 — history lag table (ms-based; see nn_inputs.h). Lags must be
// integral in ticks at the compiled cadence.
constexpr bool historyLagsIntegral() {
  for (int m : kNNHistoryLagsMsec) {
    if (m % SIM_TIME_STEP_MSEC != 0) return false;
  }
  return true;
}
static_assert(historyLagsIntegral(),
              "kNNHistoryLagsMsec entries must be integral multiples of "
              "SIM_TIME_STEP_MSEC");
constexpr int historyLagTicks(int slot) {
  return kNNHistoryLagsMsec[slot] / SIM_TIME_STEP_MSEC;
}

// 037 T018 — cadence rescale anchor. Per-tick fitness accumulators
// (path score, stability, energy) multiply by this so totals are
// denominated in 100 ms-tick-equivalent units at ANY control rate:
// ×1.0 at the historical 10 Hz (bitwise-exact no-op — the FP regression
// gate holds), ×0.5 at 20 Hz, ×0.2 at 50 Hz. Anchored to the historical
// tick rather than to seconds so totals stay on the t6 scale and the
// constant lexicase epsilon (0.5) keeps its meaning
// (project_lexicase_mad_epsilon). Parameterized form exists for the
// cadence-invariance tests; production uses the compile-time constant.
constexpr double cadenceTickScale(unsigned long intervalMsec) {
  return static_cast<double>(intervalMsec) / 100.0;
}
constexpr double kCadenceTickScale = cadenceTickScale(SIM_TIME_STEP_MSEC);

// 037 T020 — rate-independent engage-delay window (023 contract):
// ticks = ceil(delayMsec / intervalMsec), so the coast DURATION is the
// same wall-clock at every control rate. Shared by the crrcsim pathgen
// branch, both tracker-stepper mirrors, and tick_rescale_tests.cc.
constexpr int engageDelayTicks(unsigned long delayMsec, unsigned long intervalMsec) {
  return static_cast<int>((delayMsec + intervalMsec - 1) / intervalMsec);
}

/*
 * some generic path information about routes
 */
class Path {
public:
  gp_vec3 start;
  gp_vec3 orientation;
  gp_scalar distanceFromStart;
  gp_scalar radiansFromStart;

  // Default constructor for backward compatibility
  Path() : start(gp_vec3::Zero()), orientation(gp_vec3::UnitX()),
           distanceFromStart(0.0f), radiansFromStart(0.0f) {}

  // Constructor to ensure all fields are properly initialized
  Path(const gp_vec3& start_pos, const gp_vec3& orient,
       gp_scalar distance, gp_scalar radians)
    : start(start_pos), orientation(orient), distanceFromStart(distance),
      radiansFromStart(radians) {}

  // Generic constructor to cast external Eigen scalars into gp_scalar
  template <typename Scalar>
  Path(const Eigen::Matrix<Scalar, 3, 1>& start_pos, const Eigen::Matrix<Scalar, 3, 1>& orient,
       Scalar distance, Scalar radians)
    : start(start_pos.template cast<gp_scalar>()), orientation(orient.template cast<gp_scalar>()),
      distanceFromStart(static_cast<gp_scalar>(distance)),
      radiansFromStart(static_cast<gp_scalar>(radians)) {}

  void sanitize() {
    auto sanitizeScalar = [](gp_scalar value, gp_scalar fallback = 0.0f) {
      return std::isfinite(value) ? value : fallback;
    };
    for (int i = 0; i < 3; ++i) {
      start[i] = sanitizeScalar(start[i]);
      gp_scalar defaultOrient = (i == 0) ? 1.0f : 0.0f;
      orientation[i] = sanitizeScalar(orientation[i], defaultOrient);
    }
    distanceFromStart = sanitizeScalar(distanceFromStart);
    radiansFromStart = sanitizeScalar(radiansFromStart);
  }

  void dump(std::ostream& os) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Path: (%f, %f, %f), Odometer: %f, Turnmeter: %f",
      start[0], start[1], start[2], distanceFromStart, radiansFromStart);
    os << buf;
  }

#ifndef ARDUINO
  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t /*version*/) {
    ar(start, orientation, distanceFromStart, radiansFromStart);
  }
#endif
};
#ifndef ARDUINO
CEREAL_CLASS_VERSION(Path, 1)
#endif

// Maximum offset steps for path interpolation (±1 second, time-derived).
// 037 audit: currently UNUSED anywhere in the tree; derived from time so it
// cannot rot at a cadence change if revived.
constexpr int MAX_OFFSET_STEPS = 1000 / SIM_TIME_STEP_MSEC;

// Portable path provider interface for unified GP evaluation
// Abstracts path access for both vector and single-path environments
class PathProvider {
public:
    virtual ~PathProvider() = default;
    virtual const Path& getPath(int index) const = 0;
    virtual int getCurrentIndex() const = 0;
    virtual int getPathSize() const = 0;

    // Get the distance of the last waypoint in the path
    gp_scalar getMaxDistance() const {
        int size = getPathSize();
        if (size == 0) return 0;
        return getPath(size - 1).distanceFromStart;
    }
};

// Vector-based path provider
class VectorPathProvider : public PathProvider {
private:
    const std::vector<Path>& paths;
    int currentIndex;
    
public:
    VectorPathProvider(const std::vector<Path>& p, int current = 0) 
        : paths(p), currentIndex(current) {}
    
    const Path& getPath(int index) const override {
        if (index < 0) index = 0;
        if (index >= (int)paths.size()) index = (int)paths.size() - 1;
        return paths[index];
    }
    
    int getCurrentIndex() const override { return currentIndex; }
    int getPathSize() const override { return (int)paths.size(); }
    void setCurrentIndex(int index) { currentIndex = index; }
};

// Single path provider
class SinglePathProvider : public PathProvider {
private:
    const Path& singlePath;
    int currentIndex;
    
public:
    SinglePathProvider(const Path& p, int current = 0) 
        : singlePath(p), currentIndex(current) {}
    
    const Path& getPath(int index) const override {
        // For single path, always return the same path regardless of index
        return singlePath;
    }
    
    int getCurrentIndex() const override { return currentIndex; }
    int getPathSize() const override { return 1; }
    void setCurrentIndex(int index) { currentIndex = index; }
};

// Forward declaration for AircraftState
struct AircraftState;

// 038 US3 — per-state NN-mode discriminator for the SPLIT dmp serialization.
// Pathgen and tracker dmps each carry ONLY their own NN block (inputs+outputs),
// so a format change to one mode does NOT invalidate the other's dmps (the M1
// source library survives M2 format changes). Set by setNNData; serialized as a
// uint8 that selects the block on read. uint8_t underlying = stable wire byte.
enum class NNMode : std::uint8_t { Pathgen = 0, Tracker = 1 };

// ACRO rate-PID per-tick internals captured from CRRCSim inputdev for
// post-run analysis. Pitch/roll only; yaw passive (HB1 has no rudder).
// All rates rad/s, integrators in rad. See spec 026.
struct PidInternals {
  float rateCmdP = 0.0f;  // desired body-X rate (roll), rad/s
  float rateCmdQ = 0.0f;  // desired body-Y rate (pitch), rad/s
  float rateAchP = 0.0f;  // achieved body-X rate from FDM, rad/s
  float rateAchQ = 0.0f;  // achieved body-Y rate from FDM, rad/s
  float ffP = 0.0f;       // FF term contribution (post-scale)
  float ffQ = 0.0f;
  float pP  = 0.0f;       // P term contribution (post-scale)
  float pQ  = 0.0f;
  float iP  = 0.0f;       // I term contribution (post-scale)
  float iQ  = 0.0f;
  float intP = 0.0f;      // integrator state (rad)
  float intQ = 0.0f;
  uint8_t sat = 0;        // bit0=pitch axis hit ±1, bit1=roll axis hit ±1

#ifndef ARDUINO
  template<class Archive>
  void serialize(Archive& ar) {
    ar(rateCmdP, rateCmdQ, rateAchP, rateAchQ,
       ffP, ffQ, pP, pQ, iP, iQ, intP, intQ, sat);
  }
#endif
};

// REMOVED: getPathIndex() - replaced by getInterpolatedTargetPosition() in gp_evaluator_portable.cc
// The old discrete index lookup caused jitter sensitivity; interpolation provides smooth sensor values.

/*
* portable aircraft state
 */
struct AircraftState {
  public:

    AircraftState()
      : thisPathIndex(0),
        dRelVel(0.0f),
        velocity(gp_vec3::Zero()),
        aircraft_orientation(gp_quat::Identity()),
        position(gp_vec3::Zero()),
        simTimeMsec(0),
        pitchCommand(0.0f),
        rollCommand(0.0f),
        throttleCommand(0.0f),
        wind_velocity(gp_vec3::Zero()),
        rabbitPosition(gp_vec3::Zero()),
        rabbitOdometer_(0.0f),
        rabbitSpeed_(0.0f) {}
    AircraftState(int thisPathIndex, gp_scalar relVel, gp_vec3 vel, gp_quat orientation,
      gp_vec3 pos, gp_scalar pc, gp_scalar rc, gp_scalar tc,
      unsigned long int timeMsec)
      : thisPathIndex(thisPathIndex), dRelVel(relVel), velocity(vel), aircraft_orientation(orientation), position(pos), simTimeMsec(timeMsec),
      pitchCommand(pc), rollCommand(rc), throttleCommand(tc),
      wind_velocity(gp_vec3::Zero()),
      rabbitPosition(gp_vec3::Zero()) {
    }

    // Casting ctor for external Eigen scalar types while migrating callers to float
    template <typename Scalar>
    AircraftState(int thisPathIndex, Scalar relVel, const Eigen::Matrix<Scalar, 3, 1>& vel, const Eigen::Quaternion<Scalar>& orientation,
      const Eigen::Matrix<Scalar, 3, 1>& pos, Scalar pc, Scalar rc, Scalar tc,
      unsigned long int timeMsec)
      : thisPathIndex(thisPathIndex),
        dRelVel(static_cast<gp_scalar>(relVel)),
        velocity(vel.template cast<gp_scalar>()),
        aircraft_orientation(orientation.template cast<gp_scalar>()),
        position(pos.template cast<gp_scalar>()),
        simTimeMsec(timeMsec),
        pitchCommand(static_cast<gp_scalar>(pc)),
        rollCommand(static_cast<gp_scalar>(rc)),
        throttleCommand(static_cast<gp_scalar>(tc)),
        wind_velocity(gp_vec3::Zero()),
        rabbitPosition(gp_vec3::Zero()) {}

    // generate setters and getters
    int getThisPathIndex() const { return thisPathIndex; }
    void setThisPathIndex(int index) { thisPathIndex = index; }

    gp_scalar getRelVel() const { return dRelVel; }
    void setRelVel(gp_scalar relVel) { dRelVel = relVel; }
    
    gp_vec3 getVelocity() const { return velocity; }
    void setVelocity(const gp_vec3& vel) { velocity = vel; }

    gp_quat getOrientation() const { return aircraft_orientation; }
    void setOrientation(gp_quat orientation) { aircraft_orientation = orientation; }

    gp_vec3 getPosition() const { return position; }
    void setPosition(gp_vec3 pos) { position = pos; }

    // Position is always in virtual coordinates (origin at test start).
    // Producers (CRRCSim, xiao) convert raw→virtual at boundary.
    // See docs/COORDINATE_CONVENTIONS.md "Virtual Frame" section.

    unsigned long int getSimTimeMsec() const { return simTimeMsec; }
    void setSimTimeMsec(unsigned long int timeMsec) { simTimeMsec = timeMsec; }

    gp_scalar getPitchCommand() const { return pitchCommand; }
    gp_scalar setPitchCommand(gp_scalar pitch) { return (pitchCommand = CLAMP_DEF(pitch, -1.0f, 1.0f)); }
    gp_scalar getRollCommand() const { return rollCommand; }
    gp_scalar setRollCommand(gp_scalar roll) { return (rollCommand = CLAMP_DEF(roll, -1.0f, 1.0f)); }
    gp_scalar getThrottleCommand() const { return throttleCommand; }
    gp_scalar setThrottleCommand(gp_scalar throttle) { return (throttleCommand = CLAMP_DEF(throttle, -1.0f, 1.0f)); }

    gp_vec3 getWindVelocity() const { return wind_velocity; }
    void setWindVelocity(const gp_vec3& wind) { wind_velocity = wind; }

    gp_vec3 getRabbitPosition() const { return rabbitPosition; }
    void setRabbitPosition(const gp_vec3& pos) { rabbitPosition = pos; }

    gp_scalar getRabbitOdometer() const { return rabbitOdometer_; }
    void setRabbitOdometer(gp_scalar odo) { rabbitOdometer_ = odo; }

    gp_scalar getRabbitSpeed() const { return rabbitSpeed_; }
    void setRabbitSpeed(gp_scalar speed) { rabbitSpeed_ = speed; }

    // Body-frame angular rates (p, q, r) in rad/s — standard aerospace RHR convention
    // Roll (p): positive = right wing down
    // Pitch (q): positive = nose up
    // Yaw (r): positive = nose right
    // NOTE: INAV gyro has inverted pitch/yaw signs — negate at consumer boundary.
    // CRRCSim FDM provides these in standard convention (no conversion needed).
    gp_vec3 getGyroRates() const { return gyroRates_; }
    void setGyroRates(const gp_vec3& rates) { gyroRates_ = rates; }

    // 041 T039 — body-frame SPECIFIC FORCE in g (what an accelerometer reads,
    // gravity included), completing the 6-DOF inertial block beside the gyro.
    //
    // Aerospace body FRD, so steady level flight is [0, 0, -1]: body +z points
    // DOWN and the measured reaction points UP. ⚠️ Do NOT "fix" that against
    // INAV's bench table (level reads +1 g on its normal axis) — INAV's frame
    // is FLU and msplink flips y/z at the boundary, exactly as it already does
    // for the quat and the gyro. Same physical fact, two frames. Settled
    // 2026-08-11; derivation in docs/COORDINATE_CONVENTIONS.md.
    //
    // ⚠️ SPECIFIC force, not FDM kinematic acceleration. The latter reads ~0 in
    // level flight and would put a constant 1 g error in the load axis —
    // invisible in sim, wrong in the air.
    //
    // WHY IT LIVES HERE rather than being computed in the gather: on hardware
    // `acc.accADCf` arrives already finished in the same MSP round trip as the
    // quat and gyro, and the xiao's gather does no transformation — it copies.
    // The sim matches that shape (spec.md § Clarifications, 2026-08-10): the
    // worker computes it via autoc/eval/specific_force.h, stores it here, and
    // both gathers only copy. A gather-side computation would be a sim-only
    // code path with no hardware counterpart.
    //
    // Stored UNSCALED (g). kAccelScale_g is applied at the NN slot write.
    gp_vec3 getSpecificForceG() const { return specificForceG_; }
    void setSpecificForceG(const gp_vec3& sf_g) { specificForceG_ = sf_g; }

    // 041 T037/T038 — envelope occupancy, as the NN sees it.
    //
    // `inEnvelope` is the OBSERVABLE scoring-envelope condition (FR-015), never
    // the fitness machinery's internal streak counter. `envelopeSecs` is
    // min(consecutive_seconds / FitStreakRampSec, 1), already normalized.
    //
    // Both are computed ONCE per tick by the stepper, before the NN evaluates,
    // and the same value feeds the NN input and (M1) the fitness accumulation.
    // Two independent computations of this quantity are forbidden — the
    // disagreement they permit is the failure class 041 exists to remove.
    // Accumulator mechanics: autoc/eval/envelope_state.h.
    bool getInEnvelope() const { return inEnvelope_; }
    void setInEnvelope(bool inside) { inEnvelope_ = inside; }

    gp_scalar getEnvelopeSecs() const { return envelopeSecs_; }
    void setEnvelopeSecs(gp_scalar secs) { envelopeSecs_ = secs; }

    // ------------------------------------------------------------------
    // 041 P2-2 — the three new observations. Same WHY-IT-LIVES-HERE rule as
    // the specific force above: the PRODUCER computes, both gathers COPY. On
    // hardware every one of these is derivable from what the xiao already has
    // (position, velocity, the arena it resolved at engage, and — for M1 and
    // for M2 phase 1 — the virtual target), so none of them needs a sim-only
    // code path, and none of them may acquire one.
    // ------------------------------------------------------------------

    // Specific energy `Es = h_hd + v²/2g`, in METRES, UNSCALED.
    // kEnergyScale_m is applied at the NN slot write, so every other consumer —
    // dmp-dump, the Ps objective axis, the flight log — reads plain metres.
    // Datum is height above the arena FLOOR; see autoc/eval/energy_state.h for
    // why that and not AGL.
    gp_scalar getSpecificEnergy() const { return specificEnergyM_; }
    void setSpecificEnergy(gp_scalar es_m) { specificEnergyM_ = es_m; }

    // Outward radial velocity toward the cylinder wall, in METRES PER SECOND,
    // UNSCALED (kCruiseSpeed_mps is applied at the slot write). POSITIVE =
    // closing on the wall. Complements — does not replace — the along-velocity
    // DIST_TO_BOUNDARY: distance says where the wall is, this says whether it
    // is getting closer, and distance is saturated above 0.99 on 83% of ticks
    // where this still spans ±17 m/s.
    gp_scalar getBoundaryClosureRate() const { return boundaryClosureRateMps_; }
    void setBoundaryClosureRate(gp_scalar mps) { boundaryClosureRateMps_ = mps; }

    // ∂score/∂position in the BODY frame, already multiplied by the streak
    // multiplier, UNSCALED (per metre). The improvement direction: which way to
    // move to score more, weighted by how much reward is currently at stake.
    //
    // ⚠️ The slot encoding is a direction-preserving tanh of the NORM applied
    // at the gather (see kScoreGradScale) — this carries the raw vector, so a
    // recorded column and an analysis can recover the true magnitude.
    gp_vec3 getScoreGradBody() const { return scoreGradBody_; }
    void setScoreGradBody(const gp_vec3& g) { scoreGradBody_ = g; }

    // NN I/O capture — record what the NN actually saw and produced.
    //
    // 030 M9.preA (2026-05-07): Two parallel input slots — `nnInputs_` for
    // pathgen mode (33 floats per `NNInputs`), `trackerInputs_` for tracker
    // mode (45 floats per `TrackerInputs`). Only the active mode's slot
    // gets populated; the other stays zero-initialized. This closes the
    // M6d/M8a deferral at evaluator.cc:495 ("setNNData(TrackerInputs)
    // is deferred to M8") — necessary for honest data.dat capture +
    // proper diagnostic stability/energy in tracker mode (per
    // memory:feedback_honest_dmp_recording).
    //
    // The two-slot design keeps pathgen byte-identical (nnInputs_ unchanged)
    // while extending tracker honesty. Cereal v=3 writes both slots; the
    // ~45 extra floats per state × ~58K states/dmp ≈ 10MB cost is acceptable.
    // 038 US3 — nnOutputs_ holds up to TRACKER_NN_OUTPUT_COUNT (7): pathgen
    // writes 3 (control), tracker writes 7 (3 control + 4 span-aux predictor).
    void setNNData(const NNInputs& inputs, const float* outputs, int numOutputs) {
      nnInputs_ = inputs;
      for (int i = 0; i < TRACKER_NN_OUTPUT_COUNT && i < numOutputs; i++) nnOutputs_[i] = outputs[i];
      hasNNData_ = true;
      nnMode_ = NNMode::Pathgen;
    }
    void setNNData(const TrackerInputs& inputs, const float* outputs, int numOutputs) {
      trackerInputs_ = inputs;
      for (int i = 0; i < TRACKER_NN_OUTPUT_COUNT && i < numOutputs; i++) nnOutputs_[i] = outputs[i];
      hasNNData_ = true;
      nnMode_ = NNMode::Tracker;
    }
    bool hasNNData() const { return hasNNData_; }
    const NNInputs& getNNInputs() const { return nnInputs_; }
    const TrackerInputs& getTrackerInputs() const { return trackerInputs_; }
    const float* getNNOutputs() const { return nnOutputs_; }

    // ACRO PID per-tick internals (see PidInternals).
    void setPidInternals(const PidInternals& p) { pidInternals_ = p; }
    const PidInternals& getPidInternals() const { return pidInternals_; }

    // =========================================================================
    // Temporal history for GP nodes - see specs/TEMPORAL_STATE.md
    // =========================================================================
    // Ring depth derives from the deepest history lag (t10: 0.8 s):
    // max lag in ticks + 1 (= 17 at 50 ms / 20 Hz). Was a fixed 10
    // ("1 sec at 10 Hz") pre-037, then 33 at the 1.6 s window.
    static constexpr int HISTORY_SIZE = (kNNHistoryLagsMsec[0] / SIM_TIME_STEP_MSEC) + 1;

    // Record current target direction and distance to history (call before NN eval each tick)
    void recordErrorHistory(const gp_vec3& targetDir, gp_scalar distance, unsigned long timeMs) {
      targetDirHistory_[historyIndex_] = targetDir;
      distHistory_[historyIndex_] = distance;
      timeHistory_[historyIndex_] = timeMs;
      historyIndex_ = (historyIndex_ + 1) % HISTORY_SIZE;
      if (historyCount_ < HISTORY_SIZE) historyCount_++;
    }

    // Get historical target direction unit vector (n=0 is most recent, n=1 is one tick ago, etc.)
    // Returns zero vector if history not available. Uses CLAMP_DEF for portability.
    gp_vec3 getHistoricalTargetDir(int n) const {
      if (historyCount_ == 0) return gp_vec3::Zero();
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return targetDirHistory_[idx];
    }

    gp_scalar getHistoricalDist(int n) const {
      if (historyCount_ == 0) return static_cast<gp_scalar>(0.0f);
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return distHistory_[idx];
    }

    unsigned long getHistoricalTime(int n) const {
      if (historyCount_ == 0) return 0;
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return timeHistory_[idx];
    }

    int getHistoryCount() const { return historyCount_; }

    void clearHistory() {
      historyIndex_ = 0;
      historyCount_ = 0;
    }

    // Pre-fill all history slots with the current geometry so the NN starts
    // with consistent direction cosines and closing_rate = 0.  Call at
    // engage/scenario start AFTER position, orientation, and path are known.
    // targetPos: world-frame position of the rabbit/path start
    // pathTangent: unit vector along path at targetPos (singularity fallback)
    void resetHistory(const gp_vec3& targetPos, const gp_vec3& pathTangent) {
      clearHistory();
      gp_vec3 craftToTarget = targetPos - position;
      gp_vec3 target_local = aircraft_orientation.inverse() * craftToTarget;
      float distance = static_cast<float>(target_local.norm());

      gp_vec3 tangent_body = aircraft_orientation.inverse() * pathTangent;
      float tn = static_cast<float>(tangent_body.norm());
      if (tn > 1e-6f) tangent_body = tangent_body / tn;
      else tangent_body = gp_vec3::UnitX();

      gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);

      for (int h = 0; h < HISTORY_SIZE; h++) {
        recordErrorHistory(dir, distance, 0);
      }
    }

    void advanceState(gp_scalar dt) {
      gp_scalar dtSec = dt / 1000.0f;

      // get current roll state, compute left/right force (positive roll is right)
      gp_scalar delta_roll = std::remainder(rollCommand * dtSec * SIM_MAX_ROLL_RATE_RADSEC, static_cast<gp_scalar>(M_PI));

      // get current pitch state, compute up/down force (positive pitch is up)
      gp_scalar delta_pitch = std::remainder(pitchCommand * dtSec * SIM_MAX_PITCH_RATE_RADSEC, static_cast<gp_scalar>(M_PI));

      // adjust velocity as a function of throttle (-1:1)
      // throttle = -1.0 → 0.5x base velocity (10 m/s)
      // throttle =  0.0 → 1.0x base velocity (20 m/s)
      // throttle = +1.0 → 1.5x base velocity (30 m/s)
      dRelVel = SIM_INITIAL_VELOCITY * (1.0f + throttleCommand * 0.5f);

      // Save pre-rotation orientation for gyro rate computation
      gp_quat q_prev = aircraft_orientation;

      // Convert pitch and roll updates to quaternions (in the body frame)
      gp_quat delta_roll_quat(Eigen::AngleAxis<gp_scalar>(delta_roll, gp_vec3::UnitX()));
      gp_quat delta_pitch_quat(Eigen::AngleAxis<gp_scalar>(delta_pitch, gp_vec3::UnitY()));

      // Apply the roll and pitch adjustments to the aircraft's orientation
      aircraft_orientation = aircraft_orientation * delta_roll_quat;
      aircraft_orientation = aircraft_orientation * delta_pitch_quat;

      // Normalize the resulting quaternion
      aircraft_orientation.normalize();

      // Compute actual body angular rates from quaternion delta (rad/s)
      // delta_q = q_prev.inverse() * q_new = rotation in body frame over dt
      // angular velocity = 2 * vec(delta_q) / dt (small angle approx)
      if (dtSec > 0.0f) {
        gp_quat dq = q_prev.inverse() * aircraft_orientation;
        if (dq.w() < 0) { dq.coeffs() = -dq.coeffs(); }  // ensure shortest path
        gyroRates_ = gp_vec3(
            2.0f * dq.x() / dtSec,   // p (roll rate, rad/s)
            2.0f * dq.y() / dtSec,   // q (pitch rate, rad/s)
            2.0f * dq.z() / dtSec);  // r (yaw rate, rad/s)
      }

      // Define the initial velocity vector in the body frame
      gp_vec3 velocity_body(dRelVel * dtSec, 0.0f, 0.0f);

      // Rotate the velocity vector using the updated quaternion
      gp_vec3 velocity_world = aircraft_orientation * velocity_body;

      // Store the actual velocity vector (convert from distance per timestep to velocity)
      velocity = velocity_world / dtSec;

      // adjust position
      position += velocity_world;
    }

  private:
    int thisPathIndex;
    gp_scalar dRelVel;
    gp_vec3 velocity;  // Actual velocity vector (north, east, down)
    gp_quat aircraft_orientation;
    gp_vec3 position;
    unsigned long int simTimeMsec;
    gp_scalar pitchCommand;
    gp_scalar rollCommand;
    gp_scalar throttleCommand;

    // Wind diagnostic fields (for debugging non-determinism)
    gp_vec3 wind_velocity;  // Wind vector (north, east, down) from calculate_wind()

    // Interpolated rabbit position for renderer playback
    gp_vec3 rabbitPosition;  // Exact target position NN was tracking this tick

    // Odometer-based rabbit tracking
    gp_scalar rabbitOdometer_;  // Distance along path (meters)
    gp_scalar rabbitSpeed_;     // Current rabbit speed (m/s)

    // Body-frame angular rates (rad/s, standard aerospace RHR)
    gp_vec3 gyroRates_ = gp_vec3::Zero();

    // 041 T037-T039 — the observation half of the 6-DOF inertial block, plus
    // envelope occupancy. Set by the stepper each tick BEFORE the NN gathers.
    //
    // ⚠️ DELIBERATELY NOT SERIALIZED. These are NN INPUTS, and every NN input
    // is already recorded per tick through nnInputs_ / trackerInputs_ (the
    // honest-capture block). Adding them to the archive would create a second
    // recorded copy of the same numbers that could disagree with the first —
    // the exact parallel-definition shape US1 exists to retire. Readers wanting
    // accel or envelope read the NN input columns, which are what the policy
    // actually saw.
    gp_vec3 specificForceG_ = gp_vec3::Zero();  // body FRD, g, gravity included
    bool inEnvelope_ = false;
    gp_scalar envelopeSecs_ = 0;                // already normalized to [0, 1]
    // 041 P2-2 — all UNSCALED; NN normalization happens at the slot writes.
    gp_scalar specificEnergyM_ = 0;             // Es, metres above the hard deck + v²/2g
    gp_scalar boundaryClosureRateMps_ = 0;      // + = outward, toward the wall
    gp_vec3 scoreGradBody_ = gp_vec3::Zero();   // ∂score/∂position, body frame, × multiplier

    // NN I/O capture — actual values presented to/produced by the neural net.
    // Pathgen-mode populates nnInputs_ (33 floats); tracker-mode populates
    // trackerInputs_ (45 floats). The other stays zero-initialized. Mode
    // dispatch happens at consumer sites (data.dat writer, fitness_decomposition).
    NNInputs nnInputs_ = {};                 // Pathgen NN inputs (NNInputs struct = 37 floats)
    TrackerInputs trackerInputs_ = {};       // 030 M9.preA — Tracker NN inputs (58 floats)
    float nnOutputs_[TRACKER_NN_OUTPUT_COUNT] = {0};  // Raw tanh outputs (mode-agnostic; 3 pathgen / 7 tracker w/ span-aux, 038 US3)
    bool hasNNData_ = false;
    NNMode nnMode_ = NNMode::Pathgen;        // 038 US3 — set by setNNData; selects the serialized NN block

    // ACRO PID per-tick snapshot (always populated when PID runs)
    PidInternals pidInternals_;

    // Temporal history — target direction (unit vec in body frame) and distance
    gp_vec3 targetDirHistory_[HISTORY_SIZE];  // unit vectors in body frame
    gp_scalar distHistory_[HISTORY_SIZE] = {0};
    unsigned long timeHistory_[HISTORY_SIZE] = {0};
    int historyIndex_ = 0;   // Next write position (ring buffer)
    int historyCount_ = 0;   // Valid samples (0 to HISTORY_SIZE)

#ifndef ARDUINO
    friend class cereal::access;
    template<class Archive>
    void serialize(Archive& ar, const std::uint32_t version) {
      ar(thisPathIndex, dRelVel, velocity, aircraft_orientation, position,
         pitchCommand, rollCommand, throttleCommand, simTimeMsec, wind_velocity,
         rabbitPosition, hasNNData_);

      // 030 M8a — Honest-recording audit at v=2 schema-bump boundary
      // (per memory:feedback_honest_dmp_recording). v=1 had a gyroRates_
      // gap (the 030 M3a regression caught it during source dmp loading
      // when SourceTickSample needed angular rates and discovered the
      // field wasn't persisted). v=2 closes the gap.
      //
      // raw-ok: gyroRates_ is gp_vec3 (Eigen-typed); cereal byte-format
      // is governed by the existing free-function save/load handlers
      // for gp_vec3 in protocol.h.
      if (version >= 2) {
        ar(gyroRates_);
      }

      // 038 US3 — SPLIT NN serialization (mode-separated). Pathgen and tracker
      // dmps each carry ONLY their own NN block (inputs + outputs together),
      // keyed on a per-state nnMode discriminator written first. A format change
      // to one mode NO LONGER invalidates the other's dmps — critically, the M1
      // pathgen SOURCE library survives M2/tracker format changes, so we stop
      // re-baking the M1 source on every M2 iteration. Greenfield restructure
      // (no cereal version bump per project policy; all pre-split dmps obsolete,
      // one re-bake). The v=1 legacy gate is retained but dead (no v=1 dmps).
      const bool readNNData = (version >= 2) ? true : hasNNData_;
      if (readNNData) {
        uint8_t mode = static_cast<uint8_t>(nnMode_);
        ar(mode);
        nnMode_ = static_cast<NNMode>(mode);

        // 037 T023 — shared fail-loud history-layout marker (both modes).
        if (version >= 2) {
          uint32_t historyLayout = kNNHistoryLayoutVersion;
          ar(historyLayout);
          if (historyLayout != kNNHistoryLayoutVersion) {
            throw std::runtime_error(
              "AircraftState deserialization: NN history-layout mismatch — "
              "serialized layout v" + std::to_string(historyLayout) +
              " but compiled with v" + std::to_string(kNNHistoryLayoutVersion) +
              " (t10 ms-based lags {800,400,200,100,50,0}, 0.8 s window). Old "
              "dmps are not replayable through the new layout.");
          }
        }

        if (nnMode_ == NNMode::Tracker) {
          // Tracker block: 58 inputs + 7 outputs (3 control + 4 span-aux).
          uint32_t inputCount = static_cast<uint32_t>(TrackerInput::COUNT);
          uint32_t outputCount = static_cast<uint32_t>(TRACKER_NN_OUTPUT_COUNT);
          ar(inputCount, outputCount);
          if (inputCount != static_cast<uint32_t>(TrackerInput::COUNT) ||
              outputCount != static_cast<uint32_t>(TRACKER_NN_OUTPUT_COUNT)) {
            throw std::runtime_error(
              "AircraftState deserialization: TRACKER topology mismatch — "
              "serialized inputs=" + std::to_string(inputCount) +
              " outputs=" + std::to_string(outputCount) +
              " but compiled with inputs=" + std::to_string(static_cast<int>(TrackerInput::COUNT)) +
              " outputs=" + std::to_string(TRACKER_NN_OUTPUT_COUNT) +
              ". Regenerate training data with current binary.");
          }
          float* rawTracker = reinterpret_cast<float*>(&trackerInputs_);  // raw-ok: NN-byte-format buffer
          for (uint32_t i = 0; i < inputCount; i++) ar(rawTracker[i]);
          for (uint32_t i = 0; i < outputCount; i++) ar(nnOutputs_[i]);
        } else {
          // Pathgen block: 37 inputs + 3 outputs.
          uint32_t inputCount = NN_INPUT_COUNT;
          uint32_t outputCount = NN_OUTPUT_COUNT;
          ar(inputCount, outputCount);
          if (inputCount != NN_INPUT_COUNT || outputCount != NN_OUTPUT_COUNT) {
            throw std::runtime_error(
              "AircraftState deserialization: PATHGEN topology mismatch — "
              "serialized inputs=" + std::to_string(inputCount) +
              " outputs=" + std::to_string(outputCount) +
              " but compiled with inputs=" + std::to_string(NN_INPUT_COUNT) +
              " outputs=" + std::to_string(NN_OUTPUT_COUNT) +
              ". Regenerate training data with current binary.");
          }
          float* rawInputs = reinterpret_cast<float*>(&nnInputs_);  // raw-ok: NN-byte-format buffer
          for (uint32_t i = 0; i < inputCount; i++) ar(rawInputs[i]);
          for (uint32_t i = 0; i < outputCount; i++) ar(nnOutputs_[i]);
        }
      }
      ar(rabbitOdometer_, rabbitSpeed_);
      ar(pidInternals_);

      // 041 P2-4 — honest-recording audit at this schema-bump boundary
      // (memory:feedback_honest_dmp_recording says to do one at EVERY bump;
      // v=1's gyroRates_ gap is what that rule was written for).
      //
      // These three are UNSCALED sources of NN slots, and each is recorded
      // separately from the slot for a reason that is not redundancy:
      //   * under an ablation mask the slot is ZEROED before recording — by
      //     design, so the dmp describes the network that actually ran — which
      //     would erase the quantity from the record entirely;
      //   * SCORE_GRAD's slot is a tanh of the norm, so near saturation the
      //     magnitude is not recoverable from it at any precision;
      //   * dmp-dump derives Ps by differencing Es across ticks, and that has
      //     to be the true Es, not a de-normalized slot.
      ar(specificEnergyM_, boundaryClosureRateMps_, scoreGradBody_);
    }
#endif
};
#ifndef ARDUINO
// 030 M8a (2026-05-06) — bumped 1 → 2 for honest-recording audit
// (gyroRates_ added, NN data switched to always-on instead of
// hasNNData_-gated). v=1 read path unchanged (regression-tight invariant
// for gen9200.dmp baseline).
//
// 030 M9.preA (2026-05-07) — STAYS at v=2; trackerInputs_ field added
// in-place to the v=2 schema per user feedback "while inside M2 we
// don't need backward compatibility — old ones will be obsolete".
// v=2 dmps written before M9.preA become unreadable; v=1 path
// (gen9200.dmp regression gate) is unaffected.
CEREAL_CLASS_VERSION(AircraftState, 2)
#endif

// Physics trace entry - captures complete FDM state at a single timestep
// Uses crrcsim native types for bit-exact copying (SCALAR = double)
// WARNING: Do NOT introduce type conversions - causes rounding errors!
struct PhysicsTraceEntry {
  // Simulation metadata
  uint32_t step;          // Timestep number
  double simTimeMsec;     // Simulation time in milliseconds (native: double)
  double dtSec;           // Timestep delta in seconds (SCALAR)

  // Worker identity (for multi-process determinism debugging)
  int32_t workerId;       // Worker ID (0-7 typically)
  int32_t workerPid;      // Worker process ID
  int32_t evalCounter;    // Evaluation counter on this worker

  // Position, velocity, acceleration (world frame) - all SCALAR (double)
  double pos[3];          // Position [x, y, z]
  double vel[3];          // Velocity [x, y, z]
  double acc[3];          // Acceleration [x, y, z]
  double accPast[3];      // Previous acceleration

  // Orientation and rotation - all SCALAR (double)
  double quat[4];         // Quaternion [x, y, z, w]
  double quatDotPast[4];  // Previous quaternion derivative
  double omegaBody[3];    // Angular velocity in body frame
  double omegaDotBody[3]; // Angular acceleration
  double rate[3];         // Rate (may alias omegaBody)
  double ratePast[3];     // Previous rate

  // Aerodynamic state - all SCALAR (double)
  double alpha;           // Angle of attack (rad)
  double beta;            // Sideslip angle (rad)
  double vRelWind;        // Airspeed magnitude
  double velRelGround[3]; // Velocity relative to ground
  double velRelAir[3];    // Velocity relative to air
  double vLocal[3];       // Local velocity
  double vLocalDot[3];    // Local velocity derivative

  // Aero calculation details - all SCALAR (double)
  double cosAlpha, sinAlpha, cosBeta;     // Trig values
  double CL, CD;                          // Lift and drag coefficients
  double CL_left, CL_cent, CL_right;     // Spanwise lift distribution
  double CL_wing;                         // Wing lift coefficient
  double Cl, Cm, Cn;                      // Moment coefficients
  double QS;                              // Dynamic pressure × ref area

  // Forces and moments (body frame) - all SCALAR (double)
  double forceBody[3];    // Total force
  double momentBody[3];   // Total moment

  // Environment - all SCALAR (double)
  double wind[3];         // Wind velocity
  double localAirmass[3]; // Local airmass velocity
  double gustBody[6];     // Gust in body frame: [v_V_gust_body (3), v_R_omega_gust_body (3)]
  double density;         // Air density
  double gravity;         // Gravity magnitude
  double geocentricLat, geocentricLon, geocentricR;  // Geocentric position

  // Control inputs - all SCALAR (double) from TSimInputs
  double pitchCommand;    // GP pitch command
  double rollCommand;     // GP roll command
  double throttleCommand; // GP throttle command
  double elevator;        // Sim elevator input
  double aileron;         // Sim aileron input
  double rudder;          // Sim rudder input
  double throttle;        // Sim throttle input

  // RNG state - exact integer types
  uint16_t rngState16;    // 16-bit RNG state
  uint32_t rngState32;    // 32-bit RNG state

  // Path tracking
  int32_t pathIndex;      // Current path index

  PhysicsTraceEntry() { memset(this, 0, sizeof(*this)); }

#ifndef ARDUINO
  template <class Archive>
  void serialize(Archive& ar) {
    ar(step, simTimeMsec, dtSec);
    ar(workerId, workerPid, evalCounter);
    ar(cereal::binary_data(pos, sizeof(pos)));
    ar(cereal::binary_data(vel, sizeof(vel)));
    ar(cereal::binary_data(acc, sizeof(acc)));
    ar(cereal::binary_data(accPast, sizeof(accPast)));
    ar(cereal::binary_data(quat, sizeof(quat)));
    ar(cereal::binary_data(quatDotPast, sizeof(quatDotPast)));
    ar(cereal::binary_data(omegaBody, sizeof(omegaBody)));
    ar(cereal::binary_data(omegaDotBody, sizeof(omegaDotBody)));
    ar(cereal::binary_data(rate, sizeof(rate)));
    ar(cereal::binary_data(ratePast, sizeof(ratePast)));
    ar(alpha, beta, vRelWind);
    ar(cereal::binary_data(velRelGround, sizeof(velRelGround)));
    ar(cereal::binary_data(velRelAir, sizeof(velRelAir)));
    ar(cereal::binary_data(vLocal, sizeof(vLocal)));
    ar(cereal::binary_data(vLocalDot, sizeof(vLocalDot)));
    ar(cosAlpha, sinAlpha, cosBeta);
    ar(CL, CD, CL_left, CL_cent, CL_right, CL_wing, Cl, Cm, Cn, QS);
    ar(cereal::binary_data(forceBody, sizeof(forceBody)));
    ar(cereal::binary_data(momentBody, sizeof(momentBody)));
    ar(cereal::binary_data(wind, sizeof(wind)));
    ar(cereal::binary_data(localAirmass, sizeof(localAirmass)));
    ar(cereal::binary_data(gustBody, sizeof(gustBody)));
    ar(density, gravity, geocentricLat, geocentricLon, geocentricR);
    ar(pitchCommand, rollCommand, throttleCommand);
    ar(elevator, aileron, rudder, throttle);
    ar(rngState16, rngState32, pathIndex);
  }
#endif
};
