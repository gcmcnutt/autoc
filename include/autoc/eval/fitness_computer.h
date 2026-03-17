#ifndef FITNESS_COMPUTER_H
#define FITNESS_COMPUTER_H

#include "autoc/types.h"

// Fitness computation constants (must match autoc.h defines)
// These are duplicated here for test builds that don't include autoc.h
#ifndef DISTANCE_TARGET
#define DISTANCE_TARGET 7.5
#define DISTANCE_NORM 5.0
#define DISTANCE_POWER 1.5
#define ATTITUDE_NORM 0.349
#define ATTITUDE_POWER 1.5
#define CRASH_COMPLETION_WEIGHT 1e6
#endif

// Intercept-budget constants (from autoc.h)
#ifndef INTERCEPT_SCALE_FLOOR
#define INTERCEPT_SCALE_FLOOR 0.1
#define INTERCEPT_SCALE_CEILING 1.0
#define INTERCEPT_BUDGET_MAX 15.0
#define INTERCEPT_TURN_RATE (M_PI / 4.0)
#endif

// Initial velocity for intercept budget computation
#ifndef SIM_INITIAL_VELOCITY
#define SIM_INITIAL_VELOCITY 16.0
#endif

// Shared fitness computation extracted from autoc.cc eval pipeline.
// Used by all controller backends (GP tree, bytecode, neural net).
class FitnessComputer {
public:
    // Compute per-step combined distance + attitude penalty.
    // Returns: pow(interceptScale * |dist - TARGET| / DIST_NORM, DIST_POWER)
    //        + pow(interceptScale * attitude_delta / ATT_NORM, ATT_POWER)
    double computeStepPenalty(double distance, double attitude_delta, double intercept_scale);

    // Compute crash penalty from fraction of path completed.
    // Returns: (1 - fraction_completed) * CRASH_COMPLETION_WEIGHT
    double computeCrashPenalty(double fraction_completed);

    // Compute attitude scaling factor from path geometry.
    // Returns: path_distance / max(path_turn_rad, 2*PI)
    double computeAttitudeScale(double path_distance, double path_turn_rad);

    // Compute per-step intercept scale factor.
    // Quadratic ramp from FLOOR to CEILING over the intercept budget.
    static double computeInterceptScale(double stepTimeSec, double budgetSec);

    // Estimate time-to-intercept from initial conditions.
    static double computeInterceptBudget(double displacement, double headingOffset,
                                          double aircraftSpeed, double rabbitSpeed);
};

#endif
