#include "autoc/eval/fitness_computer.h"
#include <cmath>
#include <algorithm>

// Extracted from autoc.cc lines 1288-1290
// Per-step combined distance + attitude penalty
double FitnessComputer::computeStepPenalty(double distance, double attitude_delta, double intercept_scale) {
    double distDelta = fabs(distance - DISTANCE_TARGET);
    double dist_penalty = pow(intercept_scale * distDelta / DISTANCE_NORM, DISTANCE_POWER);
    double att_penalty = pow(intercept_scale * attitude_delta / ATTITUDE_NORM, ATTITUDE_POWER);
    return dist_penalty + att_penalty;
}

// Extracted from autoc.cc lines 1402-1408
// Crash penalty: soft lexicographic - completion dominates
double FitnessComputer::computeCrashPenalty(double fraction_completed) {
    return (1.0 - fraction_completed) * CRASH_COMPLETION_WEIGHT;
}

// Extracted from autoc.cc lines 1389-1393
// Attitude scaling from path geometry
double FitnessComputer::computeAttitudeScale(double path_distance, double path_turn_rad) {
    double denominator = std::max(path_turn_rad, 2.0 * M_PI);
    return path_distance / denominator;
}

// Quadratic ramp from FLOOR to CEILING over the intercept budget
double FitnessComputer::computeInterceptScale(double stepTimeSec, double budgetSec) {
    if (budgetSec <= 0.0) return INTERCEPT_SCALE_CEILING;
    double t = std::min(1.0, stepTimeSec / budgetSec);
    return INTERCEPT_SCALE_FLOOR + (INTERCEPT_SCALE_CEILING - INTERCEPT_SCALE_FLOOR) * t * t;
}

// Crude geometric estimate of time-to-intercept
double FitnessComputer::computeInterceptBudget(double displacement, double headingOffset,
                                                double aircraftSpeed, double rabbitSpeed) {
    if (aircraftSpeed <= 0.0) return INTERCEPT_BUDGET_MAX;
    double turn_time = fabs(headingOffset) / INTERCEPT_TURN_RATE;
    double closure_time = displacement / aircraftSpeed;
    double rabbit_compensation = closure_time * (rabbitSpeed / aircraftSpeed);
    double budget = turn_time + closure_time + rabbit_compensation;
    return std::clamp(budget, 0.0, INTERCEPT_BUDGET_MAX);
}
