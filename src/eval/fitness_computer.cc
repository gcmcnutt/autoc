#include "autoc/eval/fitness_computer.h"
#include <algorithm>

FitnessComputer::FitnessComputer(double behindScale, double aheadScale, double crossScale,
                                 double streakThreshold, int streakStepsToMax, double streakMultMax)
    : behindScale_(behindScale), aheadScale_(aheadScale), crossScale_(crossScale),
      streakThreshold_(streakThreshold), streakStepsToMax_(streakStepsToMax),
      streakMultMax_(streakMultMax) {}

double FitnessComputer::computeStepScore(double along, double lateralDist) const {
    double effAlong = (along <= 0.0)
        ? along / behindScale_
        : along / aheadScale_;
    double effLateral = lateralDist / crossScale_;
    double effDistSq = effAlong * effAlong + effLateral * effLateral;
    return 1.0 / (1.0 + effDistSq);
}

double FitnessComputer::applyStreak(double stepPoints) {
    if (stepPoints >= streakThreshold_) {
        streakCount_ = std::min(streakCount_ + 1, streakStepsToMax_);
        totalStreakSteps_++;
    } else {
        streakCount_ = 0;
    }
    maxStreak_ = std::max(maxStreak_, streakCount_);
    double multiplier = 1.0 + (streakMultMax_ - 1.0) * static_cast<double>(streakCount_) / static_cast<double>(streakStepsToMax_);
    maxMultiplier_ = std::max(maxMultiplier_, multiplier);
    return stepPoints * multiplier;
}

void FitnessComputer::resetStreak() {
    streakCount_ = 0;
    maxStreak_ = 0;
    totalStreakSteps_ = 0;
    maxMultiplier_ = 1.0;
}
