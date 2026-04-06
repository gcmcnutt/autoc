#ifndef FITNESS_COMPUTER_H
#define FITNESS_COMPUTER_H

// Point-accumulation fitness (022): ellipsoidal scoring surface + streak multiplier.
// Replaces the old distance-penalty + crash-cliff model.
class FitnessComputer {
public:
    FitnessComputer(double behindScale, double aheadScale, double crossScale,
                    double streakThreshold, int streakStepsToMax, double streakMultMax);

    // Compute step score from along-track and cross-track distances.
    // along: positive = ahead of rabbit, negative = behind (preferred)
    // lateralDist: perpendicular distance (always >= 0)
    // Returns: stepPoints in (0, 1], 1.0 at rabbit position.
    double computeStepScore(double along, double lateralDist) const;

    // Update streak state and return stepPoints * multiplier.
    // Streak increments if stepPoints >= threshold, hard-resets otherwise.
    double applyStreak(double stepPoints);

    // Reset streak and diagnostics (call at start of each scenario).
    void resetStreak();

    // Diagnostics
    int getMaxStreak() const { return maxStreak_; }
    int getStreakSteps() const { return totalStreakSteps_; }
    double getMaxMultiplier() const { return maxMultiplier_; }

private:
    double behindScale_, aheadScale_, crossScale_;
    double streakThreshold_;
    int streakStepsToMax_;
    double streakMultMax_;

    // Per-scenario state
    int streakCount_ = 0;
    int maxStreak_ = 0;
    int totalStreakSteps_ = 0;
    double maxMultiplier_ = 1.0;
};

#endif
