#ifndef EVAL_LOGGER_H
#define EVAL_LOGGER_H

#include "autoc/types.h"

// Shared logging extracted from autoc.cc and autoc-eval.cc.
// Writes per-step data to data.dat; per-generation stats go to the run .log
// via the logger (the data.stc breadcrumb file was retired in 034 T052).
class EvalLogger {
public:
    void logStepHeader();
    void logStep();
    void logGenerationStats();
    void logBestController();
    void logNNWeightStats();
};

#endif
