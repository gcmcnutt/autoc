#ifndef EVAL_LOGGER_H
#define EVAL_LOGGER_H

#include "autoc/types.h"

// Shared logging extracted from autoc.cc and autoc-eval.cc.
// Per-generation stats go to the run .log via the logger. (The per-step
// data.dat was retired in 035 FR-P05 — the S3 dmp is now the single training
// trace, inspected post-hoc via the dmp-dump tool; the data.stc breadcrumb
// was retired in 034 T052.)
class EvalLogger {
public:
    void logStepHeader();
    void logStep();
    void logGenerationStats();
    void logBestController();
    void logNNWeightStats();
};

#endif
