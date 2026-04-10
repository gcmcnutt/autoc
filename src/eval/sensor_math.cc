// sensor_math.cc — Navigation sensor functions for temporal history recording
// Extracted from gp_evaluator_portable during GP removal (014-nn-training-signal)
//
// 023: fastAtan2/ATAN_LUT removed — NN inputs now use direction cosines
// (unit vector in body frame) via computeTargetDir() in nn_input_computation.h.
// executeGetDPhi/executeGetDTheta removed — no longer in the NN input pipeline.

#include "autoc/eval/sensor_math.h"
#include <cmath>

// ============================================================================
// Path interpolation
// ============================================================================

gp_vec3 getInterpolatedTargetPosition(PathProvider& pathProvider,
                                       gp_scalar currentOdometer,
                                       gp_scalar offsetMeters) {
    int pathSize = pathProvider.getPathSize();
    if (pathSize == 0) {
        return gp_vec3::Zero();
    }
    if (pathSize == 1) {
        return pathProvider.getPath(0).start;
    }

    // Handle NaN offset — return current rabbit position
    if (std::isnan(offsetMeters)) {
        int currentIdx = CLAMP_DEF(pathProvider.getCurrentIndex(), 0, pathSize - 1);
        return pathProvider.getPath(currentIdx).start;
    }

    // Calculate goal distance along path
    gp_scalar goalOdometer = currentOdometer + offsetMeters;

    // Clamp goal distance to path bounds
    gp_scalar minDist = pathProvider.getPath(0).distanceFromStart;
    gp_scalar maxDist = pathProvider.getPath(pathSize - 1).distanceFromStart;

    if (goalOdometer <= minDist) {
        return pathProvider.getPath(0).start;
    }
    if (goalOdometer >= maxDist) {
        return pathProvider.getPath(pathSize - 1).start;
    }

    // Linear scan from current index — O(1) amortized since rabbit only moves forward.
    int lo = CLAMP_DEF(pathProvider.getCurrentIndex(), 0, pathSize - 2);

    // Scan forward if needed
    while (lo < pathSize - 2 && pathProvider.getPath(lo + 1).distanceFromStart <= goalOdometer) {
        lo++;
    }
    // Scan backward if needed (for negative offsets)
    while (lo > 0 && pathProvider.getPath(lo).distanceFromStart > goalOdometer) {
        lo--;
    }

    const Path& p0 = pathProvider.getPath(lo);
    const Path& p1 = pathProvider.getPath(lo + 1);

    // Calculate interpolation fraction
    gp_scalar dd = p1.distanceFromStart - p0.distanceFromStart;
    gp_scalar frac = 0.0f;
    if (dd > 0.0f) {
        frac = (goalOdometer - p0.distanceFromStart) / dd;
        frac = CLAMP_DEF(frac, 0.0f, 1.0f);
    }

    return p0.start + frac * (p1.start - p0.start);
}

