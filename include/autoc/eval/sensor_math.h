// sensor_math.h — Navigation sensor functions for temporal history recording
// Shared between minisim (history capture) and NN evaluator (reads history buffers)
// Extracted from gp_evaluator_portable during GP removal (014-nn-training-signal)
//
// 023: executeGetDPhi/executeGetDTheta removed — NN inputs now use direction
// cosines (unit vector in body frame) via computeTargetDir() in nn_input_computation.h.

#ifndef SENSOR_MATH_H
#define SENSOR_MATH_H

#include <cstdint>
#include "autoc/eval/aircraft_state.h"

// Path interpolation — returns position at goal distance along path (linear scan + lerp)
gp_vec3 getInterpolatedTargetPosition(PathProvider& pathProvider,
                                       gp_scalar currentOdometer,
                                       gp_scalar offsetMeters);

#endif
