#pragma once
#include <cstddef>

// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and sim_response.py. Reordering fields is a format-breaking change.
// DO NOT add padding or non-float members.

struct NNInputs {
    // Time samples: [-0.9s, -0.3s, -0.1s, now, +0.1s, +0.5s]
    float dPhi[6];       // body-frame bearing angle (atan2), radians
    float dTheta[6];     // body-frame pitch-to-target angle, radians
    float dist[6];       // m, euclidean distance to rabbit
    float closing_rate;  // m/s, positive = approaching
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float airspeed;      // m/s
    float gyro_p;        // rad/s, aerospace RHR
    float gyro_q;
    float gyro_r;
};

static_assert(sizeof(NNInputs) == 27 * sizeof(float),
              "NNInputs layout must be contiguous float[27] with no padding");
static_assert(alignof(NNInputs) == alignof(float),
              "NNInputs must be float-aligned for matrix multiply");

constexpr int NN_INPUT_COUNT = sizeof(NNInputs) / sizeof(float);
