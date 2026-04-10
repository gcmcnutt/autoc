#ifndef NN_TOPOLOGY_H
#define NN_TOPOLOGY_H

// Compile-time NN topology constants (T110, updated 023)
//
// Single source of truth for neural network dimensions.
// The topology is {33, 32, 16, 3}: 33 sensor inputs, two hidden layers
// (32 and 16 neurons), 3 control outputs (pitch rate, roll rate, throttle).
//
// Input layout (33 sensors):
//  0- 5: target_x [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  unit-vec x (body frame)
//  6-11: target_y [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  unit-vec y (body frame)
// 12-17: target_z [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  unit-vec z (body frame)
// 18-23: dist     [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  metres; dist to rabbit position
//    24: dDist/dt closing rate (m/s, positive = approaching)
// 25-28: quaternion (w, x, y, z)                         [-1,1]
//    29: airspeed (m/s)
// 30-32: gyro rates (p, q, r) in rad/s                   standard aerospace RHR
//
// Changed in 023 (was 27 inputs with dPhi/dTheta atan2 angles):
//    dPhi/dTheta replaced by target_x/y/z direction cosines (unit vector in body frame)
//    Hidden layers widened: 16→32, 8→16 to accommodate 6 additional inputs
//
// Output layout (3 controls):
//    0: pitch [-1, 1] via tanh  (ACRO: desired pitch rate, MANUAL: surface deflection)
//    1: roll  [-1, 1] via tanh  (ACRO: desired roll rate,  MANUAL: surface deflection)
//    2: throttle [-1, 1] via tanh
// Output interpretation (rate→surface) is handled by sim bridge, not the NN.

#include <cstddef>
#include "autoc/nn/nn_inputs.h"

// Layer sizes (NN_INPUT_COUNT derived from NNInputs struct in nn_inputs.h)
constexpr int NN_HIDDEN1_SIZE = 32;
constexpr int NN_HIDDEN2_SIZE = 16;
constexpr int NN_OUTPUT_COUNT = 3;

// Full topology as a compile-time array
constexpr int NN_NUM_LAYERS = 4;
constexpr int NN_TOPOLOGY[NN_NUM_LAYERS] = {
    NN_INPUT_COUNT, NN_HIDDEN1_SIZE, NN_HIDDEN2_SIZE, NN_OUTPUT_COUNT
};

// Total weight+bias count: (33*32+32) + (32*16+16) + (16*3+3) = 1088+528+51 = 1667
constexpr int NN_WEIGHT_COUNT =
    (NN_INPUT_COUNT * NN_HIDDEN1_SIZE + NN_HIDDEN1_SIZE) +
    (NN_HIDDEN1_SIZE * NN_HIDDEN2_SIZE + NN_HIDDEN2_SIZE) +
    (NN_HIDDEN2_SIZE * NN_OUTPUT_COUNT + NN_OUTPUT_COUNT);
static_assert(NN_WEIGHT_COUNT == 1667, "Weight count arithmetic inconsistent");

// Topology as comma-separated string (for config logging/validation)
constexpr const char* NN_TOPOLOGY_STRING = "33,32,16,3";

#endif
