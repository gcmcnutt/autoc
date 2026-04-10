#ifndef NN_TOPOLOGY_H
#define NN_TOPOLOGY_H

// Compile-time NN topology constants (T110, updated 021)
//
// Single source of truth for neural network dimensions.
// The topology is {27, 16, 8, 3}: 27 sensor inputs, two hidden layers
// (16 and 8 neurons), 3 control outputs (pitch rate, roll rate, throttle).
//
// Input layout (27 sensors):
//  0- 5: dPhi  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  radians; +0.1s/+0.5s are path-lookahead
//  6-11: dTheta[-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  radians; same lookahead
// 12-17: dist  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  metres; dist to future rabbit position
//    18: dDist/dt closing rate (m/s, positive = approaching)
// 19-22: quaternion (w, x, y, z)                      [-1,1]
//    23: airspeed (m/s)
// 24-26: gyro rates (p, q, r) in rad/s                 standard aerospace RHR
//
// Removed in 021:
//    alpha/beta (was 24-25) — invalid without airspeed sensor
//    pitchCmd/rollCmd/throttleCmd feedback (was 26-28) — unnecessary with ACRO rate feedback
//
// Output layout (3 controls):
//    0: pitch [-1, 1] via tanh  (ACRO: desired pitch rate, MANUAL: surface deflection)
//    1: roll  [-1, 1] via tanh  (ACRO: desired roll rate,  MANUAL: surface deflection)
//    2: throttle [-1, 1] via tanh
// Output interpretation (rate→surface) is handled by sim bridge, not the NN.

#include <cstddef>
#include "autoc/nn/nn_inputs.h"

// Layer sizes (NN_INPUT_COUNT derived from NNInputs struct in nn_inputs.h)
constexpr int NN_HIDDEN1_SIZE = 16;
constexpr int NN_HIDDEN2_SIZE = 8;
constexpr int NN_OUTPUT_COUNT = 3;

// Full topology as a compile-time array
constexpr int NN_NUM_LAYERS = 4;
constexpr int NN_TOPOLOGY[NN_NUM_LAYERS] = {
    NN_INPUT_COUNT, NN_HIDDEN1_SIZE, NN_HIDDEN2_SIZE, NN_OUTPUT_COUNT
};

// Total weight+bias count: (27*16+16) + (16*8+8) + (8*3+3) = 611
constexpr int NN_WEIGHT_COUNT =
    (NN_INPUT_COUNT * NN_HIDDEN1_SIZE + NN_HIDDEN1_SIZE) +
    (NN_HIDDEN1_SIZE * NN_HIDDEN2_SIZE + NN_HIDDEN2_SIZE) +
    (NN_HIDDEN2_SIZE * NN_OUTPUT_COUNT + NN_OUTPUT_COUNT);

// Topology as comma-separated string (for config logging/validation)
constexpr const char* NN_TOPOLOGY_STRING = "27,16,8,3";

#endif
