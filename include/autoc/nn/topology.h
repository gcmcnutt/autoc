#ifndef NN_TOPOLOGY_H
#define NN_TOPOLOGY_H

// Compile-time NN topology constants (T110)
//
// Single source of truth for neural network dimensions.
// The topology is {29, 16, 8, 3}: 29 sensor inputs, two hidden layers
// (16 and 8 neurons), 3 control outputs (pitch, roll, throttle).
//
// Input layout (29 sensors, all raw — no normalization):
//  0- 5: dPhi  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  radians; +0.1s/+0.5s are path-lookahead
//  6-11: dTheta[-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  radians; same lookahead
// 12-17: dist  [-0.9s,-0.3s,-0.1s,now,+0.1s,+0.5s]  metres; dist to future rabbit position
//    18: dDist/dt closing rate (m/s, positive = approaching)
// 19-22: quaternion (w, x, y, z)                      [-1,1]
//    23: airspeed (m/s)
//    24: alpha (angle of attack, rad)
//    25: beta  (sideslip, rad)
// 26-28: pitchCmd, rollCmd, throttleCmd feedback       [-1,1]
//
// Output layout (3 controls):
//    0: pitch command    [-1, 1] via tanh
//    1: roll command     [-1, 1] via tanh
//    2: throttle command [-1, 1] via tanh

#include <cstddef>

// Layer sizes
constexpr int NN_INPUT_COUNT = 29;
constexpr int NN_HIDDEN1_SIZE = 16;
constexpr int NN_HIDDEN2_SIZE = 8;
constexpr int NN_OUTPUT_COUNT = 3;

// Full topology as a compile-time array
constexpr int NN_NUM_LAYERS = 4;
constexpr int NN_TOPOLOGY[NN_NUM_LAYERS] = {
    NN_INPUT_COUNT, NN_HIDDEN1_SIZE, NN_HIDDEN2_SIZE, NN_OUTPUT_COUNT
};

// Total weight+bias count: (29*16+16) + (16*8+8) + (8*3+3) = 643
constexpr int NN_WEIGHT_COUNT =
    (NN_INPUT_COUNT * NN_HIDDEN1_SIZE + NN_HIDDEN1_SIZE) +
    (NN_HIDDEN1_SIZE * NN_HIDDEN2_SIZE + NN_HIDDEN2_SIZE) +
    (NN_HIDDEN2_SIZE * NN_OUTPUT_COUNT + NN_OUTPUT_COUNT);

// Topology as comma-separated string (for config logging/validation)
constexpr const char* NN_TOPOLOGY_STRING = "29,16,8,3";

#endif
