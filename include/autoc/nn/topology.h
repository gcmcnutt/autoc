#ifndef NN_TOPOLOGY_H
#define NN_TOPOLOGY_H

// Compile-time NN topology constants (T110, updated 023)
//
// Single source of truth for neural network dimensions.
// The topology is {NN_INPUT_COUNT, 32, 16, 3}: sensor inputs, two hidden layers
// (32 and 16 neurons), 3 control outputs (pitch rate, roll rate, throttle).
//
// 041 P2-2: NN_INPUT_COUNT is 45 (25 M1 target representation + 20 shared
// CraftCommonInputs). The historical 33-slot map below is kept for provenance
// only — nn_inputs.h is the layout of record, and the shared craft block is
// declared ONCE there rather than mirrored per mode.
//
// ⚠️ RNN width is NOT the constraint and is deliberately unchanged: measured
// W_hh effective rank 11.1–11.8 of 16, flat across all 608 generations of t1.
// Widening it would spend search on a non-problem. Settled; reopen only on new
// evidence.
//
// Input layout (33 sensors):
//  0- 5: target_x [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  unit-vec x (body frame)
//  6-11: target_y [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  unit-vec y (body frame)
// 12-17: target_z [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  unit-vec z (body frame)
// 18-23: dist     [-0.5s,-0.4s,-0.3s,-0.2s,-0.1s,now]  metres; dist to rabbit position
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

// Recurrent-layer flag (spec 027, D-simple). A recurrent layer gets a
// self-connection weight matrix W_hh (size × size, no extra bias — the
// layer's existing bias is shared). Hidden state is carried across NN
// evaluation ticks and reset on span start. Only hidden layers can be
// recurrent (input has no weights in; output is typically a projection).
// Clarify Q3 picked the 16-wide hidden2 layer.
// 028 D-alone: layer-2 (16-wide) recurrent re-enabled per spec/plan §Phase 2.
// CADENCE7-REDUX marker flipped: was {false,false,false,false} for diagnostic.
constexpr bool NN_RECURRENT[NN_NUM_LAYERS] = {
    false,   // layer 0: input (no weights in)
    false,   // layer 1: hidden1, 32-wide
    true,    // layer 2: hidden2, 16-wide — RECURRENT (028 D-alone)
    false    // layer 3: output
};

// Total weight+bias count. Feedforward: (33*32+32) + (32*16+16) + (16*3+3)
// = 1088+528+51 = 1667. Recurrent: + Σ_i size_i² for each recurrent layer i.
// With hidden2 recurrent: +16*16 = 256 → 1923.
constexpr int NN_WEIGHT_COUNT =
    (NN_INPUT_COUNT * NN_HIDDEN1_SIZE + NN_HIDDEN1_SIZE) +
    (NN_HIDDEN1_SIZE * NN_HIDDEN2_SIZE + NN_HIDDEN2_SIZE) +
    (NN_HIDDEN2_SIZE * NN_OUTPUT_COUNT + NN_OUTPUT_COUNT) +
    (NN_RECURRENT[1] ? NN_HIDDEN1_SIZE * NN_HIDDEN1_SIZE : 0) +
    (NN_RECURRENT[2] ? NN_HIDDEN2_SIZE * NN_HIDDEN2_SIZE : 0) +
    (NN_RECURRENT[3] ? NN_OUTPUT_COUNT  * NN_OUTPUT_COUNT  : 0);
static_assert(NN_WEIGHT_COUNT == 2307,
              "Weight count arithmetic inconsistent. History: 028 = 1923 at 33 in; "
              "038 FR-P0H 33→37 in (+4×32 = 128) = 2051; "
              "041 US4 37→42 in (+5×32 = 160) = 2211; "
              "041 P2-2 42→45 in (net +3: −IN_ENVELOPE −ENVELOPE_SECS "
              "+SPECIFIC_ENERGY +BOUNDARY_CLOSURE_RATE +SCORE_GRAD_X/Y/Z, "
              "+3×32 = 96) = 2307 "
              "(45·32+32 + 32·16+16 + 16·3+3 + 16·16 = 1472+528+51+256). "
              "RECOMPUTE this when the input count moves — never relax it.");

// Total recurrent hidden-state floats across all recurrent layers.
// Zero for pure-feedforward networks; 16 for the current 027 config.
constexpr int NN_HIDDEN_STATE_COUNT =
    (NN_RECURRENT[1] ? NN_HIDDEN1_SIZE : 0) +
    (NN_RECURRENT[2] ? NN_HIDDEN2_SIZE : 0) +
    (NN_RECURRENT[3] ? NN_OUTPUT_COUNT  : 0);

// Topology as comma-separated string (for config logging/validation).
// Recurrent layers marked with a trailing 'r'.
constexpr const char* NN_TOPOLOGY_STRING = "45,32,16r,3";  // 041 P2-2: 42 → 45 inputs
// ⚠️ DISPLAY ONLY, and it was STALE at "42" from 041 gen-1 through t7 launch —
// P2-2 moved the input count 42 → 45 and updated NN_WEIGHT_COUNT (which the
// static_assert above proves) but not this string. Every run log since then has
// printed the wrong first-layer size. Harmless to the maths, actively misleading
// to an investigation that reads the log to find out how big the network is.

// ============================================================================
// 030 M7a — Tracker-mode topology (FR-019 runtime mode dispatch + Session
// 2026-05-07 Q1 simplification). Parallel constants for autoc's runtime
// mode-select on desktop (one binary supports both modes); xiao firmware
// uses compile-time -DAUTOC_MODE per FR-019 to pick one.
//
// 2026-05-15 — M11.preA.5 T-102 closed: 16r baseline restored.
// 32r experiment ran to gen 544 (postdiag3); was consistently 700-1500 points
// WORSE than 16r baseline (postdiag2 -17060 vs postdiag3 -16382 best fitness).
// State capacity is NOT the binding constraint on the M2 plateau. Per
// postdiag3_report.md, 032 (derived perceptual features) is the next move,
// not bigger NN. Weight count restored to 2307; topology back to 45,32,16r,3.
//
// 2026-05-16 — 032 phase 1: input layer grows 45 → 54 (9 derived perceptual
// features). Hidden topology unchanged (32 → 16r → 3). Weight count grows
// 2307 → 2595 (~12.5%, from the wider input fan-in to hidden1). Topology
// string updated to "54,32,16r,3".
// ============================================================================

constexpr int TRACKER_NN_INPUT_COUNT = static_cast<int>(TrackerInput::COUNT);  // 66 (041 P2-2: 46 target + 20 CraftCommonInputs)
constexpr int TRACKER_NN_HIDDEN1_SIZE = 32;
constexpr int TRACKER_NN_HIDDEN2_SIZE = 16;
// 038 US3 — tracker output head grows 3 → 7: 3 actuated control outputs +
// kNumSpanAuxOutputs (4) auxiliary span/closure-predictor outputs (scored on a
// separate lexicase axis, NOT actuated). Pathgen (NN_OUTPUT_COUNT) is unchanged
// — the predictor is tracker-only (M2-direct). outputs[0..2] = control,
// outputs[3..6] = predicted span at kSpanPredictHorizonsMsec + span-rate.
constexpr int TRACKER_NN_CONTROL_OUTPUT_COUNT = 3;
constexpr int TRACKER_NN_OUTPUT_COUNT = TRACKER_NN_CONTROL_OUTPUT_COUNT + kNumSpanAuxOutputs;  // 7
constexpr int TRACKER_NN_NUM_LAYERS = 4;

constexpr int TRACKER_NN_TOPOLOGY[TRACKER_NN_NUM_LAYERS] = {
    TRACKER_NN_INPUT_COUNT,
    TRACKER_NN_HIDDEN1_SIZE,
    TRACKER_NN_HIDDEN2_SIZE,
    TRACKER_NN_OUTPUT_COUNT
};

// Same recurrent flag pattern as pathgen — hidden2 (16-wide) is recurrent.
constexpr bool TRACKER_NN_RECURRENT[TRACKER_NN_NUM_LAYERS] = {
    false, false, true, false
};

constexpr int TRACKER_NN_WEIGHT_COUNT =
    (TRACKER_NN_INPUT_COUNT * TRACKER_NN_HIDDEN1_SIZE + TRACKER_NN_HIDDEN1_SIZE) +
    (TRACKER_NN_HIDDEN1_SIZE * TRACKER_NN_HIDDEN2_SIZE + TRACKER_NN_HIDDEN2_SIZE) +
    (TRACKER_NN_HIDDEN2_SIZE * TRACKER_NN_OUTPUT_COUNT + TRACKER_NN_OUTPUT_COUNT) +
    (TRACKER_NN_RECURRENT[1] ? TRACKER_NN_HIDDEN1_SIZE * TRACKER_NN_HIDDEN1_SIZE : 0) +
    (TRACKER_NN_RECURRENT[2] ? TRACKER_NN_HIDDEN2_SIZE * TRACKER_NN_HIDDEN2_SIZE : 0) +
    (TRACKER_NN_RECURRENT[3] ? TRACKER_NN_OUTPUT_COUNT  * TRACKER_NN_OUTPUT_COUNT  : 0);
static_assert(TRACKER_NN_WEIGHT_COUNT == 3047,
              "Tracker weight count arithmetic inconsistent. History: 2787 at 60 in / 3 out; "
              "038 US3 = 2791 at 58 in / 7 out; "
              "041 US4 58→63 in (+5×32 = 160) = 2951; "
              "041 P2-2 63→66 in (same net +3 as pathgen, +3×32 = 96) = 3047 "
              "(66·32+32 + 32·16+16 + 16·7+7 + 16·16 = 2144+528+119+256). "
              "⚠️ Moves AGAIN after the M2 phase: +N innovation inputs (FR-005a), and "
              "the output count becomes 3 if the predictor head is retired at T088 "
              "(which would reclaim 16·4+4 = 68 output weights). RECOMPUTE, never relax.");

constexpr int TRACKER_NN_HIDDEN_STATE_COUNT =
    (TRACKER_NN_RECURRENT[1] ? TRACKER_NN_HIDDEN1_SIZE : 0) +
    (TRACKER_NN_RECURRENT[2] ? TRACKER_NN_HIDDEN2_SIZE : 0) +
    (TRACKER_NN_RECURRENT[3] ? TRACKER_NN_OUTPUT_COUNT  : 0);

constexpr const char* TRACKER_NN_TOPOLOGY_STRING = "66,32,16r,7";  // 041 P2-2: 63 → 66 inputs

// ============================================================================
// 041 P2-8 follow-up (2026-08-20) — MAKE THE DISPLAY STRINGS UNABLE TO GO STALE.
//
// ⛔ Both strings above are hand-maintained, and one of them WAS wrong: P2-2
// moved pathgen 42 → 45, updated NN_WEIGHT_COUNT (whose static_assert has been
// correct throughout) and updated the tracker string, but missed the pathgen
// one. Every run log from 041 gen-1 through t7 launch printed "42,32,16r,3" for
// a network whose first layer is 45 wide. Harmless arithmetically, actively
// misleading to anyone reading a log to size the network — which is exactly what
// happened while investigating the input-scale defect.
//
// ⭐ Fixing the string is not the fix. Nothing stopped it drifting, so nothing
// stops it drifting again. These asserts make the leading number a COMPILE-TIME
// consequence of the input count instead of a promise someone has to keep.
constexpr int autocLeadingInt(const char* s) {
    int v = 0;
    for (; *s >= '0' && *s <= '9'; ++s) v = v * 10 + (*s - '0');
    return v;
}
static_assert(autocLeadingInt(NN_TOPOLOGY_STRING) == NN_INPUT_COUNT,
              "NN_TOPOLOGY_STRING's first layer disagrees with NN_INPUT_COUNT. "
              "Update the string — the maths is driven by NN_INPUT_COUNT, so a "
              "mismatch means only the LOG is lying, which is worse than a crash "
              "because it survives review.");
static_assert(autocLeadingInt(TRACKER_NN_TOPOLOGY_STRING) ==
                  static_cast<int>(TrackerInput::COUNT),
              "TRACKER_NN_TOPOLOGY_STRING's first layer disagrees with "
              "TrackerInput::COUNT. Same failure, tracker side.");

#endif
