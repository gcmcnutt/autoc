#pragma once
#include <cstddef>
#include <cstdint>

// 030 M11.preA.2 (2026-05-09) — Tracker-mode input normalization constants.
//
// Applied in gather_tracker_inputs (src/nn/evaluator.cc). These rescale the
// raw chase-state values into NN-friendly ranges:
//
//   - Cruise-normalized airspeed: hb1 cruises ~13 m/s; chase at cruise ⇒ 1.0.
//     Replaces raw m/s. Range becomes ≈ [0, 2].
//     ⚠️ 041 P2-2 briefly reverted this to raw for BOTH modes; P2-8 restored it.
//
//   - Soft-saturated dist_to_boundary: cylinder always contains chase in sim
//     (top + ceiling), so distanceToBoundary ≥ 0 by construction. Raw meters
//     (range [0, ~tens]) is replaced by tanh(d / scale). With scale=20 (M11.preA.2
//     bump from 10 → 20 on 2026-05-09 to match craft's ~10-15m emergency-turn
//     budget): 10m → 0.46, 15m → 0.64, 20m → 0.76, 30m → 0.91, 40m → 0.96.
//     NN gets meaningful gradient over ~2 turn-radii of warning instead of
//     reacting only inside the commit zone. Smooth NN-space saturation; the
//     world-frame safety layer (true OOB envelope, eventually for real flight)
//     is a separate future addition that operates on raw distance, not NN input.
//
// NOT a layout change — TrackerInputs struct stays float[45]. Old genomes
// trained against raw airspeed/dist are NOT portable across this normalization
// (greenfield, no backward compat per project policy). data.dat byte format
// unchanged.
constexpr float kCruiseSpeed_mps = 13.0f;
constexpr float kDistToBoundaryScale_m = 20.0f;

// ============================================================================
// 041 P2-8 (2026-08-20) — FINISH THE JOB 030 STARTED. Scale the four remaining
// raw-units inputs so every slot reaches the first layer on comparable footing.
//
// ⛔ WHY THIS IS NOT TUNING. A unit sees Σ wᵢxᵢ, so an input's influence is
// weight × SPREAD, not weight alone. Measured over 131,493 t5 ticks the spread
// ran from 7.53 (CLOSING_RATE, raw m/s) down to 0.036 (DIST_TO_BOUNDARY) — a
// 200× span. With Xavier init every weight starts comparable, so the small-scale
// slots start ~100× quieter, and NNSigmaFloor=0.05 is the SAME mutation step for
// every weight. Reaching a useful weight on a quiet slot is therefore a long
// random walk whose every intermediate step is fitness-neutral: no gradient, no
// selection, no arrival. The weight data shows exactly that — across t5 gens
// 1→475 every input's investment sat at ~1.0 and eff_rank went 11.2 → 10.9. The
// network never differentiated between its inputs AT ALL.
//
// ⭐ So this does not move the optimum, it makes terms REACHABLE. 041's premise
// was "the policy cannot observe its own energy"; we added SPECIFIC_ENERGY and
// left it below the search's own noise floor. Symptom: throttle pegged 99.3% of
// ticks (prior M1: 37.7%) — with no usable energy signal, "more thrust is never
// worse" is correct.
//
// ⚠️ 041 P2-2 made this worse rather than better: it "unified" AIRSPEED on RAW,
// which discarded the cruise-normalization 030 M11.preA.2 had deliberately given
// tracker mode (see the block above). This restores it and applies the same
// treatment to the other three, so the shared block is consistent in BOTH modes.
//
// Scales are p95 of |x| over that same t5 tick set, rounded — the derivation
// already used for kEnergyScale_m and kScoreGradScale. AIRSPEED reuses the
// existing physical constant rather than adding a fifth.
//     DIST         p95 25.87 →  26.0 m
//     CLOSING_RATE p95 15.93 →  16.0 m/s
//     GYRO         p95  6.17 →   6.0 rad/s
//     AIRSPEED     ÷ kCruiseSpeed_mps (13.0) — chase at cruise ⇒ 1.0
//
// ⛔ BREAKING for trained weights, deliberately. Genomes trained against raw
// dist/airspeed/gyro are NOT portable across this change (greenfield, no
// backward compat per project policy). Layout is UNCHANGED — still float[45] /
// float[66] — so no schema bump and no nn2cpp count change.
constexpr float kTargetDistScale_m    = 26.0f;
constexpr float kClosingRateScale_mps = 16.0f;
constexpr float kGyroScale_radps      = 6.0f;

// 041 T032 — ACCEL_* normalization. Body specific force is divided by this, so
// the channel carries "g's, scaled". Sized from the observed envelope rather
// than a round number: the standing flight record is +11.2 g / -8.4 g, and
// loads have crept up flight over flight. At 8 g the raw ratio is 1.0, which
// keeps the common flight regime (0..3 g) inside [0, 0.4] where tanh is close
// to linear, while an 11 g excursion reaches 1.4 -- large and clearly distinct
// without saturating the unit. A scale of 1 g would put every manoeuvre deep in
// tanh saturation and destroy exactly the resolution the load axis needs.
constexpr float kAccelScale_g = 8.0f;

// 041 P0-3 (2026-08-18) — SPECIFIC_ENERGY normalization, in metres of energy
// height. Es = h_hd + v²/2g, where h_hd is height above the arena FLOOR (the
// hard deck) and NEVER AGL. That datum choice is not cosmetic: in sim
// `checkArenaBounds` derives altitude from a ground plane the sim knows about,
// while in flight `resolveEngageArena` builds the band from the engage point
// because the aircraft has no ground reference. An AGL input would therefore
// have been a sim-only quantity. Both sides carry an explicit floor, so
// "height above the floor" is the one definition that means the same thing in
// both — and negative Es (below the deck) is MEANINGFUL, not an error to clamp.
//
// Sized like kAccelScale_g, from the record rather than for tidiness: the 041
// band is 100 m (arena 10..110 m AGL) and the maximum kinetic term MEASURED
// over 131,127 t1 ticks is 44.41 m ⇒ 144.41, rounded up. Those same ticks
// remapped into the new frame put realized Es in [0.39, 0.88] — inside the
// unit, unsaturated, with headroom above the observed maximum.
constexpr float kEnergyScale_m = 145.0f;

// 041 P0-3 — SCORE_GRAD_* normalization, in INVERSE metres (the slot is a
// spatial gradient of a dimensionless score, so its natural unit is 1/m).
//
// = the in-envelope p95 of |∇score| × streak multiplier over the same 131,127
// t1 ticks. In-envelope, not pooled: 84% of ticks are nowhere near the cone,
// and sizing against gradients the policy cannot act on would have picked a
// constant ~4× too small. At this divisor the in-envelope median (0.2016) reads
// 0.26 and 95% of in-envelope ticks fall below tanh(1).
//
// ⚠️ USE THE tanh-OF-NORM FORM, NOT A PLAIN DIVIDE. |∇θ| = 1/d diverges as the
// chase closes on the rabbit: every measured tick above 2.0 had d < 1.52 m
// (median 0.23 m). The quantity is genuinely unbounded, so a divide alone would
// hand the network an occasional 19.0. And apply tanh to the NORM, keeping the
// unit direction — per-component tanh would bound the slot while ROTATING the
// vector, and the direction is the entire content of this input:
//     v_body = ĝ_body · tanh(|∇score| · mult / kScoreGradScale)
// (Same idiom as DIST_TO_BOUNDARY's tanh(d / kDistToBoundaryScale_m).)
constexpr float kScoreGradScale = 0.78f;

// 038 P0-D FR-P0H (A) — time-since-seen normalization. Blind-tick count × dt
// (seconds) fed through tanh(t / kTimeSinceSeenScale_s): 1 s → 0.46, 2 s →
// 0.76, 4 s → 0.96. Gives the tracker NN a graded "how long since the target
// was last visible" signal that saturates within a few seconds of blindness;
// 0.0 exactly when a beacon is visible this tick.
constexpr float kTimeSinceSeenScale_s = 2.0f;

// ============================================================================
// 038 US3 — auxiliary span/closure predictor head (TRACKER-ONLY).
// ----------------------------------------------------------------------------
// The aux NN outputs predict the beacon-pair `span` (closure) at fixed
// MILLISECOND horizons — NOT raw bearing positions (Clarifications 2026-07-05):
// span is ≈ invariant to camera rotation and moves only with range, so it IS
// the closure/overtake signal and needs no ego-motion subtraction; it predicts
// cleanly at the ~150 ms actuation-lag horizon that gives *actionable*
// anti-overrun lead. Horizons are defined in MS (cadence-invariant, mirroring
// kNNHistoryLagsMsec) so a cadence change (e.g. 20→25 Hz) re-derives the tick
// offsets from the fixed physical horizons instead of silently shrinking the
// lookahead. Tick derivation + integral-at-cadence static_assert live at the
// consumer (src/eval/fitness_decomposition.cc), same pattern as historyLagTicks.
//
// Aux output vector (kNumSpanAuxOutputs = 4): predicted span at +50/+100/+150 ms
// (kSpanPredictHorizonsMsec) + a predicted closure rate (span-rate, rad/s).
// Scored on a SEPARATE lexicase axis (prediction_score) vs the realized span,
// must beat the persistence baseline (SC-002). NOT actuated.
constexpr int kSpanPredictHorizonsMsec[] = {50, 100, 150};
constexpr int kNumSpanPredictHorizons =
    sizeof(kSpanPredictHorizonsMsec) / sizeof(kSpanPredictHorizonsMsec[0]);
constexpr int kNumSpanAuxOutputs = kNumSpanPredictHorizons + 1;  // + 1 span-rate = 4

// ============================================================================
// 037 R5/T021 — History time-basis (ms-based, log-spaced lags)
// ----------------------------------------------------------------------------
// History lags are defined in MILLISECONDS, log-spaced (octaves), oldest
// first; the slot order matches the *_TM5..NOW array layout below. Tick
// offsets derive at the consumer (lagMsec / SIM_TIME_STEP_MSEC,
// static_assert-ed integral in aircraft_state.h).
//
// t10 (2026-06-13): window HALVED 1.6 s → 0.8 s (operator: 1.6 s is an
// eternity for this regime, even in patrol) AND the finest slot tightened
// 100 ms → 50 ms for close-in LOS-rate resolution. The 50 ms finest slot
// only divides integer ticks at >=20 Hz, so this set is NOT 10 Hz-compatible
// (a 10 Hz build trips historyLagsIntegral — intended):
//   50 ms tick (20 Hz) → ticks {16,8,4,2,1,0}   ← t10 operating point
//   25 ms tick (40 Hz) → ticks {32,16,8,4,2,0}
//   10 ms tick (100 Hz)→ ticks {80,40,20,10,5,0}
// Derivatives (closing_rate, span_rate) divide by the NOW↔TM1 gap
// (kNNHistoryRecentGapSec, now 50 ms), never an implicit one-tick assumption.
constexpr int kNNHistoryLagsMsec[6] = {800, 400, 200, 100, 50, 0};
constexpr float kNNHistoryRecentGapSec =
    static_cast<float>(kNNHistoryLagsMsec[4] - kNNHistoryLagsMsec[5]) / 1000.0f;

// 037 T023 — fail-loud layout marker (Principle V). The slot COUNT did not
// change at the R5 re-lag (NN_INPUT_COUNT stays 33, TrackerInput::COUNT
// stays 54), so the existing count checks cannot catch the semantic
// mismatch; this version is serialized in the per-state NN block
// (aircraft_state.h) and checked on read.
//   v1 = uniform one-tick lags {5,4,3,2,1,0} (pre-037, 100 ms grid)
//   v2 = ms-based log-spaced lags {1600,800,400,200,100,0} (1.6 s window)
//   v3 = ms-based log-spaced lags {800,400,200,100,50,0}   (t10, 0.8 s window)
constexpr uint32_t kNNHistoryLayoutVersion = 3;

// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and sim_response.py. Reordering fields is a format-breaking change.
// DO NOT add padding or non-float members.

// ============================================================================
// 041 P2-1 — CraftCommonInputs: the block M1 and M2 SHARE, defined ONCE.
// ----------------------------------------------------------------------------
// Operator 2026-08-17: *"sensor inputs to m1 and m2 are the same except for
// target representation."* That is exactly true, and until now it was expressed
// by declaring the same 17 slots twice — in `NNInputs` and in `TrackerInputs` —
// so a change to what `ACCEL_Z` MEANS had to be made in two unrelated structs
// or the modes silently diverged. This is the parallel-definition hazard the
// index-coupling inventory exists to retire.
//
// ⚠️ THE TWO STRUCTS HAD ALREADY DIVERGED, and the refactor is what exposed it:
// `AIRSPEED` was RAW m/s in M1 and CRUISE-NORMALIZED (÷ kCruiseSpeed_mps) in M2.
// A "shared" slot with two different scales. Unified here on RAW m/s per the
// input-vector proposal's shared-block table. M2 genomes retrain across this
// boundary anyway (greenfield, no backward compat), so the fix is free now and
// would not have been later.
//
// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and analysis tooling. This sub-struct is embedded as the LAST member
// of both mode structs, so each mode's layout is [target representation][common]
// and the common block occupies the final 20 slots in both.
// DO NOT add padding or non-float members.
struct CraftCommonInputs {  // raw-ok: NN-byte-format struct, all members fp32 by xiao-firmware-locked contract
    float quat_w;            // raw-ok: NN-byte-format buffer — attitude q_EB, aerospace FRD, unit norm
    float quat_x;            // raw-ok: NN-byte-format buffer
    float quat_y;            // raw-ok: NN-byte-format buffer
    float quat_z;            // raw-ok: NN-byte-format buffer
    float airspeed;          // raw-ok: NN-byte-format buffer — ÷ kCruiseSpeed_mps (041 P2-8)
    float gyro_p;            // raw-ok: NN-byte-format buffer — rad/s ÷ kGyroScale_radps, body, aerospace RHR
    float gyro_q;            // raw-ok: NN-byte-format buffer
    float gyro_r;            // raw-ok: NN-byte-format buffer
    // The other half of the IMU. Body-frame specific force INCLUDING gravity
    // (a_world - g_world rotated into body), scaled by kAccelScale_g — NOT FDM
    // kinematic acceleration, which would put a constant ~1 g offset in the most
    // load-relevant axis. autoc/eval/specific_force.h is the one definition,
    // shared with dmp-dump so reader and input cannot disagree.
    float accel_x;           // raw-ok: NN-byte-format buffer — longitudinal
    float accel_y;           // raw-ok: NN-byte-format buffer — lateral
    float accel_z;           // raw-ok: NN-byte-format buffer — normal, the load axis
    // ----- 041 P2-2 — the observation the energy objective had no way to see --
    // TA03 found the vector carried AIRSPEED but no altitude/height term at all,
    // so Es = h + v²/2g was UNOBSERVABLE: the policy had the v² half and never
    // the h half. That is the mechanical explanation for 035's energy objective
    // muting the whole regiment rather than trimming waste. Es is given as the
    // STATE, not only Ps the rate — the integral is what a recurrent layer would
    // otherwise have to accumulate, and its capacity is already unfilled
    // (effective rank 11.3 of 16).
    float specific_energy;   // raw-ok: NN-byte-format buffer — (h_hd + v²/2g) / kEnergyScale_m
    // Outward radial velocity (p·v)/‖p‖ over the horizontal radius, normalized
    // by kCruiseSpeed_mps. POSITIVE = closing on the wall.
    //
    // ⚠️ This REPLACES the retracted time-to-boundary idea, and the retraction
    // matters: a straight-line time-to-impact is a systematically wrong input
    // for this vehicle, which turns 66.6% of ticks at a radius under 20 m in an
    // 80 m arena — a 3-second ray reaches 46 m while the craft turns ~225°.
    // The derivative makes NO trajectory assumption at all. It is also
    // informative exactly where DIST_TO_BOUNDARY is blind: on the 83% of ticks
    // where distance is saturated above 0.99 (std 0.042), closure rate still
    // spans −17.3 to +16.5 m/s.
    //
    // Cadence-invariant BY CONSTRUCTION — it is an instantaneous dot product of
    // position and velocity, so no dt appears anywhere in it and there is no
    // one-tick assumption to re-derive at another rate.
    float boundary_closure_rate;  // raw-ok: NN-byte-format buffer — (p̂·v)/kCruiseSpeed_mps, + = outward
    // Position to the closure rate's rate — kept, NOT replaced. Ablation settled
    // it: −40.7% pooled / −25.0% on path 5, the third most important input in
    // the whole vector. The two are complementary (where the wall is vs whether
    // it is getting closer); the saturation problem is fixed by ADDING the rate,
    // not by removing the distance.
    float dist_to_boundary;  // raw-ok: NN-byte-format buffer — tanh(d/kDistToBoundaryScale_m), [0,1), along-velocity ray
    float inward_body_x;     // raw-ok: NN-byte-format buffer — chase_quat.conjugate()*normalize(center-pos), body frame
    float inward_body_y;     // raw-ok: NN-byte-format buffer
    float inward_body_z;     // raw-ok: NN-byte-format buffer
    // ----- 041 P2-2 — the improvement DIRECTION, replacing the envelope pair --
    // Operator: *"streak was a crude proxy for rewarding in-track range."* A
    // binary flag is a STATE LABEL — a controller can only switch on it. A
    // gradient is an IMPROVEMENT DIRECTION: which way to move, in body axes, to
    // score more, weighted by the streak multiplier so it carries how much
    // reward is currently at stake. Analytically available from the Lorentzian
    // in FitnessComputer::decomposeStepScore, so nothing is estimated in M1.
    //
    // ⚠️ Shared SLOT, mode- AND PHASE-specific SOURCE. Exact closed form for M1
    // sim, M1 flight (the xiao knows the virtual target) and M2 flight phase 1
    // (virtual target + synthetic camera). Only at M2 phase 2 — a real craft
    // with real beacons — must it be proxied from camera range estimation, and
    // that is a 043 study deferred BEHIND phase 1, not a gate on it. Feeding the
    // true gradient to a tracker that cannot see it in the air would be an
    // oracle, exactly what T038 refused for the M2 envelope estimator.
    float score_grad_x;      // raw-ok: NN-byte-format buffer — direction-preserving tanh, see kScoreGradScale
    float score_grad_y;      // raw-ok: NN-byte-format buffer
    float score_grad_z;      // raw-ok: NN-byte-format buffer
};

static_assert(sizeof(CraftCommonInputs) == 20 * sizeof(float),
              "CraftCommonInputs must be contiguous float[20] with no padding "
              "(4 quat + 1 airspeed + 3 gyro + 3 accel + 1 Es + 1 closure-rate "
              "+ 1 dist-to-boundary + 3 inward + 3 score-grad)");
static_assert(alignof(CraftCommonInputs) == alignof(float),
              "CraftCommonInputs must be float-aligned for matrix multiply");

constexpr int NN_COMMON_INPUT_COUNT = sizeof(CraftCommonInputs) / sizeof(float);

// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and sim_response.py. Reordering fields is a format-breaking change.
// DO NOT add padding or non-float members.

struct NNInputs {
    // ----- M1 TARGET REPRESENTATION (25 slots) -----
    // Time samples: [-0.8s, -0.4s, -0.2s, -0.1s, -0.05s, now]
    // (kNNHistoryLagsMsec — t10 0.8 s log-spaced lags; past-only per
    // 029 US1, no future lookahead. The *_TM5..NOW slot NAMES are lag-slot
    // indices, not tick counts.)
    float target_x[6];   // body-frame unit-vec x component (was dPhi)
    float target_y[6];   // body-frame unit-vec y component (was dTheta partial)
    float target_z[6];   // body-frame unit-vec z component (NEW)
    float dist[6];       // m ÷ kTargetDistScale_m (041 P2-8), distance to rabbit
    float closing_rate;  // m/s ÷ kClosingRateScale_mps (041 P2-8), + = approaching
    // ----- CRAFT STATE (20 slots) — one definition, shared with TrackerInputs
    CraftCommonInputs common;
};

static_assert(sizeof(NNInputs) == 45 * sizeof(float),
              "NNInputs layout must be contiguous float[45] with no padding "
              "(25 M1 target representation + 20 CraftCommonInputs). History: "
              "33 pre-038; 37 at 038 FR-P0H; 42 at 041 US4; 45 at 041 P2-2 "
              "(-IN_ENVELOPE -ENVELOPE_SECS +SPECIFIC_ENERGY "
              "+BOUNDARY_CLOSURE_RATE +SCORE_GRAD_X/Y/Z)");
static_assert(alignof(NNInputs) == alignof(float),
              "NNInputs must be float-aligned for matrix multiply");
static_assert(offsetof(NNInputs, common) == 25 * sizeof(float),
              "CraftCommonInputs must start at slot 25 in NNInputs — the enum, "
              "the metadata table and the flat float view all assume it");

constexpr int NN_INPUT_COUNT = sizeof(NNInputs) / sizeof(float);

// ============================================================================
// 030 M2 — Type-safe NN sensor interface (FR-006 + FR-019)
// ----------------------------------------------------------------------------
// Two enums name the per-mode NN input slots. Values are indices into the
// flat float array (a typed view over NNInputs / future TrackerInputs storage).
// Enum order MUST match the underlying struct field order (= cereal byte
// order). The COUNT trailer terminates the enum and serves as the slot count.
//
// Pluggable mode dispatch (FR-019): one autoc binary supports both modes;
// the active mode is selected at startup by config-file path (autoc.ini →
// pathgen, autoc-tracker.ini → tracker). The NN forward-pass code is
// mode-agnostic (consumes a flat float[N] of typed dimensionality
// N = ModeInput::COUNT).
//
// Xiao firmware uses preprocessor-driven compile-time mode selection
// (-DAUTOC_MODE=PATHGEN | =TRACKER) so only the active mode's enum compiles
// in — runtime branching avoided on the embedded target.
// ============================================================================

struct SensorInputMeta {
    const char* name;          // enum name as string (e.g. "TARGET_X_TM5")
    const char* display_name;  // data.dat header column label (e.g. "tgX-5")
    int header_width;          // data.dat column width incl. leading separator
                               // space (matches the corresponding `% N.Mf`
                               // format spec in src/autoc.cc per-tick line:
                               // 7 = " % 6.Xf", 8 = " % 7.Xf"). Right-aligned
                               // header text uses this; static text and format
                               // string must remain mutually consistent.
};

// ----------------------------------------------------------------------------
// 041 P2-1/P2-2 — the two enums now share a TAIL, because the structs share a
// sub-struct. Both end with the same 20 CraftCommonInputs slots, in the same
// order, with the same names and the same semantics. Only the leading target
// representation differs: 25 slots for M1 (direction cosines + range), 46 for
// M2 (two points of light and what can be inferred from them).
//
// ⚠️ Keep the two COMMON tails textually identical. They are declared twice
// because C++ enums do not compose, not because they are allowed to differ —
// the struct behind them is one definition and the static_asserts below pin the
// counts, but nothing but review pins the ORDER. If they drift, the modes read
// each other's slots and every value is plausible.
// ----------------------------------------------------------------------------

// PathgenInput — 45 inputs: 25 M1 target representation + 20 craft common.
enum class PathgenInput : uint16_t {
    TARGET_X_TM5 = 0, TARGET_X_TM4, TARGET_X_TM3, TARGET_X_TM2, TARGET_X_TM1, TARGET_X_NOW,
    TARGET_Y_TM5,     TARGET_Y_TM4, TARGET_Y_TM3, TARGET_Y_TM2, TARGET_Y_TM1, TARGET_Y_NOW,
    TARGET_Z_TM5,     TARGET_Z_TM4, TARGET_Z_TM3, TARGET_Z_TM2, TARGET_Z_TM1, TARGET_Z_NOW,
    DIST_TM5,         DIST_TM4,     DIST_TM3,     DIST_TM2,     DIST_TM1,     DIST_NOW,
    CLOSING_RATE,
    // ----- CraftCommonInputs (25..44) — shared tail, see note above -----
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    ACCEL_X, ACCEL_Y, ACCEL_Z,
    SPECIFIC_ENERGY,
    BOUNDARY_CLOSURE_RATE,
    DIST_TO_BOUNDARY,
    INWARD_BODY_X, INWARD_BODY_Y, INWARD_BODY_Z,
    SCORE_GRAD_X, SCORE_GRAD_Y, SCORE_GRAD_Z,
    COUNT
};

// The shared tail's metadata, written once and spliced into both tables by the
// CRAFT_COMMON_INPUT_META macro. Same reason as the struct: a display name or a
// column width that differs between modes is a silent reporting bug.
#define CRAFT_COMMON_INPUT_META                                    \
    {"QUAT_W", "qw", 8}, {"QUAT_X", "qx", 8},                      \
    {"QUAT_Y", "qy", 8}, {"QUAT_Z", "qz", 8},                      \
    {"AIRSPEED", "vel", 8},                                        \
    {"GYRO_P", "gyrP", 7}, {"GYRO_Q", "gyrQ", 7}, {"GYRO_R", "gyrR", 7}, \
    {"ACCEL_X", "acX", 7}, {"ACCEL_Y", "acY", 7}, {"ACCEL_Z", "acZ", 7}, \
    {"SPECIFIC_ENERGY", "Es", 8},                                  \
    {"BOUNDARY_CLOSURE_RATE", "bClR", 8},                          \
    {"DIST_TO_BOUNDARY", "dBnd", 8},                               \
    {"INWARD_BODY_X", "inX", 7}, {"INWARD_BODY_Y", "inY", 7},      \
    {"INWARD_BODY_Z", "inZ", 7},                                   \
    {"SCORE_GRAD_X", "sgX", 7}, {"SCORE_GRAD_Y", "sgY", 7},        \
    {"SCORE_GRAD_Z", "sgZ", 7}

constexpr SensorInputMeta kPathgenInputMeta[] = {
    {"TARGET_X_TM5", "tgX-5", 7}, {"TARGET_X_TM4", "tgX-4", 7}, {"TARGET_X_TM3", "tgX-3", 7},
    {"TARGET_X_TM2", "tgX-2", 7}, {"TARGET_X_TM1", "tgX-1", 7}, {"TARGET_X_NOW", "tgX0",  7},
    {"TARGET_Y_TM5", "tgY-5", 7}, {"TARGET_Y_TM4", "tgY-4", 7}, {"TARGET_Y_TM3", "tgY-3", 7},
    {"TARGET_Y_TM2", "tgY-2", 7}, {"TARGET_Y_TM1", "tgY-1", 7}, {"TARGET_Y_NOW", "tgY0",  7},
    {"TARGET_Z_TM5", "tgZ-5", 7}, {"TARGET_Z_TM4", "tgZ-4", 7}, {"TARGET_Z_TM3", "tgZ-3", 7},
    {"TARGET_Z_TM2", "tgZ-2", 7}, {"TARGET_Z_TM1", "tgZ-1", 7}, {"TARGET_Z_NOW", "tgZ0",  7},
    {"DIST_TM5",     "ds-5",  7}, {"DIST_TM4",     "ds-4",  7}, {"DIST_TM3",     "ds-3",  7},
    {"DIST_TM2",     "ds-2",  7}, {"DIST_TM1",     "ds-1",  7}, {"DIST_NOW",     "ds0",   7},
    {"CLOSING_RATE", "dd/dt", 7},
    CRAFT_COMMON_INPUT_META,
};

static_assert(static_cast<size_t>(PathgenInput::COUNT) ==
              sizeof(kPathgenInputMeta) / sizeof(SensorInputMeta),
              "PathgenInput enum count must match kPathgenInputMeta length");
static_assert(static_cast<int>(PathgenInput::COUNT) == NN_INPUT_COUNT,
              "PathgenInput::COUNT must equal NN_INPUT_COUNT (NNInputs struct size)");
static_assert(static_cast<int>(PathgenInput::QUAT_W) ==
              static_cast<int>(PathgenInput::COUNT) - NN_COMMON_INPUT_COUNT,
              "The CraftCommonInputs block must be the LAST 20 slots of "
              "PathgenInput — offsetof(NNInputs, common) says so and the flat "
              "float view the forward pass uses depends on it");

// ----------------------------------------------------------------------------
// TrackerInput — 66 inputs: 46 M2 target representation + 20 craft common.
//
// M2 target representation (46):
//   36 beacon (left + right x 6 history slots x {x, y, CEP})
//    9 032 phase-1 derived (span x6 + span_rate + tilt sin/cos)
//    1 TIME_SINCE_SEEN — the target-lost cue. Tracker-only: M1's rabbit is
//      never invisible, so this belongs to the TARGET block, not the shared one.
//
// ⚠️ 041 P2-1 REORDERED THIS ENUM. Pre-041 the shared craft slots were SPLIT by
// the target block (quat/airspeed/gyro/boundary at 36..44, then span and tilt,
// then inward/envelope/accel at 54..62). A shared sub-struct cannot be
// non-contiguous, so the craft slots are now gathered into one tail. Legal
// inside this format break, and M2 genomes retrain across it regardless.
//
// Also folded in: DIST_TO_BOUNDARY_ALONG_VEL is now just DIST_TO_BOUNDARY. The
// two names described the SAME quantity computed by the SAME function — M1's
// gather calls autoc::eval::distanceToBoundary along velocity too — and two
// names for one thing is the hazard this refactor exists to retire.
//
// M2 has NO direct distance and NO direct bearing to the target: range is
// inferred from SPAN (wider = closer), which is why span and its rate carry
// what DIST_*/CLOSING_RATE carry in M1. That is the whole M1↔M2 difference —
// M1 is TOLD where the target is; M2 must infer it from two points of light.
// ----------------------------------------------------------------------------
enum class TrackerInput : uint16_t {
    BEACON_L_X_TM5 = 0, BEACON_L_X_TM4, BEACON_L_X_TM3, BEACON_L_X_TM2, BEACON_L_X_TM1, BEACON_L_X_NOW,
    BEACON_L_Y_TM5,     BEACON_L_Y_TM4, BEACON_L_Y_TM3, BEACON_L_Y_TM2, BEACON_L_Y_TM1, BEACON_L_Y_NOW,
    BEACON_L_CEP_TM5,   BEACON_L_CEP_TM4, BEACON_L_CEP_TM3, BEACON_L_CEP_TM2, BEACON_L_CEP_TM1, BEACON_L_CEP_NOW,
    BEACON_R_X_TM5,     BEACON_R_X_TM4, BEACON_R_X_TM3, BEACON_R_X_TM2, BEACON_R_X_TM1, BEACON_R_X_NOW,
    BEACON_R_Y_TM5,     BEACON_R_Y_TM4, BEACON_R_Y_TM3, BEACON_R_Y_TM2, BEACON_R_Y_TM1, BEACON_R_Y_NOW,
    BEACON_R_CEP_TM5,   BEACON_R_CEP_TM4, BEACON_R_CEP_TM3, BEACON_R_CEP_TM2, BEACON_R_CEP_TM1, BEACON_R_CEP_NOW,
    // ----- 032 phase-1 derived perceptual features (36..44) -----
    BEACON_PAIR_SPAN_TM5, BEACON_PAIR_SPAN_TM4, BEACON_PAIR_SPAN_TM3,
    BEACON_PAIR_SPAN_TM2, BEACON_PAIR_SPAN_TM1, BEACON_PAIR_SPAN_NOW,
    SPAN_RATE,
    TARGET_TILT_SIN,
    TARGET_TILT_COS,
    // ----- 038 FR-P0H (A) target-lost cue (45) — tracker-only -----
    // (038 US3, 2026-07-05: EXIT_DIR_SIN/COS removed — redundant with the beacon
    //  history inside the 0.8 s window, stale beyond it; low-value hand-built cue.)
    TIME_SINCE_SEEN,
    // ----- CraftCommonInputs (46..65) — shared tail, identical to PathgenInput
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    ACCEL_X, ACCEL_Y, ACCEL_Z,
    SPECIFIC_ENERGY,
    BOUNDARY_CLOSURE_RATE,
    DIST_TO_BOUNDARY,
    INWARD_BODY_X, INWARD_BODY_Y, INWARD_BODY_Z,
    SCORE_GRAD_X, SCORE_GRAD_Y, SCORE_GRAD_Z,
    COUNT
};

constexpr SensorInputMeta kTrackerInputMeta[] = {
    {"BEACON_L_X_TM5", "blX-5", 7},   {"BEACON_L_X_TM4", "blX-4", 7},   {"BEACON_L_X_TM3", "blX-3", 7},
    {"BEACON_L_X_TM2", "blX-2", 7},   {"BEACON_L_X_TM1", "blX-1", 7},   {"BEACON_L_X_NOW", "blX0",  7},
    {"BEACON_L_Y_TM5", "blY-5", 7},   {"BEACON_L_Y_TM4", "blY-4", 7},   {"BEACON_L_Y_TM3", "blY-3", 7},
    {"BEACON_L_Y_TM2", "blY-2", 7},   {"BEACON_L_Y_TM1", "blY-1", 7},   {"BEACON_L_Y_NOW", "blY0",  7},
    {"BEACON_L_CEP_TM5", "blC-5", 7}, {"BEACON_L_CEP_TM4", "blC-4", 7}, {"BEACON_L_CEP_TM3", "blC-3", 7},
    {"BEACON_L_CEP_TM2", "blC-2", 7}, {"BEACON_L_CEP_TM1", "blC-1", 7}, {"BEACON_L_CEP_NOW", "blC0",  7},
    {"BEACON_R_X_TM5", "brX-5", 7},   {"BEACON_R_X_TM4", "brX-4", 7},   {"BEACON_R_X_TM3", "brX-3", 7},
    {"BEACON_R_X_TM2", "brX-2", 7},   {"BEACON_R_X_TM1", "brX-1", 7},   {"BEACON_R_X_NOW", "brX0",  7},
    {"BEACON_R_Y_TM5", "brY-5", 7},   {"BEACON_R_Y_TM4", "brY-4", 7},   {"BEACON_R_Y_TM3", "brY-3", 7},
    {"BEACON_R_Y_TM2", "brY-2", 7},   {"BEACON_R_Y_TM1", "brY-1", 7},   {"BEACON_R_Y_NOW", "brY0",  7},
    {"BEACON_R_CEP_TM5", "brC-5", 7}, {"BEACON_R_CEP_TM4", "brC-4", 7}, {"BEACON_R_CEP_TM3", "brC-3", 7},
    {"BEACON_R_CEP_TM2", "brC-2", 7}, {"BEACON_R_CEP_TM1", "brC-1", 7}, {"BEACON_R_CEP_NOW", "brC0",  7},
    {"BEACON_PAIR_SPAN_TM5", "spn-5", 7}, {"BEACON_PAIR_SPAN_TM4", "spn-4", 7},
    {"BEACON_PAIR_SPAN_TM3", "spn-3", 7}, {"BEACON_PAIR_SPAN_TM2", "spn-2", 7},
    {"BEACON_PAIR_SPAN_TM1", "spn-1", 7}, {"BEACON_PAIR_SPAN_NOW", "spn0",  7},
    {"SPAN_RATE",            "dspn",  7},
    {"TARGET_TILT_SIN",      "tltS",  7},
    {"TARGET_TILT_COS",      "tltC",  7},
    {"TIME_SINCE_SEEN",      "tSee",  7},
    CRAFT_COMMON_INPUT_META,
};

static_assert(static_cast<size_t>(TrackerInput::COUNT) ==
              sizeof(kTrackerInputMeta) / sizeof(SensorInputMeta),
              "TrackerInput enum count must match kTrackerInputMeta length");
static_assert(static_cast<int>(TrackerInput::COUNT) == 66,
              "TrackerInput::COUNT must equal 66 (46 M2 target representation + "
              "20 CraftCommonInputs). History: 58 pre-041; 63 at 041 US4; 66 at "
              "041 P2-2. ⚠️ This count changes ONCE MORE after the M2 phase "
              "(66 + N innovation channels, FR-005a) -- that second change is "
              "legal, see contracts/nn-input-layout.md");

// 030 M6d — Tracker-mode NN input storage struct (FR-006 + FR-016).
//
// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and analysis tooling. Reordering fields is a format-breaking
// change. DO NOT add padding or non-float members. Layout MUST match
// TrackerInput enum order so reinterpret_cast<float*>(&trackerInputs)
// agrees with enum-indexed access for the NN forward pass + the data.dat
// header walk via kTrackerInputMeta.
//
// Time samples: [-0.8s, -0.4s, -0.2s, -0.1s, -0.05s, now] — kNNHistoryLagsMsec
// (t10 0.8 s log-spaced lags; slot names are lag-slot indices).
struct TrackerInputs {  // raw-ok: NN-byte-format struct, all members fp32 by xiao-firmware-locked contract
    // ----- M2 TARGET REPRESENTATION (46 slots) -----
    float beacon_l_x[6];     // raw-ok: NN-byte-format buffer (per Principle VI whitelist: NN-byte-format buffers)
    float beacon_l_y[6];     // raw-ok: NN-byte-format buffer
    float beacon_l_cep[6];   // raw-ok: NN-byte-format buffer
    float beacon_r_x[6];     // raw-ok: NN-byte-format buffer
    float beacon_r_y[6];     // raw-ok: NN-byte-format buffer
    float beacon_r_cep[6];   // raw-ok: NN-byte-format buffer

    // ----- 032 PHASE 1 — derived perceptual features (FR per spec.md §1.5) -----
    // beacon_pair_span: raw Euclidean distance between port + starboard
    // beacon bearings, in RADIANS as of 040 T031 — `sqrt(dx² + dy²)`
    // with no scaling, no normalization, no clipping. Same units as
    // BeaconObservation::screen_x/y. 6-slot history at kNNHistoryLagsMsec.
    // Substituted to 0.0 per-tick when EITHER beacon's CEP at "now"
    // >= CepGateThreshold.
    float beacon_pair_span[6];   // raw-ok: NN-byte-format buffer
    // span_rate: signed RATE (rad/s) over the NOW↔TM1 lag gap =
    // (span[5] - span[4]) / kNNHistoryRecentGapSec. 037 T022 changed this
    // from a raw one-tick diff to a true per-second rate (cadence- and
    // lag-invariant); M2 genomes retrain from scratch at the boundary.
    float span_rate;             // raw-ok: NN-byte-format buffer
    // target_tilt encoded as (sin θ, cos θ) where θ = atan2(y_r-y_l, x_r-x_l)
    // over bearings. θ = 0 ⇔ port→starboard line is horizontal in image plane (chase
    // and target wings level relative). Substituted to (0, 1) per-tick when
    // CEP-gated OR when beacon pair is geometrically degenerate (<1e-4 rad).
    float target_tilt_sin;       // raw-ok: NN-byte-format buffer
    float target_tilt_cos;       // raw-ok: NN-byte-format buffer
    // 038 FR-P0H (A) target-lost cue — TRACKER-ONLY, hence part of the target
    // representation and not of CraftCommonInputs. Stateful (held/decaying),
    // reset per scenario/engage in the stepper.
    float time_since_seen;   // raw-ok: NN-byte-format buffer — tanh(ticks_lost·scale), [0,1); 0 = visible now

    // ----- CRAFT STATE (20 slots) — one definition, shared with NNInputs -----
    // The RNN receives identical input channels in both modes; only the SOURCE
    // of a channel may differ (see CraftCommonInputs for which, and why).
    CraftCommonInputs common;
};

static_assert(sizeof(TrackerInputs) == 66 * sizeof(float),
              "TrackerInputs layout must be contiguous float[66] with no padding "
              "(46 M2 target representation + 20 CraftCommonInputs)");
static_assert(alignof(TrackerInputs) == alignof(float),
              "TrackerInputs must be float-aligned for matrix multiply");
static_assert(static_cast<int>(TrackerInput::COUNT) ==
              sizeof(TrackerInputs) / sizeof(float),
              "TrackerInputs struct size must match TrackerInput::COUNT");
static_assert(offsetof(TrackerInputs, common) == 46 * sizeof(float),
              "CraftCommonInputs must start at slot 46 in TrackerInputs");
static_assert(static_cast<int>(TrackerInput::QUAT_W) ==
              static_cast<int>(TrackerInput::COUNT) - NN_COMMON_INPUT_COUNT,
              "The CraftCommonInputs block must be the LAST 20 slots of "
              "TrackerInput, exactly as it is of PathgenInput");

// ⚠️ The one assertion that pins the two enums' shared tails to each other.
// The structs are one definition; the enums are two, and nothing else stops
// them drifting apart. If they do, each mode reads the other's slot meanings
// and every resulting value looks entirely plausible.
static_assert(
    static_cast<int>(TrackerInput::SCORE_GRAD_Z) - static_cast<int>(TrackerInput::QUAT_W) ==
    static_cast<int>(PathgenInput::SCORE_GRAD_Z) - static_cast<int>(PathgenInput::QUAT_W),
    "PathgenInput and TrackerInput must lay out CraftCommonInputs identically");
