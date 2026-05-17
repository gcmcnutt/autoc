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

// CONSTITUTIONAL NOTE -- SERIALIZATION CONTRACT
// Field declaration order IS the on-disk byte order for cereal, data.dat,
// nn2cpp, and sim_response.py. Reordering fields is a format-breaking change.
// DO NOT add padding or non-float members.

struct NNInputs {
    // Time samples: [-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now]
    // 029 US1: past-only distribution, 100 ms uniform grid (no future lookahead).
    float target_x[6];   // body-frame unit-vec x component (was dPhi)
    float target_y[6];   // body-frame unit-vec y component (was dTheta partial)
    float target_z[6];   // body-frame unit-vec z component (NEW)
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

static_assert(sizeof(NNInputs) == 33 * sizeof(float),
              "NNInputs layout must be contiguous float[33] with no padding");
static_assert(alignof(NNInputs) == alignof(float),
              "NNInputs must be float-aligned for matrix multiply");

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
// PathgenInput — 33 inputs, layout matches NNInputs struct field order.
// ----------------------------------------------------------------------------
enum class PathgenInput : uint16_t {
    TARGET_X_TM5 = 0, TARGET_X_TM4, TARGET_X_TM3, TARGET_X_TM2, TARGET_X_TM1, TARGET_X_NOW,
    TARGET_Y_TM5,     TARGET_Y_TM4, TARGET_Y_TM3, TARGET_Y_TM2, TARGET_Y_TM1, TARGET_Y_NOW,
    TARGET_Z_TM5,     TARGET_Z_TM4, TARGET_Z_TM3, TARGET_Z_TM2, TARGET_Z_TM1, TARGET_Z_NOW,
    DIST_TM5,         DIST_TM4,     DIST_TM3,     DIST_TM2,     DIST_TM1,     DIST_NOW,
    CLOSING_RATE,
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    COUNT
};

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
    {"QUAT_W", "qw", 8}, {"QUAT_X", "qx", 8}, {"QUAT_Y", "qy", 8}, {"QUAT_Z", "qz", 8},
    {"AIRSPEED", "vel", 8},
    {"GYRO_P", "gyrP", 7}, {"GYRO_Q", "gyrQ", 7}, {"GYRO_R", "gyrR", 7},
};

static_assert(static_cast<size_t>(PathgenInput::COUNT) ==
              sizeof(kPathgenInputMeta) / sizeof(SensorInputMeta),
              "PathgenInput enum count must match kPathgenInputMeta length");
static_assert(static_cast<int>(PathgenInput::COUNT) == NN_INPUT_COUNT,
              "PathgenInput::COUNT must equal NN_INPUT_COUNT (NNInputs struct size)");

// ----------------------------------------------------------------------------
// TrackerInput — 45 inputs (FR-006 + FR-016 + Session 2026-05-07 Q1 simplification).
//   36 beacon (left + right × 6 history slots × {x, y, CEP})
//    8 aircraft state (quat 4 + airspeed 1 + gyro 3)
//    1 arena-awareness (DIST_TO_BOUNDARY_ALONG_VEL — single ray-projection
//      scalar shared with arena.h::distanceToBoundary())
// Was 48 in the 2026-05-04 shape (HOME_X/Y/Z + HOME_DIST); simplified per
// Session 2026-05-07 Q1 — cylinder-shaped arena needs cylinder-shaped
// signal, single source of truth with the OOB termination check.
// ----------------------------------------------------------------------------
enum class TrackerInput : uint16_t {
    BEACON_L_X_TM5 = 0, BEACON_L_X_TM4, BEACON_L_X_TM3, BEACON_L_X_TM2, BEACON_L_X_TM1, BEACON_L_X_NOW,
    BEACON_L_Y_TM5,     BEACON_L_Y_TM4, BEACON_L_Y_TM3, BEACON_L_Y_TM2, BEACON_L_Y_TM1, BEACON_L_Y_NOW,
    BEACON_L_CEP_TM5,   BEACON_L_CEP_TM4, BEACON_L_CEP_TM3, BEACON_L_CEP_TM2, BEACON_L_CEP_TM1, BEACON_L_CEP_NOW,
    BEACON_R_X_TM5,     BEACON_R_X_TM4, BEACON_R_X_TM3, BEACON_R_X_TM2, BEACON_R_X_TM1, BEACON_R_X_NOW,
    BEACON_R_Y_TM5,     BEACON_R_Y_TM4, BEACON_R_Y_TM3, BEACON_R_Y_TM2, BEACON_R_Y_TM1, BEACON_R_Y_NOW,
    BEACON_R_CEP_TM5,   BEACON_R_CEP_TM4, BEACON_R_CEP_TM3, BEACON_R_CEP_TM2, BEACON_R_CEP_TM1, BEACON_R_CEP_NOW,
    QUAT_W, QUAT_X, QUAT_Y, QUAT_Z,
    AIRSPEED,
    GYRO_P, GYRO_Q, GYRO_R,
    DIST_TO_BOUNDARY_ALONG_VEL,
    // ----- 032 PHASE 1 NEW SLOTS (45..53) -----
    BEACON_PAIR_SPAN_TM5, BEACON_PAIR_SPAN_TM4, BEACON_PAIR_SPAN_TM3,
    BEACON_PAIR_SPAN_TM2, BEACON_PAIR_SPAN_TM1, BEACON_PAIR_SPAN_NOW,
    SPAN_RATE,
    TARGET_TILT_SIN,
    TARGET_TILT_COS,
    COUNT
};

// Tracker meta header_widths are placeholders (7 each) — actual data.dat
// tracker-mode column widths land in M5 alongside the projection module.
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
    {"QUAT_W", "qw", 8}, {"QUAT_X", "qx", 8}, {"QUAT_Y", "qy", 8}, {"QUAT_Z", "qz", 8},
    {"AIRSPEED", "vel", 8},
    {"GYRO_P", "gyrP", 7}, {"GYRO_Q", "gyrQ", 7}, {"GYRO_R", "gyrR", 7},
    {"DIST_TO_BOUNDARY_ALONG_VEL", "dBnd", 8},
    // ----- 032 PHASE 1 NEW META (slots 45..53) -----
    {"BEACON_PAIR_SPAN_TM5", "spn-5", 7}, {"BEACON_PAIR_SPAN_TM4", "spn-4", 7},
    {"BEACON_PAIR_SPAN_TM3", "spn-3", 7}, {"BEACON_PAIR_SPAN_TM2", "spn-2", 7},
    {"BEACON_PAIR_SPAN_TM1", "spn-1", 7}, {"BEACON_PAIR_SPAN_NOW", "spn0",  7},
    {"SPAN_RATE",            "dspn",  7},
    {"TARGET_TILT_SIN",      "tltS",  7},
    {"TARGET_TILT_COS",      "tltC",  7},
};

static_assert(static_cast<size_t>(TrackerInput::COUNT) ==
              sizeof(kTrackerInputMeta) / sizeof(SensorInputMeta),
              "TrackerInput enum count must match kTrackerInputMeta length");
static_assert(static_cast<int>(TrackerInput::COUNT) == 54,
              "TrackerInput::COUNT must equal 54 per 032 phase 1 (36 beacon + 8 state + 1 arena + 9 derived)");

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
// Time samples: [-0.5s, -0.4s, -0.3s, -0.2s, -0.1s, now] — 100ms grid.
struct TrackerInputs {  // raw-ok: NN-byte-format struct, all members fp32 by xiao-firmware-locked contract
    float beacon_l_x[6];     // raw-ok: NN-byte-format buffer (per Principle VI whitelist: NN-byte-format buffers)
    float beacon_l_y[6];     // raw-ok: NN-byte-format buffer
    float beacon_l_cep[6];   // raw-ok: NN-byte-format buffer
    float beacon_r_x[6];     // raw-ok: NN-byte-format buffer
    float beacon_r_y[6];     // raw-ok: NN-byte-format buffer
    float beacon_r_cep[6];   // raw-ok: NN-byte-format buffer

    float quat_w, quat_x, quat_y, quat_z;  // raw-ok: NN-byte-format buffer
    float airspeed;                         // raw-ok: NN-byte-format buffer — M11.preA.2: cruise-normalized (relVel / kCruiseSpeed_mps), dimensionless ≈[0,2]
    float gyro_p, gyro_q, gyro_r;           // raw-ok: NN-byte-format buffer (rad/s, body-frame, aerospace RHR)

    // 1 arena-awareness input (FR-016 + Session 2026-05-07 Q1):
    // M11.preA.2 — soft-saturated tanh(d / kDistToBoundaryScale_m). Raw d
    // is the meters-along-velocity to nearest cylinder/floor/ceiling
    // intersection (autoc::eval::distanceToBoundary). Soft-sat keeps the
    // gradient sharp near the boundary and saturates at ~1 far from it;
    // the safety layer is a separate world-frame addition (deferred).
    float dist_to_boundary_along_vel;        // raw-ok: NN-byte-format buffer — dimensionless tanh, [0,1)

    // ----- 032 PHASE 1 — derived perceptual features (FR per spec.md §1.5) -----
    // beacon_pair_span: raw Euclidean distance between port + starboard
    // beacon centroids in the NDC (x, y) coordinate frame — `sqrt(dx² + dy²)`
    // with no scaling, no normalization, no clipping. Same units as
    // BeaconObservation::screen_x/y. 6-tick history (100ms grid).
    // Substituted to 0.0 per-tick when EITHER beacon's CEP at "now"
    // >= CepGateThreshold.
    float beacon_pair_span[6];   // raw-ok: NN-byte-format buffer
    // span_rate: one-tick raw diff = span[5] - span[4]. Signed.
    float span_rate;             // raw-ok: NN-byte-format buffer
    // target_tilt encoded as (sin θ, cos θ) where θ = atan2(y_r-y_l, x_r-x_l)
    // over NDC. θ = 0 ⇔ port→starboard line is horizontal in image plane (chase
    // and target wings level relative). Substituted to (0, 1) per-tick when
    // CEP-gated OR when beacon pair is geometrically degenerate (<1e-4 in NDC).
    float target_tilt_sin;       // raw-ok: NN-byte-format buffer
    float target_tilt_cos;       // raw-ok: NN-byte-format buffer
};

static_assert(sizeof(TrackerInputs) == 54 * sizeof(float),
              "TrackerInputs layout must be contiguous float[54] with no padding");
static_assert(alignof(TrackerInputs) == alignof(float),
              "TrackerInputs must be float-aligned for matrix multiply");
static_assert(static_cast<int>(TrackerInput::COUNT) ==
              sizeof(TrackerInputs) / sizeof(float),
              "TrackerInputs struct size must match TrackerInput::COUNT");
