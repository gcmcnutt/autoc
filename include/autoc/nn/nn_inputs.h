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

// 038 P0-D FR-P0H (A) — time-since-seen normalization. Blind-tick count × dt
// (seconds) fed through tanh(t / kTimeSinceSeenScale_s): 1 s → 0.46, 2 s →
// 0.76, 4 s → 0.96. Gives the tracker NN a graded "how long since the target
// was last visible" signal that saturates within a few seconds of blindness;
// 0.0 exactly when a beacon is visible this tick.
constexpr float kTimeSinceSeenScale_s = 2.0f;

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

struct NNInputs {
    // Time samples: [-0.8s, -0.4s, -0.2s, -0.1s, -0.05s, now]
    // (kNNHistoryLagsMsec — t10 0.8 s log-spaced lags; past-only per
    // 029 US1, no future lookahead. The *_TM5..NOW slot NAMES are lag-slot
    // indices, not tick counts.)
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
    // ----- 038 P0-D FR-P0H arena-awareness (B) — M1 gets it too (clean/fast
    // early read on the arena cue; M1 never goes blind so (A) is tracker-only).
    float dist_to_boundary;  // raw-ok: NN-byte-format buffer — dimensionless tanh(d/scale), [0,1), along-velocity ray to arena bound
    float inward_body_x;     // raw-ok: NN-byte-format buffer — body-frame radial-inward unit vec, chase_quat.conjugate()*normalize(center-pos)
    float inward_body_y;     // raw-ok: NN-byte-format buffer
    float inward_body_z;     // raw-ok: NN-byte-format buffer
};

static_assert(sizeof(NNInputs) == 37 * sizeof(float),
              "NNInputs layout must be contiguous float[37] with no padding (33 pre-038 + 4 FR-P0H arena)");
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
// PathgenInput — 37 inputs, layout matches NNInputs struct field order.
// (33 pre-038 + 4 038 FR-P0H arena-awareness (B): dist_to_boundary + inward_body xyz)
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
    // ----- 038 P0-D FR-P0H arena-awareness (B), M1 gets it too (33..36) -----
    DIST_TO_BOUNDARY,
    INWARD_BODY_X, INWARD_BODY_Y, INWARD_BODY_Z,
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
    // ----- 038 P0-D FR-P0H arena-awareness meta (slots 33..36) -----
    {"DIST_TO_BOUNDARY", "dBnd", 8},
    {"INWARD_BODY_X",    "inX",  7},
    {"INWARD_BODY_Y",    "inY",  7},
    {"INWARD_BODY_Z",    "inZ",  7},
};

static_assert(static_cast<size_t>(PathgenInput::COUNT) ==
              sizeof(kPathgenInputMeta) / sizeof(SensorInputMeta),
              "PathgenInput enum count must match kPathgenInputMeta length");
static_assert(static_cast<int>(PathgenInput::COUNT) == NN_INPUT_COUNT,
              "PathgenInput::COUNT must equal NN_INPUT_COUNT (NNInputs struct size)");

// ----------------------------------------------------------------------------
// TrackerInput — 60 inputs (FR-006 + FR-016 + 032 phase 1 + 038 FR-P0H).
//   36 beacon (left + right × 6 history slots × {x, y, CEP})
//    8 aircraft state (quat 4 + airspeed 1 + gyro 3)
//    1 arena-awareness (DIST_TO_BOUNDARY_ALONG_VEL — single ray-projection
//      scalar shared with arena.h::distanceToBoundary())
//    9 032 phase-1 derived (span×6 + span_rate + tilt sin/cos)
//    6 038 FR-P0H situational-awareness (time_since_seen + exit_dir sin/cos +
//      inward_body x/y/z)
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
    // ----- 038 P0-D FR-P0H situational-awareness inputs (54..59) -----
    // (A) target-lost cues (tracker only — M1's rabbit is always visible):
    TIME_SINCE_SEEN,   // tanh-normalized ticks since any beacon last visible
    EXIT_DIR_SIN,      // last-seen beacon-pair bearing sin (camera-frame, held)
    EXIT_DIR_COS,      // last-seen beacon-pair bearing cos
    // (B) arena-inward body-frame unit vector (radial toward cylinder axis):
    INWARD_BODY_X,     // chase_quat.conjugate()*normalize(center-pos), body frame
    INWARD_BODY_Y,
    INWARD_BODY_Z,
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
    // ----- 038 P0-D FR-P0H situational-awareness meta (slots 54..59) -----
    {"TIME_SINCE_SEEN",      "tSee",  7},
    {"EXIT_DIR_SIN",         "exS",   7},
    {"EXIT_DIR_COS",         "exC",   7},
    {"INWARD_BODY_X",        "inX",   7},
    {"INWARD_BODY_Y",        "inY",   7},
    {"INWARD_BODY_Z",        "inZ",   7},
};

static_assert(static_cast<size_t>(TrackerInput::COUNT) ==
              sizeof(kTrackerInputMeta) / sizeof(SensorInputMeta),
              "TrackerInput enum count must match kTrackerInputMeta length");
static_assert(static_cast<int>(TrackerInput::COUNT) == 60,
              "TrackerInput::COUNT must equal 60 (54 pre-038 + 038 FR-P0H: 3 target-lost + 3 arena-inward)");

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
    // BeaconObservation::screen_x/y. 6-slot history at kNNHistoryLagsMsec.
    // Substituted to 0.0 per-tick when EITHER beacon's CEP at "now"
    // >= CepGateThreshold.
    float beacon_pair_span[6];   // raw-ok: NN-byte-format buffer
    // span_rate: signed RATE (NDC-units/s) over the NOW↔TM1 lag gap =
    // (span[5] - span[4]) / kNNHistoryRecentGapSec. 037 T022 changed this
    // from a raw one-tick diff to a true per-second rate (cadence- and
    // lag-invariant); M2 genomes retrain from scratch at the boundary.
    float span_rate;             // raw-ok: NN-byte-format buffer
    // target_tilt encoded as (sin θ, cos θ) where θ = atan2(y_r-y_l, x_r-x_l)
    // over NDC. θ = 0 ⇔ port→starboard line is horizontal in image plane (chase
    // and target wings level relative). Substituted to (0, 1) per-tick when
    // CEP-gated OR when beacon pair is geometrically degenerate (<1e-4 in NDC).
    float target_tilt_sin;       // raw-ok: NN-byte-format buffer
    float target_tilt_cos;       // raw-ok: NN-byte-format buffer

    // ----- 038 P0-D FR-P0H situational-awareness (baseline enrichment) -----
    // (A) target-lost cues (tracker only; M1's rabbit is always visible).
    // Stateful (held/decaying), reset per scenario/engage in the stepper.
    float time_since_seen;   // raw-ok: NN-byte-format buffer — tanh(ticks_lost·scale), [0,1); 0 = visible now
    float exit_dir_sin;      // raw-ok: NN-byte-format buffer — last-seen beacon-pair bearing (camera-frame), held through blindness
    float exit_dir_cos;      // raw-ok: NN-byte-format buffer
    // (B) arena-inward body-frame unit vector — radial toward the cylinder axis,
    // rotated into body frame by the chase quat (all-attitude, smooth). Paired
    // with dist_to_boundary_along_vel above as the magnitude/urgency term.
    float inward_body_x;     // raw-ok: NN-byte-format buffer — chase_quat.conjugate()*normalize(center-pos), body frame
    float inward_body_y;     // raw-ok: NN-byte-format buffer
    float inward_body_z;     // raw-ok: NN-byte-format buffer
};

static_assert(sizeof(TrackerInputs) == 60 * sizeof(float),
              "TrackerInputs layout must be contiguous float[60] with no padding");
static_assert(alignof(TrackerInputs) == alignof(float),
              "TrackerInputs must be float-aligned for matrix multiply");
static_assert(static_cast<int>(TrackerInput::COUNT) ==
              sizeof(TrackerInputs) / sizeof(float),
              "TrackerInputs struct size must match TrackerInput::COUNT");
