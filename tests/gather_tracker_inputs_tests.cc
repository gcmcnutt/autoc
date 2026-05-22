// 030 M6d / M7a — gather_tracker_inputs contract tests.
//
// Verifies the TrackerInputs struct is filled in field order matching the
// TrackerInput enum, and that the layout supports the
// reinterpret_cast<float*>(&inputs) trick that nn_forward relies on.
//
// 030 M7a (Session 2026-05-07 Q1): TrackerInput::COUNT reduced 48 → 45.
// HOME_X/Y/Z/HOME_DIST replaced with single DIST_TO_BOUNDARY_ALONG_VEL
// computed via arena.h::distanceToBoundary() (single source of truth
// with the per-tick OOB termination check).

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "autoc/eval/aircraft_state.h"
#include "autoc/eval/arena.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/types.h"

using autoc::eval::FlightArena;

namespace {

// Build a chase AircraftState with known values so each gathered field
// has a verifiable expected output.
AircraftState makeChaseState() {
    gp_quat orientation =
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitZ())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitY())) *
        gp_quat(Eigen::AngleAxis<gp_scalar>(0, gp_vec3::UnitX()));
    AircraftState s{0, /*relVel=*/12.5f,
                    gp_vec3(12.5f, 0.0f, 0.0f),
                    orientation,
                    gp_vec3(10.0f, 0.0f, 0.0f),
                    /*pitchCmd=*/0.0f,
                    /*rollCmd=*/0.0f,
                    /*throttleCmd=*/0.0f,
                    /*pathIndex=*/0};
    return s;
}

}  // namespace

TEST(GatherTrackerInputs, FillsHistoryFieldsInEnumOrder) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    // Distinct values per slot so out-of-order copy would be caught.
    for (int i = 0; i < 6; ++i) {
        history.left_x[i]   = 0.10f + 0.01f * i;
        history.left_y[i]   = 0.20f + 0.01f * i;
        history.left_cep[i] = 0.30f + 0.01f * i;
        history.right_x[i]  = 0.40f + 0.01f * i;
        history.right_y[i]  = 0.50f + 0.01f * i;
        history.right_cep[i]= 0.60f + 0.01f * i;
    }

    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);

    for (int i = 0; i < 6; ++i) {
        EXPECT_FLOAT_EQ(out.beacon_l_x[i],   history.left_x[i])   << i;
        EXPECT_FLOAT_EQ(out.beacon_l_y[i],   history.left_y[i])   << i;
        EXPECT_FLOAT_EQ(out.beacon_l_cep[i], history.left_cep[i]) << i;
        EXPECT_FLOAT_EQ(out.beacon_r_x[i],   history.right_x[i])  << i;
        EXPECT_FLOAT_EQ(out.beacon_r_y[i],   history.right_y[i])  << i;
        EXPECT_FLOAT_EQ(out.beacon_r_cep[i], history.right_cep[i])<< i;
    }
}

TEST(GatherTrackerInputs, FillsAircraftStateFieldsFromAircraftState) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);

    // Quat: identity ⇒ (1, 0, 0, 0).
    EXPECT_FLOAT_EQ(out.quat_w, 1.0f);
    EXPECT_FLOAT_EQ(out.quat_x, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_y, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_z, 0.0f);
    // M11.preA.2: cruise-normalized airspeed (relVel / kCruiseSpeed_mps = 13).
    EXPECT_FLOAT_EQ(out.airspeed, 12.5f / kCruiseSpeed_mps);
}

TEST(GatherTrackerInputs, DistToBoundaryComputedFromArenaAndVelocity) {
    // Chase at virtual (10, 0, 0) — 10m from arena center horizontally,
    // velocity (12.5, 0, 0) — pure +X. Wall at radius 80m. Wall ahead is
    // 80 - 10 = 70m along +X. Velocity unit = (1, 0, 0). Ray hits wall
    // at t=70.
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    FlightArena arena;  // defaults: 80m radius, 5m floor, 100m ceiling

    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, out);

    // M11.preA.2: dist_to_boundary is now tanh(d / kDistToBoundaryScale_m).
    // d_raw ≈ 70m (wall hit, no floor/ceiling intersection since vz = 0).
    // tanh(70/20) = tanh(3.5) ≈ 0.998 (saturated, far from boundary).
    EXPECT_NEAR(out.dist_to_boundary_along_vel,
                std::tanh(70.0f / kDistToBoundaryScale_m), 1e-4f);
    EXPECT_GT(out.dist_to_boundary_along_vel, 0.99f);
}

TEST(GatherTrackerInputs, DistToBoundaryWithDescendingVelocityHitsFloor) {
    // Chase at virtual (10, 0, 0); virtual_z=0 corresponds to raw z=-25
    // (i.e., 25m AGL). Floor at 5m AGL = raw z=-5 = virtual_z=+20.
    // Velocity (10, 0, 5) — descending at 5 m/s. Distance to floor in
    // virtual_z = 20 - 0 = 20. Time to floor = 20 / 5 = 4 sec
    // along velocity = 4 sec * |v| ≈ ... wait, distanceToBoundary
    // returns t in normalized-velocity units (meters along unit vector),
    // so for unit v with vz_unit = 5/sqrt(125) ≈ 0.447, distance to floor
    // along unit vec = 20 / 0.447 ≈ 44.7m.
    AircraftState chase{0, /*relVel=*/sqrtf(125.0f),
                        gp_vec3(10.0f, 0.0f, 5.0f),  // descending
                        gp_quat::Identity(),
                        gp_vec3(10.0f, 0.0f, 0.0f),
                        0.0f, 0.0f, 0.0f, 0};
    TrackerHistoryWindow history{};
    FlightArena arena;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, out);

    // M11.preA.2: tanh-saturated. d_raw ≈ 44.7m → tanh(44.7/20) = tanh(2.235)
    // ≈ 0.977 (still saturated; would need chase within ~10-20m of boundary
    // to see gradient under scale=20). Floor (~44.7m) is closer than wall.
    EXPECT_NEAR(out.dist_to_boundary_along_vel,
                std::tanh(44.7f / kDistToBoundaryScale_m), 1e-3f);
    EXPECT_GT(out.dist_to_boundary_along_vel, 0.95f);
}

// 030 M11.preA.2 — soft-sat shape exercised in the gradient region. Place
// chase 10m from the wall along velocity so d_raw ≈ 10m → tanh(10/20) = tanh(0.5)
// ≈ 0.462. Verifies the NN sees a meaningful sub-1.0 signal as it approaches
// the boundary (which is the whole point of the tanh wrap). With scale=20
// the gradient region spans ~0-30m, matching the craft's ~10-15m emergency-
// turn budget.
TEST(GatherTrackerInputs, DistToBoundarySoftSatShapeNearWall) {
    // Arena defaults: 80m radius. Place chase at virtual (70, 0, 0) with
    // velocity (10, 0, 0). Wall ahead at +x: 80 - 70 = 10m along +X.
    AircraftState chase{0, /*relVel=*/10.0f,
                        gp_vec3(10.0f, 0.0f, 0.0f),
                        gp_quat::Identity(),
                        gp_vec3(70.0f, 0.0f, 0.0f),
                        0.0f, 0.0f, 0.0f, 0};
    TrackerHistoryWindow history{};
    FlightArena arena;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, out);

    EXPECT_NEAR(out.dist_to_boundary_along_vel,
                std::tanh(10.0f / kDistToBoundaryScale_m), 1e-3f);
    EXPECT_NEAR(out.dist_to_boundary_along_vel, 0.4621f, 1e-3f);
}

// reinterpret_cast<float*>(&trackerInputs) is the path nn_forward uses;
// verify the struct can be safely treated as a flat float[54] post-032 phase 1.
TEST(GatherTrackerInputs, LayoutIsContiguousFloat54) {
    static_assert(sizeof(TrackerInputs) == 54 * sizeof(float),
                  "TrackerInputs must be float[54] for nn_forward to consume "
                  "(032 phase 1: 45 existing + 9 derived = 54)");

    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    history.left_x[0] = 0.123f;
    history.right_cep[5] = 0.987f;
    history.span[3] = 0.456f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);

    const float* flat = reinterpret_cast<const float*>(&out);
    // First slot should be beacon_l_x[0] per enum order.
    EXPECT_FLOAT_EQ(flat[0], 0.123f);
    // Slot 35 (last beacon channel: right_cep[5]) should be 0.987f.
    EXPECT_FLOAT_EQ(flat[35], 0.987f);
    // Slot 36 should be quat_w (identity = 1.0).
    EXPECT_FLOAT_EQ(flat[36], 1.0f);
    // Slot 44 should be dist_to_boundary_along_vel.
    EXPECT_FLOAT_EQ(flat[44], out.dist_to_boundary_along_vel);
    // 032 phase 1 — slot 48 = beacon_pair_span[3] (cached from history).
    EXPECT_FLOAT_EQ(flat[48], 0.456f);
}

// ============================================================================
// 032 PHASE 1 — Derived perceptual features
// ============================================================================

namespace {

// Populate history.span and the underlying NDC slots so the span values are
// internally consistent. CEP defaults to 0 (visible).
void fillVisibleHistoryWithSpan(TrackerHistoryWindow& h,
                                 float lx, float ly,
                                 float rx, float ry) {
    for (int i = 0; i < 6; ++i) {
        h.left_x[i] = lx;
        h.left_y[i] = ly;
        h.left_cep[i] = 0.5f;  // visible (below default 1.25 gate)
        h.right_x[i] = rx;
        h.right_y[i] = ry;
        h.right_cep[i] = 0.5f;
        const float dx = rx - lx;
        const float dy = ry - ly;
        h.span[i] = std::sqrt(dx * dx + dy * dy);
    }
}

}  // namespace

TEST(GatherTrackerInputs, CopiesSpanHistoryIntoOutputSlots) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    // Distinct span values per slot so out-of-order copy would be caught.
    for (int i = 0; i < 6; ++i) {
        history.left_x[i] = 0.0f;
        history.left_y[i] = 0.0f;
        history.left_cep[i] = 0.5f;
        history.right_x[i] = 0.0f;
        history.right_y[i] = 0.0f;
        history.right_cep[i] = 0.5f;
        history.span[i] = 0.10f + 0.05f * i;  // 0.10, 0.15, 0.20, ...
    }
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    for (int i = 0; i < 6; ++i) {
        EXPECT_FLOAT_EQ(out.beacon_pair_span[i], history.span[i]) << i;
    }
}

TEST(GatherTrackerInputs, SpanRateIsOneTickRawDiff) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history, 0.0f, 0.0f, 0.5f, 0.0f);  // span = 0.5
    history.span[5] = 0.50f;
    history.span[4] = 0.30f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    EXPECT_FLOAT_EQ(out.span_rate, 0.20f);  // 0.50 - 0.30
}

TEST(GatherTrackerInputs, TiltAtMidRangeHorizontalPair) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history,
                                -0.15f, 0.0f,   // port at image-left
                                 0.15f, 0.0f);  // starboard at image-right
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    // θ = 0 → (sin, cos) = (0, 1)
    EXPECT_NEAR(out.target_tilt_sin, 0.0f, 1e-5f);
    EXPECT_NEAR(out.target_tilt_cos, 1.0f, 1e-5f);
    // Span ~ 0.3
    EXPECT_NEAR(out.beacon_pair_span[5], 0.30f, 1e-5f);
}

TEST(GatherTrackerInputs, TiltRolledNinetyDegreesRelative) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history,
                                0.0f, -0.15f,   // port below center
                                0.0f, 0.15f);   // starboard above
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    // θ = +π/2 → (sin, cos) = (1, 0)
    EXPECT_NEAR(out.target_tilt_sin, 1.0f, 1e-5f);
    EXPECT_NEAR(out.target_tilt_cos, 0.0f, 1e-5f);
}

TEST(GatherTrackerInputs, CepGatedLeftBeaconSubstitutesNeutral) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history, -0.15f, 0.0f, 0.15f, 0.0f);
    // Override LEFT cep at "now" to be sentinel (1.5 ≥ kCepGateThreshold).
    history.left_cep[5] = 1.5f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    // span_now: gate substitution at gather layer would set to 0; here it's
    // sourced from history.span[5] (computed upstream in projectAndShift-
    // History). For the gather contract, what matters is the tilt + span_rate
    // gating decisions.
    // Tilt is gate-substituted at gather-time → (0, 1).
    EXPECT_FLOAT_EQ(out.target_tilt_sin, 0.0f);
    EXPECT_FLOAT_EQ(out.target_tilt_cos, 1.0f);
}

TEST(GatherTrackerInputs, CepGatedRightBeaconAlsoSubstitutes) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history, -0.15f, 0.0f, 0.15f, 0.0f);
    history.right_cep[5] = 1.5f;  // RIGHT sentinel
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    EXPECT_FLOAT_EQ(out.target_tilt_sin, 0.0f);
    EXPECT_FLOAT_EQ(out.target_tilt_cos, 1.0f);
}

TEST(GatherTrackerInputs, BothBeaconsVisibleNoSubstitution) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history,
                                0.0f, 0.0f,
                                1.0f, 1.0f);  // diagonal pair, θ = π/4
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    // θ = π/4 → (sin, cos) = (√2/2, √2/2) ≈ 0.707
    EXPECT_NEAR(out.target_tilt_sin, std::sqrt(0.5f), 1e-5f);
    EXPECT_NEAR(out.target_tilt_cos, std::sqrt(0.5f), 1e-5f);
}

TEST(GatherTrackerInputs, DeterministicAcrossConsecutiveCalls) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history, -0.1f, 0.0f, 0.3f, 0.2f);
    history.span[4] = 0.4f;
    history.span[5] = 0.45f;
    TrackerInputs a{}, b{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, a);
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, b);
    for (size_t i = 0; i < sizeof(TrackerInputs) / sizeof(float); ++i) {
        const float* fa = reinterpret_cast<const float*>(&a);
        const float* fb = reinterpret_cast<const float*>(&b);
        EXPECT_FLOAT_EQ(fa[i], fb[i]) << "slot " << i;
    }
}

// ============================================================================
// 032 PHASE 1 — Identity-stable beacon ordering (Feature A)
// ============================================================================
// Per contracts/identity_invariant.md: the existing sim pipeline preserves
// port/starboard identity by mount convention (beacon_left_ → left_* slots,
// beacon_right_ → right_* slots). This is a no-op in sim today (R1); the
// test guards against future refactors that might accidentally re-introduce
// NDC-x sorting.
//
// gather_tracker_inputs is a pure pass-through from TrackerHistoryWindow to
// TrackerInputs — it has no knowledge of mounting. The identity invariant
// lives in projectAndShiftHistory (TrackerStepper + crrcsim_tracker_helper).
// At this layer we verify only the pass-through: whatever the history
// window holds in left_*[i] lands in beacon_l_*[i].

TEST(GatherTrackerInputs, IdentityStablePassThrough) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    // Simulate target oriented away from chase: port beacon (body -y) projects
    // to image-plane left in this geometry. But what matters is identity-
    // stability: whatever the upstream code put in left_*, gather forwards to
    // beacon_l_*.
    history.left_x[5] = -0.2f;
    history.left_y[5] = 0.1f;
    history.right_x[5] = 0.2f;
    history.right_y[5] = 0.1f;
    history.left_cep[5] = 0.5f;
    history.right_cep[5] = 0.5f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    EXPECT_FLOAT_EQ(out.beacon_l_x[5], -0.2f);
    EXPECT_FLOAT_EQ(out.beacon_r_x[5], 0.2f);

    // Now reverse the synthetic NDC values (simulating target oriented
    // toward chase: port still at body -y but lands on image-plane right).
    // gather must still forward left_* → beacon_l_* without sorting.
    history.left_x[5] = 0.2f;
    history.right_x[5] = -0.2f;
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, out);
    EXPECT_FLOAT_EQ(out.beacon_l_x[5], 0.2f);   // port still on the left slot
    EXPECT_FLOAT_EQ(out.beacon_r_x[5], -0.2f);  // starboard still on the right slot
}
