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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);

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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);

    // Quat: identity ⇒ (1, 0, 0, 0).
    EXPECT_FLOAT_EQ(out.common.quat_w, 1.0f);
    EXPECT_FLOAT_EQ(out.common.quat_x, 0.0f);
    EXPECT_FLOAT_EQ(out.common.quat_y, 0.0f);
    EXPECT_FLOAT_EQ(out.common.quat_z, 0.0f);
    // 041 P2-8: CRUISE-NORMALIZED, both modes. The shared CraftCommonInputs
    // sub-struct still makes an M1/M2 divergence impossible — P2-1 unified the
    // slot on RAW, P2-8 keeps the unification but on the SCALED side, because
    // raw m/s made this input 35x louder than SPECIFIC_ENERGY and the quiet
    // slots were never selected on (t5: throttle pegged 99.3% of ticks).
    // Written as physical/scale so the intent survives a constant change.
    EXPECT_FLOAT_EQ(out.common.airspeed, 12.5f / kCruiseSpeed_mps);
}

TEST(GatherTrackerInputs, DistToBoundaryComputedFromArenaAndVelocity) {
    // Chase at virtual (10, 0, 0) — 10 m from the arena centre horizontally,
    // velocity (12.5, 0, 0) — pure +X. Wall at radius 70 m, so the wall ahead
    // is 70 − 10 = 60 m along +X. Velocity unit = (1, 0, 0) ⇒ ray hits at t=60.
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    FlightArena arena;  // 041 P2-3 defaults: 70 m radius, 10 m deck, 110 m ceiling

    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, SituationalAwarenessState{}, out);

    // M11.preA.2: dist_to_boundary is tanh(d / kDistToBoundaryScale_m).
    // d_raw ≈ 60 m (wall hit; no floor/ceiling intersection since vz = 0).
    // tanh(60/20) = tanh(3) ≈ 0.995 — saturated, i.e. far from the boundary.
    EXPECT_NEAR(out.common.dist_to_boundary,
                std::tanh(60.0f / kDistToBoundaryScale_m), 1e-4f);
    EXPECT_GT(out.common.dist_to_boundary, 0.99f);
}

TEST(GatherTrackerInputs, DistToBoundaryWithDescendingVelocityHitsFloor) {
    // 041 P2-3: virtual_z=0 is the arm point (55 m AGL) and the hard deck is
    // 25 m AGL = virtual_z = +30 — the DOWN-extent. Velocity (6, 0, 8) descends,
    // |v| = 10 so vz_unit = 0.8 and vx_unit = 0.6. distanceToBoundary returns
    // metres along the UNIT velocity vector, so the deck is 30/0.8 = 37.5 m away
    // and the wall (70 − 10 = 60 m out) is 60/0.6 = 100 m.
    //
    // ⭐ The DECK is the nearer bound, which is what this test is named for.
    // ⚠️ The velocity is (6,0,8) rather than the (10,0,5) used previously
    // BECAUSE (10,0,5) is exactly degenerate at this geometry: 30/0.4472 and
    // 60/0.8944 are both 67.08 m, so the minimum would be a tie and the test
    // would pass or fail on floating-point noise while appearing to assert
    // which bound wins.
    AircraftState chase{0, /*relVel=*/10.0f,
                        gp_vec3(6.0f, 0.0f, 8.0f),  // descending
                        gp_quat::Identity(),
                        gp_vec3(10.0f, 0.0f, 0.0f),
                        0.0f, 0.0f, 0.0f, 0};
    TrackerHistoryWindow history{};
    FlightArena arena;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, SituationalAwarenessState{}, out);

    // The MINIMUM over wall/deck/ceiling is taken, and it is the deck: 37.5 m.
    EXPECT_NEAR(out.common.dist_to_boundary,
                std::tanh(37.5f / kDistToBoundaryScale_m), 1e-3f);
    // Distinguishable from the wall's 100 m, which would read 0.99999 — the
    // assertion that the MINIMUM was taken, not merely that something was.
    EXPECT_LT(out.common.dist_to_boundary,
              std::tanh(100.0f / kDistToBoundaryScale_m) - 0.01f);
}

// 030 M11.preA.2 — soft-sat shape exercised in the gradient region. Place
// chase 10m from the wall along velocity so d_raw ≈ 10m → tanh(10/20) = tanh(0.5)
// ≈ 0.462. Verifies the NN sees a meaningful sub-1.0 signal as it approaches
// the boundary (which is the whole point of the tanh wrap). With scale=20
// the gradient region spans ~0-30m, matching the craft's ~10-15m emergency-
// turn budget.
TEST(GatherTrackerInputs, DistToBoundarySoftSatShapeNearWall) {
    // Arena defaults: 70 m radius. Place the chase at virtual (60, 0, 0) with
    // velocity (10, 0, 0). Wall ahead at +x: 70 − 60 = 10 m along +X.
    AircraftState chase{0, /*relVel=*/10.0f,
                        gp_vec3(10.0f, 0.0f, 0.0f),
                        gp_quat::Identity(),
                        gp_vec3(60.0f, 0.0f, 0.0f),
                        0.0f, 0.0f, 0.0f, 0};
    TrackerHistoryWindow history{};
    FlightArena arena;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, arena, 1.25f, SituationalAwarenessState{}, out);

    EXPECT_NEAR(out.common.dist_to_boundary,
                std::tanh(10.0f / kDistToBoundaryScale_m), 1e-3f);
    EXPECT_NEAR(out.common.dist_to_boundary, 0.4621f, 1e-3f);
}

// reinterpret_cast<float*>(&trackerInputs) is the path nn_forward uses;
// verify the struct can be safely treated as a flat float array.
TEST(GatherTrackerInputs, LayoutIsContiguousFloatArray) {
    // Derived, not literal: the count lives in one place (TrackerInput::COUNT,
    // itself asserted against the struct size in nn_inputs.h), so a layout
    // change fails at its source instead of in three restated numbers.
    static_assert(sizeof(TrackerInputs) ==
                      static_cast<size_t>(TRACKER_NN_INPUT_COUNT) * sizeof(float),
                  "TrackerInputs must be a flat float[TRACKER_NN_INPUT_COUNT] for "
                  "nn_forward to consume");

    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    history.left_x[0] = 0.123f;
    history.right_cep[5] = 0.987f;
    history.span[3] = 0.456f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);

    const float* flat = reinterpret_cast<const float*>(&out);
    // ⚠️ 041 P2-1 REORDERED this struct: the craft block is now a contiguous
    // 20-slot tail (46..65) instead of being split around the derived
    // perceptual features. Indices below are the post-reorder ones.
    //
    // First slot is beacon_l_x[0] per enum order.
    EXPECT_FLOAT_EQ(flat[0], 0.123f);
    // Slot 35 (last beacon channel: right_cep[5]).
    EXPECT_FLOAT_EQ(flat[35], 0.987f);
    // Slot 39 = beacon_pair_span[3] — the derived block now begins at 36.
    EXPECT_FLOAT_EQ(flat[39], 0.456f);
    // Slot 46 is the first CraftCommonInputs slot, quat_w (identity = 1.0).
    EXPECT_FLOAT_EQ(flat[46], 1.0f);
    // And the sub-struct really is embedded there, not merely coincident.
    EXPECT_EQ(offsetof(TrackerInputs, common), 46u * sizeof(float));
    EXPECT_FLOAT_EQ(flat[59], out.common.dist_to_boundary);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
    for (int i = 0; i < 6; ++i) {
        EXPECT_FLOAT_EQ(out.beacon_pair_span[i], history.span[i]) << i;
    }
}

TEST(GatherTrackerInputs, SpanRateIsRateOverRecentLagGap) {
    // 037 T022 — span_rate is a true rate (NDC-units/s) over the NOW↔TM1
    // lag gap (kNNHistoryRecentGapSec = 100 ms at every cadence); was a raw
    // one-tick diff pre-037.
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history, 0.0f, 0.0f, 0.5f, 0.0f);  // span = 0.5
    history.span[5] = 0.50f;
    history.span[4] = 0.30f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
    EXPECT_FLOAT_EQ(out.span_rate, 0.20f / kNNHistoryRecentGapSec);  // 2.0/s
}

TEST(GatherTrackerInputs, TiltAtMidRangeHorizontalPair) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    fillVisibleHistoryWithSpan(history,
                                -0.15f, 0.0f,   // port at image-left
                                 0.15f, 0.0f);  // starboard at image-right
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, a);
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, b);
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
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
    EXPECT_FLOAT_EQ(out.beacon_l_x[5], -0.2f);
    EXPECT_FLOAT_EQ(out.beacon_r_x[5], 0.2f);

    // Now reverse the synthetic NDC values (simulating target oriented
    // toward chase: port still at body -y but lands on image-plane right).
    // gather must still forward left_* → beacon_l_* without sorting.
    history.left_x[5] = 0.2f;
    history.right_x[5] = -0.2f;
    gather_tracker_inputs(chase, history, FlightArena{}, 1.25f, SituationalAwarenessState{}, out);
    EXPECT_FLOAT_EQ(out.beacon_l_x[5], 0.2f);   // port still on the left slot
    EXPECT_FLOAT_EQ(out.beacon_r_x[5], -0.2f);  // starboard still on the right slot
}
