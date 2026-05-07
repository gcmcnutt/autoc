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
    gather_tracker_inputs(chase, history, FlightArena{}, out);

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
    gather_tracker_inputs(chase, history, FlightArena{}, out);

    // Quat: identity ⇒ (1, 0, 0, 0).
    EXPECT_FLOAT_EQ(out.quat_w, 1.0f);
    EXPECT_FLOAT_EQ(out.quat_x, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_y, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_z, 0.0f);
    EXPECT_FLOAT_EQ(out.airspeed, 12.5f);
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
    gather_tracker_inputs(chase, history, arena, out);

    // dist_to_boundary should be ~70m (wall hit, no floor/ceiling intersection
    // since velocity vz = 0).
    EXPECT_NEAR(out.dist_to_boundary_along_vel, 70.0f, 0.5f);
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
    gather_tracker_inputs(chase, history, arena, out);

    // Either floor (~44.7m) or wall (~70/0.894=78.3m) — whichever is closer.
    // Floor is closer. Allow tolerance for floating-point.
    EXPECT_NEAR(out.dist_to_boundary_along_vel, 44.7f, 1.0f);
}

// reinterpret_cast<float*>(&trackerInputs) is the path nn_forward uses;
// verify the struct can be safely treated as a flat float[45] post-M7a.
TEST(GatherTrackerInputs, LayoutIsContiguousFloat45) {
    static_assert(sizeof(TrackerInputs) == 45 * sizeof(float),
                  "TrackerInputs must be float[45] for nn_forward to consume (M7a Session 2026-05-07)");

    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    history.left_x[0] = 0.123f;
    history.right_cep[5] = 0.987f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, FlightArena{}, out);

    const float* flat = reinterpret_cast<const float*>(&out);
    // First slot should be beacon_l_x[0] per enum order.
    EXPECT_FLOAT_EQ(flat[0], 0.123f);
    // Slot 35 (last beacon channel: right_cep[5]) should be 0.987f.
    EXPECT_FLOAT_EQ(flat[35], 0.987f);
    // Slot 36 should be quat_w (identity = 1.0).
    EXPECT_FLOAT_EQ(flat[36], 1.0f);
    // Slot 44 (last) should be dist_to_boundary_along_vel.
    EXPECT_FLOAT_EQ(flat[44], out.dist_to_boundary_along_vel);
}
