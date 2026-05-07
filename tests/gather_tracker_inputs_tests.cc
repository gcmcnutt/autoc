// 030 M6d — gather_tracker_inputs contract tests.
//
// Verifies the TrackerInputs struct is filled in field order matching the
// TrackerInput enum, and that the layout supports the
// reinterpret_cast<float*>(&inputs) trick that nn_forward relies on.

#include <gtest/gtest.h>

#include <cstring>

#include "autoc/eval/aircraft_state.h"
#include "autoc/nn/evaluator.h"
#include "autoc/nn/nn_inputs.h"
#include "autoc/types.h"

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
    gather_tracker_inputs(chase, history, gp_vec3::Zero(), out);

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
    gather_tracker_inputs(chase, history, gp_vec3::Zero(), out);

    // Quat: identity ⇒ (1, 0, 0, 0).
    EXPECT_FLOAT_EQ(out.quat_w, 1.0f);
    EXPECT_FLOAT_EQ(out.quat_x, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_y, 0.0f);
    EXPECT_FLOAT_EQ(out.quat_z, 0.0f);
    EXPECT_FLOAT_EQ(out.airspeed, 12.5f);
}

TEST(GatherTrackerInputs, HomeUnitVecAndDistanceFromChasePos) {
    AircraftState chase = makeChaseState();  // chase at (10, 0, 0)
    TrackerHistoryWindow history{};
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, gp_vec3(0.0f, 0.0f, 0.0f), out);

    // home is at world origin, chase at (10, 0, 0). Vector chase→home is
    // (-10, 0, 0); norm = 10. Identity orientation → body frame = world.
    EXPECT_NEAR(out.home_dist, 10.0f, 1e-5f);
    EXPECT_NEAR(out.home_x, -1.0f, 1e-5f);
    EXPECT_NEAR(out.home_y, 0.0f, 1e-5f);
    EXPECT_NEAR(out.home_z, 0.0f, 1e-5f);
}

TEST(GatherTrackerInputs, HomeAtChasePositionEmitsZeroDirection) {
    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    TrackerInputs out{};
    // home == chase position → singular case, expect zero dir + zero dist.
    gather_tracker_inputs(chase, history, chase.getPosition(), out);

    EXPECT_FLOAT_EQ(out.home_dist, 0.0f);
    EXPECT_FLOAT_EQ(out.home_x, 0.0f);
    EXPECT_FLOAT_EQ(out.home_y, 0.0f);
    EXPECT_FLOAT_EQ(out.home_z, 0.0f);
}

// reinterpret_cast<float*>(&trackerInputs) is the path nn_forward uses;
// verify the struct can be safely treated as a flat float[48].
TEST(GatherTrackerInputs, LayoutIsContiguousFloat48) {
    static_assert(sizeof(TrackerInputs) == 48 * sizeof(float),
                  "TrackerInputs must be float[48] for nn_forward to consume");

    AircraftState chase = makeChaseState();
    TrackerHistoryWindow history{};
    history.left_x[0] = 0.123f;
    history.right_cep[5] = 0.987f;
    TrackerInputs out{};
    gather_tracker_inputs(chase, history, gp_vec3::Zero(), out);

    const float* flat = reinterpret_cast<const float*>(&out);
    // First slot should be beacon_l_x[0] per enum order.
    EXPECT_FLOAT_EQ(flat[0], 0.123f);
    // Slot 35 (last beacon channel: right_cep[5]) should be 0.987f.
    EXPECT_FLOAT_EQ(flat[35], 0.987f);
    // Slot 36 should be quat_w (identity = 1.0).
    EXPECT_FLOAT_EQ(flat[36], 1.0f);
    // Slot 47 (last) should be home_dist.
    EXPECT_FLOAT_EQ(flat[47], out.home_dist);
}
