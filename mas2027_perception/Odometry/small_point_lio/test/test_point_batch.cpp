#include "small_point_lio/eskf.h"

#include <gtest/gtest.h>

namespace small_point_lio {
namespace {

TEST(PointBatch, CombinesLinearMeasurementsIntoOneStateAndCovarianceUpdate) {
    eskf filter;
    filter.P.setIdentity();
    filter.P(0, state::velocity_index) = 0.3;
    filter.P(state::velocity_index, 0) = 0.3;
    std::size_t selected = 0;
    eskf::measurement_model_imu imu_model = [](const state &, imu_measurement_result &) {};
    filter.init(
        [&selected](const state &, point_measurement_result &result) {
            result.valid = selected != 1;
            result.H.setZero();
            result.H(0) = 1.0;
            result.z = 1.0;
            result.laser_point_cov = 0.5;
        }, imu_model);

    const auto accepted = filter.update_point_batch(3, [&selected](std::size_t index) {
        selected = index;
    });

    EXPECT_EQ(accepted, 2U);
    EXPECT_NEAR(filter.x.position.x(), 0.8, 1e-10);
    EXPECT_NEAR(filter.P(0, 0), 0.2, 1e-10);
    EXPECT_NEAR(filter.x.velocity.x(), 0.24, 1e-10);
    EXPECT_NEAR(filter.P(0, state::velocity_index), 0.06, 1e-10);
    EXPECT_NEAR(filter.P(1, 1), 1.0, 1e-10);
    EXPECT_NEAR((filter.P - filter.P.transpose()).norm(), 0.0, 1e-12);
}

TEST(PointBatch, InvalidMeasurementsLeaveStateUntouched) {
    eskf filter;
    filter.P.setIdentity();
    eskf::measurement_model_imu imu_model = [](const state &, imu_measurement_result &) {};
    filter.init(
        [](const state &, point_measurement_result &result) {
            result.valid = false;
        }, imu_model);

    EXPECT_EQ(filter.update_point_batch(2, [](std::size_t) {}), 0U);
    EXPECT_DOUBLE_EQ(filter.x.position.x(), 0.0);
    EXPECT_DOUBLE_EQ(filter.P(0, 0), 1.0);
}

}  // namespace
}  // namespace small_point_lio
