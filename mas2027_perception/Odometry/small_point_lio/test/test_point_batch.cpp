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

TEST(PointBatch, SixObservedDimensionsMatchFullSolveWithStateCrossCovariance) {
    for (int trial = 0; trial < 5; ++trial) {
        eskf full, reduced;
        const Eigen::Matrix<double, state::DIM, state::DIM> basis =
            Eigen::Matrix<double, state::DIM, state::DIM>::Random();
        full.P = basis * basis.transpose() * .01 +
            Eigen::Matrix<double, state::DIM, state::DIM>::Identity() * .1;
        reduced.P = full.P;
        std::size_t selected = 0;
        const auto model = [&selected](const state &, point_measurement_result &r) {
            r.valid = selected % 7 != 0;
            r.H.setZero();
            for (int j = 0; j < 6; ++j) r.H(j) = std::sin((selected + 1) * (j + 1) * .17);
            r.z = .01 * std::cos(selected * .3);
            r.laser_point_cov = .02 + .001 * selected;
        };
        const auto select = [&selected](std::size_t i) { selected = i; };
        full.init(model, [](const state &, imu_measurement_result &) {});
        reduced.init(model, [](const state &, imu_measurement_result &) {});
        Eigen::Matrix<double, 6, 6> full_info, reduced_info;
        const auto accepted = full.update_point_batch<12>(32, select, &full_info);
        EXPECT_EQ(reduced.update_point_batch<6>(32, select, &reduced_info), accepted);
        EXPECT_GT(accepted, 0U);
        EXPECT_NEAR((reduced.P - full.P).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced_info - full_info).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.position - full.x.position).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.rotation - full.x.rotation).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.velocity - full.x.velocity).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.omg - full.x.omg).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.acceleration - full.x.acceleration).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.gravity - full.x.gravity).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.bg - full.x.bg).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.ba - full.x.ba).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.offset_R_L_I - full.x.offset_R_L_I).norm(), 0, 1e-11);
        EXPECT_NEAR((reduced.x.offset_T_L_I - full.x.offset_T_L_I).norm(), 0, 1e-11);
    }
}

TEST(PointBatch, DefaultSolveStillUpdatesOnlineExtrinsics) {
    eskf filter;
    filter.P.setIdentity();
    filter.init([](const state &, point_measurement_result &r) {
        r.valid = true; r.H.setZero(); r.H(9) = 1;
        r.z = 1; r.laser_point_cov = 1;
    }, [](const state &, imu_measurement_result &) {});
    EXPECT_EQ(filter.update_point_batch(1, [](std::size_t) {}), 1U);
    EXPECT_NEAR(filter.x.offset_T_L_I.x(), .5, 1e-12);
    EXPECT_NEAR(filter.P(9,9), .5, 1e-12);
}

}  // namespace
}  // namespace small_point_lio
