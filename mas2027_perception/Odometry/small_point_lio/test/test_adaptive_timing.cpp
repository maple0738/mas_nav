#include "small_point_lio/adaptive_timing.h"
#include "small_point_lio/eskf.h"
#include <gtest/gtest.h>
#include <limits>

namespace small_point_lio {
    namespace {
        AdaptiveTiming ready(const AdaptiveTimingConfig &c) {
            AdaptiveTiming t;
            t.observeImu(1, Eigen::Vector3d::Zero(), true, c);
            t.observeImu(1.005, Eigen::Vector3d::Zero(), true, c);
            for (int i = 0; i < 20; ++i) t.observeBatch(Eigen::Vector3d::Zero(), AdaptiveTiming::Information::Identity(), 1, .005, c);
            return t;
        }
        TEST(AdaptiveTiming, StableMotionGrowsWithinBoundsAndResetRestoresFallback) {
            AdaptiveTimingConfig c;
            auto t = ready(c);
            for (int i = 0; i < 20; ++i) t.observeBatch(Eigen::Vector3d::Zero(), AdaptiveTiming::Information::Identity(), 10, .005, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), .005);
            t.reset();
            EXPECT_DOUBLE_EQ(t.interval(.005, c), .005);
        }
        TEST(AdaptiveTiming, AccelerationAndStrongerConstraintsShortenInterval) {
            AdaptiveTimingConfig c;
            auto weak = ready(c), strong = ready(c);
            Eigen::Vector3d acceleration(10000, 0, 0);
            weak.observeBatch(acceleration, AdaptiveTiming::Information::Identity(), 10, .005, c);
            strong.observeBatch(acceleration, 100 * AdaptiveTiming::Information::Identity(), 10, .005, c);
            EXPECT_LT(strong.interval(.005, c), weak.interval(.005, c));
            EXPECT_NEAR(weak.interval(.005, c), std::pow(.01 / 1e8, .25), 1e-12);
            EXPECT_DOUBLE_EQ(strong.interval(.005, c), c.min_interval);
        }
        TEST(AdaptiveTiming, AngularAccelerationRespondsOnlyInObservedDirections) {
            AdaptiveTimingConfig c;
            auto observable = ready(c), unobservable = ready(c);
            observable.observeImu(1.01, Eigen::Vector3d(100, 0, 0), true, c);
            unobservable.observeImu(1.01, Eigen::Vector3d(100, 0, 0), true, c);
            auto info = AdaptiveTiming::Information::Zero().eval();
            info(3, 3) = 1e6;
            observable.observeBatch(Eigen::Vector3d::Zero(), info, 1, .005, c);
            info.setZero();
            for (int i = 0; i < 20; ++i) unobservable.observeBatch(Eigen::Vector3d::Zero(), info, 1, .005, c);
            EXPECT_DOUBLE_EQ(observable.interval(.005, c), c.min_interval);
            EXPECT_DOUBLE_EQ(unobservable.interval(.005, c), c.max_interval);
        }
        TEST(AdaptiveTiming, ImuFaultsAndNonfiniteInputsUseMinimumThenRecoverGradually) {
            AdaptiveTimingConfig c;
            auto t = ready(c);
            t.observeImu(1.01, Eigen::Vector3d::Zero(), false, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), c.min_interval);
            t.observeImu(1.015, Eigen::Vector3d::Zero(), true, c);
            t.observeImu(1.020, Eigen::Vector3d::Zero(), true, c);
            t.observeBatch(Eigen::Vector3d::Zero(), AdaptiveTiming::Information::Identity(), 1, .005, c);
            EXPECT_NEAR(t.interval(.005, c), .0012, 1e-12);
            t.observeImu(.5, Eigen::Vector3d::Zero(), true, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), c.min_interval);
            t.observeImu(2, Eigen::Vector3d::Zero(), true, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), c.min_interval);
            auto info = AdaptiveTiming::Information::Identity().eval();
            info(0, 0) = std::numeric_limits<double>::quiet_NaN();
            t.observeBatch(Eigen::Vector3d::Zero(), info, 10, .005, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), c.min_interval);
        }
        TEST(AdaptiveTiming, MissingMatchesUseBoundedFallback) {
            AdaptiveTimingConfig c;
            auto t = ready(c);
            for (int i = 0; i < 20; ++i) t.observeBatch(Eigen::Vector3d(1000, 0, 0), AdaptiveTiming::Information::Zero(), 0, .003, c);
            EXPECT_DOUBLE_EQ(t.interval(.005, c), .003);
            c.max_interval = .2;
            EXPECT_THROW(c.validate(), std::invalid_argument);
            c = {};
            c.normalized_error = std::numeric_limits<double>::quiet_NaN();
            EXPECT_THROW(c.validate(), std::invalid_argument);
        }
        TEST(AdaptiveTiming, BatchInformationUsesAcceptedMeasurementsAndPreservesFilterResult) {
            eskf a, b;
            a.P.setIdentity();
            b.P.setIdentity();
            std::size_t selected = 0;
            const auto model = [&selected](const state &, point_measurement_result &r) {
                r.valid = selected != 1;
                r.H.setZero();
                r.H(0) = 2;
                r.H(3) = 1;
                r.z = 1;
                r.laser_point_cov = .5;
            };
            a.init(model, [](const state &, imu_measurement_result &) {});
            b.init(model, [](const state &, imu_measurement_result &) {});
            AdaptiveTiming::Information info;
            auto select = [&selected](std::size_t i) { selected = i; };
            ASSERT_EQ(a.update_point_batch(3, select, &info), 2U);
            ASSERT_EQ(b.update_point_batch(3, select), 2U);
            EXPECT_DOUBLE_EQ(info(0, 0), 8);
            EXPECT_DOUBLE_EQ(info(0, 3), 4);
            EXPECT_DOUBLE_EQ(info(3, 3), 2);
            EXPECT_NEAR((a.P - b.P).norm(), 0, 1e-12);
            EXPECT_NEAR((a.x.position - b.x.position).norm(), 0, 1e-12);
            info.setOnes();
            a.update_point_batch(0, select, &info);
            EXPECT_DOUBLE_EQ(info.norm(), 0);
        }
    }// namespace
}// namespace small_point_lio
