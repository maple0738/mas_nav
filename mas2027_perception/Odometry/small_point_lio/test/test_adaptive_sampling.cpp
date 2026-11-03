#include "small_point_lio/adaptive_sampling.h"
#include "small_point_lio/preprocess.h"

#include <gtest/gtest.h>
#include <limits>

namespace small_point_lio {
namespace {

TEST(AdaptiveSampling, FeedbackDirectionBoundsAndReset) {
    AdaptiveSamplingConfig config;
    AdaptiveSampling sampler;
    EXPECT_DOUBLE_EQ(sampler.leaf(0.5, config), 0.5);
    std::vector<double> dense(5000, 5.0);
    sampler.observe(dense, 0.05, config);
    EXPECT_GT(sampler.leaf(0.5, config), 0.5);
    for (int i = 0; i < 100; ++i) sampler.observe(dense, 0.05, config);
    EXPECT_DOUBLE_EQ(sampler.leaf(0.5, config), config.max_leaf);
    std::vector<double> sparse(10, 5.0);
    for (int i = 0; i < 100; ++i) sampler.observe(sparse, 0.05, config);
    EXPECT_DOUBLE_EQ(sampler.leaf(0.5, config), config.min_leaf);
    sampler.reset();
    EXPECT_DOUBLE_EQ(sampler.target(), 0);
    EXPECT_DOUBLE_EQ(sampler.leaf(0.5, config), 0.5);
}

TEST(AdaptiveSampling, TargetTracksSceneAndAcquisitionDuration) {
    AdaptiveSamplingConfig config;
    AdaptiveSampling near, far, short_input;
    near.leaf(0.5, config); far.leaf(0.5, config); short_input.leaf(0.5, config);
    std::vector<double> near_ranges(800, 1.0), far_ranges(2000, 30.0), half_ranges(1000, 30.0);
    near.observe(near_ranges, 0.05, config);
    far.observe(far_ranges, 0.05, config);
    short_input.observe(half_ranges, 0.025, config);
    EXPECT_GT(far.target(), near.target());
    EXPECT_DOUBLE_EQ(far.target(), config.max_points_per_second * 0.05);
    EXPECT_DOUBLE_EQ(short_input.target(), far.target() / 2);
    // Equal sampling rate at different input frequencies has equal steady state.
    EXPECT_DOUBLE_EQ(far.leaf(0.5, config), short_input.leaf(0.5, config));
}

TEST(AdaptiveSampling, EmptyDegenerateAndInterruptedInputsDoNotChangeLeaf) {
    AdaptiveSamplingConfig config;
    AdaptiveSampling sampler;
    sampler.leaf(0.5, config);
    std::vector<double> empty, ranges(5000, 5.0);
    sampler.observe(empty, 0.05, config);
    for (double dt : {0.0, -0.1, 0.5, std::numeric_limits<double>::quiet_NaN()}) {
        sampler.observe(ranges, dt, config);
    }
    EXPECT_DOUBLE_EQ(sampler.leaf(0.5, config), 0.5);
    EXPECT_DOUBLE_EQ(sampler.target(), 0);
}

TEST(AdaptiveSampling, RejectsInvalidConfiguration) {
    AdaptiveSamplingConfig config;
    config.min_leaf = 0;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config = {};
    config.max_points_per_second = config.min_points_per_second - 1;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config = {};
    config.reference_range = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(config.validate(), std::invalid_argument);
}

Parameters parameters() {
    Parameters result{};
    result.point_filter_num = 1;
    result.min_distance_squared = 0;
    result.max_distance_squared = 10000;
    result.blind_center = Eigen::Vector3f::Zero();
    result.space_downsample = true;
    result.space_downsample_leaf_size = 0.5F;
    // A low target makes reduction observable on the compact synthetic plane.
    result.adaptive_sampling.min_points_per_second = 1000;
    result.adaptive_sampling.max_points_per_second = 1000;
    return result;
}

std::vector<common::Point> plane(double start) {
    std::vector<common::Point> points;
    for (int x = 0; x < 40; ++x) {
        for (int y = 0; y < 40; ++y) {
            points.push_back({start + 0.05 * (x * 40 + y) / 1599.0,
                Eigen::Vector3f(2.0F + x * 0.1F, y * 0.1F, 1.0F)});
        }
    }
    return points;
}

TEST(AdaptiveSampling, ReducesLocalizationPointsPreservesDenseCloudAndResets) {
    auto params = parameters();
    Preprocess preprocess;
    preprocess.parameters = &params;
    std::size_t first_count = 0;
    for (int frame = 0; frame < 30; ++frame) {
        const auto input = plane(1.0 + frame * 0.06);
        preprocess.on_point_cloud_callback(input);
        if (frame == 0) {
            first_count = preprocess.point_deque.size();
        }
        EXPECT_EQ(preprocess.dense_point_deque.size(), input.size());
        ASSERT_EQ(preprocess.dense_frame_remaining.size(), 1U);
        EXPECT_EQ(preprocess.dense_frame_remaining.front(), input.size());
        double previous = -1;
        for (const auto &point : preprocess.point_deque) {
            EXPECT_GE(point.timestamp, previous);
            previous = point.timestamp;
        }
        if (frame == 29) {
            EXPECT_LT(preprocess.point_deque.size(), first_count);
        }
        preprocess.point_deque.clear();
        while (!preprocess.dense_point_deque.empty()) preprocess.finish_dense_point();
    }
    EXPECT_GT(preprocess.sampling_leaf_size(), 0.5);
    preprocess.reset();
    EXPECT_DOUBLE_EQ(preprocess.sampling_leaf_size(), 0.5);
}

TEST(AdaptiveSampling, DownsamplingBypassLeavesInputIntactAndControllerIdle) {
    auto params = parameters();
    params.space_downsample = false;
    Preprocess preprocess;
    preprocess.parameters = &params;
    const auto input = plane(1.0);
    preprocess.on_point_cloud_callback(input);
    EXPECT_EQ(preprocess.point_deque.size(), input.size());
    EXPECT_EQ(preprocess.dense_point_deque.size(), input.size());
    EXPECT_DOUBLE_EQ(preprocess.sampling_leaf_size(), 0.5);
}

TEST(AdaptiveSampling, FirstInputUsesConfiguredInitialResolution) {
    auto params = parameters();
    Preprocess preprocess;
    preprocess.parameters = &params;
    const auto input = plane(1.0);
    util::VoxelgridSampling fixed;
    std::vector<common::Point> expected;
    fixed.voxelgrid_sampling(input, expected, 0.5);
    std::sort(expected.begin(), expected.end(), [](const auto &a, const auto &b) {
        return a.timestamp < b.timestamp;
    });
    preprocess.on_point_cloud_callback(input);
    ASSERT_EQ(preprocess.point_deque.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        EXPECT_DOUBLE_EQ(preprocess.point_deque[i].timestamp, expected[i].timestamp);
        EXPECT_TRUE(preprocess.point_deque[i].position.isApprox(expected[i].position));
    }
    EXPECT_GT(preprocess.sampling_leaf_size(), 0.5);
}

}  // namespace
}  // namespace small_point_lio
