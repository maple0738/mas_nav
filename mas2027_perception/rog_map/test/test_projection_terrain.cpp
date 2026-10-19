#include <rog_map/projection_layer.hpp>
#include <rog_map/body_return_filter.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr int kSize = 21;
constexpr double kResolution = 0.05;

TEST(BodyReturnFilter, UsesRotatedSquareAndPreservesNearbyWallReturns)
{
  super_utils::Pose pose{
    super_utils::Vec3f(1.0, 2.0, 0.0),
    super_utils::Quatf(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ()))};
  const double half_side = 0.302 / std::sqrt(2.0);
  const Eigen::Vector2d axis(std::sqrt(0.5), std::sqrt(0.5));
  const Eigen::Vector2d center(1.0, 2.0);
  const Eigen::Vector2d self = center + axis * (half_side - 0.01);
  const Eigen::Vector2d wall = center + axis * (half_side + 0.01);
  const rog_map::BodyReturnFilter filter(pose, 0.302);
  EXPECT_TRUE(filter.contains(self.x(), self.y()));
  EXPECT_FALSE(filter.contains(wall.x(), wall.y()));
  EXPECT_FALSE(rog_map::BodyReturnFilter(pose, 0.0).contains(self.x(), self.y()));
}

rog_map::ColumnStats ground(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 10;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = static_cast<int>(std::round(z / kResolution));
  stats.occupied_z_index_max = stats.occupied_z_index_min;
  stats.occupied_z_min_abs = z;
  stats.occupied_z_max_abs = z;
  return stats;
}

std::vector<uint8_t> labelsFor(const rog_map::ProjectionLayer::ColumnScanner & scanner)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    config, scanner);
  return layer.terrainLabels();
}

TEST(ProjectionTerrain, FlatGroundIsNotSlope)
{
  const auto labels = labelsFor([](int, int) { return ground(1.0); });
  EXPECT_EQ(labels[10 * kSize + 10], 0U);
}

TEST(ProjectionTerrain, ContinuousInclineIsSlope)
{
  const auto labels = labelsFor([](int x, int) {
    return ground(1.0 + 0.2 * x * kResolution);
  });
  EXPECT_EQ(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, NoisyContinuousInclineStillSlope)
{
  const auto labels = labelsFor([](int x, int y) {
    const double noise = ((x * 13 + y * 7) % 11 - 5) * 0.004;
    return ground(1.0 + 0.2 * x * kResolution + noise);
  });
  EXPECT_EQ(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, UphillVectorFollowsHeightGradient)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    config, [](int x, int y) { return ground(1.0 + (0.24 * x - 0.18 * y) * kResolution); });
  const auto geometry = layer.terrainGeometry();
  const size_t center = 10 * kSize + 10;
  ASSERT_EQ(geometry.labels[center], 5U);
  EXPECT_NEAR(geometry.uphill_x[center], 0.8, 0.05);
  EXPECT_NEAR(geometry.uphill_y[center], -0.6, 0.05);
}

TEST(ProjectionTerrain, RaisedThinSurfaceDoesNotTiltGroundPlane)
{
  const auto labels = labelsFor([](int x, int y) {
    if (y < 9 || y > 11 || x < 5 || x > 15) {
      return rog_map::ColumnStats{};
    }
    // One 20 cm raised thin surface gives a single high-leverage sample.
    // A least-squares plane can still fit the remaining flat ground with low
    // RMS, but the max residual exposes the step.
    return ground(x == 15 && y == 9 ? 1.2 : 1.0);
  });
  EXPECT_NE(labels[10 * kSize + 10], 5U);
}

TEST(ProjectionTerrain, ObservedGapWithNeighborSupportIsTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 21;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 7;
    stats.occupied_z_max_abs = 0.35;
    stats.free_between_occupied_count = 6;
    return stats;
  });
  EXPECT_EQ(labels[10 * kSize + 10], 6U);
}

TEST(ProjectionTerrain, ConfiguredTunnelSpanIsUsed)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  config.tunnel_height_delta_max = 0.75;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int, int) {
      auto stats = ground(0.0);
      stats.observed_count = 21;
      stats.occupied_count = 2;
      stats.occupied_z_index_min = 0;
      stats.occupied_z_index_max = 13;
      stats.occupied_z_max_abs = 0.65;
      stats.free_between_occupied_count = 12;
      return stats;
    });
  const size_t center = 10 * kSize + 10;
  EXPECT_EQ(layer.cells()[center].type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(layer.terrainLabels()[center], 6U);
}

TEST(ProjectionTerrain, OccupiedGapIsNotTunnel)
{
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int, int) {
      auto stats = ground(0.0);
      stats.observed_count = 21;
      stats.occupied_count = 2;
      stats.occupied_z_index_min = 0;
      stats.occupied_z_index_max = 20;
      stats.occupied_z_max_abs = 1.0;
      stats.free_between_occupied_count = 19;
      return stats;
    });
  const size_t center = 10 * kSize + 10;
  EXPECT_EQ(layer.cells()[center].type, rog_map::CellType::OCCUPIED);
  EXPECT_NE(layer.terrainLabels()[center], 6U);
}

TEST(ProjectionTerrain, UnknownGapIsNotTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 2;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 7;
    stats.occupied_z_max_abs = 0.35;
    return stats;
  });
  EXPECT_NE(labels[10 * kSize + 10], 6U);
}

rog_map::ProjectionLayerConfig groundConfig()
{
  rog_map::ProjectionLayerConfig config;
  config.min_observed_voxels = 1;
  config.surface_height_delta_max = 0.08;
  config.wall_height_delta_min = 0.8;
  config.ground_filter_en = true;
  config.hysteresis_en = false;
  config.mask_filter_en = false;
  config.obstacle_hold_time = 1.5;
  return config;
}

rog_map::ColumnStats span(double low, double high)
{
  auto stats = ground(low);
  stats.occupied_z_index_max = static_cast<int>(std::round(high / kResolution));
  stats.occupied_z_max_abs = high;
  stats.occupied_count = stats.occupied_z_index_max - stats.occupied_z_index_min + 1;
  stats.observed_count = std::max(10, stats.occupied_count);
  return stats;
}

TEST(ProjectionGround, NoisyGroundSpanUsesSurroundingPlane)
{
  rog_map::ProjectionLayer layer;
  const auto config = groundConfig();
  const auto scanner = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config, scanner);
  const auto & cell = layer.cells()[10 * kSize + 10];
  EXPECT_EQ(cell.raw_type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cell.type, rog_map::CellType::PASSABLE);
  EXPECT_EQ(cell.mask, 1U);
}

TEST(ProjectionGround, RaisedTenCentimeterObstacleAndTallWallStayBlocked)
{
  rog_map::ProjectionLayer layer;
  const auto scanner = [](int x, int y) {
    if (x == 10 && y == 10) return span(0.0, 0.1);
    if (x == 15 && y == 15) return span(0.0, 1.0);
    return ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    groundConfig(), scanner);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(layer.cells()[15 * kSize + 15].type, rog_map::CellType::OCCUPIED);
}

TEST(ProjectionGround, UnknownAndNonPlanarNeighborsCannotProveGround)
{
  for (bool unknown : {true, false}) {
    rog_map::ProjectionLayer layer;
    const auto scanner = [unknown](int x, int y) {
      if (x == 10 && y == 10) return span(-0.05, 0.05);
      return unknown ? rog_map::ColumnStats{} : ground((x + y) % 2 ? -0.1 : 0.1);
    };
    layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
      groundConfig(), scanner);
    EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
  }
}

TEST(ProjectionGround, ContinuousSlopeCanSupportNoisySurface)
{
  rog_map::ProjectionLayer layer;
  const auto scanner = [](int x, int y) {
    const double z = 0.2 * (x - 10) * kResolution;
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(z);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0,
    groundConfig(), scanner);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::PASSABLE);
}

TEST(ProjectionGround, DirtyNeighborChangesInvalidateOldCorrection)
{
  rog_map::ProjectionLayer layer;
  const auto config = groundConfig();
  const auto initial = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0);
  };
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config, initial);
  ASSERT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::PASSABLE);
  std::vector<int> dirty;
  for (int y = 7; y <= 13; ++y) for (int x = 7; x <= 13; ++x) {
    if (x != 10 || y != 10) dirty.push_back(y * kSize + x);
  }
  const auto changed = [](int x, int y) {
    return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.2);
  };
  layer.updateDirty(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 2.0,
    config, changed, dirty, false);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].raw_type, rog_map::CellType::OCCUPIED);
  EXPECT_EQ(layer.cells()[10 * kSize + 10].mask, 0U);
}

TEST(ProjectionGround, DisablingFilterPreservesOriginalClassification)
{
  rog_map::ProjectionLayer layer;
  auto config = groundConfig(); config.ground_filter_en = false;
  layer.updateFull(kSize, kSize, kResolution, Eigen::Vector2d::Zero(), 1.0, config,
    [](int x, int y) { return x == 10 && y == 10 ? span(-0.05, 0.05) : ground(0.0); });
  EXPECT_EQ(layer.cells()[10 * kSize + 10].type, rog_map::CellType::OCCUPIED);
}

}  // namespace
