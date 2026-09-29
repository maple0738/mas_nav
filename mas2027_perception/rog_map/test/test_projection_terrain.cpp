#include <rog_map/projection_layer.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace {

constexpr int kSize = 21;
constexpr double kResolution = 0.05;

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

TEST(ProjectionTerrain, ObservedGapWithNeighborSupportIsTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 21;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 20;
    stats.occupied_z_max_abs = 1.0;
    stats.free_between_occupied_count = 19;
    return stats;
  });
  EXPECT_EQ(labels[10 * kSize + 10], 6U);
}

TEST(ProjectionTerrain, UnknownGapIsNotTunnel)
{
  const auto labels = labelsFor([](int, int) {
    auto stats = ground(0.0);
    stats.observed_count = 2;
    stats.occupied_count = 2;
    stats.occupied_z_index_min = 0;
    stats.occupied_z_index_max = 20;
    stats.occupied_z_max_abs = 1.0;
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
