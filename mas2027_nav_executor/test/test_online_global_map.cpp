#include <gtest/gtest.h>

#include <limits>

#include "rog_map/query_adapter.hpp"
#include "rog_map/projection_layer.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"

#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "mas2027_nav_executor/common/environment/history_frontier.hpp"
#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "minco_core/components/planner_mode_context.hpp"

namespace {

using mas2027_nav_executor::OnlineGlobalMap;
using mas2027_nav_executor::OnlineGlobalMapConfig;
using Result = OnlineGlobalMap::UpdateResult;

OnlineGlobalMapConfig immediateConfig()
{
  OnlineGlobalMapConfig config;
  config.occupied_confirmations = 1;
  config.terrain_confirmations = 1;
  return config;
}

rog_map::MapSnapshot source(uint64_t sequence, double x = 0.0, double y = 0.0,
  uint8_t type = 1, int width = 10, int height = 10, double resolution = 0.1)
{
  rog_map::MapSnapshot s;
  s.sequence = sequence;
  s.stamp = static_cast<double>(sequence);
  s.origin_x = x; s.origin_y = y;
  s.width = width; s.height = height; s.resolution = resolution;
  s.types.assign(static_cast<size_t>(width) * height, type);
  s.values.assign(s.types.size(), type == 1 || type == 2 ? 0U : 254U);
  return s;
}

uint8_t valueAt(const std::shared_ptr<mas2027_nav_executor::OnlineGlobalMapSnapshot> & s,
  double x, double y)
{
  unsigned int mx = 0, my = 0;
  if (!s || !s->worldToMap(x, y, mx, my)) return 254U;
  return s->value(mx, my);
}

TEST(OnlineGlobalMap, SingleObstacleBecomesUnknownAndFreeReobservationClearsIt)
{
  OnlineGlobalMap map({}, "odom");
  ASSERT_EQ(map.update(source(1)), Result::UPDATED);
  ASSERT_EQ(map.update(source(2, 0, 0, 3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 255U);
  const auto pending = map.snapshot();
  unsigned int x, y;
  ASSERT_TRUE(pending->worldToMap(0.45, 0.45, x, y));
  EXPECT_FALSE(pending->isFree(x, y));
  EXPECT_LT(pending->query({0.45, 0.45, 0}).distance, 0.0);
  ASSERT_EQ(map.update(source(3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 0U);
}

TEST(OnlineGlobalMap, SpecialTerrainPersistsAndSlopeBoundaryRejectsSideEntry)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto slope = source(1);
  slope.terrain_labels.assign(slope.types.size(), 0U);
  slope.uphill_x.assign(slope.types.size(), 0.0F);
  slope.uphill_y.assign(slope.types.size(), 0.0F);
  for (int y = 4; y <= 6; ++y) for (int x = 4; x <= 6; ++x) {
    const size_t index = static_cast<size_t>(y) * slope.width + x;
    slope.terrain_labels[index] = 5U;
    slope.uphill_x[index] = 1.0F;
  }
  ASSERT_EQ(map.update(slope), Result::UPDATED);
  const auto first = map.snapshot();
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->terrainTransition(0.35, 0.45, 0.45, 0.45));
  EXPECT_FALSE(first->terrainTransition(0.45, 0.35, 0.45, 0.45));
  EXPECT_FALSE(first->terrainTransition(0.45, 0.35, 0.55, 0.35));
  EXPECT_EQ(valueAt(first, 0.45, 0.35), 254U);
  EXPECT_EQ(valueAt(first, 0.35, 0.45), 0U);
  const auto labels = first->terrainGrid();
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(first->worldToMap(0.45, 0.45, mx, my));
  EXPECT_EQ(labels.data[static_cast<size_t>(my) * labels.info.width + mx], 5);

  ASSERT_EQ(map.update(source(2, 2.0, 0.0)), Result::UPDATED);
  const auto remembered = map.snapshot();
  EXPECT_FALSE(remembered->terrainTransition(0.45, 0.35, 0.45, 0.45));
  EXPECT_EQ(valueAt(remembered, 0.45, 0.35), 254U);
  ASSERT_EQ(map.update(source(3)), Result::UPDATED);
  EXPECT_FALSE(map.snapshot()->terrainTransition(0.45, 0.35, 0.45, 0.45));
}

TEST(OnlineGlobalMap, TerrainNeedsRepeatedEvidenceAndCompleteOrdinaryReobservationClearsIt)
{
  OnlineGlobalMapConfig config = immediateConfig();
  config.terrain_confirmations = 3;
  config.terrain_clear_confirmations = 2;
  OnlineGlobalMap map(config, "odom");
  auto slope = [](uint64_t sequence) {
    auto s = source(sequence);
    s.terrain_labels.assign(s.types.size(), 0U);
    s.uphill_x.assign(s.types.size(), 0.0F);
    s.uphill_y.assign(s.types.size(), 0.0F);
    for (int y = 4; y <= 6; ++y) for (int x = 4; x <= 6; ++x) {
      const size_t i = static_cast<size_t>(y) * s.width + x;
      s.terrain_labels[i] = 5U;
      s.uphill_x[i] = 1.0F;
    }
    return s;
  };
  auto label_at = [&map]() {
    const auto snapshot = map.snapshot();
    const auto grid = snapshot->terrainGrid();
    unsigned int x = 0, y = 0;
    EXPECT_TRUE(snapshot->worldToMap(0.45, 0.45, x, y));
    return grid.data[static_cast<size_t>(y) * grid.info.width + x];
  };
  ASSERT_EQ(map.update(slope(1)), Result::UPDATED);
  EXPECT_EQ(label_at(), -1);
  auto ordinary = source(2);
  ordinary.terrain_labels.assign(ordinary.types.size(), 0U);
  ordinary.uphill_x.assign(ordinary.types.size(), 0.0F);
  ordinary.uphill_y.assign(ordinary.types.size(), 0.0F);
  ASSERT_EQ(map.update(ordinary), Result::UPDATED);
  ASSERT_EQ(map.update(slope(3)), Result::UPDATED);
  ASSERT_EQ(map.update(slope(4)), Result::UPDATED);
  EXPECT_EQ(label_at(), -1);
  ASSERT_EQ(map.update(slope(5)), Result::UPDATED);
  EXPECT_EQ(label_at(), 5);
  EXPECT_FALSE(map.snapshot()->terrainTransition(0.45, 0.35, 0.45, 0.45));
  ASSERT_EQ(map.update(source(6)), Result::UPDATED);  // No terrain observation.
  EXPECT_EQ(label_at(), 5);
  ordinary.sequence = 7; ordinary.stamp = 7;
  ASSERT_EQ(map.update(ordinary), Result::UPDATED);
  EXPECT_EQ(label_at(), 5);
  ordinary.sequence = 8; ordinary.stamp = 8;
  ASSERT_EQ(map.update(ordinary), Result::UPDATED);
  EXPECT_EQ(label_at(), -1);
  EXPECT_TRUE(map.snapshot()->terrainTransition(0.45, 0.35, 0.45, 0.45));
}

TEST(OnlineGlobalMap, ConfirmationsNeedConsecutiveFreshSnapshots)
{
  OnlineGlobalMap map({}, "odom");
  auto hit = source(1, 0, 0, 3);
  ASSERT_EQ(map.update(hit), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 255U);
  EXPECT_EQ(map.update(hit), Result::NO_NEW_OBSERVATION);
  EXPECT_EQ(map.update(source(2, 0, 0, 0)), Result::NO_NEW_OBSERVATION);
  ASSERT_EQ(map.update(source(3, 0, 0, 3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 255U);
  auto late = source(4, 0, 0, 3); late.stamp = 6.0;
  ASSERT_EQ(map.update(late), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 255U);
  auto fresh = source(5, 0, 0, 3); fresh.stamp = 6.5;
  ASSERT_EQ(map.update(fresh), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
}

TEST(OnlineGlobalMap, ObservedFreeCellsFilledOrHeldByLocalSafetyAreNotStoredAsObstacles)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto filled = source(1, 0, 0, 3);
  filled.raw_types.assign(filled.types.size(), 1U);
  filled.confidence.assign(filled.types.size(), 1.0F);
  EXPECT_EQ(map.update(filled), Result::NO_NEW_OBSERVATION);
  EXPECT_FALSE(map.snapshot());
  filled.sequence = 2; filled.stamp = 2;
  filled.raw_types.assign(filled.types.size(), 3U);
  ASSERT_EQ(map.update(filled), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
  auto invalid = source(3); invalid.raw_types.assign(1, 3U);
  EXPECT_EQ(map.update(invalid), Result::INVALID_INPUT);
}

TEST(OnlineGlobalMap, MeasuredHeightEdgesSurviveFreeAirAndNeedContinuousGroundToClear)
{
  OnlineGlobalMapConfig config;
  config.resolution = 0.1;
  OnlineGlobalMap map(config, "odom");
  auto edge = source(1, 0, 0, 2);
  edge.raw_types.assign(100, 2U);
  edge.ground_heights.assign(100, 0.0F);
  edge.height_discontinuities.assign(100, 0U);
  edge.confidence.assign(100, 1.0F);
  const size_t i = 4 * 10 + 5;
  edge.types[i] = 3U;
  edge.values[i] = 254U;
  edge.height_discontinuities[i] = 1U;
  ASSERT_EQ(map.update(edge), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.55, 0.45), 254U);
  EXPECT_FALSE(map.snapshot()->terrainTransition(0.45, 0.45, 0.65, 0.45));
  for (uint64_t sequence = 2; sequence <= 5; ++sequence) {
    auto air = source(sequence);
    air.ground_heights.assign(100, std::numeric_limits<float>::quiet_NaN());
    air.height_discontinuities.assign(100, 0U);
    ASSERT_EQ(map.update(air), Result::UPDATED);
    EXPECT_EQ(valueAt(map.snapshot(), 0.55, 0.45), 254U);
  }
  auto floor = source(6, 0, 0, 2);
  floor.raw_types.assign(100, 2U);
  floor.ground_heights.assign(100, 0.0F);
  floor.height_discontinuities.assign(100, 0U);
  ASSERT_EQ(map.update(floor), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.55, 0.45), 254U);
  floor.sequence = 7; floor.stamp = 7;
  ASSERT_EQ(map.update(floor), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.55, 0.45), 0U);
  EXPECT_TRUE(map.snapshot()->terrainTransition(0.45, 0.45, 0.65, 0.45));
}

TEST(OnlineGlobalMap, OnlyAdjacentMeasuredStepsEnterHistory)
{
  // 0 = gap-inferred on unseen cell, 1 = gap-inferred on seen ground,
  // 2 = adjacent measured jump on seen ground.
  for (const int scenario : {0, 1, 2}) {
    OnlineGlobalMapConfig config;
    config.resolution = 0.1;
    OnlineGlobalMap map(config, "odom");
    uint64_t sequence = 1;
    const auto floor = [&sequence]() {
      auto f = source(sequence, 0, 0, 2);
      f.stamp = static_cast<double>(sequence);
      ++sequence;
      f.raw_types.assign(100, 2U);
      f.ground_heights.assign(100, 0.0F);
      f.height_discontinuities.assign(100, 0U);
      f.confidence.assign(100, 1.0F);
      return f;
    };
    if (scenario > 0) {
      ASSERT_EQ(map.update(floor()), Result::UPDATED);
      ASSERT_EQ(map.update(floor()), Result::UPDATED);
    }
    for (int repeat = 0; repeat < 3; ++repeat) {
      auto edge = floor();
      const size_t i = 4 * 10 + 5;
      edge.types[i] = 3U;
      edge.values[i] = 254U;
      edge.height_discontinuities[i] = scenario == 2 ? 1U : 2U;
      ASSERT_EQ(map.update(edge), Result::UPDATED);
    }
    const auto snapshot = map.snapshot();
    unsigned int mx = 0, my = 0;
    ASSERT_TRUE(snapshot->worldToMap(0.55, 0.45, mx, my));
    EXPECT_EQ(snapshot->heightDiscontinuityMask()[my * snapshot->sizeX() + mx] != 0U,
      scenario == 2) << scenario;
    // Repeated inferred-only marks are not an observed obstacle either.
    if (scenario == 1) EXPECT_EQ(valueAt(snapshot, 0.55, 0.45), 0U);
    if (scenario == 2) {
      EXPECT_EQ(valueAt(snapshot, 0.55, 0.45), 254U);
      EXPECT_FALSE(snapshot->terrainTransition(0.45, 0.45, 0.65, 0.45));
    }
  }
}

TEST(OnlineGlobalMap, RejectsUnknownHeightDiscontinuityCode)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto edge = source(1, 0, 0, 2);
  edge.raw_types.assign(100, 2U);
  edge.ground_heights.assign(100, 0.0F);
  edge.height_discontinuities.assign(100, 0U);
  edge.height_discontinuities[45] = 3U;
  EXPECT_EQ(map.update(edge), Result::INVALID_INPUT);
}

TEST(OnlineGlobalMap, HeightEvidenceAcrossAnUnobservedScanGapIsRetained)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto edge = source(1, 0, 0, 0);
  edge.raw_types.assign(100, 0U);
  edge.height_discontinuities.assign(100, 0U);
  edge.confidence.assign(100, 0.0F);
  edge.types[45] = 3U;
  edge.height_discontinuities[45] = 1U;
  ASSERT_EQ(map.update(edge), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.55, 0.45), 254U);
}

TEST(OnlineGlobalMap, SparseEnclosingGroundCorrectsRememberedGapButStepsAndOneSidedReturnsDoNot)
{
  constexpr int size = 31, center = 15;
  constexpr double res = .05;
  for (int scenario = 0; scenario < 4; ++scenario) {
    OnlineGlobalMapConfig cfg; cfg.resolution = res;
    OnlineGlobalMap map(cfg, "odom");
    auto edge = source(1, 0, 0, 1, size, size, res);
    edge.raw_types = edge.types;
    edge.height_discontinuities.assign(size * size, 0U);
    const size_t target = center * size + center;
    edge.types[target] = 3U; edge.values[target] = 254U;
    edge.height_discontinuities[target] = 1U;
    ASSERT_EQ(map.update(edge), Result::UPDATED);
    auto floor = source(2, 0, 0, 2, size, size, res);
    floor.raw_types = floor.types;
    floor.ground_heights.resize(size * size);
    floor.height_discontinuities.assign(size * size, 0U);
    for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) {
      const size_t i = y * size + x;
      floor.ground_heights[i] = scenario == 1 ? .2F * x * res :
        (scenario == 2 && x >= center ? .10F : 0.F);
      if (std::hypot(x-center, y-center) < 7 || (scenario == 3 && x >= center)) {
        floor.types[i] = floor.raw_types[i] = 1U;
        floor.ground_heights[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    ASSERT_EQ(map.update(floor), Result::UPDATED);
    EXPECT_EQ(valueAt(map.snapshot(), .775, .775), 254U);
    floor.sequence = 3; floor.stamp = 3;
    ASSERT_EQ(map.update(floor), Result::UPDATED);
    EXPECT_EQ(valueAt(map.snapshot(), .775, .775), scenario < 2 ? 0U : 254U);
  }
}

TEST(OnlineGlobalMap, MirroredPlatformsUseOnlyTheirContinuousCornerRampInBothDirections)
{
  constexpr int size = 61;
  constexpr double res = 0.05, extent = size * res;
  for (const bool mirrored : {false, true}) {
    rog_map::ProjectionLayer projection;
    rog_map::ProjectionLayerConfig config;
    config.hysteresis_en = false;
    config.mask_filter_en = false;
    projection.updateFull(size, size, res, Eigen::Vector2d::Zero(), 1, config,
      [mirrored](int x, int y) {
        if (mirrored) { x = size - 1 - x; y = size - 1 - y; }
        const double z = y < 12 ? 0.2 * std::clamp((x - 20) / 20.0, 0.0, 1.0) :
          (x < 30 ? 0.0 : 0.2);
        rog_map::ColumnStats s;
        s.observed_count = 10; s.occupied_count = 1;
        s.occupied_z_index_min = s.occupied_z_index_max = static_cast<int>(std::round(z / res));
        s.occupied_z_min_abs = s.occupied_z_max_abs = z;
        return s;
      });
    auto observation = source(1, 0, 0, 2, size, size, res);
    observation.values = projection.values();
    observation.raw_types.resize(size * size);
    observation.ground_heights.resize(size * size);
    observation.height_discontinuities.resize(size * size);
    for (size_t i = 0; i < observation.types.size(); ++i) {
      const auto & cell = projection.cells()[i];
      observation.types[i] = static_cast<uint8_t>(cell.type);
      observation.raw_types[i] = static_cast<uint8_t>(cell.raw_type);
      observation.ground_heights[i] = cell.ground_reference_z;
      observation.height_discontinuities[i] = cell.height_discontinuity ? 1U : 0U;
    }
    auto cfg = immediateConfig(); cfg.resolution = res;
    OnlineGlobalMap map(cfg, "odom");
    ASSERT_EQ(map.update(observation), Result::UPDATED);
    // Remove the platform from the live window: history must keep its drop.
    ASSERT_EQ(map.update(source(2, 10, 0, 1, size, size, res)), Result::UPDATED);
    const auto history = map.snapshot();
    mas2027_nav_executor::smac::SmacPlanner2DSimple planner;
    planner.setMap(history); planner.setESDFQuery(history);
    planner.setMinimumClearance(0.1);
    planner.setParameters(false, 100000, 0.0F);
    for (const bool reverse : {false, true}) {
      Eigen::Vector2d start(0.55, 2.0), goal(2.55, 2.0);
      if (mirrored) { start = Eigen::Vector2d::Constant(extent) - start;
        goal = Eigen::Vector2d::Constant(extent) - goal; }
      if (reverse) std::swap(start, goal);
      unsigned int sx, sy, gx, gy;
      ASSERT_TRUE(history->worldToMap(start.x(), start.y(), sx, sy));
      ASSERT_TRUE(history->worldToMap(goal.x(), goal.y(), gx, gy));
      mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector path;
      ASSERT_TRUE(planner.createPath(sx, sy, gx, gy, path));
      bool used_ramp = false;
      for (const auto & point : path) {
        double wx, wy;
        history->mapToWorld(static_cast<unsigned int>(point.x),
          static_cast<unsigned int>(point.y), wx, wy);
        if (mirrored) { wx = extent - wx; wy = extent - wy; }
        if (wx > 1.4 && wx < 1.6) {
          EXPECT_LT(wy, 0.6);
          used_ramp = true;
        }
      }
      EXPECT_TRUE(used_ramp);
    }
  }
}

TEST(OnlineGlobalMap, RememberedDropRequiresRobotClearanceEvenWhenLiveRaysAreFree)
{
  auto cfg = immediateConfig(); cfg.resolution = .1;
  OnlineGlobalMap map(cfg, "odom");
  auto edge = source(1, 0, 0, 1, 31, 31, .1);
  edge.raw_types = edge.types;
  edge.height_discontinuities.assign(31*31, 0U);
  for (int y = 0; y < 31; ++y) {
    const size_t i = y * 31 + 15;
    edge.types[i] = 3U; edge.values[i] = 254U; edge.height_discontinuities[i] = 1U;
  }
  ASSERT_EQ(map.update(edge), Result::UPDATED);
  ASSERT_EQ(map.update(source(2, 0, 0, 1, 31, 31, .1)), Result::UPDATED);
  const auto history = map.snapshot();
  const auto field = history->queryHeight({1.25, 1.55, 0});
  ASSERT_TRUE(field.ok);
  EXPECT_NEAR(field.distance, .25, 1e-6);
  EXPECT_LT(field.gradient.x(), -.9);
  EXPECT_TRUE(history->terrainTransition(1.25, 1.25, 1.25, 1.85));
  EXPECT_FALSE(history->terrainTransition(1.25, 1.25, 1.25, 1.85, .28));
  const Eigen::Vector2d near_start(1.25, 1.25);
  EXPECT_TRUE(history->terrainTransition(1.25, 1.25, 1.10, 1.25, .28, near_start));
  EXPECT_FALSE(history->terrainTransition(1.25, 1.25, 1.35, 1.25, .28, near_start));
  EXPECT_FALSE(history->terrainTransition(1.25, 1.25, 1.25, 1.85, .28, near_start));
  EXPECT_TRUE(history->terrainTransition(1.15, 1.25, 1.15, 1.85, .28));
  EXPECT_FALSE(history->terrainTransition(1.15, 1.25, 1.95, 1.25, .28));
  const auto barriers = history->heightGrid();
  EXPECT_EQ(std::count(barriers.data.begin(), barriers.data.end(), 100), 31);
}

TEST(OnlineGlobalMap, UnknownBlockingCostsDoNotCreatePermanentWalls)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  EXPECT_EQ(map.update(source(1, 0, 0, 0)), Result::NO_NEW_OBSERVATION);
  EXPECT_FALSE(map.snapshot());
  ASSERT_EQ(map.update(source(2)), Result::UPDATED);
  const auto before = map.snapshot();
  EXPECT_EQ(map.update(source(3, 0, 0, 0)), Result::NO_NEW_OBSERVATION);
  EXPECT_EQ(map.snapshot(), before);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 0U);
}

TEST(OnlineGlobalMap, FilterFilledUnobservedCellsAreNotMemorized)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto s = source(1, 0, 0, 3);
  s.confidence.assign(s.types.size(), 0.0F);
  EXPECT_EQ(map.update(s), Result::NO_NEW_OBSERVATION);
  EXPECT_FALSE(map.snapshot());
  s.sequence = 2; s.stamp = 2;
  s.confidence.assign(s.types.size(), 1.0F);
  ASSERT_EQ(map.update(s), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
}

TEST(OnlineGlobalMap, RollingAndNegativeOriginsRetainHistoryAndPinnedQueries)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(1)), Result::UPDATED);
  const auto old = map.snapshot();
  const double old_origin = old->originX();
  ASSERT_EQ(map.update(source(2, -2.0, -1.0)), Result::UPDATED);
  const auto current = map.snapshot();
  EXPECT_EQ(valueAt(current, 0.45, 0.45), 0U);
  EXPECT_EQ(valueAt(current, -1.55, -0.55), 0U);
  EXPECT_EQ(current->grid().header.frame_id, "odom");
  EXPECT_GT(current->sizeX(), old->sizeX());
  EXPECT_EQ(old->originX(), old_origin);
  EXPECT_EQ(old->revision(), 1U);
  EXPECT_EQ(current->revision(), 2U);
  EXPECT_EQ(valueAt(current, -0.5, 0.45), 255U);
  unsigned int x, y;
  EXPECT_FALSE(old->worldToMap(-1.55, -0.55, x, y));
}

TEST(OnlineGlobalMap, ReobservedFreeSpaceClearsOldObstacleButDuplicateFramesDoNot)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(1, 0, 0, 3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
  auto free = source(2);
  ASSERT_EQ(map.update(free), Result::UPDATED);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(map.update(free), Result::NO_NEW_OBSERVATION);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
  ASSERT_EQ(map.update(source(3)), Result::UPDATED);
  ASSERT_EQ(map.update(source(4)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
  ASSERT_EQ(map.update(source(5)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 0U);
  ASSERT_EQ(map.update(source(6, 0, 0, 3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
}

TEST(OnlineGlobalMap, DownsamplingNeedsFullFreeCoverageAndAnyObstacleWins)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto s = source(1, 0, 0, 0, 4, 4, 0.05);
  s.types[0] = 1; s.values[0] = 0;
  EXPECT_EQ(map.update(s), Result::NO_NEW_OBSERVATION);
  s.sequence = 2; s.stamp = 2;
  for (const auto i : {0, 1, 4, 5}) { s.types[i] = 1; s.values[i] = 0; }
  ASSERT_EQ(map.update(s), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.05, 0.05), 0U);
  s.sequence = 3; s.stamp = 3; s.types[5] = 3; s.values[5] = 254;
  ASSERT_EQ(map.update(s), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.05, 0.05), 254U);
}

TEST(OnlineGlobalMap, HalfCoarseCellSlideKeepsWorldAlignmentAndPartialEdgesUnknown)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(1, -0.05, -0.05, 1, 4, 4, 0.05)), Result::UPDATED);
  const auto s = map.snapshot();
  EXPECT_EQ(valueAt(s, 0.05, 0.05), 0U);
  EXPECT_EQ(valueAt(s, -0.05, 0.05), 255U);
  EXPECT_EQ(valueAt(s, 0.15, 0.05), 255U);
}

TEST(OnlineGlobalMap, CellLimitDoesNotDiscardHistoryOrPartiallyApplyUpdate)
{
  OnlineGlobalMapConfig config;
  config.max_cells = 225;
  config.occupied_confirmations = 1;
  OnlineGlobalMap map(config, "odom");
  ASSERT_EQ(map.update(source(1)), Result::UPDATED);
  const auto before = map.snapshot();
  EXPECT_EQ(map.update(source(2, 100.0)), Result::CELL_LIMIT);
  EXPECT_EQ(map.snapshot(), before);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 0U);
  ASSERT_EQ(map.update(source(3, 0, 0, 3)), Result::UPDATED);
  EXPECT_EQ(valueAt(map.snapshot(), 0.45, 0.45), 254U);
}

TEST(OnlineGlobalMap, InvalidGeometryAndRegressedTimeLeaveSnapshotUnchanged)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(5)), Result::UPDATED);
  const auto before = map.snapshot();
  EXPECT_EQ(map.update(source(4, 0, 0, 3)), Result::NO_NEW_OBSERVATION);
  auto s = source(6); s.origin_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(map.update(s), Result::INVALID_INPUT);
  s = source(6); s.types.pop_back();
  EXPECT_EQ(map.update(s), Result::INVALID_INPUT);
  s = source(6); s.types[2] = 42;
  EXPECT_EQ(map.update(s), Result::INVALID_INPUT);
  s = source(6); s.resolution = 0.03;
  EXPECT_EQ(map.update(s), Result::INVALID_INPUT);
  s = source(6); s.origin_x = 0.025;
  EXPECT_EQ(map.update(s), Result::INVALID_INPUT);
  EXPECT_EQ(map.snapshot(), before);
  EXPECT_FALSE(before->query({std::numeric_limits<double>::infinity(), 0, 0}).ok);
}

TEST(OnlineGlobalMap, DistanceFieldUsesCellCentersAndUnknownBorders)
{
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(1)), Result::UPDATED);
  const auto s = map.snapshot();
  const auto center = s->query({0.45, 0.45, 0});
  ASSERT_TRUE(center.ok);
  EXPECT_NEAR(center.distance, 0.5, 1e-5);
  const auto unknown = s->query({-0.15, 0.45, 0});
  ASSERT_TRUE(unknown.ok);
  EXPECT_LT(unknown.distance, 0);
  EXPECT_FALSE(s->query({-10, 0, 0}).ok);
  std::vector<unsigned char> costs;
  ASSERT_TRUE(s->copyValues(costs));
  EXPECT_EQ(costs.size(), static_cast<size_t>(s->sizeX()) * s->sizeY());
  EXPECT_EQ(s->values(), nullptr);
}

TEST(OnlineGlobalMap, ExistingSearchInterfaceCanReturnThroughRetainedArea)
{
  rclcpp::init(0, nullptr);
  OnlineGlobalMap map(immediateConfig(), "odom");
  ASSERT_EQ(map.update(source(1, 0, 0, 1, 40, 20)), Result::UPDATED);
  ASSERT_EQ(map.update(source(2, 3.0, 0, 1, 40, 20)), Result::UPDATED);
  const auto query = map.snapshot();
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  params.exploration_unknown_as_occupied = true;
  minco_planner::PlannerModeContext context;
  context.configure(params, query, nullptr, rclcpp::get_logger("global_map_test"));
  minco_planner::Astar astar(query->sizeX(), query->sizeY());
  minco_planner::GlobalPathSearcher search;
  search.configure(nullptr, &astar, nullptr, false, false, 0.15, rclcpp::get_logger("global_map_test"));
  search.setQuery(query);
  geometry_msgs::msg::PoseStamped start, goal;
  start.header.frame_id = goal.header.frame_id = "odom";
  start.pose.position.x = 6.45; start.pose.position.y = 1.05;
  goal.pose.position.x = 0.55; goal.pose.position.y = 1.05;
  start.pose.orientation.w = goal.pose.orientation.w = 1;
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_TRUE(search.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_NEAR(path.back().pose.position.x, 0.55, 0.15);
  rclcpp::shutdown();
}

}  // namespace

using mas2027_nav_executor::HistoryFrontierQuery;
using mas2027_nav_executor::findHistoryFrontierRoute;

TEST(HistoryFrontier, DeadEndPocketEscapesTowardTheOnlyUnexploredSide)
{
  // 6 x 4 m corridor, closed to the east (toward the goal), never observed
  // west of x = 0.5 m. The live window from the east end sees no progress.
  constexpr int width = 60, height = 40;
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto scan = source(1, 0, 0, 1, width, height, 0.1);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t i = static_cast<size_t>(y) * width + x;
      if (x < 5) { scan.types[i] = 0U; scan.values[i] = 255U; }
      if (y < 2 || y >= height - 2 || x >= width - 1) { scan.types[i] = 3U; scan.values[i] = 254U; }
    }
  }
  ASSERT_EQ(map.update(scan), Result::UPDATED);
  const auto snapshot = map.snapshot();
  ASSERT_TRUE(snapshot);
  HistoryFrontierQuery request;
  request.start = {5.0, 2.0};
  request.goal = {10.0, 2.0};
  const auto route = findHistoryFrontierRoute(*snapshot, request);
  ASSERT_TRUE(route);
  EXPECT_LT(route->target.x(), 1.6);
  EXPECT_GE((route->target - request.start).norm(), request.minimum_advance);
  ASSERT_GE(route->target_to_start.size(), 2U);
  EXPECT_LT((route->target_to_start.front() - route->target).norm(), 1e-9);
  EXPECT_LT((route->target_to_start.back() - request.start).norm(), 0.15);
  for (const auto & p : route->target_to_start) {
    unsigned int mx = 0, my = 0;
    ASSERT_TRUE(snapshot->worldToMap(p.x(), p.y(), mx, my));
    EXPECT_EQ(snapshot->value(mx, my), 0U);  // never enters unknown or walls
  }

  request.fixed_target = route->target;
  request.start = {4.0, 2.0};
  const auto again = findHistoryFrontierRoute(*snapshot, request);
  ASSERT_TRUE(again);
  EXPECT_LT((again->target - route->target).norm(), 1e-9);

  request.fixed_target.reset();
  request.excluded = [](const Eigen::Vector2d & p) { return p.x() < 3.0; };
  EXPECT_FALSE(findHistoryFrontierRoute(*snapshot, request));
}

TEST(HistoryFrontier, PreferredEdgeMarginDoesNotEraseTheOnlySafeFrontier)
{
  constexpr int width = 80, height = 30;
  OnlineGlobalMap map(immediateConfig(), "odom");
  auto scan = source(1, 0, 0, 1, width, height, .1);
  scan.height_discontinuities.assign(width * height, 0U);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t i = static_cast<size_t>(y) * width + x;
      if (x < 5) { scan.types[i] = 0U; scan.values[i] = 255U; }
      if (y <= 10 || y >= 18 || x == width - 1) {
        scan.types[i] = 3U; scan.values[i] = 254U;
      }
      if (y == 10 || y == 18) scan.height_discontinuities[i] = 1U;
    }
  }
  scan.raw_types = scan.types;
  ASSERT_EQ(map.update(scan), Result::UPDATED);
  const auto snapshot = map.snapshot();
  ASSERT_TRUE(snapshot);
  HistoryFrontierQuery request;
  request.start = {6.5, 1.45};
  request.goal = {10.0, 1.45};
  request.height_clearance = .45;
  const auto route = findHistoryFrontierRoute(*snapshot, request);
  ASSERT_TRUE(route);
  EXPECT_LT(route->target.x(), 1.6);
  EXPECT_LT(snapshot->queryHeight({route->target.x(), route->target.y(), 0}).distance,
    request.height_clearance);
  for (const auto & point : route->target_to_start) {
    const auto field = snapshot->query({point.x(), point.y(), 0});
    ASSERT_TRUE(field.ok);
    EXPECT_GE(field.distance, request.route_clearance);
  }
  // A genuinely unsafe slot is still refused by the original clearance gate.
  request.route_clearance = .5;
  EXPECT_FALSE(findHistoryFrontierRoute(*snapshot, request));
}
