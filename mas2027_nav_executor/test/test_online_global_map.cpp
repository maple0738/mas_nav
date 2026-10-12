#include <gtest/gtest.h>

#include <limits>

#include "rog_map/query_adapter.hpp"

#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
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
