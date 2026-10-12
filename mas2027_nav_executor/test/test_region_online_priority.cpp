#include "mas2027_nav_executor/common/environment/region_control.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"

#include <gtest/gtest.h>

#include <cstring>

#include <rclcpp/clock.hpp>

namespace {

using mas2027_nav_executor::RegionController;
using mas2027_nav_executor::RegionPlan;
using mas2027_nav_executor::RegionRule;
using mas2027_nav_executor::RegionSegment;
using mas2027_nav_executor::TerrainGrid;
using mas2027_nav_executor::annotateRegions;

TerrainGrid::Snapshot staticTerrain(uint8_t label)
{
  TerrainGrid::Snapshot terrain;
  terrain.cost.header.frame_id = "map";
  terrain.cost.info.resolution = 0.1F;
  terrain.cost.info.width = 20;
  terrain.cost.info.height = 20;
  terrain.cost.info.origin.orientation.w = 1.0;
  terrain.cost.data.assign(400, 0);
  terrain.labels.header.frame_id = "map";
  terrain.labels.width = 20;
  terrain.labels.height = 20;
  terrain.labels.step = 20;
  terrain.labels.encoding = "mono8";
  terrain.labels.data.assign(400, label);
  return terrain;
}

nav_msgs::msg::OccupancyGrid onlineTerrain(int8_t label)
{
  auto grid = staticTerrain(0).cost;
  grid.data.assign(400, label);
  return grid;
}

interfaces::msg::MpcPositionCommand trajectory()
{
  interfaces::msg::MpcPositionCommand command;
  command.header.frame_id = "map";
  command.cmds.resize(2);
  command.cmds[0].trajectory_id = 42;
  command.cmds[1].trajectory_id = 42;
  command.cmds[0].position.x = 0.25;
  command.cmds[0].position.y = 0.25;
  command.cmds[1].position.x = 0.75;
  command.cmds[1].position.y = 0.25;
  return command;
}

std::array<RegionRule, 3> rules()
{
  std::array<RegionRule, 3> result{};
  for (size_t i = 0; i < result.size(); ++i) {
    auto & rule = result[i];
    rule.label = static_cast<uint8_t>(5 + i);
    rule.mode = rule.label;
    rule.max_speed = 1.0;
    rule.max_acceleration = 1.0;
  }
  return result;
}

TEST(RegionOnlinePriority, OnlineSpecialLabelOverridesStatic)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const auto terrain = staticTerrain(5);
  const auto online = onlineTerrain(6);
  const auto plan = annotateRegions(trajectory(), terrain, buffer, rules(), "map", &online);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->segments.size(), 1U);
  EXPECT_EQ(plan->segments.front().label, 6U);
}

TEST(RegionOnlinePriority, RememberedUndulatingLabelIsApplied)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const auto terrain = staticTerrain(0);
  const auto remembered = onlineTerrain(7);
  const auto plan = annotateRegions(trajectory(), terrain, buffer, rules(), "map", &remembered);
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan->segments.size(), 1U);
  EXPECT_EQ(plan->segments.front().label, 7U);
}

TEST(TerrainGrid, PriorSlopeDirectionBlocksSideBoundary)
{
  auto terrain = staticTerrain(0);
  terrain.directions.header.frame_id = "map";
  terrain.directions.width = 20;
  terrain.directions.height = 20;
  terrain.directions.step = 20 * 2 * sizeof(float);
  terrain.directions.encoding = "32FC2";
  terrain.directions.data.assign(static_cast<size_t>(terrain.directions.step) * 20, 0U);
  for (int y = 4; y <= 6; ++y) for (int x = 4; x <= 6; ++x) {
    terrain.labels.data[static_cast<size_t>(y) * 20 + x] = 5U;
    const float uphill_x = 1.0F;
    std::memcpy(terrain.directions.data.data() +
      static_cast<size_t>(y) * terrain.directions.step + x * 2 * sizeof(float),
      &uphill_x, sizeof(float));
  }
  std::vector<float> uphill_x(400, 0.0F), uphill_y(400, 0.0F);
  for (int y = 4; y <= 6; ++y) for (int x = 4; x <= 6; ++x) {
    uphill_x[static_cast<size_t>(y) * 20 + x] = 1.0F;
  }
  terrain.slope_side_mask = mas2027_nav_executor::buildSlopeSideMask(
    20, 20, terrain.labels.data, uphill_x, uphill_y);
  EXPECT_TRUE(terrain.transition({0.35, 0.45}, {0.45, 0.45}));
  EXPECT_FALSE(terrain.transition({0.45, 0.35}, {0.45, 0.45}));
  EXPECT_FALSE(terrain.traversable({0.45, 0.35}));
  EXPECT_FALSE(terrain.transition({0.45, 0.35}, {0.55, 0.35}));
  EXPECT_TRUE(terrain.transition({0.65, 0.45}, {0.75, 0.45}));

  TerrainGrid grid;
  grid.updateDirections(terrain.directions);
  grid.updateCost(terrain.cost);
  grid.updateLabels(terrain.labels);
  const auto constraints = grid.planningConstraints();
  ASSERT_TRUE(constraints);
  EXPECT_EQ(constraints->data[3 * 20 + 4], 100);
  EXPECT_EQ(constraints->data[4 * 20 + 3], 0);
}

TEST(TerrainGrid, RotatedSlopeBlocksLongSideAndKeepsEndOpen)
{
  constexpr int width = 25;
  std::vector<uint8_t> labels(width * width, 0U);
  std::vector<float> uphill_x(labels.size(), 0.0F), uphill_y(labels.size(), 0.0F);
  for (int y = 0; y < width; ++y) for (int x = 0; x < width; ++x) {
    if (std::abs(x + y - 20) > 8 || std::abs(y - x) > 2) continue;
    const size_t index = static_cast<size_t>(y) * width + x;
    labels[index] = 5U;
    uphill_x[index] = uphill_y[index] = 0.70710678F;
  }
  const auto sides = mas2027_nav_executor::buildSlopeSideMask(
    width, width, labels, uphill_x, uphill_y);
  EXPECT_EQ(sides[static_cast<size_t>(12) * width + 9], 1U);
  EXPECT_EQ(sides[static_cast<size_t>(6) * width + 5], 0U);
  EXPECT_FALSE(mas2027_nav_executor::terrainSideTransitionAllowed(
    width, width, 0.1, 0.0, 0.0, sides, 0.95, 1.25, 0.95, 1.25, true));
  EXPECT_TRUE(mas2027_nav_executor::terrainSideTransitionAllowed(
    width, width, 0.1, 0.0, 0.0, sides, 0.55, 0.65, 0.65, 0.65, true));
}

TEST(RegionOnlinePriority, UnknownAndOrdinaryOnlineCellsFallBackToStatic)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const auto terrain = staticTerrain(5);
  for (const int8_t label : {-1, 0}) {
    const auto online = onlineTerrain(label);
    const auto plan = annotateRegions(trajectory(), terrain, buffer, rules(), "map", &online);
    ASSERT_TRUE(plan);
    ASSERT_EQ(plan->segments.size(), 1U);
    EXPECT_EQ(plan->segments.front().label, 5U);
  }
}

TEST(RegionOnlinePriority, SameTrajectoryCanAdoptNewRegionWithoutLosingProgress)
{
  RegionController controller(4, 2.0, 2.0, 2.0, 2.0);
  auto first = std::make_shared<RegionPlan>();
  first->trajectory_id = 42;
  first->positions_odom = {{0.0, 0.0}, {1.0, 0.0}};
  first->arc_lengths = {0.0, 1.0};
  first->segments.push_back(RegionSegment{5, 5, 1.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.8, 1.0});
  controller.setPlan(first);
  controller.update({0.2, 0.0});
  ASSERT_EQ(controller.mode(), 5U);
  const double progress = controller.progress();

  auto replacement = std::make_shared<RegionPlan>(*first);
  replacement->segments[0].label = 6;
  replacement->segments[0].mode = 6;
  controller.setPlan(replacement);
  controller.update({0.2, 0.0});
  EXPECT_EQ(controller.mode(), 6U);
  EXPECT_GE(controller.progress(), progress);
}

TEST(RegionOnlinePriority, FasterSlopeProfileStaysWithinOrdinaryModeLimits)
{
  RegionController controller(4, 3.0, 4.0, 100.0, 100.0);
  EXPECT_DOUBLE_EQ(controller.speedLimit(), 3.0);
  EXPECT_DOUBLE_EQ(controller.accelerationLimit(), 4.0);

  auto plan = std::make_shared<RegionPlan>();
  plan->trajectory_id = 43;
  plan->positions_odom = {{0.0, 0.0}, {3.0, 0.0}};
  plan->arc_lengths = {0.0, 3.0};
  plan->segments.push_back(RegionSegment{5, 5, 4.0, 5.0,
    0.7, 0.8, 0.9, 1.0, 1.5, 1.7});
  controller.setPlan(plan);
  controller.update({0.5, 0.0});
  controller.update({0.75, 0.0});
  controller.tickProfile(0.05);
  EXPECT_EQ(controller.mode(), 4U);
  EXPECT_DOUBLE_EQ(controller.speedLimit(), 3.0);

  controller.update({0.85, 0.0});
  controller.tickProfile(0.05);
  EXPECT_EQ(controller.mode(), 5U);
  EXPECT_DOUBLE_EQ(controller.speedLimit(), 4.0);
  EXPECT_DOUBLE_EQ(controller.accelerationLimit(), 5.0);

  controller.update({2.0, 0.0});
  controller.tickProfile(0.05);
  EXPECT_EQ(controller.mode(), 4U);
  EXPECT_DOUBLE_EQ(controller.speedLimit(), 3.0);
  EXPECT_DOUBLE_EQ(controller.accelerationLimit(), 4.0);
}

TEST(RegionOnlinePriority, OnlineOnlyWorksWithoutStaticMapOrMapTF)
{
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  auto traj = trajectory();
  traj.header.frame_id = "odom";
  const auto ordinary = mas2027_nav_executor::annotateOnlineRegions(traj, buffer, rules(), "odom");
  ASSERT_TRUE(ordinary);
  EXPECT_TRUE(ordinary->segments.empty());
  EXPECT_EQ(ordinary->trajectory_id, 42U);
  auto online = onlineTerrain(5);
  online.header.frame_id = "odom";
  const auto slope = mas2027_nav_executor::annotateOnlineRegions(traj, buffer, rules(), "odom", &online);
  ASSERT_TRUE(slope);
  ASSERT_EQ(slope->segments.size(), 1U);
  EXPECT_EQ(slope->segments.front().label, 5U);
  online.data.assign(400, -1);
  const auto unknown = mas2027_nav_executor::annotateOnlineRegions(traj, buffer, rules(), "odom", &online);
  ASSERT_TRUE(unknown);
  EXPECT_TRUE(unknown->segments.empty());
}

}  // namespace
