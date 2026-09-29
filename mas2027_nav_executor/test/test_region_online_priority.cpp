#include "mas2027_nav_executor/common/environment/region_control.hpp"

#include <gtest/gtest.h>

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

}  // namespace
