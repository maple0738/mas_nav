#include <gtest/gtest.h>

#include "mas2027_nav_executor/path_planner/trajectory/replan_continuity.hpp"

namespace {

using mas2027_nav_executor::selectReplanContinuityPrefix;

TEST(ReplanContinuity, DoesNotUseWaypointBeyondTimeWindow)
{
  const std::vector<Eigen::Vector3d> new_path = {
    {0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.4, 0.0, 0.0}};
  const super_utils::vec_Vec3f old_path = {
    {0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.4, 0.0, 0.0}};
  super_utils::VecDf durations(2);
  durations << 0.8, 0.2;

  EXPECT_EQ(selectReplanContinuityPrefix(new_path, old_path, durations, 0.3), 0);
  durations[0] = 0.3;
  EXPECT_EQ(selectReplanContinuityPrefix(new_path, old_path, durations, 0.3), 1);
  EXPECT_EQ(selectReplanContinuityPrefix(new_path, old_path, durations, 0.0), 0);
}

TEST(ReplanContinuity, ShortensPrefixAtRouteJump)
{
  const std::vector<Eigen::Vector3d> new_path = {
    {0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {0.4, 0.0, 0.0}, {0.6, 0.0, 0.0}};
  const super_utils::vec_Vec3f old_path = {
    {0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}, {1.5, 0.0, 0.0}, {1.7, 0.0, 0.0}};
  super_utils::VecDf durations(3);
  durations << 0.1, 0.1, 0.1;

  EXPECT_EQ(selectReplanContinuityPrefix(new_path, old_path, durations, 0.3), 1);
  EXPECT_EQ(selectReplanContinuityPrefix(new_path, old_path, durations, 0.05), 0);
}

}
