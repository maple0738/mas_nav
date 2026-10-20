#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>
#include "mas2027_nav_executor/common/environment/region_control.hpp"
#include "mas2027_nav_executor/common/control/gimbal_follow_adapter.hpp"

namespace mas2027_nav_executor {
namespace {
constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<RegionPlan> makePlan(uint8_t label = 5)
{
  auto plan = std::make_shared<RegionPlan>();
  plan->trajectory_id = 1;
  // A right-angle turn at the terrain entry: the entry direction is +Y.
  plan->positions_odom = {{0, 0}, {2, 0}, {2, 2}, {4, 2}};
  plan->arc_lengths = {0, 2, 4, 6};
  plan->segments.push_back({label, label, 1, 2, 1, 1.5, 1.8, 2, 4, 4.5});
  return plan;
}

TEST(RegionYawRef, StartsBeforeActivationAndPersistsThroughReleaseWindow)
{
  for (const uint8_t label : {5, 6, 7}) {
    RegionController controller(4, 3, 4, 2, 4);
    GimbalFollowAdapter adapter(GimbalFollowParams{});
    controller.setPlan(makePlan(label));
    controller.update({0, 0});
    EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_FALSE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
    controller.update({1.1, 0});
    EXPECT_EQ(controller.phase(), RegionPhase::PREPARING);
    ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 0.0, 1e-8);
    EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
    controller.update({1.6, 0});
    EXPECT_EQ(controller.phase(), RegionPhase::ARMED);
    EXPECT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
    controller.update({1.9, 0});
    EXPECT_EQ(controller.phase(), RegionPhase::COMMITTED);
    EXPECT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
    controller.update({2, 1});
    EXPECT_EQ(controller.phase(), RegionPhase::INSIDE);
    EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(kPi / 4), 45.0, 1e-8);
    EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(kPi / 4), 0.05).follow_gimbal);
    controller.update({2.2, 2});
    EXPECT_EQ(controller.phase(), RegionPhase::RELEASING);
    EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(kPi / 2), -90.0, 1e-8);
    EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(kPi / 2), 0.05).follow_gimbal);
    controller.update({2.6, 2});
    EXPECT_EQ(controller.phase(), RegionPhase::NORMAL);
    EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_FALSE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
  }
}

TEST(RegionYawRef, DegreeReferenceWrapsAt180AndRejectsInvalidRadianYaw)
{
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(makePlan());
  controller.update({0, 0});
  controller.update({1.1, 0});
  EXPECT_NEAR(std::abs(*controller.trajectoryHeadingErrorDegrees(-kPi)), 180.0, 1e-8);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(std::numeric_limits<double>::infinity()));
}

TEST(RegionYawRef, CancelsOnMissingPlanNewTrajectoryOrInvalidPosition)
{
  RegionController controller(4, 3, 4, 2, 4);
  auto plan = makePlan();
  controller.setPlan(plan);
  controller.update({0, 0});
  controller.update({1.1, 0});
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({std::numeric_limits<double>::quiet_NaN(), 0});
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({1.1, 0});
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.setPlan(nullptr);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
  auto normal = makePlan();
  normal->trajectory_id = 2;
  normal->segments.clear();
  controller.setPlan(normal);
  controller.update({0, 0});
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, SkipsDuplicateEntryPointsAndClearsOnSemanticRefresh)
{
  auto plan = makePlan();
  plan->positions_odom.insert(plan->positions_odom.begin() + 2, {2, 0});
  plan->arc_lengths.insert(plan->arc_lengths.begin() + 2, 2);
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(plan);
  controller.update({0, 0});
  controller.update({1.1, 0});
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 0.0, 1e-8);
  controller.update({2, 0});
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 90.0, 1e-8);
  auto refreshed = makePlan();
  refreshed->segments.clear();
  controller.setPlan(refreshed);
  controller.update({1.1, 0});
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}
}  // namespace
}  // namespace mas2027_nav_executor



namespace mas2027_nav_executor {
TEST(GimbalFollowAdapter, IntegratesMpcOmegaAndHoldsTheFinalAngle)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  EXPECT_TRUE(adapter.updateHeading(0.0, 0.05).follow_gimbal);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(1.0, 0.05),
    0.05 * 180.0 / 3.14159265358979323846);
  const double angle = adapter.integrateMpcYawRate(0.0, 0.05);
  for (int i = 0; i < 10; ++i) {
    adapter.updateHeading(0.0, 0.05);
    EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(0.0, 0.05), angle);
  }
  EXPECT_TRUE(adapter.updateHeading(0.0, 0.05).heading_ready);
}
TEST(GimbalFollowAdapter, HeadingTrimAlignsTheAxisWhenMpcYawIsZero)
{
  GimbalFollowParams params;
  params.heading_error_gain_s = 1.0;
  params.heading_deadband_deg = 0.5;
  params.max_heading_rate_deg_s = 30.0;
  GimbalFollowAdapter adapter(params);
  adapter.updateHeading(80.0, 0.05);
  const double first = adapter.integrateMpcYawRate(0.0, 0.05);
  EXPECT_NEAR(first, 1.5, 1e-9);  // 30 deg/s limit, then 0.05 s.
  double previous = first;
  for (int i = 0; i < 200; ++i) {
    adapter.updateHeading(80.0 - previous, 0.05);
    const double angle = adapter.integrateMpcYawRate(0.0, 0.05);
    EXPECT_GE(angle, previous);
    EXPECT_LE(angle - previous, 1.5 + 1e-9);
    previous = angle;
  }
  EXPECT_NEAR(previous, 79.5, 0.6);
  adapter.updateHeading(0.25, 0.05);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(0.0, 0.05), previous);
}
TEST(GimbalFollowAdapter, HeadingTrimClosesTheMeasuredAxisWithServoLag)
{
  GimbalFollowParams params;
  params.heading_error_gain_s = 1.0;
  params.heading_deadband_deg = 0.5;
  params.max_heading_rate_deg_s = 60.0;
  GimbalFollowAdapter adapter(params);
  double measured_yaw_deg = 0.0;
  const double target_deg = 80.0;
  constexpr double kLowerDeadbandDeg = 12.0;
  for (int i = 0; i < 500; ++i) {
    adapter.updateHeading(target_deg - measured_yaw_deg, 0.05);
    const double command_deg = adapter.integrateMpcYawRate(0.0, 0.05);
    const double delta = command_deg - measured_yaw_deg;
    if (std::abs(delta) > kLowerDeadbandDeg) {
      measured_yaw_deg += 0.4 * (delta - std::copysign(kLowerDeadbandDeg, delta));
    }
  }
  EXPECT_NEAR(measured_yaw_deg, target_deg, 1.0);
}
TEST(GimbalFollowAdapter, SettlingHysteresisAndReset)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  EXPECT_FALSE(adapter.updateHeading(0.0, 0.1).heading_ready);
  EXPECT_FALSE(adapter.updateHeading(0.0, 0.1).heading_ready);
  EXPECT_TRUE(adapter.updateHeading(0.0, 0.1).heading_ready);
  EXPECT_TRUE(adapter.updateHeading(6.0, 0.05).heading_ready);
  EXPECT_FALSE(adapter.updateHeading(9.0, 0.05).heading_ready);
  adapter.integrateMpcYawRate(1.0, 0.05);
  EXPECT_FALSE(adapter.updateHeading(std::nullopt, 0.05).follow_gimbal);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(1.0, 0.05), 0.0);
  EXPECT_FALSE(adapter.updateHeading(std::numeric_limits<double>::quiet_NaN(), 0.05).follow_gimbal);
}
TEST(GimbalFollowAdapter, PositionLimitDoesNotWrapAndInvalidOmegaDoesNotMoveTarget)
{
  GimbalFollowParams params;
  params.heading_error_gain_s = 0.0;
  GimbalFollowAdapter adapter(params);
  adapter.updateHeading(120.0, 0.05);
  for (int i = 0; i < 1000; ++i) adapter.integrateMpcYawRate(2.0, 0.05);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(2.0, 0.05), 180.0);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(std::numeric_limits<double>::quiet_NaN(), 0.05), 180.0);
  for (int i = 0; i < 1000; ++i) adapter.integrateMpcYawRate(-2.0, 0.05);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(-2.0, 0.05), -180.0);
}
TEST(RegionYawRef, HeadingFollowsTheForwardTangentInsideTheRegion)
{
  auto plan = makePlan();
  plan->segments[0].exit_s = 5.5;
  plan->segments[0].release_s = 6;
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(plan);
  controller.update({0, 0});controller.update({1.1, 0});
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 0, 1e-8);
  controller.update({2, 1});
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 90, 1e-8);
  controller.update({3, 2});
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 0, 1e-8);
}
}  // namespace mas2027_nav_executor

namespace mas2027_nav_executor {
TEST(GimbalFollowAdapter, SelectsTheNearerReverseAxisAndAcceptsOppositeAlignment)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  const auto selected = adapter.updateHeading(-118.0, 0.05);
  EXPECT_TRUE(selected.reverse_heading);
  EXPECT_DOUBLE_EQ(selected.heading_error_deg, 62.0);
  EXPECT_FALSE(selected.heading_ready);
  for (int i = 0; i < 6; ++i) adapter.updateHeading(180.0, 0.05);
  const auto aligned = adapter.updateHeading(-180.0, 0.05);
  EXPECT_TRUE(aligned.reverse_heading);
  EXPECT_NEAR(aligned.heading_error_deg, 0.0, 1e-8);
  EXPECT_TRUE(aligned.heading_ready);
  GimbalFollowParams legacy; legacy.allow_reverse = false;
  GimbalFollowAdapter forward_only(legacy);
  const auto forward = forward_only.updateHeading(-118.0, 0.05);
  EXPECT_FALSE(forward.reverse_heading);
  EXPECT_DOUBLE_EQ(forward.heading_error_deg, -118.0);
}
TEST(GimbalFollowAdapter, DirectionHysteresisAvoidsFlippingNearNinetyDegrees)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  EXPECT_FALSE(adapter.updateHeading(89.0, 0.05).reverse_heading);
  EXPECT_FALSE(adapter.updateHeading(91.0, 0.05).reverse_heading);
  EXPECT_FALSE(adapter.updateHeading(99.0, 0.05).reverse_heading);
  EXPECT_TRUE(adapter.updateHeading(101.0, 0.05).reverse_heading);
  EXPECT_TRUE(adapter.updateHeading(89.0, 0.05).reverse_heading);
  EXPECT_TRUE(adapter.updateHeading(81.0, 0.05).reverse_heading);
  EXPECT_FALSE(adapter.updateHeading(79.0, 0.05).reverse_heading);
  adapter.updateHeading(std::nullopt, 0.05);
  EXPECT_TRUE(adapter.updateHeading(-179.0, 0.05).reverse_heading);
  EXPECT_NEAR(adapter.updateHeading(179.0, 0.05).heading_error_deg, -1.0, 1e-8);
}
TEST(GimbalFollowAdapter, HoldsLastAngleAcrossAPlannerGapAndResumesFromIt)
{
  GimbalFollowParams params;
  params.heading_error_gain_s = 0.0;
  GimbalFollowAdapter adapter(params);
  EXPECT_FALSE(adapter.hasCommand());
  EXPECT_TRUE(adapter.updateHeading(30.0, 0.05).follow_gimbal);
  for (int i = 0; i < 3; ++i) adapter.integrateMpcYawRate(1.0, 0.05);
  const double held_angle = adapter.commandDegrees();
  EXPECT_GT(held_angle, 0.0);

  const auto held = adapter.hold();
  EXPECT_TRUE(held.follow_gimbal);
  EXPECT_FALSE(held.heading_ready);
  EXPECT_TRUE(adapter.hasCommand());
  EXPECT_DOUBLE_EQ(adapter.commandDegrees(), held_angle);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(1.0, 0.05), held_angle);

  EXPECT_TRUE(adapter.updateHeading(30.0, 0.05).follow_gimbal);
  EXPECT_DOUBLE_EQ(adapter.commandDegrees(), held_angle);
  EXPECT_GT(adapter.integrateMpcYawRate(1.0, 0.05), held_angle);

  adapter.updateHeading(std::nullopt, 0.05);
  EXPECT_FALSE(adapter.hasCommand());
  EXPECT_DOUBLE_EQ(adapter.commandDegrees(), 0.0);
}

TEST(GimbalFollowAdapter, PlannerGapPreservesReadyHeadingAndReverseSelection)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  // Facing the reverse path axis is ready after the normal 0.3 s settle.
  EXPECT_TRUE(adapter.updateHeading(180.0, 0.1).reverse_heading);
  EXPECT_FALSE(adapter.updateHeading(180.0, 0.1).heading_ready);
  EXPECT_TRUE(adapter.updateHeading(180.0, 0.1).heading_ready);
  const auto held = adapter.hold();
  EXPECT_TRUE(held.follow_gimbal);
  EXPECT_FALSE(held.heading_ready);  // The node still commands zero translation.
  EXPECT_TRUE(held.reverse_heading);
  EXPECT_TRUE(adapter.updateHeading(180.0, 0.05).heading_ready);

  adapter.hold();
  // A changed measured heading must revoke readiness on the first resumed tick.
  const auto misaligned = adapter.updateHeading(165.0, 0.05);
  EXPECT_TRUE(misaligned.reverse_heading);
  EXPECT_FALSE(misaligned.heading_ready);
  adapter.hold();
  // The reverse choice keeps its 10 degree hysteresis through another gap.
  EXPECT_TRUE(adapter.updateHeading(85.0, 0.05).reverse_heading);
}

TEST(GimbalFollowAdapter, ReversingTravelOnTheSameAxisKeepsHeadingReady)
{
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  adapter.updateHeading(0.0, 0.1);
  adapter.updateHeading(0.0, 0.1);
  ASSERT_TRUE(adapter.updateHeading(0.0, 0.1).heading_ready);
  const auto reversed = adapter.updateHeading(180.0, 0.05);
  EXPECT_TRUE(reversed.reverse_heading);
  EXPECT_NEAR(reversed.heading_error_deg, 0.0, 1.0e-8);
  EXPECT_TRUE(reversed.heading_ready);
  EXPECT_FALSE(adapter.updateHeading(165.0, 0.05).heading_ready);
}

TEST(GimbalFollowAdapter, ReversingTheRequestedTurnCanLeaveTheAngleLimit)
{
  GimbalFollowParams params;
  params.heading_error_gain_s = 0.0;
  GimbalFollowAdapter adapter(params);
  adapter.updateHeading(-118.0, 0.05);
  for (int i = 0; i < 40; ++i) adapter.integrateMpcYawRate(-2.0, 0.05);
  EXPECT_DOUBLE_EQ(adapter.integrateMpcYawRate(-2.0, 0.05), -180.0);
  const auto reverse = adapter.updateHeading(-118.0, 0.05);
  EXPECT_GT(reverse.heading_error_deg, 0.0);
  EXPECT_GT(adapter.integrateMpcYawRate(0.2, 0.05), -180.0);
}
}  // namespace mas2027_nav_executor
