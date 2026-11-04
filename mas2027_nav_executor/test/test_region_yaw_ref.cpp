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
    controller.setUndulatingAlignmentLatch(false, 3.2);
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


TEST(RegionYawRef, UndulatingLatchHoldsFixedDistanceAfterLabelsDisappear)
{
  auto undulating = std::make_shared<RegionPlan>();
  undulating->trajectory_id = 1;
  undulating->positions_odom = {{0, 0}, {6, 0}};
  undulating->arc_lengths = {0, 6};
  undulating->segments.push_back({7, 7, 1, 2, 0.5, 0.8, 0.9, 1.0, 2.8, 3.0});
  RegionController controller(4, 3, 4, 2, 4);
  controller.setUndulatingAlignmentLatch(true, 3.2);
  controller.setPlan(undulating);
  controller.update({0, 0});
  controller.update({0.6, 0});
  controller.update({1.0, 0});
  ASSERT_EQ(controller.mode(), 7);
  ASSERT_TRUE(controller.label());
  EXPECT_EQ(*controller.label(), 7);
  EXPECT_TRUE(controller.alignmentActive());
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0), 0.0, 1e-8);

  // A planner gap must not release the fixed-length alignment.
  controller.setPlan(nullptr);
  controller.update({1.2, 0});
  EXPECT_EQ(controller.mode(), 7);
  EXPECT_TRUE(controller.label());
  EXPECT_TRUE(controller.alignmentActive());
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));

  // A fresh flat plan has no label 7, but the latch still holds for 3.2 m.
  auto flat = std::make_shared<RegionPlan>();
  flat->trajectory_id = 2;
  flat->positions_odom = {{0, 0}, {8, 0}};
  flat->arc_lengths = {0, 8};
  controller.setPlan(flat);
  controller.update({2.0, 0});
  EXPECT_EQ(controller.mode(), 7);
  ASSERT_TRUE(controller.label());
  EXPECT_EQ(*controller.label(), 7);
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({4.1, 0});  // 3.1 m from the entry
  EXPECT_EQ(controller.mode(), 7);
  controller.update({4.3, 0});  // 3.3 m, past the latch
  EXPECT_EQ(controller.mode(), 4);
  EXPECT_FALSE(controller.label());
}

TEST(RegionYawRef, OrdinaryPlanNeverRequestsAlignment)
{
  RegionController controller(4,3,4,2,4);
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  auto ordinary=makePlan();ordinary->segments.clear();controller.setPlan(ordinary);
  for(const auto &position:ordinary->positions_odom) {
    controller.update(position);
    EXPECT_EQ(controller.mode(),4U);
    EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_FALSE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0),.05).follow_gimbal);
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

TEST(RegionYawRef, ReplanningAtThePreparationBoundaryKeepsTheLowerFollowBaseline)
{
  RegionController controller(4, 3, 4, 2, 4);
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  controller.setPlan(makePlan());
  controller.update({0, 0});
  controller.update({1.1, 0});
  ASSERT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
  const double angle = adapter.integrateMpcYawRate(1.0, 0.05);
  auto replanned = makePlan();
  replanned->trajectory_id = 2;
  replanned->segments.front().prepare_s = 1.15;
  controller.setPlan(replanned);
  controller.update({1.1, 0});
  EXPECT_EQ(controller.phase(), RegionPhase::PREPARING);
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0), 0.05).follow_gimbal);
  EXPECT_DOUBLE_EQ(adapter.commandDegrees(), angle);
}

TEST(RegionYawRef, MissingSemanticSamplesKeepAlignmentUntilTheReleaseDistanceIsTravelled)
{
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(makePlan());
  controller.update({0, 0});
  controller.update({1.6, 0});
  ASSERT_EQ(controller.mode(), 5U);
  auto gap = makePlan();
  gap->trajectory_id = 2;
  gap->segments.clear();
  controller.setPlan(gap);
  controller.update({1.7, 0});
  EXPECT_EQ(controller.mode(), 5U);
  EXPECT_EQ(controller.phase(), RegionPhase::RELEASING);
  EXPECT_EQ(controller.label(), 5U);
  EXPECT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({2.2, 0});
  EXPECT_EQ(controller.mode(), 4U);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, ACancelledPlanClearsRetainedAlignmentAndMode)
{
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(makePlan());
  controller.update({0, 0});
  controller.update({1.6, 0});
  ASSERT_EQ(controller.mode(), 5U);
  controller.setPlan(nullptr);
  EXPECT_EQ(controller.mode(), 4U);
  EXPECT_FALSE(controller.label());
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, BrakingDistancePreparesBeforeTheFixedWindowAndSurvivesStopping)
{
  RegionController controller(4, 3, 4, 2, 4);
  controller.setPlan(makePlan());
  controller.update({0, 0}, 2.5);
  EXPECT_EQ(controller.phase(), RegionPhase::PREPARING);
  EXPECT_EQ(controller.mode(), 4U);  // Activation stays at the configured point.
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({0.1, 0}, 0.0);
  EXPECT_EQ(controller.phase(), RegionPhase::PREPARING);
  EXPECT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, ATrajectoryEndingInsideTheRegionHasFiniteSpatialRelease)
{
  RegionController controller(4, 3, 4, 2, 4);
  auto plan = makePlan();
  plan->segments.front().exit_s = std::numeric_limits<double>::infinity();
  plan->segments.front().release_s = std::numeric_limits<double>::infinity();
  plan->segments.front().release_distance = 0.5;
  controller.setPlan(plan);
  controller.update({0, 0});
  controller.update({1.6, 0});
  auto gap = makePlan();
  gap->segments.clear();
  controller.setPlan(gap);
  controller.update({1.7, 0});
  EXPECT_EQ(controller.mode(), 5U);
  EXPECT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({2.2, 0});
  EXPECT_EQ(controller.mode(), 4U);
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

namespace mas2027_nav_executor {
TEST(GimbalFollowAdapter, SwitchesAxisInsteadOfSaturatingAtTheCommandLimit)
{
  // Ideal servo: the measured base_link yaw equals the integrated command.
  // Path direction 155 deg; starting near -170 the nearer axis needs -205 deg,
  // beyond the non-wrapping +/-180 deg position command. Without an axis switch
  // the command saturates at -180 with a constant 25 deg error.
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  const auto error_to = [](double path_deg, double yaw_deg) {
    return std::remainder(path_deg - yaw_deg, 360.0);
  };
  for (int i = 0; i <= 340 + 100; ++i) {  // sweep the path slowly to -170 deg
    const double path = -0.5 * std::min(i, 340);
    adapter.updateHeading(error_to(path, adapter.commandDegrees()), 0.05);
    adapter.integrateMpcYawRate(0.0, 0.05);
  }
  ASSERT_NEAR(adapter.commandDegrees(), -170.0, 1.0);
  bool ready = false;
  for (int i = 0; i < 400 && !ready; ++i) {
    ready = adapter.updateHeading(error_to(155.0, adapter.commandDegrees()), 0.05).heading_ready;
    adapter.integrateMpcYawRate(0.0, 0.05);
    ASSERT_LE(std::abs(adapter.commandDegrees()), 180.0);
  }
  EXPECT_TRUE(ready);
  EXPECT_NEAR(std::abs(std::remainder(155.0 - adapter.commandDegrees(), 180.0)), 0.0, 5.0);
}

TEST(RegionYawRef, HairpinMovesToRampFrontBeforeStartingAlignment)
{
  auto plan=std::make_shared<RegionPlan>();plan->trajectory_id=99;
  plan->positions_odom={{0,0},{1,0},{1.6,0},{1.6,0.7},{1.6,1.5}};
  plan->arc_lengths={0,1,1.6,2.3,3.1};
  RegionSegment segment{5,5,4,5,0,2.1,2.2,2.3,3.1,3.6};
  segment.entry_position={1.6,0.7};segment.entry_axis={0,1};
  plan->segments.push_back(segment);
  RegionController controller(4,3,4,2,4);controller.setFrontEntryGate(true);
  controller.setPlan(plan);
  for(const Eigen::Vector2d p : std::vector<Eigen::Vector2d>{{0,0},{0.8,0},{1.4,0},{1.5,0}}) {
    controller.update(p,5.0); // even a large braking lookahead must not align on the side
    EXPECT_EQ(controller.mode(),4);EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
    EXPECT_FALSE(controller.alignmentActive());
  }
  controller.update({1.6,0.1},5.0);
  EXPECT_EQ(controller.phase(),RegionPhase::PREPARING);
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0),90,1e-9);
  controller.update({1.6,0.55},5.0);EXPECT_EQ(controller.mode(),5);
  controller.update({1.6,0.85},5.0);EXPECT_EQ(controller.phase(),RegionPhase::INSIDE);
}
TEST(RegionYawRef, FrontGateUsesActualPositionAndDoesNotRetainSideApproachAfterReplan)
{
  auto plan=makePlan();plan->positions_odom={{0,0},{2,0},{2,2},{4,2}};
  plan->segments[0].entry_position={2,0};plan->segments[0].entry_axis={0,1};
  RegionController controller(4,3,4,2,4);controller.setFrontEntryGate(true);
  controller.setPlan(plan);controller.update({0,0},4);controller.update({1.9,0},4);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
  // A short-lived old prepare phase must not re-enable follow from memory.
  auto next=std::make_shared<RegionPlan>(*plan);next->trajectory_id++;
  controller.setPlan(next);controller.update({1.9,0},4);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, FrontEntryDoesNotRestartAlignmentForOneCellOfLateralJitter)
{
  auto plan = std::make_shared<RegionPlan>();
  plan->trajectory_id = 100;
  plan->positions_odom = {{0, 0}, {0, 1}, {0, 2}};
  plan->arc_lengths = {0, 1, 2};
  RegionSegment segment{5, 5, 4, 5, 0, 0.8, 0.9, 1, 1.5, 2};
  segment.entry_position = {0, 1};
  segment.entry_axis = {0, 1};
  plan->segments.push_back(segment);
  RegionController controller(4, 3, 4, 2, 4);
  controller.setFrontEntryGate(true);
  controller.setPlan(plan);
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  for (const double lateral : {0.14, 0.16, 0.14, 0.19, 0.16}) {
    auto replanned = std::make_shared<RegionPlan>(*plan);
    replanned->trajectory_id++;
    controller.setPlan(replanned);
    controller.update({lateral, 0.2}, 1.0);
    const auto follow = adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(kPi / 2), 0.1);
    EXPECT_TRUE(follow.follow_gimbal) << lateral;
  }
  EXPECT_TRUE(adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(kPi / 2), 0.1).heading_ready);
  // Hysteresis cannot start alignment from the side or retain a real departure.
  controller.update({0.26, 0.2}, 1.0);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({0.19, 0.2}, 1.0);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(RegionYawRef, FrontEntryKeepsSpinningUntilTheConfiguredPrepareWindowEvenAtHighSpeed)
{
  auto plan = std::make_shared<RegionPlan>();
  plan->trajectory_id = 120;
  plan->positions_odom = {{0, 0}, {0, 1}, {0, 2}};
  plan->arc_lengths = {0, 1, 2};
  RegionSegment segment{5, 5, 4, 5, 0.7, 0.8, 0.9, 1, 1.5, 2};
  segment.entry_position = {0, 1};
  segment.entry_axis = {0, 1};
  plan->segments.push_back(segment);
  RegionController controller(4, 3, 4, 2, 4);
  controller.setFrontEntryGate(true);
  controller.setPlan(plan);
  for (const double y : {0.0, 0.2, 0.5, 0.69}) {
    controller.update({0, y}, 2.5);
    EXPECT_EQ(controller.phase(), RegionPhase::NORMAL);
    EXPECT_FALSE(controller.alignmentActive());
    EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
    ASSERT_TRUE(controller.distanceToSlopeAlignment());
    EXPECT_NEAR(*controller.distanceToSlopeAlignment(), 0.7 - y, 1e-9);
  }
  controller.update({0, 0.71}, 2.5);
  EXPECT_EQ(controller.phase(), RegionPhase::PREPARING);
  EXPECT_TRUE(controller.alignmentActive());
  EXPECT_FALSE(controller.distanceToSlopeAlignment());
  controller.update({0, 0.9}, 0.0);
  EXPECT_EQ(controller.mode(), 5);
  EXPECT_TRUE(controller.alignmentActive());
  controller.setPlan(nullptr);
  EXPECT_FALSE(controller.distanceToSlopeAlignment());
}

TEST(RegionYawRef, LateralHysteresisDoesNotTransferToADifferentRampAxis)
{
  auto plan = std::make_shared<RegionPlan>();
  plan->trajectory_id = 110;
  plan->positions_odom = {{0, 0}, {0, 1}, {0, 2}};
  plan->arc_lengths = {0, 1, 2};
  RegionSegment segment{5, 5, 4, 5, 0, 0.8, 0.9, 1, 1.5, 2};
  segment.entry_position = {0, 1};
  segment.entry_axis = {0, 1};
  plan->segments.push_back(segment);
  RegionController controller(4, 3, 4, 2, 4);
  controller.setFrontEntryGate(true);
  controller.setPlan(plan);
  controller.update({0.14, 0.81}, 1.0);
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  auto next = std::make_shared<RegionPlan>(*plan);
  next->trajectory_id++;
  next->positions_odom = {{-0.2, 0.81}, {0.8, 0.81}, {1.8, 0.81}};
  next->segments[0].entry_position = {0.1, 1};
  next->segments[0].entry_axis = {1, 0};
  controller.setPlan(next);
  controller.update({-0.1, 0.81}, 1.0);
  EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
}

TEST(GimbalFollowAdapter, FasterHeadingTrimKeepsTheRateAndReadinessLimits)
{
  const auto settle = [](double gain) {
    GimbalFollowParams params;
    params.heading_error_gain_s = gain;
    GimbalFollowAdapter adapter(params);
    double previous = 0.0;
    for (int i = 0; i < 200; ++i) {
      // Ideal lower angle servo, used only to compare the adapter's convergence.
      // This is not a model of the real chassis or the Gazebo dynamics.
      const double error = 60.0 - previous;
      const auto output = adapter.updateHeading(error, 0.05);
      const double command = adapter.integrateMpcYawRate(0.0, 0.05);
      EXPECT_LE(std::abs(command - previous), params.max_heading_rate_deg_s * 0.05 + 1e-9);
      previous = command;
      if (output.heading_ready) {
        EXPECT_LE(std::abs(error), params.ready_tolerance_deg);
        return (i + 1) * 0.05;
      }
    }
    ADD_FAILURE() << "Heading trim never settled";
    return 10.0;
  };
  EXPECT_LT(settle(2.0), 0.7 * settle(1.0));
}
}  // namespace mas2027_nav_executor

namespace mas2027_nav_executor {
TEST(RegionYawRef, TunnelWaitsForHeadingReadinessAndTracksTheCurrentPath)
{
  RegionController controller(4,3,4,2,4);
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  controller.setPlan(makePlan(6));controller.update({0,0});controller.update({1.1,0});
  ASSERT_EQ(controller.label(),6U);
  ASSERT_TRUE(requiresPathAlignment(controller.label()));
  auto follow=adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(kPi/2),.1);
  EXPECT_TRUE(follow.follow_gimbal);EXPECT_FALSE(follow.heading_ready);
  for(int i=0;i<4;++i)follow=adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0),.1);
  EXPECT_TRUE(follow.follow_gimbal);EXPECT_TRUE(follow.heading_ready);
  controller.update({1.6,0});EXPECT_EQ(controller.mode(),6U);
  controller.update({2,1});
  follow=adapter.updateHeading(controller.trajectoryHeadingErrorDegrees(0),.1);
  EXPECT_TRUE(follow.follow_gimbal);EXPECT_FALSE(follow.heading_ready);
  EXPECT_NEAR(*controller.trajectoryHeadingErrorDegrees(0),90.,1e-8);
}
TEST(RegionYawRef, TunnelSemanticGapRetainsAlignmentOnlyWithinReleaseDistance)
{
  RegionController controller(4,3,4,2,4);
  controller.setPlan(makePlan(6));controller.update({0,0});controller.update({1.6,0});
  ASSERT_EQ(controller.mode(),6U);
  auto gap=makePlan(6);gap->trajectory_id=2;gap->segments.clear();
  controller.setPlan(gap);controller.update({1.7,0});
  EXPECT_EQ(controller.mode(),6U);EXPECT_TRUE(controller.alignmentActive());
  EXPECT_TRUE(requiresPathAlignment(controller.label()));
  ASSERT_TRUE(controller.trajectoryHeadingErrorDegrees(0));
  controller.update({2.2,0});
  EXPECT_EQ(controller.mode(),4U);EXPECT_FALSE(controller.trajectoryHeadingErrorDegrees(0));
  EXPECT_FALSE(requiresPathAlignment(controller.label()));
}
}  // namespace mas2027_nav_executor
