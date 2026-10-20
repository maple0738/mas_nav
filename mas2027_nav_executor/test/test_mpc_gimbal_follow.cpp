#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <gtest/gtest.h>
#include "mas2027_nav_executor/path_executor/path_executor.hpp"
#include "mas2027_nav_executor/common/control/gimbal_follow_adapter.hpp"

namespace mas2027_nav_executor {
namespace {
constexpr double kPi = 3.14159265358979323846;
class FreeMap final : public rog_map::MapQueryInterface
{
public:
  double stamp{10.0};
  double distance{10.0};
  std::function<double(const Eigen::Vector3d &)> distance_fn;
  std::function<Eigen::Vector3d(const Eigen::Vector3d &)> gradient_fn;
  bool worldToMap(double, double, unsigned int & x, unsigned int & y) const override {x = y = 0; return true;}
  void mapToWorld(unsigned int, unsigned int, double & x, double & y) const override {x = y = 0;}
  unsigned int sizeX() const override {return 1;}
  unsigned int sizeY() const override {return 1;}
  double resolution() const override {return 0.05;}
  double originX() const override {return 0.0;}
  double originY() const override {return 0.0;}
  double snapshotStampSeconds() const override {return stamp;}
  uint8_t value(unsigned int, unsigned int) const override {return 0;}
  const unsigned char * values() const override {static const unsigned char data = 0; return &data;}
  bool isValid(unsigned int, unsigned int) const override {return true;}
  bool isFree(unsigned int, unsigned int) const override {return true;}
  bool evaluate(const Eigen::Vector3d & pos, double & d, Eigen::Vector3d & gradient) const override {
    d = distance_fn ? distance_fn(pos) : distance; gradient.setZero(); return true;
  }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override {
    rog_map::QueryResult result; result.ok = true; result.status = rog_map::QueryStatus::OK;
    result.distance = distance_fn ? distance_fn(pos) : distance;
    result.gradient = gradient_fn ? gradient_fn(pos) : Eigen::Vector3d::Zero();
    return result;
  }
};
double betweenSideWalls(const Eigen::Vector3d & pos, double half_width)
{
  return std::min(std::abs(pos.x() - half_width), std::abs(pos.x() + half_width));
}
minco_controller::MPCConfig config()
{
  minco_controller::MPCConfig c; c.dt = 0.05; c.horizon = 20;
  c.use_acc_constraints = true; c.omega_min = -2; c.omega_max = 2; c.alpha_min = -4; c.alpha_max = 4;
  return c;
}
// 高速用例需要放开默认 ±1 m/s 的速度约束，否则测不出接近减速包络的差别。
minco_controller::MPCConfig fastConfig()
{
  auto c = config();
  c.vx_min = -3.0; c.vx_max = 3.0; c.vy_min = -3.0; c.vy_max = 3.0;
  return c;
}
PathExecutorParams params()
{
  PathExecutorParams p; p.planner_frequency = 20; p.trajectory_timeout_s = 0.6; p.odom_timeout_s = 0.5;
  p.output_in_body_frame = true; p.odom_frame = "odom"; p.deadzone_speed = 0.02;
  p.rog_map_clearance = 0.28; p.rog_map_timeout_s = 0.5; return p;
}
ExecutorInput input()
{
  ExecutorInput in; in.stamp = rclcpp::Time(10, 0, RCL_ROS_TIME); in.allow_motion = true;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(); odom->header.stamp = in.stamp; odom->pose.pose.orientation.w = 1;
  in.odom = odom;
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(); trajectory->header.stamp = in.stamp;
  trajectory->header.frame_id = "odom";
  for (int i = 0; i <= 30; ++i) {
    interfaces::msg::PositionCommand command; command.position.y = i * 0.05; command.velocity.y = 1.0;
    command.yaw = 0; command.yaw_dot = 0; trajectory->cmds.push_back(command);
  }
  in.trajectory = trajectory; return in;
}
TEST(MpcGimbalFollow, TracksForwardMincoTangentInsteadOfTheOldYawField)
{
  auto map = std::make_shared<FreeMap>(); PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true;
  const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_GT(result.command.angular.z, 0.0); EXPECT_LE(result.command.angular.z, 0.20001);
  EXPECT_GT(result.command.linear.y, 0.0);
  PathExecutor ordinary(config(), params(), nullptr, nullptr, map); in.follow_gimbal = false;
  EXPECT_NEAR(ordinary.computeCommand(in).command.angular.z, 0.0, 1e-6);
}
TEST(MpcGimbalFollow, SecondReferenceSlowsProgressInANarrowBand)
{
  auto open_map = std::make_shared<FreeMap>();
  open_map->distance = 0.40;
  auto narrow_map = std::make_shared<FreeMap>();
  narrow_map->distance = 0.25;
  auto c = config();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 2.0;  // No output cap masks the reference change.
  p.secondary_reference_enable = true;
  p.secondary_reference_narrow_time_scale = 0.50;
  PathExecutor open(c, p, nullptr, nullptr, open_map);
  PathExecutor narrow(c, p, nullptr, nullptr, narrow_map);
  const auto straight = input();
  const auto open_output = open.computeCommand(straight);
  const auto narrow_output = narrow.computeCommand(straight);
  ASSERT_EQ(open_output.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(narrow_output.status, ExecutorStatus::PUBLISHED);
  EXPECT_LT(narrow_output.command.linear.y, open_output.command.linear.y - 0.05);
  EXPECT_FALSE(open_output.secondary_reference_applied);
  EXPECT_TRUE(narrow_output.secondary_reference_applied);
  EXPECT_TRUE(narrow_output.secondary_reference_narrow);
}

TEST(MpcGimbalFollow, SafeNarrowSamplesSurviveOneBelowFloorAhead)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & point) {
    if (point.y() >= 0.75) return 0.23;
    if (point.y() >= 0.20) return 0.25;
    return 0.40;
  };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 0.20;
  p.secondary_reference_enable = true;
  p.secondary_reference_narrow_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto result = executor.computeCommand(input());
  EXPECT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_TRUE(result.secondary_reference_narrow);
}

TEST(MpcGimbalFollow, BrakesToNarrowSpeedAtTheEntranceInsteadOfAcrossTheLookahead)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & point) {
    return point.y() >= 1.0 ? 0.25 : 0.40;
  };
  auto c = fastConfig();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 0.5;
  p.clearance_braking.deceleration = 1.2;
  p.clearance_braking.reaction_time = 0.2;
  auto run_at = [&](double y) {
    PathExecutor executor(c, p, nullptr, nullptr, map);
    auto in = input();
    auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
    odom->pose.pose.position.y = y;
    in.odom = odom;
    return executor.computeCommand(in);
  };
  const auto approach = run_at(0.0);
  const auto entrance = run_at(0.9);
  const auto inside = run_at(1.0);
  ASSERT_EQ(approach.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(entrance.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(inside.status, ExecutorStatus::PUBLISHED);
  EXPECT_GT(approach.command.linear.y, 0.7);
  EXPECT_LE(entrance.command.linear.y, 0.50001);
  EXPECT_LE(inside.command.linear.y, 0.50001);
  p.clearance_braking.deceleration = 0.0;
  const auto no_braking_model = run_at(0.0);
  ASSERT_EQ(no_braking_model.status, ExecutorStatus::PUBLISHED);
  EXPECT_LE(no_braking_model.command.linear.y, 0.50001);
}

TEST(MpcGimbalFollow, SecondSolveUsesTheOriginalAccelerationAnchor)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.25;
  auto c = config();
  c.horizon = 40;
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 2.0;
  p.secondary_reference_enable = true;
  PathExecutor executor(c, p, nullptr, nullptr, map);
  const auto result = executor.computeCommand(input());
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  // With a fresh zero anchor, one tick permits at most ay_max * dt = 0.1 m/s.
  EXPECT_GT(result.command.linear.y, 0.0);
  EXPECT_LE(result.command.linear.y, c.ay_max * c.dt + 1.0e-6);
}

TEST(MpcGimbalFollow, SecondReferencePrioritizesCrossTrackRecoveryInOpenSpace)
{
  auto map = std::make_shared<FreeMap>();
  auto c = config();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.secondary_reference_enable = true;
  PathExecutor refined(c, p, nullptr, nullptr, map);
  p.secondary_reference_enable = false;
  PathExecutor original(c, p, nullptr, nullptr, map);
  auto in = input();
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
  odom->pose.pose.position.x = 0.6;
  in.odom = odom;
  const auto first = original.computeCommand(in);
  const auto second = refined.computeCommand(in);
  ASSERT_EQ(first.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(second.status, ExecutorStatus::PUBLISHED);
  EXPECT_LT(second.command.linear.y, first.command.linear.y - 0.02);
  EXPECT_LT(second.command.linear.x, 0.0);
  EXPECT_TRUE(second.secondary_reference_applied);
  EXPECT_FALSE(second.secondary_reference_narrow);
}

TEST(MpcGimbalFollow, CrossTrackWeightFollowsTravelTangentAtDiagonalYawMismatch)
{
  auto c = config();
  c.dt = 1.0;
  c.horizon = 1;
  c.use_acc_constraints = false;
  c.vx_min = c.vy_min = -10.0;
  c.vx_max = c.vy_max = 10.0;
  c.q_along = 1.0;
  c.q_cross = 20.0;
  minco_controller::ReferencePoint point;
  point.vel = Eigen::Vector2d(0.1, 0.1);  // Tangent at 45 degrees, yaw at zero.
  point.yaw = 0.0;
  const std::vector<minco_controller::ReferencePoint> reference{point};
  minco_controller::State baseline, along, cross;
  along.x = along.y = 0.1;
  cross.x = -0.1;
  cross.y = 0.1;
  minco_controller::Control baseline_control, along_control, cross_control;
  minco_controller::MpcSolver solver(c);
  ASSERT_TRUE(solver.solve(baseline, reference, baseline_control));
  ASSERT_TRUE(solver.solve(along, reference, along_control));
  ASSERT_TRUE(solver.solve(cross, reference, cross_control));
  const double along_correction =
    ((along_control.vx - baseline_control.vx) +
     (along_control.vy - baseline_control.vy)) / std::sqrt(2.0);
  const double cross_correction =
    ((cross_control.vx - baseline_control.vx) -
     (cross_control.vy - baseline_control.vy)) / std::sqrt(2.0);
  EXPECT_GT(std::abs(cross_correction), 1.5 * std::abs(along_correction));
}
TEST(MpcGimbalFollow, AlongWeightCanDominateCrossWeightWhenConfigured)
{
  auto c = config();
  c.dt = 1.0;
  c.horizon = 1;
  c.use_acc_constraints = false;
  c.vx_min = c.vy_min = -10.0;
  c.vx_max = c.vy_max = 10.0;
  c.q_along = 20.0;
  c.q_cross = 1.0;
  minco_controller::ReferencePoint point;
  point.vel = Eigen::Vector2d(0.1, 0.1);  // Tangent at 45 degrees, yaw at zero.
  point.yaw = 0.0;
  const std::vector<minco_controller::ReferencePoint> reference{point};
  minco_controller::State baseline, along, cross;
  along.x = along.y = 0.1;
  cross.x = -0.1;
  cross.y = 0.1;
  minco_controller::Control baseline_control, along_control, cross_control;
  minco_controller::MpcSolver solver(c);
  ASSERT_TRUE(solver.solve(baseline, reference, baseline_control));
  ASSERT_TRUE(solver.solve(along, reference, along_control));
  ASSERT_TRUE(solver.solve(cross, reference, cross_control));
  const double along_correction =
    ((along_control.vx - baseline_control.vx) +
     (along_control.vy - baseline_control.vy)) / std::sqrt(2.0);
  const double cross_correction =
    ((cross_control.vx - baseline_control.vx) -
     (cross_control.vy - baseline_control.vy)) / std::sqrt(2.0);
  EXPECT_GT(std::abs(along_correction), 1.5 * std::abs(cross_correction));
}

TEST(MpcGimbalFollow, SecondaryReferenceAngleTriggerCanBeDisabled)
{
  auto map = std::make_shared<FreeMap>();
  auto c = config();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.secondary_reference_enable = true;
  p.secondary_reference_angle_enable = false;
  p.secondary_reference_narrow_enable = false;
  PathExecutor disabled(c, p, nullptr, nullptr, map);
  p.secondary_reference_enable = false;
  PathExecutor baseline(c, p, nullptr, nullptr, map);
  auto in = input();
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
  odom->pose.pose.position.x = 0.6;
  in.odom = odom;
  const auto disabled_output = disabled.computeCommand(in);
  const auto baseline_output = baseline.computeCommand(in);
  ASSERT_EQ(disabled_output.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(baseline_output.status, ExecutorStatus::PUBLISHED);
  EXPECT_NEAR(disabled_output.command.linear.x, baseline_output.command.linear.x, 1e-9);
  EXPECT_NEAR(disabled_output.command.linear.y, baseline_output.command.linear.y, 1e-9);
  EXPECT_FALSE(disabled_output.secondary_reference_applied);
}

TEST(MpcGimbalFollow, SecondaryReferenceNarrowBranchCanBeDisabled)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.25;
  auto c = config();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 2.0;
  p.secondary_reference_enable = true;
  p.secondary_reference_angle_enable = false;
  p.secondary_reference_narrow_enable = false;
  PathExecutor disabled(c, p, nullptr, nullptr, map);
  p.secondary_reference_narrow_enable = true;
  PathExecutor enabled(c, p, nullptr, nullptr, map);
  const auto in = input();
  const auto disabled_output = disabled.computeCommand(in);
  const auto enabled_output = enabled.computeCommand(in);
  ASSERT_EQ(disabled_output.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(enabled_output.status, ExecutorStatus::PUBLISHED);
  EXPECT_GT(disabled_output.command.linear.y, enabled_output.command.linear.y + 0.05);
  EXPECT_FALSE(disabled_output.secondary_reference_applied);
  EXPECT_TRUE(enabled_output.secondary_reference_applied);
  EXPECT_TRUE(enabled_output.secondary_reference_narrow);
}

TEST(MpcGimbalFollow, NarrowTimeScaleRespectsMinimumFloor)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.25;
  auto c = config();
  c.horizon = 40;
  c.use_acc_constraints = false;
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 2.0;
  p.secondary_reference_enable = true;
  p.secondary_reference_angle_enable = false;
  p.secondary_reference_narrow_enable = true;
  p.secondary_reference_min_time_scale = 0.9;
  p.secondary_reference_narrow_time_scale = 0.5;
  PathExecutor floored(c, p, nullptr, nullptr, map);
  p.secondary_reference_narrow_time_scale = 0.9;
  PathExecutor capped(c, p, nullptr, nullptr, map);
  const auto in = input();
  const auto floored_output = floored.computeCommand(in);
  const auto capped_output = capped.computeCommand(in);
  ASSERT_EQ(floored_output.status, ExecutorStatus::PUBLISHED);
  ASSERT_EQ(capped_output.status, ExecutorStatus::PUBLISHED);
  EXPECT_NEAR(
    floored_output.command.linear.y, capped_output.command.linear.y, 1e-9);
  EXPECT_NEAR(floored_output.secondary_reference_min_time_scale, 0.9, 1e-9);
  EXPECT_NEAR(capped_output.secondary_reference_min_time_scale, 0.9, 1e-9);
}

TEST(MpcGimbalFollow, AnticipatesTheChangingTangentOnACurvedTrajectory)
{
  auto map = std::make_shared<FreeMap>(); PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true;
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(*in.trajectory);
  double x = 0, y = 0;
  for (size_t i = 0; i < trajectory->cmds.size(); ++i) {
    const double theta = static_cast<double>(i) * kPi / 60;
    auto & command = trajectory->cmds[i];
    command.position.x = x; command.position.y = y;
    command.velocity.x = std::cos(theta); command.velocity.y = std::sin(theta);
    command.acceleration.x = -std::sin(theta) * kPi / 3;
    command.acceleration.y = std::cos(theta) * kPi / 3;
    x += 0.05 * command.velocity.x; y += 0.05 * command.velocity.y;
  }
  in.trajectory = trajectory; const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_GT(result.command.angular.z, 0.0); EXPECT_LE(result.command.angular.z, 0.20001);
}
TEST(MpcGimbalFollow, AlignsInPlaceAndDoesNotAddOrdinarySpin)
{
  auto map = std::make_shared<FreeMap>(); PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true; in.hold_translation = true; in.alignment_heading_odom = kPi / 2; in.spin_speed = 4;
  const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_DOUBLE_EQ(result.command.linear.x, 0); EXPECT_DOUBLE_EQ(result.command.linear.y, 0);
  EXPECT_GT(result.command.angular.z, 0); EXPECT_LE(result.command.angular.z, 0.20001);
  GimbalFollowAdapter adapter(GimbalFollowParams{}); adapter.updateHeading(0.0, 0.05);
  EXPECT_NEAR(adapter.integrateMpcYawRate(result.command.angular.z, 0.05),
    result.command.angular.z * 0.05 * 180.0 / kPi, 1e-8);
}
TEST(MpcGimbalFollow, UsesForwardPositionsAtZeroSpeedAndRejectsUndefinedTangent)
{
  auto map = std::make_shared<FreeMap>(); auto in = input(); in.follow_gimbal = true;
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(*in.trajectory);
  for (auto & command : trajectory->cmds) command.velocity.y = 0;
  in.trajectory = trajectory; PathExecutor executor(config(), params(), nullptr, nullptr, map);
  const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED); EXPECT_GT(result.command.angular.z, 0);
  for (auto & command : trajectory->cmds) command.position.y = 0;
  EXPECT_EQ(executor.computeCommand(in).status, ExecutorStatus::REFERENCE_FAILED);
}
TEST(MpcGimbalFollow, StopsIntegrationWhenSafetyRejectsAndRejectsInvalidPose)
{
  auto map = std::make_shared<FreeMap>(); map->stamp = 0;
  PathExecutor executor(config(), params(), nullptr, nullptr, map); auto in = input(); in.follow_gimbal = true;
  const auto result = executor.computeCommand(in);
  EXPECT_EQ(result.status, ExecutorStatus::DYNAMIC_BLOCKED); EXPECT_DOUBLE_EQ(result.command.angular.z, 0);
  map->stamp = 10;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom); odom->pose.pose.orientation.w = 0; in.odom = odom;
  EXPECT_EQ(executor.computeCommand(in).status, ExecutorStatus::WAITING_INPUT);
}
TEST(MpcGimbalFollow, AllowsOnlyCreepWhenAlreadyInsideNarrowClearance)
{
  auto map = std::make_shared<FreeMap>(); map->distance = 0.25;
  PathExecutorParams p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 0.2;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto result = executor.computeCommand(input());
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_LE(std::hypot(result.command.linear.x, result.command.linear.y), 0.20001);

  minco_controller::State current;
  current.x = 0.0; current.y = 0.0;
  minco_controller::Control control;
  control.vx = 1.0; control.vy = 0.0;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.28, false, 0.24, false, 0.5,
    0.05, current, control, stamp), ExecutorStatus::DYNAMIC_BLOCKED);
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.28, true, 0.24, false, 0.5,
    0.05, current, control, stamp), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, CapsSpeedAndAllowsCommandGateWhenNarrowPassageIsAhead)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.30 : (pos.y() < 1.2 ? 0.25 : 0.30);
    };
  PathExecutorParams p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_speed = 0.2;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto result = executor.computeCommand(input());
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_LE(std::hypot(result.command.linear.x, result.command.linear.y), 0.20001);

  // The current pose is still in open space (0.30 m), but the gate is allowed
  // to use the floor because the committed trajectory is narrow ahead.
  minco_controller::State current;
  current.x = 0.0;
  current.y = 0.0;
  minco_controller::Control control;
  control.vx = 0.0;
  control.vy = 0.2;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.28, true, 0.24, true, 0.5,
    0.05, current, control, stamp), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, NarrowPassageAlignmentUsesEntryTangent)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.25);
    };
  PathExecutorParams p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_lookahead = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto in = input();
  const auto error = executor.narrowPassageHeadingErrorDegrees(*in.trajectory, 0.0, 0.0, 0.0);
  ASSERT_TRUE(error.has_value());
  EXPECT_NEAR(*error, 90.0, 1.0e-6);

  map->distance_fn = [](const Eigen::Vector3d &) {return 0.40;};
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(*in.trajectory, 0.0, 0.0, 0.0));
  executor.resetNarrowPassageAlignment();
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(*in.trajectory, 0.0, 0.0, 0.0));
}

TEST(MpcGimbalFollow, NarrowHeadingIgnoresSmallAlternatingWaypointOffsets)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.45 : betweenSideWalls(pos, 0.25);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  auto trajectory = *input().trajectory;
  for (size_t i = 0; i < trajectory.cmds.size(); ++i) {
    trajectory.cmds[i].position.x = i % 2U == 0U ? -0.005 : 0.005;
  }
  const auto error = executor.narrowPassageHeadingErrorDegrees(
    trajectory, 0.0, 0.0, 0.0);
  ASSERT_TRUE(error.has_value());
  EXPECT_NEAR(*error, 90.0, 5.0);
}

TEST(MpcGimbalFollow, NarrowAlignmentKeepsEntryHeadingAcrossReplans)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.25);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto straight = *input().trajectory;
  const auto first = executor.narrowPassageHeadingErrorDegrees(straight, 0.0, 0.0, 0.0);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(*first, 90.0, 1.0e-6);

  // A refreshed path aims into a diagonal corridor. While the previous
  // narrow alignment is active, that new tangent must not rotate the square
  // body inside the slot. Once released, the new section can select its own
  // heading on entry.
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      if (pos.y() < 0.15) return 0.40;
      const double normal = (pos.y() - pos.x()) / std::sqrt(2.0);
      return std::min(std::abs(normal - 0.25), std::abs(normal + 0.25));
    };
  auto diagonal = straight;
  for (auto & command : diagonal.cmds) {
    command.position.x = command.position.y;
  }
  const auto held = executor.narrowPassageHeadingErrorDegrees(diagonal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(held.has_value());
  EXPECT_NEAR(*held, 90.0, 1.0e-6);

  executor.resetNarrowPassageAlignment();
  const auto new_entry = executor.narrowPassageHeadingErrorDegrees(diagonal, 0.0, 0.0, 0.0);
  ASSERT_TRUE(new_entry.has_value());
  EXPECT_NEAR(*new_entry, 45.0, 1.0e-6);
}

TEST(MpcGimbalFollow, NarrowAlignmentAllowsReverseTravelOnTheSameBodyAxis)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.25);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto straight = *input().trajectory;
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(straight, 0.0, 0.0, kPi / 2));

  auto return_path = straight;
  const double end_y = straight.cmds.back().position.y;
  for (auto & command : return_path.cmds) {
    command.position.y = end_y - command.position.y;
  }
  const auto reversed = executor.narrowPassageHeadingErrorDegrees(
    return_path, 0.0, end_y, kPi / 2);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_NEAR(std::abs(*reversed), 180.0, 1.0e-6);
}

TEST(MpcGimbalFollow, DistantTwoSidedNarrowPointDoesNotAlignInOpenSpace)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 1.0 ? 0.55 : betweenSideWalls(pos, 0.25);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_lookahead = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto trajectory = *input().trajectory;
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
  EXPECT_FALSE(executor.narrowPassageAlignmentActive());
  const auto near_entry = executor.narrowPassageHeadingErrorDegrees(
    trajectory, 0.0, 0.6, 0.0);
  ASSERT_TRUE(near_entry.has_value());
  EXPECT_TRUE(executor.narrowPassageAlignmentActive());
}

TEST(MpcGimbalFollow, OneSidedWallDoesNotTriggerNarrowAlignment)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : std::abs(pos.x() - 0.25);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));
  EXPECT_FALSE(executor.narrowPassageAlignmentActive());
}

TEST(MpcGimbalFollow, OffCenterTwoSidedSlotStillTriggersAlignment)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 :
        std::min(std::abs(pos.x() + 0.35), std::abs(pos.x() - 0.60));
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  auto trajectory = *input().trajectory;
  for (auto & command : trajectory.cmds) command.position.x = -0.10;
  const auto error = executor.narrowPassageHeadingErrorDegrees(trajectory, -0.10, 0.0, 0.0);
  ASSERT_TRUE(error.has_value());
  EXPECT_NEAR(*error, 90.0, 1.0e-6);
}

TEST(MpcGimbalFollow, WideTwoSidedSlotDoesNotTriggerAlignedBodyNarrowThreshold)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.35);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));
  EXPECT_FALSE(executor.narrowPassageAlignmentActive());

  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.30);
    };
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));

  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.3 ? 0.40 : betweenSideWalls(pos, 0.29);
    };
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));
}

TEST(MpcGimbalFollow, NarrowAlignmentStaysLatchedUntilPastTheCorridor)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() >= 0.5 && pos.y() <= 1.0 ? betweenSideWalls(pos, 0.25) : 0.40;
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_lookahead = 2.0;
  p.narrow_passage_alignment_hold_distance = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  auto in = input();
  auto trajectory = *in.trajectory;
  for (int i = 31; i <= 60; ++i) {
    auto command = trajectory.cmds.back();
    command.position.y = i * 0.05;
    trajectory.cmds.push_back(command);
  }
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
  // A momentarily wide map must not switch the chassis back to free spin.
  map->distance_fn = [](const Eigen::Vector3d &) {return 0.40;};
  auto replanned = trajectory;
  int next_odom_decimeter = 1;
  auto advance_odom_to = [&](double y) {
      while (next_odom_decimeter * 0.1 <= y + 1.0e-9) {
        executor.updateNarrowPassageAlignmentPose(0.0, next_odom_decimeter * 0.1);
        ++next_odom_decimeter;
      }
    };
  advance_odom_to(0.6);
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(replanned, 0.0, 0.6, 0.0));
  // Odom still advances while a planner gap has no new heading query.
  advance_odom_to(1.3);
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(replanned, 0.0, 1.3, 0.0));
  advance_odom_to(1.6);
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(replanned, 0.0, 1.6, 0.0));
  advance_odom_to(1.99);
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(replanned, 0.0, 1.99, 0.0));
  advance_odom_to(2.05);
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(replanned, 0.0, 2.05, 0.0));

  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() >= 0.5 && pos.y() <= 1.0 ? betweenSideWalls(pos, 0.25) : 0.40;
    };
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
  executor.resetNarrowPassageAlignment();
  map->distance_fn = [](const Eigen::Vector3d &) {return 0.40;};
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
}

TEST(MpcGimbalFollow, ReleasesAfterTwoMetersWithoutChasingADistantNextNarrowPoint)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return (pos.y() >= 0.5 && pos.y() <= 1.0) ||
             (pos.y() >= 3.2 && pos.y() <= 3.5) ? betweenSideWalls(pos, 0.25) : 0.40;
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_lookahead = 2.0;
  p.narrow_passage_alignment_hold_distance = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  auto trajectory = *input().trajectory;
  for (int i = 31; i <= 80; ++i) {
    auto command = trajectory.cmds.back();
    command.position.y = i * 0.05;
    trajectory.cmds.push_back(command);
  }
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
  for (int i = 1; i <= 19; ++i) {
    executor.updateNarrowPassageAlignmentPose(0.0, i * 0.1);
  }
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 1.99, 0.0));
  executor.updateNarrowPassageAlignmentPose(0.0, 2.05);
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 2.05, 0.0));
  EXPECT_GE(executor.narrowPassageAlignmentTraveled(), 2.0);
  // The next narrow sample is over a metre away, so the next tick stays ordinary.
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 2.06, 0.0));
  // Approaching that separate narrow section can enter a new alignment interval.
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 2.80, 0.0));
}

TEST(MpcGimbalFollow, KeepsAlignmentAcrossAShortWideGapBetweenNarrowSections)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return (pos.y() >= 0.5 && pos.y() <= 2.5) ||
             (pos.y() >= 3.1 && pos.y() <= 3.5) ? betweenSideWalls(pos, 0.25) : 0.40;
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_hold_distance = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  auto trajectory = *input().trajectory;
  for (int i = 31; i <= 80; ++i) {
    auto command = trajectory.cmds.back();
    command.position.y = i * 0.05;
    trajectory.cmds.push_back(command);
  }
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 0.0, 0.0));
  for (int i = 1; i <= 26; ++i) {
    executor.updateNarrowPassageAlignmentPose(0.0, i * 0.1);
  }
  EXPECT_TRUE(executor.narrowPassageHeadingErrorDegrees(trajectory, 0.0, 2.55, 0.0));
  EXPECT_TRUE(executor.narrowPassageAlignmentActive());
}

TEST(MpcGimbalFollow, CompletedGoalReleasesOnlyAfterTheTwoMeterHold)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() >= 0.5 && pos.y() <= 1.0 ? betweenSideWalls(pos, 0.25) : 0.40;
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_hold_distance = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));
  for (int i = 1; i <= 19; ++i) {
    executor.updateNarrowPassageAlignmentPose(0.0, i * 0.1);
  }
  EXPECT_FALSE(executor.releaseNarrowPassageAlignmentAtGoal());
  EXPECT_TRUE(executor.narrowPassageAlignmentActive());
  executor.updateNarrowPassageAlignmentPose(0.0, 2.0);
  EXPECT_TRUE(executor.releaseNarrowPassageAlignmentAtGoal());
  EXPECT_FALSE(executor.narrowPassageAlignmentActive());
  EXPECT_GE(executor.narrowPassageAlignmentTraveled(), 2.0);
}

TEST(MpcGimbalFollow, CompletedGoalKeepsAlignmentUntilCornersCanRotate)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() >= 0.5 && pos.y() <= 1.0 ? betweenSideWalls(pos, 0.25) :
        betweenSideWalls(pos, 0.35);
    };
  auto p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_hold_distance = 2.0;
  p.swept_radius = 0.302;
  p.rotation_margin = 0.05;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  // The 0.25 m section starts alignment; the 0.35 m exit is still too tight
  // to hand free spin back to the lower board.
  ASSERT_TRUE(executor.narrowPassageHeadingErrorDegrees(*input().trajectory, 0.0, 0.0, 0.0));
  for (int i = 1; i <= 20; ++i) {
    executor.updateNarrowPassageAlignmentPose(0.0, i * 0.1);
  }
  EXPECT_FALSE(executor.releaseNarrowPassageAlignmentAtGoal());
  EXPECT_TRUE(executor.narrowPassageAlignmentActive());
  map->distance_fn = [](const Eigen::Vector3d &) {return 0.40;};
  EXPECT_TRUE(executor.releaseNarrowPassageAlignmentAtGoal());
  EXPECT_FALSE(executor.narrowPassageAlignmentActive());
}

TEST(MpcGimbalFollow, DoesNotAlignToCurrentTightCellWhenRouteOpens)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.12 ? 0.25 : 0.40;
    };
  PathExecutorParams p = params();
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.24;
  p.narrow_passage_alignment_enable = true;
  p.narrow_passage_alignment_lookahead = 2.0;
  PathExecutor executor(config(), p, nullptr, nullptr, map);
  const auto in = input();
  EXPECT_FALSE(executor.narrowPassageHeadingErrorDegrees(*in.trajectory, 0.0, 0.0, 0.0));
}

TEST(MpcGimbalFollow, DoesNotRelaxAheadNarrowGateWhenCurrentPoseIsBelowFloor)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & pos) {
      return pos.y() < 0.05 ? 0.23 : 0.25;
    };
  map->stamp = 10.0;
  minco_controller::State current;
  current.x = 0.0;
  current.y = 0.0;
  minco_controller::Control control;
  control.vx = 0.0;
  control.vy = 1.0;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.28, true, 0.24, false, 0.5,
    0.05, current, control, stamp), ExecutorStatus::DYNAMIC_BLOCKED);
}

TEST(MpcGimbalFollow, AcceptsCommandClearanceExactlyAtThreshold)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d &) {return 0.26;};
  map->stamp = 10.0;
  minco_controller::State current;
  current.x = 0.0;
  current.y = 0.0;
  minco_controller::Control control;
  control.vx = 0.0;
  control.vy = 1.0;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.28, false, 0.24, false, 0.5,
    0.05, current, control, stamp), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, BrakingClearanceCountsOnlyMotionTowardWall)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.70;
  map->gradient_fn = [](const Eigen::Vector3d &) { return Eigen::Vector3d(-1.0, 0.0, 0.0); };
  minco_controller::State current;
  minco_controller::Control control;
  control.vx = 1.0;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const ClearanceBrakingParams braking{1.2, 0.20};
  CommandSafetyDetail detail;
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, false, 0.37, false,
    0.5, 0.05, current, control, stamp, &detail, braking), ExecutorStatus::DYNAMIC_BLOCKED);
  EXPECT_GT(detail.threshold, 1.01);  // 0.40 m baseline + 0.20 m reaction + 1/2.4 m brake.
  control.vx = 0.0;
  control.vy = 1.0;
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, false, 0.37, false,
    0.5, 0.05, current, control, stamp, &detail, braking), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, CircularBodyGapRemainsAboveFiveCentimetersInNarrowBand)
{
  auto map = std::make_shared<FreeMap>();
  map->gradient_fn = [](const Eigen::Vector3d &) { return Eigen::Vector3d(-1.0, 0.0, 0.0); };
  minco_controller::State current;
  minco_controller::Control control;
  control.vy = 0.2;  // Parallel to the wall; only the physical floor applies.
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const ClearanceBrakingParams braking{1.2, 0.20};
  map->distance = 0.34;  // 0.30 m radius leaves only 0.04 m nominal gap.
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, true, 0.37, true,
    0.5, 0.05, current, control, stamp, nullptr, braking), ExecutorStatus::DYNAMIC_BLOCKED);
  map->distance = 0.37;
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, true, 0.37, true,
    0.5, 0.05, current, control, stamp, nullptr, braking), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, FlatEsdfGradientDoesNotFreezeCenteredNarrowCorridor)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.37;
  minco_controller::State current;
  minco_controller::Control control;
  control.vy = 0.2;
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, true, 0.37, true,
    0.5, 0.05, current, control, rclcpp::Time(10, 0, RCL_ROS_TIME), nullptr,
    ClearanceBrakingParams{1.2, 0.20}), ExecutorStatus::PUBLISHED);
}

TEST(MpcGimbalFollow, AlreadyInsideBodyGapCanOnlyMoveOutward)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & p) { return 0.34 + p.x(); };
  map->gradient_fn = [](const Eigen::Vector3d &) { return Eigen::Vector3d(1.0, 0.0, 0.0); };
  minco_controller::State current;
  minco_controller::Control control;
  control.vx = 0.2;
  const auto stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const ClearanceBrakingParams braking{1.2, 0.20};
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, true, 0.37, false,
    0.5, 0.05, current, control, stamp, nullptr, braking), ExecutorStatus::PUBLISHED);
  control.vx = 0.0;
  control.vy = 0.2;
  EXPECT_EQ(checkCommandSafety(nullptr, map, nullptr, "odom", 0.42, true, 0.37, false,
    0.5, 0.05, current, control, stamp, nullptr, braking), ExecutorStatus::DYNAMIC_BLOCKED);
}

TEST(MpcGimbalFollow, ClearanceBrakePreservesWallParallelVelocity)
{
  double vx = -0.20;
  double vy = 0.50;
  ASSERT_TRUE(limitObstacleApproachVelocity(vx, vy, 1.0, 0.0, 0.03));
  EXPECT_NEAR(vx, -0.03, 1.0e-9);
  EXPECT_NEAR(vy, 0.50, 1.0e-9);
  EXPECT_FALSE(limitObstacleApproachVelocity(vx, vy, 0.0, 0.0, 0.03));
}

TEST(MpcGimbalFollow, ClearanceBrakeCapsCommandBeforeHardGate)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.70;
  map->gradient_fn = [](const Eigen::Vector3d &) { return Eigen::Vector3d(0.0, -1.0, 0.0); };
  auto c = fastConfig();
  c.use_acc_constraints = false;
  auto p = params();
  p.rog_map_clearance = 0.42;
  p.clearance_braking = {1.2, 0.20};
  PathExecutor guarded(c, p, nullptr, nullptr, map);
  const auto guarded_output = guarded.computeCommand(input());
  ASSERT_EQ(guarded_output.status, ExecutorStatus::PUBLISHED);
  const double allowed = clearanceSpeedLimit(0.70 - 0.40, p.clearance_braking);
  EXPECT_LE(guarded_output.command.linear.y, allowed + 1e-3);
  p.clearance_braking.deceleration = 0.0;
  PathExecutor unguarded(c, p, nullptr, nullptr, map);
  const auto unguarded_output = unguarded.computeCommand(input());
  ASSERT_EQ(unguarded_output.status, ExecutorStatus::PUBLISHED);
  EXPECT_GT(unguarded_output.command.linear.y, guarded_output.command.linear.y + 0.05);
}

TEST(MpcGimbalFollow, ClearanceBrakeRechecksTheSlowedCommandHorizon)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & point) {return 0.37 - point.x();};
  map->gradient_fn = [](const Eigen::Vector3d &) {
      return Eigen::Vector3d(-1.0, 0.0, 0.0);
    };
  auto in = input();
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(*in.trajectory);
  for (auto & command : trajectory->cmds) command.position.x = 0.10;
  in.trajectory = trajectory;
  auto c = fastConfig();
  c.use_acc_constraints = false;
  auto p = params();
  auto open_map = std::make_shared<FreeMap>();
  PathExecutor open(c, p, nullptr, nullptr, open_map);
  const auto unscaled = open.computeCommand(in);
  ASSERT_EQ(unscaled.status, ExecutorStatus::PUBLISHED);
  EXPECT_LT(0.37 - 0.35 * unscaled.command.linear.x, 0.26);
  p.clearance_braking = {1.2, 0.20};
  PathExecutor executor(c, p, nullptr, nullptr, map);
  const auto output = executor.computeCommand(in);
  ASSERT_EQ(output.status, ExecutorStatus::PUBLISHED);
  EXPECT_TRUE(output.clearance_braking_capped);
  // The original 0.35 s projection reaches the wall, while a reduced command
  // can still move toward the path without crossing the 0.26 m hard limit.
  EXPECT_GT(output.command.linear.x, p.deadzone_speed);
  EXPECT_LT(output.command.linear.x, unscaled.command.linear.x);
}

TEST(MpcGimbalFollow, ClearanceBrakeKeepsAlongWallCommandWhenLateralSpeedIsLimited)
{
  auto map = std::make_shared<FreeMap>();
  map->distance_fn = [](const Eigen::Vector3d & point) {return 0.37 - point.x();};
  map->gradient_fn = [](const Eigen::Vector3d &) {
      return Eigen::Vector3d(-1.0, 0.0, 0.0);
    };
  auto in = input();
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(*in.trajectory);
  for (auto & command : trajectory->cmds) command.position.x = 0.10;
  in.trajectory = trajectory;
  auto c = fastConfig();
  c.use_acc_constraints = false;
  auto p = params();
  p.clearance_braking = {1.2, 0.20};
  PathExecutor executor(c, p, nullptr, nullptr, map);
  const auto output = executor.computeCommand(in);
  ASSERT_EQ(output.status, ExecutorStatus::PUBLISHED);
  EXPECT_TRUE(output.clearance_braking_capped);
  EXPECT_GT(output.command.linear.y, 0.4);
  EXPECT_LT(output.command.linear.x, 0.25);
}

TEST(MpcGimbalFollow, NarrowCellBrakeCapUsesTheCommandGatesNearFieldBase)
{
  auto map = std::make_shared<FreeMap>();
  map->distance = 0.395;
  map->gradient_fn = [](const Eigen::Vector3d &) {
      return Eigen::Vector3d(0.0, -1.0, 0.0);
    };
  auto c = fastConfig();
  c.use_acc_constraints = false;
  auto p = params();
  p.rog_map_clearance = 0.42;  // Effective full requirement 0.40 m.
  p.narrow_passage_enable = true;
  p.narrow_passage_clearance = 0.37;
  p.clearance_braking = {1.2, 0.20};
  PathExecutor executor(c, p, nullptr, nullptr, map);
  const auto output = executor.computeCommand(input());
  ASSERT_EQ(output.status, ExecutorStatus::PUBLISHED);
  EXPECT_TRUE(output.clearance_braking_capped);
  const double command_gate_base = 0.395 - kEsdfJitterTolerance;
  const double allowed = clearanceSpeedLimit(
    0.395 - command_gate_base - kClearanceComparisonTolerance,
    p.clearance_braking);
  EXPECT_LE(output.command.linear.y, allowed + 1e-3);
}

// 目标接近减速带：机器人已经在 2 m 直线的最后 0.2 m，参考速度必须按实测刹车
// 能力（1.2 m/s²）压到爬行；关闭该功能时同一条轨迹仍按 2 m/s 巡航。
// 这条用例守住 20261018_194203 里"目标前 0.3 m 还有 1.1 m/s、过冲 0.6–1.1 m"
// 的回归（MPC 每拍最多加 0.1 m/s，故用多拍爬到稳态）。
namespace {
std::shared_ptr<interfaces::msg::MpcPositionCommand> straightTrajectory(
  const rclcpp::Time & stamp, double speed, double length)
{
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>();
  trajectory->header.stamp = stamp;
  trajectory->header.frame_id = "odom";
  const int steps = static_cast<int>(std::lround(length / 0.1));
  for (int i = 0; i <= steps; ++i) {
    interfaces::msg::PositionCommand command;
    command.position.y = i * 0.1;
    command.velocity.y = speed;
    command.yaw = 0.0;
    command.yaw_dot = 0.0;
    trajectory->cmds.push_back(command);
  }
  return trajectory;
}

void setOdom(ExecutorInput & in, double y)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.stamp = in.stamp;
  odom->pose.pose.position.x = 0.0;
  odom->pose.pose.position.y = y;
  odom->pose.pose.orientation.w = 1.0;
  in.odom = odom;
}

double settleSpeed(
  PathExecutor & executor, ExecutorInput & in, int ticks = 30)
{
  ExecutorOutput output;
  for (int i = 0; i < ticks; ++i) {
    output = executor.computeCommand(in);
  }
  return std::hypot(output.command.linear.x, output.command.linear.y);
}
}  // namespace

TEST(MpcGimbalFollow, GoalApproachEnvelopeSlowsTheLastMeters)
{
  auto map = std::make_shared<FreeMap>();
  auto approach_params = params();
  approach_params.goal_approach.deceleration = 1.2;
  approach_params.goal_approach.stop_margin = 0.15;
  approach_params.goal_approach.min_speed = 0.12;
  auto in = input();
  in.trajectory = straightTrajectory(in.stamp, 2.0, 2.0);

  setOdom(in, 1.8);
  PathExecutor capped(fastConfig(), approach_params, nullptr, nullptr, map);
  const auto first = capped.computeCommand(in);
  ASSERT_EQ(first.status, ExecutorStatus::PUBLISHED);
  EXPECT_TRUE(first.goal_approach_capped);
  EXPECT_TRUE(std::isfinite(first.goal_approach_remaining_m));
  EXPECT_NEAR(first.goal_approach_remaining_m, 0.2, 0.06);
  EXPECT_LT(first.goal_approach_allowed_speed, 0.5);
  const double capped_speed = settleSpeed(capped, in);
  EXPECT_LT(capped_speed, 0.6);

  auto disabled_params = params();
  disabled_params.goal_approach.deceleration = 0.0;
  PathExecutor disabled(fastConfig(), disabled_params, nullptr, nullptr, map);
  const auto disabled_first = disabled.computeCommand(in);
  ASSERT_EQ(disabled_first.status, ExecutorStatus::PUBLISHED);
  EXPECT_FALSE(disabled_first.goal_approach_capped);
  // 关闭包络时 MPC 仍会因"参考位置在轨迹末端停住"而自然收敛，但收敛速度是
  // 包络生效时的两倍左右；这里只要两者有明显差距即可（几何相同、只有包络不同）。
  const double disabled_speed = settleSpeed(disabled, in);
  EXPECT_GT(disabled_speed, 0.8);
  EXPECT_GT(disabled_speed, capped_speed + 0.3);
}

// 离终点还很远时不封顶：20 m 直线剩余 ~19.9 m，包络允许 6.9 m/s，巡航仍是 2 m/s。
TEST(MpcGimbalFollow, GoalApproachEnvelopeIgnoresTheFarSection)
{
  auto map = std::make_shared<FreeMap>();
  auto approach_params = params();
  approach_params.goal_approach.deceleration = 1.2;
  approach_params.goal_approach.stop_margin = 0.15;
  approach_params.goal_approach.min_speed = 0.12;
  auto in = input();
  in.trajectory = straightTrajectory(in.stamp, 2.0, 20.0);

  setOdom(in, 0.0);
  PathExecutor executor(fastConfig(), approach_params, nullptr, nullptr, map);
  const auto first = executor.computeCommand(in);
  ASSERT_EQ(first.status, ExecutorStatus::PUBLISHED);
  EXPECT_FALSE(first.goal_approach_capped);
  EXPECT_GT(settleSpeed(executor, in), 1.5);
}

TEST(MpcGimbalFollow, UsesTheShortTurnAcrossPi)
{
  auto map = std::make_shared<FreeMap>(); PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true; in.hold_translation = true; in.alignment_heading_odom = -179 * kPi / 180;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
  odom->pose.pose.orientation.z = std::sin(179 * kPi / 360); odom->pose.pose.orientation.w = std::cos(179 * kPi / 360); in.odom = odom;
  const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED); EXPECT_GT(result.command.angular.z, 0); EXPECT_LE(result.command.angular.z, 0.20001);
}
TEST(MpcGimbalFollow, RealMpcControlsAnAngleOnlyLowerServoWithUnknownZero)
{
  const auto c = config(); minco_controller::MpcSolver solver(c);
  GimbalFollowParams forward_only; forward_only.allow_reverse = false;
  GimbalFollowAdapter adapter(forward_only);
  const double lower_zero = -kPi / 6;
  minco_controller::State state; state.yaw = lower_zero;
  std::vector<minco_controller::ReferencePoint> reference(c.horizon);
  for (auto & point : reference) point.yaw = kPi / 2;
  GimbalFollowOutput follow;
  double ref_angle = 0;
  for (int i = 0; i < 500; ++i) {
    follow = adapter.updateHeading((kPi / 2 - state.yaw) * 180.0 / kPi, c.dt);
    solver.setLastControl({0, 0, state.omega}); minco_controller::Control u;
    ASSERT_TRUE(solver.solve(state, reference, u));
    ASSERT_LE(std::abs(u.omega), 2.0001);
    ASSERT_LE(std::abs(u.omega - state.omega), 4 * c.dt + 1e-4);
    ref_angle = adapter.integrateMpcYawRate(u.omega, c.dt);
    const double next_yaw = state.yaw + 0.4 * (lower_zero + ref_angle * kPi / 180 - state.yaw);
    state.omega = (next_yaw - state.yaw) / c.dt; state.yaw = next_yaw;
  }
  EXPECT_NEAR(state.yaw, kPi / 2, 2 * kPi / 180); EXPECT_TRUE(follow.heading_ready);
  EXPECT_NEAR(ref_angle, 120.0, 2.0);
}
}  // namespace
}  // namespace mas2027_nav_executor

namespace mas2027_nav_executor {
TEST(MpcGimbalFollow, MovesAlongTheSameTrajectoryWithNegativeBodyForwardVelocity)
{
  auto map = std::make_shared<FreeMap>();
  PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true; in.reverse_follow_heading = true;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
  odom->pose.pose.orientation.z = std::sin(-kPi / 4);
  odom->pose.pose.orientation.w = std::cos(-kPi / 4); in.odom = odom;
  const auto result = executor.computeCommand(in);
  ASSERT_EQ(result.status, ExecutorStatus::PUBLISHED);
  EXPECT_LT(result.command.linear.x, -0.02);
  EXPECT_NEAR(result.command.linear.y, 0.0, 1e-5);
  EXPECT_NEAR(result.command.angular.z, 0.0, 1e-5);
  // The body faces -Y; negative body vx still moves toward +Y on the original path.
  EXPECT_GT(-result.command.linear.x, 0.02);
}
TEST(MpcGimbalFollow, PreservesCurveRotationAndWorldTranslationWhenFacingBackward)
{
  auto map = std::make_shared<FreeMap>();
  auto forward = input(); forward.follow_gimbal = true;
  auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>(*forward.trajectory);
  double x = 0, y = 0;
  for (size_t i = 0; i < trajectory->cmds.size(); ++i) {
    const double theta = static_cast<double>(i) * kPi / 60;
    auto & c = trajectory->cmds[i]; c.position.x = x; c.position.y = y;
    c.velocity.x = std::cos(theta); c.velocity.y = std::sin(theta);
    c.acceleration.x = -std::sin(theta) * kPi / 3;
    c.acceleration.y = std::cos(theta) * kPi / 3;
    x += .05 * c.velocity.x; y += .05 * c.velocity.y;
  }
  forward.trajectory = trajectory;
  auto reverse = forward; reverse.reverse_follow_heading = true;
  auto odom = std::make_shared<nav_msgs::msg::Odometry>(*reverse.odom);
  odom->pose.pose.orientation.z = 1; odom->pose.pose.orientation.w = 0; reverse.odom = odom;
  PathExecutor a(config(), params(), nullptr, nullptr, map), b(config(), params(), nullptr, nullptr, map);
  const auto f = a.computeCommand(forward), r = b.computeCommand(reverse);
  ASSERT_EQ(f.status, ExecutorStatus::PUBLISHED); ASSERT_EQ(r.status, ExecutorStatus::PUBLISHED);
  EXPECT_NEAR(r.command.linear.x, -f.command.linear.x, 1e-5);
  EXPECT_NEAR(r.command.linear.y, -f.command.linear.y, 1e-5);
  EXPECT_NEAR(r.command.angular.z, f.command.angular.z, 1e-5);
  EXPECT_GT(r.command.angular.z, 0.0);
}
TEST(MpcGimbalFollow, RampsHealthyCommandsDespiteLaggingFeedbackAndReanchorsAfterAStop)
{
  auto map = std::make_shared<FreeMap>();
  PathExecutor executor(config(), params(), nullptr, nullptr, map);
  auto in = input(); in.follow_gimbal = true;
  double previous_w = 0, first_w = 0, first_speed = 0, last_speed = 0;
  for (int i = 0; i < 8; ++i) {
    const auto out = executor.computeCommand(in);
    ASSERT_EQ(out.status, ExecutorStatus::PUBLISHED);
    EXPECT_LE(std::abs(out.command.angular.z - previous_w), 0.20001);
    EXPECT_LE(std::abs(out.command.angular.z), 2.00001);
    last_speed = std::hypot(out.command.linear.x, out.command.linear.y);
    if (i == 0) {first_w = out.command.angular.z; first_speed = last_speed;}
    previous_w = out.command.angular.z;
  }
  EXPECT_GT(previous_w, first_w + 0.2);
  EXPECT_GT(last_speed, first_speed + 0.2);
  RecordProperty("first_omega_rad_s", std::to_string(first_w));
  RecordProperty("eighth_omega_rad_s", std::to_string(previous_w));
  RecordProperty("first_speed_m_s", std::to_string(first_speed));
  RecordProperty("eighth_speed_m_s", std::to_string(last_speed));
  // A real stale-map stop must reset the anchor to actual speed, not keep accelerating.
  map->stamp = 0;
  EXPECT_EQ(executor.computeCommand(in).status, ExecutorStatus::DYNAMIC_BLOCKED);
  map->stamp = 10;
  const auto resumed = executor.computeCommand(in);
  ASSERT_EQ(resumed.status, ExecutorStatus::PUBLISHED);
  EXPECT_LE(std::abs(resumed.command.angular.z), 0.20001);
}
}  // namespace mas2027_nav_executor

namespace mas2027_nav_executor {
TEST(MpcGimbalFollow, ReverseAngleServoConvergesWithLagWithoutTurningAround)
{
  auto map = std::make_shared<FreeMap>();
  PathExecutor executor(config(), params(), nullptr, nullptr, map);
  GimbalFollowAdapter adapter(GimbalFollowParams{});
  auto in = input(); in.follow_gimbal = true;
  const double lower_zero = -kPi / 6, dt = config().dt;
  double yaw = lower_zero, omega = 0, angle = 0;
  GimbalFollowOutput follow;
  for (int i = 0; i < 400; ++i) {
    follow = adapter.updateHeading((kPi / 2 - yaw) * 180 / kPi, dt);
    ASSERT_TRUE(follow.reverse_heading);
    in.reverse_follow_heading = follow.reverse_heading;
    in.hold_translation = !follow.heading_ready;
    in.alignment_heading_odom = yaw + follow.heading_error_deg * kPi / 180;
    auto odom = std::make_shared<nav_msgs::msg::Odometry>(*in.odom);
    odom->pose.pose.orientation.z = std::sin(yaw / 2);
    odom->pose.pose.orientation.w = std::cos(yaw / 2);
    odom->twist.twist.angular.z = omega; in.odom = odom;
    const auto out = executor.computeCommand(in);
    ASSERT_EQ(out.status, ExecutorStatus::PUBLISHED);
    angle = adapter.integrateMpcYawRate(out.command.angular.z, dt);
    const double next = yaw + .4 * (lower_zero + angle * kPi / 180 - yaw);
    omega = (next - yaw) / dt; yaw = next;
    EXPECT_LT(std::abs(angle), 100.0);
  }
  EXPECT_NEAR(yaw, -kPi / 2, 2 * kPi / 180);
  EXPECT_NEAR(angle, -60.0, 2.0);
  EXPECT_TRUE(follow.heading_ready);
}
} // namespace mas2027_nav_executor
