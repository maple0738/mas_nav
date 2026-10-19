#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/minco_planner.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace mas2027_nav_executor {

// Owns the goal and replanning lifecycle. MincoPlanner only computes paths.
class TaskManager final
{
public:
  TaskManager(minco_planner::MincoPlanner::Ptr planner, rclcpp::Logger logger, double odom_timeout_s);

  void submitGoal(const geometry_msgs::msg::PoseStamped & goal);
  void tick();
  // Two-gate safety: trajectory publication validates the full path, while
  // PathExecutor::checkCommandSafety() is the only runtime stop gate.
  bool allowsMotion() const { return allows_motion_.load(); }
  /// Goal lifecycle is independent of motion permission: while a goal is being
  /// planned or retried, the executor may hold the last gimbal alignment even
  /// though no trajectory is currently executable.
  bool hasActiveGoal() const { return mission_active_.load(); }
  bool acceptsTrajectory(const rclcpp::Time & stamp) const
  {
    return allowsMotion() && stamp.nanoseconds() >= minimum_trajectory_stamp_ns_.load();
  }

private:
  enum class State { IDLE, PLANNING, FOLLOWING, RECOVERING, GOAL_INVALID };
  void activateCurrentGoal();
  void handleFailure(const geometry_msgs::msg::PoseStamped & pose, double now_s);
  void markGoalInvalid(const geometry_msgs::msg::PoseStamped & pose);
  void holdRetryableBlockedGoal(const geometry_msgs::msg::PoseStamped & pose, double now_s);

  minco_planner::MincoPlanner::Ptr planner_;
  rclcpp::Logger logger_;
  double odom_timeout_s_;
  std::mutex pending_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::atomic_bool allows_motion_{false};
  std::atomic_bool mission_active_{false};
  std::atomic<int64_t> minimum_trajectory_stamp_ns_{0};
  Eigen::Vector2d escape_velocity_{Eigen::Vector2d::Zero()};
  State state_{State::IDLE};
  bool retryable_goal_blocked_{false};
  double last_plan_attempt_s_{-1.0};
  double last_global_replan_s_{-1.0};
  double last_local_replan_s_{-1.0};
};

}  // namespace mas2027_nav_executor
