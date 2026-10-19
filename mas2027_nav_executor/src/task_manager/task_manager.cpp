#include "mas2027_nav_executor/task_manager/task_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace mas2027_nav_executor {

TaskManager::TaskManager(
  minco_planner::MincoPlanner::Ptr planner, rclcpp::Logger logger, double odom_timeout_s)
: planner_(std::move(planner)), logger_(logger), odom_timeout_s_(odom_timeout_s) {}

void TaskManager::submitGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_goal_ = goal;  // Latest goal wins; old planning results cannot become active.
  minimum_trajectory_stamp_ns_.store(rclcpp::Clock().now().nanoseconds());
  mission_active_.store(true);
  allows_motion_.store(false);
}

void TaskManager::activateCurrentGoal()
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  // A newer RViz goal may have arrived while a slow search was running.
  if (!pending_goal_) allows_motion_.store(true);
}

void TaskManager::handleFailure(const geometry_msgs::msg::PoseStamped & pose, double now_s)
{
  const Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, 0.0);
  if (planner_->getEsdfDistance(position) >= 0.25) return;
  const auto recovery = planner_->recoveryServer();
  if (!recovery) return;
  Eigen::Vector2d velocity;
  const auto decision = recovery->handleReplanFailure(
    now_s, pose, [this](const Eigen::Vector3d & point) {
      return planner_->getEsdfDistance(point);
    }, velocity);
  if (decision == minco_planner::RecoverServer::RecoveryDecision::DO_ESCAPE) {
    escape_velocity_ = velocity;
    state_ = State::RECOVERING;
    RCLCPP_WARN(logger_, "Robot is too close to an obstacle; attempting bounded escape");
  }
}

void TaskManager::markGoalInvalid(const geometry_msgs::msg::PoseStamped & pose)
{
  const bool already_stopped = retryable_goal_blocked_;
  retryable_goal_blocked_ = false;
  RCLCPP_ERROR(logger_,
    "Goal is known to be inside an obstacle with no safe ejection; "
    "stopping once and waiting for a new goal");
  allows_motion_.store(false);
  mission_active_.store(false);
  if (!already_stopped) planner_->publishEmergencyStop(pose);
  planner_->invalidateGlobalPath();
  if (const auto recovery = planner_->recoveryServer()) recovery->clearMissionGoal();
  state_ = State::GOAL_INVALID;
}

void TaskManager::holdRetryableBlockedGoal(
  const geometry_msgs::msg::PoseStamped & pose, double now_s)
{
  allows_motion_.store(false);
  if (!retryable_goal_blocked_) {
    RCLCPP_WARN(logger_,
      "Goal blocked in the live map without accumulated confirmation; "
      "stopping once and retrying the same goal");
    planner_->publishEmergencyStop(pose);
    planner_->invalidateGlobalPath();
  }
  retryable_goal_blocked_ = true;
  state_ = State::PLANNING;
  last_plan_attempt_s_ = now_s;
}

void TaskManager::tick()
{
  if (!planner_) return;
  std::optional<geometry_msgs::msg::PoseStamped> incoming;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    incoming.swap(pending_goal_);
  }
  if (incoming) {
    retryable_goal_blocked_ = false;
    const bool same_goal = goal_ &&
      std::hypot(incoming->pose.position.x - goal_->pose.position.x,
        incoming->pose.position.y - goal_->pose.position.y) <= 0.15;
    goal_ = std::move(incoming);
    mission_active_.store(true);
    last_global_replan_s_ = -1.0;
    if (const auto recovery = planner_->recoveryServer()) {
      recovery->setMissionGoal(*goal_);
    }
    if (same_goal && state_ == State::FOLLOWING) {
      activateCurrentGoal();
    } else {
      planner_->invalidateGlobalPath();
      state_ = State::PLANNING;
      last_plan_attempt_s_ = -1.0;
    }
  }

  // A terminally invalid goal is latched. Map updates and control ticks must
  // not retry its SMAC search or publish another emergency-stop trajectory.
  // The incoming-goal branch above is the only transition out of this state.
  if (state_ == State::GOAL_INVALID) {
    mission_active_.store(false);
    allows_motion_.store(false);
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!planner_->getRobotPose(pose) ||
    std::abs(planner_->nowSeconds() - rclcpp::Time(pose.header.stamp).seconds()) > odom_timeout_s_) {
    allows_motion_.store(false);
    return;
  }
  if (!goal_) {
    state_ = State::IDLE;
    mission_active_.store(false);
    allows_motion_.store(false);
    return;
  }

  geometry_msgs::msg::PoseStamped terminal_goal = *goal_;
  const bool using_shifted_goal = planner_->lastGoalShifted() &&
    planner_->copyLatestGlobalGoal(terminal_goal);
  if (planner_->checkGoalReached(pose, terminal_goal) &&
    planner_->getCurrentSpeed().head<2>().norm() < 0.3)
  {
    goal_.reset();
    if (const auto recovery = planner_->recoveryServer()) recovery->clearMissionGoal();
    planner_->invalidateGlobalPath();
    state_ = State::IDLE;
    mission_active_.store(false);
    allows_motion_.store(false);
    if (using_shifted_goal) {
      RCLCPP_WARN(logger_,
        "Navigation goal reached at safe approach/ejection point (%.2f, %.2f); "
        "original goal was not traversable",
        terminal_goal.pose.position.x, terminal_goal.pose.position.y);
    } else {
      RCLCPP_INFO(logger_, "Navigation goal reached");
    }
    return;
  }

  const double now_s = planner_->nowSeconds();
  if (state_ == State::RECOVERING) {
    const auto recovery = planner_->recoveryServer();
    const Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, 0.0);
    const bool escaped = planner_->getEsdfDistance(position) > 0.40;
    if (!recovery || escaped || !recovery->inRecovery(now_s)) {
      if (recovery) recovery->finishRecovery(escaped, now_s);
      planner_->clearRecoveryDebugVisualization();
      allows_motion_.store(false);
      planner_->invalidateGlobalPath();
      state_ = State::PLANNING;
      last_plan_attempt_s_ = -1.0;
      return;
    }
    const auto escape_started_ns = rclcpp::Clock().now().nanoseconds();
    planner_->publishEscapeCommand(pose, escape_velocity_);
    minimum_trajectory_stamp_ns_.store(escape_started_ns);
    activateCurrentGoal();
    return;
  }
  if (state_ == State::FOLLOWING) {
    // A rolling map path ends at a temporary subgoal. Rebuild toward the
    // original mission goal as the window advances; only the mission goal
    // above can complete the task.
    if (planner_->usesOnlineMapOnly() &&
      (last_global_replan_s_ < 0.0 || now_s - last_global_replan_s_ >= 1.0 ||
       planner_->checkGoalReached(pose))) {
      last_global_replan_s_ = now_s;
      if (!planner_->PlanGlobalPath(pose, *goal_)) {
        if (planner_->lastGoalInvalid()) {
          // 目标已确认落在已知障碍内，且附近没有安全的接近/排出点。此时
          // 不能继续沿用旧轨迹“朝目标方向”开；宁愿原地急停并等待新目标。
          markGoalInvalid(pose);
          return;
        }
        if (planner_->lastGoalBlockedRetryable()) {
          holdRetryableBlockedGoal(pose, now_s);
          return;
        }
        // 在线模式下全局折线只负责给局部规划提供引导。刷新失败不代表当前
        // 已提交的局部轨迹变坏：保留未过期的局部轨迹继续跟随，等下一个
        // 刷新周期重试；只有局部轨迹过期时才急停。指令安全由执行前门负责。
        if (!planner_->isTrajectoryTimeExpired(now_s)) {
          RCLCPP_WARN(logger_, "Global path refresh failed; keeping current unexpired local trajectory");
          return;
        }
        last_plan_attempt_s_ = now_s;
        allows_motion_.store(false);
        planner_->publishEmergencyStop(pose);
        planner_->invalidateGlobalPath();
        state_ = State::PLANNING;
        return;
      }
      retryable_goal_blocked_ = false;
      last_local_replan_s_ = -1.0;
    }
    const bool expired = planner_->isTrajectoryTimeExpired(now_s);
    const bool due = last_local_replan_s_ < 0.0 ||
      now_s - last_local_replan_s_ >= planner_->getForceReplanPeriod();
    if (!expired && !due) return;
    last_local_replan_s_ = now_s;
    const auto plan_started_ns = rclcpp::Clock().now().nanoseconds();
    if (planner_->ReplanLocal(pose)) {
      if (const auto recovery = planner_->recoveryServer()) recovery->onReplanSuccess();
      minimum_trajectory_stamp_ns_.store(plan_started_ns);
      activateCurrentGoal();
      return;
    }
    if (!expired) return;  // The previous safe trajectory is still executable.
    allows_motion_.store(false);
    state_ = State::PLANNING;
    planner_->invalidateGlobalPath();
    handleFailure(pose, now_s);
    if (state_ == State::RECOVERING) return;
  }

  if (state_ != State::PLANNING) return;
  if (last_plan_attempt_s_ >= 0.0 && now_s - last_plan_attempt_s_ < 0.5) return;
  last_plan_attempt_s_ = now_s;
  if ((planner_->usesOnlineMapOnly() || !planner_->hasGlobalPath()) &&
    !planner_->PlanGlobalPath(pose, *goal_)) {
    if (planner_->lastGoalInvalid()) {
      markGoalInvalid(pose);
      return;
    }
    if (planner_->lastGoalBlockedRetryable()) {
      holdRetryableBlockedGoal(pose, now_s);
      return;
    }
    retryable_goal_blocked_ = false;
    RCLCPP_WARN(logger_, "Global path search failed; retrying");
    handleFailure(pose, now_s);
    return;
  }
  retryable_goal_blocked_ = false;
  last_global_replan_s_ = now_s;
  const auto plan_started_ns = rclcpp::Clock().now().nanoseconds();
  if (!planner_->ReplanLocal(pose)) {
    RCLCPP_WARN(logger_, "MINCO path generation failed; retrying");
    handleFailure(pose, now_s);
    return;
  }
  if (const auto recovery = planner_->recoveryServer()) recovery->onReplanSuccess();
  state_ = State::FOLLOWING;
  last_local_replan_s_ = now_s;
  minimum_trajectory_stamp_ns_.store(plan_started_ns);
  activateCurrentGoal();
}

}  // namespace mas2027_nav_executor
