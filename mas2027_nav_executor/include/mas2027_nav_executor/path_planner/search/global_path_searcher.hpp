#ifndef MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
#define MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_

#include <mutex>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "minco_core/header.hpp"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"

namespace mas2027_nav_executor { class OnlineGlobalMap; }

namespace minco_planner {

class PlannerModeContext;

class GlobalPathSearcher
{
public:
  void configure(std::shared_ptr<tf2_ros::Buffer> tf,
    Astar * astar,
    mas2027_nav_executor::smac::SmacPlanner2DSimple * smac,
    bool use_smac,
    bool allow_unknown,
    double tolerance,
    rclcpp::Logger logger);

  void setOnlineSubgoalClearance(double clearance) { online_subgoal_clearance_ = clearance; }
  void setGoalApproachMaxShorten(double distance)
  { goal_approach_max_shorten_ = std::max(0.0, distance); }
  /// 主搜索在 minimum_clearance 下无解时，允许用该更小净空再搜索一次。
  /// 0 表示关闭；必须小于 SMAC 主 minimum_clearance。局部轨迹还必须通过
  /// 窄道续行门，搜索放宽本身不会降低执行侧硬阈值。
  void setRelaxedMinimumClearance(double clearance)
  { relaxed_minimum_clearance_ = std::max(0.0, clearance); }

  /// 最近一次 plan() 是否确认“任务目标在已知障碍内且附近没有安全排出点”。
  /// 调用方据此停止旧轨迹，不能把它当成普通的临时搜索失败继续跟随。
  bool lastGoalInvalid() const { return last_goal_invalid_; }
  /// The live goal is blocked with no safe ejection, but the accumulated map
  /// has not confirmed a persistent obstacle. Stop and retry this same goal.
  bool lastGoalBlockedRetryable() const { return last_goal_blocked_retryable_; }

  /// Conservative admission check: reject only a goal confirmed occupied in
  /// both the live rolling map and the accumulated map, with no safe approach
  /// in the fully observed local window. Never rejects a window-external goal.
  bool goalKnownInvalid(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context) const;

  /// 最近一次成功路径是否把原始目标替换成接近点/排出点。若是，任务层应把
  /// 该路径终点视为本目标的合法终止点，而不是继续追逐障碍内的原始坐标。
  bool lastGoalShifted() const { return last_goal_shifted_; }

  void setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query);
  void setTerrainGrid(std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain)
  { terrain_ = std::move(terrain); }
  void setOnlineTerrainMap(std::shared_ptr<mas2027_nav_executor::OnlineGlobalMap> terrain)
  { online_terrain_ = std::move(terrain); }

  // 全局搜索不再接收运动学输入（实测速度 / max_speed / max_acceleration）：主搜索是地形图上的
  // 纯栅格 A*，与 mas_nav_2027 的 SMAC2D 同口径，不含速度/加速度可行性约束。
  bool plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const PlannerModeContext & mode_context,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

private:
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  bool planExploration(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  bool makePlanOnQuery(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const std::string & output_frame,
    const std::string & failure_source,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
    mas2027_nav_executor::smac::SmacPlanner2DSimple::EdgeFilter edge_filter = {},
    bool relaxed_search = false);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  Astar * astar_{nullptr};
  mas2027_nav_executor::smac::SmacPlanner2DSimple * smac_{nullptr};
  bool use_smac_{false};
  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain_;
  std::shared_ptr<mas2027_nav_executor::OnlineGlobalMap> online_terrain_;
  bool allow_unknown_{true};
  double tolerance_{0.5};
  double online_subgoal_clearance_{0.33};
  double goal_approach_max_shorten_{1.5};
  double relaxed_minimum_clearance_{0.0};
  bool last_goal_invalid_{false};
  bool last_goal_blocked_retryable_{false};
  bool last_goal_shifted_{false};
  // Keep one reachable observed subgoal across rolling-map replans. Remember
  // completed and failed areas for this mission goal, and briefly preserve a
  // remembered-route direction when live connectivity changes.
  std::optional<geometry_msgs::msg::PoseStamped> online_exploration_goal_;
  std::optional<Eigen::Vector2d> committed_online_subgoal_;
  bool committed_online_from_history_{false};
  double committed_online_best_distance_{0.0};
  double committed_online_progress_stamp_{0.0};
  std::vector<Eigen::Vector2d> completed_online_subgoals_;
  struct FailedOnlineSubgoal
  {
    Eigen::Vector2d position;
    double stamp{0.0};
  };
  std::vector<FailedOnlineSubgoal> failed_online_subgoals_;
  std::optional<Eigen::Vector2d> suspended_history_subgoal_;
  double suspended_history_stamp_{0.0};
  mutable std::mutex search_mutex_;
  rclcpp::Logger logger_{rclcpp::get_logger("GlobalPathSearcher")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
