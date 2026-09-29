#ifndef MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
#define MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_

#include "minco_core/header.hpp"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"

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

  void setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query);
  void setTerrainGrid(std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain)
  { terrain_ = std::move(terrain); }

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
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  Astar * astar_{nullptr};
  mas2027_nav_executor::smac::SmacPlanner2DSimple * smac_{nullptr};
  bool use_smac_{false};
  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain_;
  bool allow_unknown_{true};
  double tolerance_{0.5};
  double online_subgoal_clearance_{0.33};
  rclcpp::Logger logger_{rclcpp::get_logger("GlobalPathSearcher")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
