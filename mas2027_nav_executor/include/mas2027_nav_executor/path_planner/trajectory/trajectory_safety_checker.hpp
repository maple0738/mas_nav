#ifndef MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class TrajectorySafetyChecker
{
public:
  void configure(double sample_dt, rclcpp::Logger logger);
  void setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query);
  /// Narrow-corridor continuation: enabled when either the measured start
  /// clearance or the reference route itself is already between this floor and
  /// the full threshold. A route tight point only relaxes trajectory samples
  /// that stay near that same route; it never lowers the gate globally.
  void setNarrowPassage(bool enable, double minimum_clearance);

  bool checkPoint(const Eigen::Vector3d & pos, double check_dist) const;
  /// 发布前整条轨迹检查：near_field 以内按“不比起点实测净空更差”，其余按 check_dist。
  /// reference_path 是本次 MINCO 种子的稠密折线，用于把窄道放行限制在计划走廊内。
  bool checkTrajectory(
    const traj_opt::Trajectory & traj, double check_dist, double near_field,
    const std::vector<Eigen::Vector3d> & reference_path = {}) const;
  double getDistance(const Eigen::Vector3d & pos) const;
  bool projectOutOfObstacle(Eigen::Vector3d & pos, double margin) const;

private:
  bool ensureQueryAvailable() const;

  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  double sample_dt_{0.05};
  bool narrow_passage_enable_{false};
  double narrow_passage_clearance_{0.0};
  rclcpp::Logger logger_{rclcpp::get_logger("TrajectorySafetyChecker")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
