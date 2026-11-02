#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "mas2027_nav_executor/path_planner/search/directed_terrain.hpp"

namespace mas2027_nav_executor {

// A route on the accumulated session map, ordered from target back to start.
struct HistoryFrontierRoute
{
  Eigen::Vector2d target{Eigen::Vector2d::Zero()};
  std::vector<Eigen::Vector2d> target_to_start;
};

struct HistoryFrontierQuery
{
  DirectedTerrainConfig directed_terrain;
  Eigen::Vector2d start{Eigen::Vector2d::Zero()};
  Eigen::Vector2d goal{Eigen::Vector2d::Zero()};
  // Route cells keep this distance from remembered obstacles and height edges.
  double route_clearance{0.27};
  // Preferred target distance from height edges, used as a soft ranking cost.
  // Hard route/target clearance and terrain transition gates are independent.
  double height_clearance{0.40};
  // The selected target also keeps the online subgoal clearance.
  double target_clearance{0.28};
  // Ignore frontiers that would not move the robot out of its current area.
  double minimum_advance{2.0};
  // Standoff from the unknown boundary, as in the live frontier search.
  double frontier_standoff{0.85};
  // How far the start may be projected to a searchable cell.
  double start_projection{0.6};
  // Candidate exclusions (dead ends, recently completed or failed subgoals).
  std::function<bool(const Eigen::Vector2d &)> excluded;
  // Re-route to a previously chosen target instead of selecting a new one.
  std::optional<Eigen::Vector2d> fixed_target;
};

// Escape a dead end the rolling live window cannot see past: search the
// accumulated map from the robot for a reachable standoff next to never
// observed space, preferring short routes and frontiers toward the goal.
// Only remembered free cells are used; unknown space is never entered.
std::optional<HistoryFrontierRoute> findHistoryFrontierRoute(
  const OnlineGlobalMapSnapshot & map, const HistoryFrontierQuery & request);

}  // namespace mas2027_nav_executor
