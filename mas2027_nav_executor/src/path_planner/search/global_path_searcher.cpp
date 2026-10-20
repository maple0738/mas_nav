#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "rog_map/projection_layer.hpp"
#include "rog_map/query_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include "tf2/utils.h"

namespace minco_planner {

namespace {

bool projectStartToFreeCell(
  const std::shared_ptr<rog_map::MapQueryInterface> & map, unsigned int & mx, unsigned int & my)
{
  if (!map || map->isFree(mx, my)) {
    return true;
  }
  const unsigned int nx = map->sizeX();
  const unsigned int ny = map->sizeY();
  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};
  for (int k = 0; k < 4; ++k) {
    int sx = static_cast<int>(mx) + dx4[k];
    int sy = static_cast<int>(my) + dy4[k];
    if (sx < 0 || sy < 0 || sx >= static_cast<int>(nx) || sy >= static_cast<int>(ny)) {
      continue;
    }
    if (map->isFree(static_cast<unsigned int>(sx), static_cast<unsigned int>(sy))) {
      mx = static_cast<unsigned int>(sx);
      my = static_cast<unsigned int>(sy);
      return true;
    }
  }
  constexpr int kMaxRadius = 50;
  for (int r = 1; r <= kMaxRadius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::abs(dx) != r && std::abs(dy) != r) {
          continue;
        }
        int cx = static_cast<int>(mx) + dx;
        int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(nx) || cy >= static_cast<int>(ny)) {
          continue;
        }
        if (map->isFree(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy))) {
          mx = static_cast<unsigned int>(cx);
          my = static_cast<unsigned int>(cy);
          return true;
        }
      }
    }
  }
  return false;
}

bool cellHasSearchClearance(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  unsigned int mx,
  unsigned int my,
  double required_clearance)
{
  if (!map || !(required_clearance > 0.0)) {
    return true;
  }
  double wx = 0.0;
  double wy = 0.0;
  map->mapToWorld(mx, my, wx, wy);
  const auto field = map->query({wx, wy, 0.0});
  return field.ok && std::isfinite(field.distance) &&
    static_cast<float>(field.distance) >= static_cast<float>(required_clearance);
}

// USTC's planner ejects both start and goal from the obstacle before search.
// The existing projectStartToFreeCell() only looks at cost; a start cell that
// is free but within the search ESDF threshold still makes SMAC reject with
// iterations=1. Search outward for a nearby cell that satisfies both cost and
// clearance, but keep the shift bounded so the global path stays attached to
// the robot. The local planner still starts from the actual measured pose.
bool projectStartToSearchableCell(
  const std::shared_ptr<rog_map::MapQueryInterface> & map,
  unsigned int & mx,
  unsigned int & my,
  double required_clearance,
  double max_shift)
{
  if (!map || !(required_clearance > 0.0) || !(max_shift > 0.0)) {
    return true;
  }
  if (map->isFree(mx, my) && cellHasSearchClearance(map, mx, my, required_clearance)) {
    return true;
  }
  const double resolution = map->resolution();
  if (!std::isfinite(resolution) || resolution <= 1.0e-6) {
    return false;
  }
  const int max_radius = std::max(1, static_cast<int>(std::ceil(max_shift / resolution)));
  const int nx = static_cast<int>(map->sizeX());
  const int ny = static_cast<int>(map->sizeY());
  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) {
          continue;
        }
        if (std::hypot(static_cast<double>(dx) * resolution,
            static_cast<double>(dy) * resolution) > max_shift + 1.0e-9) {
          continue;
        }
        const int cx = static_cast<int>(mx) + dx;
        const int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) {
          continue;
        }
        const auto ux = static_cast<unsigned int>(cx);
        const auto uy = static_cast<unsigned int>(cy);
        if (!map->isFree(ux, uy) ||
            !cellHasSearchClearance(map, ux, uy, required_clearance)) {
          continue;
        }
        mx = ux;
        my = uy;
        return true;
      }
    }
  }
  return false;
}

const char * cellCostLabel(const unsigned char cost)
{
  if (cost == kUnknownCost) {
    return "unknown";
  }
  if (cost >= kInscribedCost) {
    return "blocked/inflated";
  }
  if (cost > 128u) {
    return "high-cost";
  }
  return "free";
}

bool smacTraversableCost(const unsigned char cost, const bool allow_unknown)
{
  if (cost == kUnknownCost) {
    return allow_unknown;
  }
  return cost < kInscribedCost;
}

// 目标格本身不可通行或净空不足时，沿 start→goal 的方向从目标往后退，
// 找最近一个满足搜索净空的可通行格作为接近点。搜索路径仍然经过完整
// SMAC/MINCO 安全和地形校验；这里只避免整个目标因为最后一格被直接拒掉。
bool findGoalApproach(
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double required_clearance,
  bool allow_unknown,
  double max_shorten,
  geometry_msgs::msg::Pose & approach,
  double & shorten_distance)
{
  if (!query || !(required_clearance > 0.0) || !(max_shorten > 0.0) ||
    !std::isfinite(start.position.x) || !std::isfinite(start.position.y) ||
    !std::isfinite(goal.position.x) || !std::isfinite(goal.position.y)) {
    return false;
  }

  unsigned int sx = 0U;
  unsigned int sy = 0U;
  unsigned int gx = 0U;
  unsigned int gy = 0U;
  if (!query->worldToMap(start.position.x, start.position.y, sx, sy) ||
    !query->worldToMap(goal.position.x, goal.position.y, gx, gy)) {
    return false;
  }

  const double dx = start.position.x - goal.position.x;
  const double dy = start.position.y - goal.position.y;
  const double length = std::hypot(dx, dy);
  if (!std::isfinite(length) || length <= 1e-6) {
    return false;
  }
  const double ux = dx / length;
  const double uy = dy / length;
  const double resolution = query->resolution();
  const double step = (std::isfinite(resolution) && resolution > 1e-6) ?
    std::max(0.05, resolution) : 0.05;

  for (double distance = step; distance <= max_shorten + 1e-9 && distance < length;
    distance += step) {
    const double wx = goal.position.x + ux * distance;
    const double wy = goal.position.y + uy * distance;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!query->worldToMap(wx, wy, mx, my)) {
      continue;
    }
    // A zero-length approach path is not useful: if the robot is already on the
    // only safe cell, let the normal fallback/recovery path handle the blockage.
    if (mx == sx && my == sy) {
      continue;
    }
    if (!smacTraversableCost(query->value(mx, my), allow_unknown)) {
      continue;
    }
    // SMAC checks the ESDF at the cell center, not at the arbitrary point on
    // the ray. Query and command the cell center so the approach endpoint is
    // exactly the cell the search will accept.
    double cx = 0.0;
    double cy = 0.0;
    query->mapToWorld(mx, my, cx, cy);
    const auto clearance = query->query({cx, cy, 0.0});
    if (!clearance.ok || !std::isfinite(clearance.distance) ||
      static_cast<float>(clearance.distance) < static_cast<float>(required_clearance)) {
      continue;
    }
    approach = goal;
    approach.position.x = cx;
    approach.position.y = cy;
    shorten_distance = std::hypot(cx - goal.position.x, cy - goal.position.y);
    return true;
  }
  return false;
}

// 在目标格被占据、且沿 start→goal 的直线接近点也找不到时，退化为在目标周围
// 的环带内寻找最近的可通行格。只接受朝机器人一侧的候选，避免把目标“弹出”
// 到墙的另一面；如果目标周围只有远侧自由格，仍交给子目标回退处理。
bool findGoalEjection(
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double required_clearance,
  bool allow_unknown,
  double max_shift,
  geometry_msgs::msg::Pose & ejected,
  double & shift_distance,
  bool & used_radial)
{
  used_radial = false;
  if (!query || !(required_clearance > 0.0) || !(max_shift > 0.0) ||
    !std::isfinite(start.position.x) || !std::isfinite(start.position.y) ||
    !std::isfinite(goal.position.x) || !std::isfinite(goal.position.y)) {
    return false;
  }
  unsigned int sx = 0U;
  unsigned int sy = 0U;
  unsigned int gx = 0U;
  unsigned int gy = 0U;
  if (!query->worldToMap(start.position.x, start.position.y, sx, sy) ||
    !query->worldToMap(goal.position.x, goal.position.y, gx, gy)) {
    return false;
  }
  const double resolution = query->resolution();
  if (!std::isfinite(resolution) || resolution <= 1.0e-6) {
    return false;
  }
  const Eigen::Vector2d goal_xy(goal.position.x, goal.position.y);
  const Eigen::Vector2d start_xy(start.position.x, start.position.y);
  const Eigen::Vector2d toward_start = start_xy - goal_xy;
  const double toward_start_norm = toward_start.norm();
  if (!(toward_start_norm > 1.0e-6)) {
    return false;
  }
  const Eigen::Vector2d toward_start_unit = toward_start / toward_start_norm;
  const int max_radius = std::max(1, static_cast<int>(std::ceil(max_shift / resolution)));
  const int nx = static_cast<int>(query->sizeX());
  const int ny = static_cast<int>(query->sizeY());

  double best_score = std::numeric_limits<double>::infinity();
  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::abs(dx) != radius && std::abs(dy) != radius) {
          continue;
        }
        const int cx = static_cast<int>(gx) + dx;
        const int cy = static_cast<int>(gy) + dy;
        if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) {
          continue;
        }
        if (cx == static_cast<int>(sx) && cy == static_cast<int>(sy)) {
          continue;
        }
        double wx = 0.0;
        double wy = 0.0;
        query->mapToWorld(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy), wx, wy);
        const double shift = std::hypot(wx - goal.position.x, wy - goal.position.y);
        if (shift > max_shift + 1.0e-9) {
          continue;
        }
        // Keep the ejection on the robot side of the goal. This is what
        // distinguishes a valid approach point from a free cell behind the wall.
        const double side = Eigen::Vector2d(wx - goal.position.x, wy - goal.position.y)
          .dot(toward_start_unit);
        if (side < -0.05) {
          continue;
        }
        const auto ux = static_cast<unsigned int>(cx);
        const auto uy = static_cast<unsigned int>(cy);
        if (!smacTraversableCost(query->value(ux, uy), allow_unknown)) {
          continue;
        }
        const auto field = query->query({wx, wy, 0.0});
        if (!field.ok || !std::isfinite(field.distance) ||
          static_cast<float>(field.distance) < static_cast<float>(required_clearance)) {
          continue;
        }
        // Among candidates on the same ring, prefer the one closest to the
        // robot; that is the shortest detour around the blocked goal.
        const double score = shift + 0.05 * std::hypot(wx - start.position.x, wy - start.position.y);
        if (score < best_score) {
          best_score = score;
          ejected = goal;
          ejected.position.x = wx;
          ejected.position.y = wy;
          shift_distance = shift;
          used_radial = true;
        }
      }
    }
    if (used_radial) {
      return true;
    }
  }
  return false;
}

bool knownBlockedWithoutEjection(
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double required_clearance,
  bool allow_unknown,
  double max_shorten)
{
  if (!query || !(required_clearance > 0.0) || !(max_shorten > 0.0)) return false;
  unsigned int gx = 0U, gy = 0U;
  if (!query->worldToMap(goal.position.x, goal.position.y, gx, gy) ||
    smacTraversableCost(query->value(gx, gy), allow_unknown)) {
    return false;
  }
  geometry_msgs::msg::Pose approach;
  double shorten_distance = 0.0;
  bool radial = false;
  return !findGoalApproach(query, start, goal, required_clearance,
      allow_unknown, max_shorten, approach, shorten_distance) &&
    !findGoalEjection(query, start, goal, required_clearance,
      allow_unknown, max_shorten, approach, shorten_distance, radial);
}

bool hasCompleteApproachWindow(
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const geometry_msgs::msg::Pose & goal,
  double max_shorten)
{
  if (!query || !(max_shorten > 0.0)) return false;
  unsigned int gx = 0U, gy = 0U;
  const double x = goal.position.x, y = goal.position.y;
  if (!query->worldToMap(x, y, gx, gy)) return false;
  const double margin = max_shorten + query->resolution();
  const double left = x - query->originX();
  const double bottom = y - query->originY();
  const double right = query->sizeX() * query->resolution() - left;
  const double top = query->sizeY() * query->resolution() - bottom;
  return std::min({left, bottom, right, top}) > margin;
}

void logCellDiagnostics(const rclcpp::Logger & logger,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & source,
  const std::string & label,
  unsigned int mx,
  unsigned int my,
  bool allow_unknown,
  double minimum_clearance = 0.0)
{
  if (!query || !query->isValid(mx, my)) {
    RCLCPP_WARN(logger,
      "[MincoPlanner] %s %s cell=(%u,%u) is invalid for map size=%ux%u",
      source.c_str(),
      label.c_str(),
      mx,
      my,
      query ? query->sizeX() : 0U,
      query ? query->sizeY() : 0U);
    return;
  }

  double wx = 0.0;
  double wy = 0.0;
  query->mapToWorld(mx, my, wx, wy);
  const unsigned char cost = query->value(mx, my);
  const auto field = query->query({wx, wy, 0.0});
  const bool field_ok = field.ok && std::isfinite(field.distance);
  const bool cost_traversable = smacTraversableCost(cost, allow_unknown);
  const bool clearance_ok = minimum_clearance <= 0.0 ||
    (field_ok && static_cast<float>(field.distance) >= static_cast<float>(minimum_clearance));

  int valid = 0;
  int unknown = 0;
  int free = 0;
  int blocked = 0;
  int cost_traversable_count = 0;
  int invalid = 0;
  constexpr int kRadius = 2;
  for (int dy = -kRadius; dy <= kRadius; ++dy) {
    for (int dx = -kRadius; dx <= kRadius; ++dx) {
      const int cx = static_cast<int>(mx) + dx;
      const int cy = static_cast<int>(my) + dy;
      if (cx < 0 || cy < 0 || cx >= static_cast<int>(query->sizeX()) ||
          cy >= static_cast<int>(query->sizeY())) {
        ++invalid;
        continue;
      }
      const unsigned int ux = static_cast<unsigned int>(cx);
      const unsigned int uy = static_cast<unsigned int>(cy);
      const unsigned char nb_cost = query->value(ux, uy);
      ++valid;
      if (nb_cost == kUnknownCost) {
        ++unknown;
      } else if (nb_cost >= kInscribedCost) {
        ++blocked;
      } else {
        ++free;
      }
      if (smacTraversableCost(nb_cost, allow_unknown)) {
        ++cost_traversable_count;
      }
    }
  }

  RCLCPP_WARN(logger,
    "[MincoPlanner] %s %s diag: cell=(%u,%u) world=(%.3f,%.3f) cost=%u(%s) query_free=%s "
    "cost_traversable=%s smac_traversable=%s clearance=%.3f minimum_clearance=%.3f "
    "field_status=%s snapshot=%llu projection=%llu mask=%llu field=%llu "
    "allow_unknown=%s 5x5(valid=%d invalid=%d free=%d unknown=%d blocked=%d "
    "cost_traversable=%d)",
    source.c_str(),
    label.c_str(),
    mx,
    my,
    wx,
    wy,
    static_cast<unsigned int>(cost),
    cellCostLabel(cost),
    query->isFree(mx, my) ? "true" : "false",
    cost_traversable ? "true" : "false",
    cost_traversable && clearance_ok ? "true" : "false",
    field.distance,
    minimum_clearance,
    rog_map::queryStatusName(field.status),
    static_cast<unsigned long long>(field.snapshot_sequence),
    static_cast<unsigned long long>(field.projection_sequence),
    static_cast<unsigned long long>(field.mask_sequence),
    static_cast<unsigned long long>(field.field_sequence),
    allow_unknown ? "true" : "false",
    valid,
    invalid,
    free,
    unknown,
    blocked,
    cost_traversable_count);
}

}  // namespace

void GlobalPathSearcher::configure(std::shared_ptr<tf2_ros::Buffer> tf,
  Astar * astar,
  mas2027_nav_executor::smac::SmacPlanner2DSimple * smac,
  bool use_smac,
  bool allow_unknown,
  double tolerance,
  rclcpp::Logger logger)
{
  tf_ = std::move(tf);
  astar_ = astar;
  smac_ = smac;
  use_smac_ = use_smac;
  allow_unknown_ = allow_unknown;
  tolerance_ = tolerance;
  logger_ = logger;
}

void GlobalPathSearcher::setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query)
{
  global_query_ = global_query;
  if (astar_) {
    astar_->setMap(global_query_);
  }
}

bool GlobalPathSearcher::normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
  const std::string & fallback_frame,
  const std::string & target_frame,
  const std::string & context,
  geometry_msgs::msg::PoseStamped & out) const
{
  out = in;
  if (out.header.frame_id.empty()) {
    out.header.frame_id = fallback_frame;
    RCLCPP_WARN(logger_,
      "[MincoPlanner] %s pose frame is empty, treating it as %s.",
      context.c_str(),
      fallback_frame.c_str());
  }

  if (out.header.frame_id == target_frame) {
    out.header.frame_id = target_frame;
    return true;
  }

  if (!tf_) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Cannot transform %s pose from %s to %s: TF buffer is null.",
      context.c_str(),
      out.header.frame_id.c_str(),
      target_frame.c_str());
    return false;
  }

  try {
    out = tf_->transform(out, target_frame);
    out.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_,
      "[MincoPlanner] Failed to transform %s pose from %s to %s: %s",
      context.c_str(),
      in.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return false;
  }
}

bool GlobalPathSearcher::goalKnownInvalid(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context) const
{
  std::lock_guard<std::mutex> lock(search_mutex_);
  if (terrain_ || !use_smac_ || !smac_ || !online_terrain_) return false;
  const auto query = mode_context.globalQuery();
  const auto remembered = online_terrain_->snapshot();
  if (!query || !remembered) return false;

  geometry_msgs::msg::PoseStamped start_rog, goal_rog;
  if (!normalizePoseToFrame(start, mode_context.rogFrame(), mode_context.rogFrame(),
      "goal admission start", start_rog) ||
    !normalizePoseToFrame(goal, mode_context.rogFrame(), mode_context.rogFrame(),
      "goal admission goal", goal_rog)) {
    return false;
  }
  unsigned int gx = 0U, gy = 0U, hx = 0U, hy = 0U;
  const double x = goal_rog.pose.position.x;
  const double y = goal_rog.pose.position.y;
  if (!query->worldToMap(x, y, gx, gy) ||
    !remembered->worldToMap(x, y, hx, hy) || remembered->value(hx, hy) != 254U) {
    return false;
  }
  // A missing approach candidate near the rolling-window edge is not proof
  // that no candidate exists. Defer such goals to normal online exploration.
  if (!hasCompleteApproachWindow(query, goal_rog.pose, goal_approach_max_shorten_)) return false;
  const double primary = smac_->minimumClearance();
  const double relaxed = relaxed_minimum_clearance_ > 0.0 &&
    relaxed_minimum_clearance_ < primary ? relaxed_minimum_clearance_ : primary;
  return knownBlockedWithoutEjection(query, start_rog.pose, goal_rog.pose,
    relaxed, allow_unknown_, goal_approach_max_shorten_);
}

bool GlobalPathSearcher::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  std::lock_guard<std::mutex> lock(search_mutex_);
  return planExploration(start, goal, mode_context, latest_global_path);
}

bool GlobalPathSearcher::planExploration(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  last_goal_invalid_ = false;
  last_goal_blocked_retryable_ = false;
  last_goal_shifted_ = false;
  const auto query = mode_context.globalQuery();
  if (!astar_ || !query) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap global search query is unavailable.");
    return false;
  }

  geometry_msgs::msg::PoseStamped start_rog;
  geometry_msgs::msg::PoseStamped goal_rog;
  if (!normalizePoseToFrame(
        start, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION start", start_rog) ||
      !normalizePoseToFrame(
        goal, mode_context.rogFrame(), mode_context.rogFrame(), "EXPLORATION goal", goal_rog)) {
    return false;
  }

  const auto & goal_position = goal_rog.pose.position;
  if (!online_exploration_goal_ ||
    std::hypot(goal_position.x - online_exploration_goal_->pose.position.x,
      goal_position.y - online_exploration_goal_->pose.position.y) > 1e-3 ||
    goal_rog.header.stamp.sec != online_exploration_goal_->header.stamp.sec ||
    goal_rog.header.stamp.nanosec != online_exploration_goal_->header.stamp.nanosec) {
    online_exploration_goal_ = goal_rog;
    committed_online_subgoal_.reset();
    committed_online_from_history_ = false;
    completed_online_subgoals_.clear();
    failed_online_subgoals_.clear();
    suspended_history_subgoal_.reset();
  }

  std::function<bool()> cancel_checker = []() {
    return !rclcpp::ok();
  };

  // Static terrain owns global topology; ROGMap owns all online-obstacle handling.
  if (terrain_) {
    const auto terrain = terrain_->snapshot();
    if (!terrain) {
      RCLCPP_WARN(logger_, "Static terrain layers are not ready");
      return false;
    }
    try {
      const auto rog_to_map = tf_->lookupTransform(
        terrain->cost.header.frame_id, mode_context.rogFrame(), tf2::TimePointZero);
      const auto map_to_rog = tf_->lookupTransform(
        mode_context.rogFrame(), terrain->cost.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped start_map, goal_map;
      tf2::doTransform(start_rog, start_map, rog_to_map);
      tf2::doTransform(goal_rog, goal_map, rog_to_map);
      // 静态地形图上的纯栅格 A* 作为主搜索：无速度/加速度可行性约束、无扩展预算，
      // 目标可通行即出解，远处目标也能毫秒级返回。切勿改回全向 Kino 状态格点搜索：
      // 它按速度/加速度扩展状态格点，单节点最多 248 个后继，真实障碍图上 3 m 起大量方向无解。
      auto terrain_query = std::make_shared<mas2027_nav_executor::TerrainMapQuery>(terrain_);
      nav_msgs::msg::Path terrain_plan;
      terrain_plan.header.stamp = rclcpp::Clock().now();
      terrain_plan.header.frame_id = mode_context.outputFrame();
      auto slope_edge = [terrain, terrain_query](unsigned int x, unsigned int y,
          unsigned int next_x, unsigned int next_y) {
        double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
        terrain_query->mapToWorld(x, y, ax, ay);
        terrain_query->mapToWorld(next_x, next_y, bx, by);
        return terrain->slopeTransition({ax, ay}, {bx, by});
      };
      if (!makePlanOnQuery(start_map.pose, goal_map.pose, terrain_query,
            mode_context.outputFrame(), "Terrain", tolerance_, cancel_checker, terrain_plan,
            latest_global_path, slope_edge)) {
        return false;
      }

      // makePlanOnQuery 的 output_frame 只写 header.frame_id、不做坐标变换，而路径点来自
      // query->mapToWorld()（地形图查询坐标系 map）：必须换算到 outputFrame，否则种子路径
      // 整体错位，MINCO 只能沿错误参考线优化，表现为"出解但车一动不动"。
      for (auto & pose : latest_global_path) {
        geometry_msgs::msg::PoseStamped in = pose;
        in.header.frame_id = terrain->cost.header.frame_id;
        geometry_msgs::msg::PoseStamped out;
        tf2::doTransform(in, out, map_to_rog);
        out.header.frame_id = mode_context.outputFrame();
        pose = out;
      }
      return latest_global_path.size() >= 2U;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Cannot align terrain map with planning frame: %s", ex.what());
      return false;
    }
  }

  // Online-only mode searches the rolling ROGMap and advances using observed subgoals.
  const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(query);
  const auto semantic = adapter ? adapter->snapshot() : nullptr;
  const auto remembered = online_terrain_ ? online_terrain_->snapshot() : nullptr;

  unsigned int sx = 0;
  unsigned int sy = 0;
  if (!query->worldToMap(start_rog.pose.position.x, start_rog.pose.position.y, sx, sy)) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] ROGMap boundary check failed: EXPLORATION start (%.2f, %.2f) is outside ROGMap.",
      start_rog.pose.position.x,
      start_rog.pose.position.y);
    return false;
  }

  unsigned int gx = 0;
  unsigned int gy = 0;
  const bool goal_inside = query->worldToMap(goal_rog.pose.position.x, goal_rog.pose.position.y, gx, gy);
  const auto live_sides = semantic && semantic->width == static_cast<int>(query->sizeX()) &&
      semantic->height == static_cast<int>(query->sizeY()) ?
    std::make_shared<const std::vector<uint8_t>>(
      mas2027_nav_executor::buildSlopeSideMask(semantic->width, semantic->height,
        semantic->terrain_labels, semantic->uphill_x, semantic->uphill_y)) : nullptr;
  const auto online_edge = [remembered, live_sides, query](unsigned int x, unsigned int y,
      unsigned int next_x, unsigned int next_y) {
    if (live_sides && live_sides->size() ==
        static_cast<size_t>(query->sizeX()) * query->sizeY()) {
      const size_t a = static_cast<size_t>(y) * query->sizeX() + x;
      const size_t b = static_cast<size_t>(next_y) * query->sizeX() + next_x;
      if ((*live_sides)[a] || (*live_sides)[b]) return false;
      if (x != next_x && y != next_y &&
          ((*live_sides)[static_cast<size_t>(y) * query->sizeX() + next_x] ||
           (*live_sides)[static_cast<size_t>(next_y) * query->sizeX() + x])) return false;
    }
    if (remembered) {
      double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
      query->mapToWorld(x, y, ax, ay);
      query->mapToWorld(next_x, next_y, bx, by);
      if (!remembered->terrainTransition(ax, ay, bx, by)) return false;
    }
    return true;
  };
  const double goal_distance = std::hypot(
    goal_rog.pose.position.x - start_rog.pose.position.x,
    goal_rog.pose.position.y - start_rog.pose.position.y);
  bool defer_direct_goal = false;
  if (goal_inside && remembered &&
    goal_distance > goal_approach_max_shorten_) {
    unsigned int hx = 0U, hy = 0U;
    defer_direct_goal = remembered->worldToMap(
      goal_rog.pose.position.x, goal_rog.pose.position.y, hx, hy) &&
      remembered->isFree(hx, hy) &&
      !smacTraversableCost(query->value(gx, gy), allow_unknown_);
  }
  // A remembered-free destination can be temporarily occupied in the live
  // rolling window. Plan a reachable observed subgoal until the robot nears
  // it; do not promote a distant, transient ejection to the mission endpoint.
  // The live map remains authoritative for every planned local segment.
  if (defer_direct_goal) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Goal (%.2f,%.2f) is free in the accumulated map but "
      "blocked in the live window; deferring endpoint ejection until closer.",
      goal_rog.pose.position.x, goal_rog.pose.position.y);
  }
  if (goal_inside && !defer_direct_goal) {
    nav_msgs::msg::Path direct_plan;
    direct_plan.header.stamp = rclcpp::Clock().now();
    direct_plan.header.frame_id = mode_context.outputFrame();
    if (makePlanOnQuery(start_rog.pose,
          goal_rog.pose,
          query,
          mode_context.outputFrame(),
          "ROGMap",
          tolerance_,
          cancel_checker,
          direct_plan,
          latest_global_path,
          online_edge)) {
      committed_online_subgoal_.reset();
      committed_online_from_history_ = false;
      suspended_history_subgoal_.reset();
      return true;
    }
    RCLCPP_WARN(logger_,
      "[MincoPlanner] Exploration goal direct search failed; "
      "see endpoint diagnostics above; fallback to a reachable observed subgoal.");

    // Direct search also tries the line approach and radial ejection. If the
    // live goal is blocked and neither exists, the fallback search must not
    // keep driving toward that obstacle. B1 confirmation distinguishes a
    // terminally invalid goal from a transient blockage that should be retried.
    if (use_smac_ && smac_) {
      const unsigned char requested_goal_cost = query->value(gx, gy);
      if (!smacTraversableCost(requested_goal_cost, allow_unknown_)) {
        const double primary = smac_->minimumClearance();
        const double relaxed = relaxed_minimum_clearance_ > 0.0 &&
          relaxed_minimum_clearance_ < primary ? relaxed_minimum_clearance_ : primary;
        if (hasCompleteApproachWindow(query, goal_rog.pose, goal_approach_max_shorten_) &&
          knownBlockedWithoutEjection(query, start_rog.pose, goal_rog.pose,
            relaxed, allow_unknown_, goal_approach_max_shorten_)) {
          unsigned int hx = 0U, hy = 0U;
          const bool confirmed = remembered &&
            remembered->worldToMap(goal_rog.pose.position.x, goal_rog.pose.position.y, hx, hy) &&
            remembered->value(hx, hy) == 254U;
          last_goal_invalid_ = confirmed;
          last_goal_blocked_retryable_ = !confirmed;
          RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
            "[MincoPlanner] Requested goal (%.2f,%.2f) is blocked in the live map "
            "(cost=%u, clearance=%.3f), with no safe approach/ejection within %.2f m; "
            "%s instead of exploring toward the obstacle.",
            goal_rog.pose.position.x, goal_rog.pose.position.y,
            static_cast<unsigned int>(requested_goal_cost),
            query->query({goal_rog.pose.position.x, goal_rog.pose.position.y, 0.0}).distance,
            goal_approach_max_shorten_,
            confirmed ? "declaring the confirmed goal invalid" : "stopping and retrying");
          return false;
        }
      }
    }
  }

  // Direct goal search failed. The fallback path below is a rolling subgoal,
  // not a replacement for the mission goal, so it must not inherit the direct
  // approach/ejection endpoint flag.
  last_goal_shifted_ = false;

  const unsigned int nx = query->sizeX();
  const unsigned int ny = query->sizeY();
  if (nx == 0U || ny == 0U) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap boundary search failed: empty ROGMap.");
    return false;
  }

  std::vector<unsigned char> costs;
  const size_t map_size = static_cast<size_t>(nx) * static_cast<size_t>(ny);
  if (!query->copyValues(costs) || costs.size() != map_size) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] ROGMap boundary search failed: cannot copy ROGMap values.");
    return false;
  }

  const auto index_of = [nx](unsigned int x, unsigned int y) {
    return static_cast<size_t>(y) * static_cast<size_t>(nx) + static_cast<size_t>(x);
  };

  const bool allow_unknown = !mode_context.explorationUnknownAsOccupied();
  // The fallback route must obey the same clearance gate as the direct SMAC search,
  // rather than checking only its endpoint and leaving MINCO to reject a tight route.
  const double route_clearance = use_smac_ && smac_ ? smac_->minimumClearance() : 0.0;
  std::vector<int8_t> traversability(map_size, -1);
  const auto traversable = [&](size_t idx) {
    if (traversability[idx] >= 0) return traversability[idx] != 0;
    bool allowed = smacTraversableCost(costs[idx], allow_unknown);
    if (allowed) allowed = online_edge(static_cast<unsigned int>(idx % nx),
      static_cast<unsigned int>(idx / nx), static_cast<unsigned int>(idx % nx),
      static_cast<unsigned int>(idx / nx));
    if (allowed && route_clearance > 0.0) {
      double wx = 0.0, wy = 0.0;
      query->mapToWorld(static_cast<unsigned int>(idx % nx),
        static_cast<unsigned int>(idx / nx), wx, wy);
      const auto field = query->query({wx, wy, 0.0});
      allowed = field.ok && std::isfinite(field.distance) &&
        static_cast<float>(field.distance) >= static_cast<float>(route_clearance);
    }
    traversability[idx] = allowed ? 1 : 0;
    return allowed;
  };

  const size_t start_idx = index_of(sx, sy);
  if (!smacTraversableCost(costs[start_idx], allow_unknown)) {
    projectStartToFreeCell(query, sx, sy);
  }

  const double resolution = std::max(1e-6, query->resolution());

  struct QueueNode
  {
    size_t idx{0};
    double cost{0.0};
    bool operator>(const QueueNode & other) const { return cost > other.cost; }
  };

  std::vector<double> dist(map_size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(map_size, -1);
  std::vector<uint8_t> closed(map_size, 0U);
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  const size_t updated_start_idx = index_of(sx, sy);
  if (!smacTraversableCost(costs[updated_start_idx], allow_unknown)) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] EXPLORATION start cell is not traversable in ROGMap.");
    return false;
  }
  dist[updated_start_idx] = 0.0;
  open.push(QueueNode{updated_start_idx, 0.0});

  const Eigen::Vector2d start_xy(start_rog.pose.position.x, start_rog.pose.position.y);
  const double snapshot_stamp = query->snapshotStampSeconds();
  constexpr double subgoal_reached_distance = 0.40;
  if (suspended_history_subgoal_ &&
      (!std::isfinite(snapshot_stamp) || snapshot_stamp <= 0.0 ||
       snapshot_stamp < suspended_history_stamp_ ||
       snapshot_stamp - suspended_history_stamp_ > 8.0 ||
       (start_xy - *suspended_history_subgoal_).norm() <= subgoal_reached_distance)) {
    suspended_history_subgoal_.reset();
  }
  const auto recently_completed = [this, snapshot_stamp](const Eigen::Vector2d & candidate) {
    const bool completed = std::any_of(
      completed_online_subgoals_.begin(), completed_online_subgoals_.end(),
      [&candidate](const Eigen::Vector2d & completed) {
        return (candidate - completed).norm() < 0.80;
      });
    const bool failed = std::any_of(
      failed_online_subgoals_.begin(), failed_online_subgoals_.end(),
      [&candidate, snapshot_stamp](const FailedOnlineSubgoal & entry) {
        return std::isfinite(snapshot_stamp) && snapshot_stamp >= entry.stamp &&
          snapshot_stamp - entry.stamp < 30.0 &&
          (candidate - entry.position).norm() < 1.20;
      });
    return completed || failed;
  };
  const auto reject_subgoal = [this, snapshot_stamp](bool lost_live_connectivity) {
    if (!committed_online_subgoal_) return;
    if (committed_online_from_history_ && lost_live_connectivity) {
      suspended_history_subgoal_ = *committed_online_subgoal_;
      suspended_history_stamp_ = snapshot_stamp;
      RCLCPP_WARN(logger_, "[MincoPlanner] Accumulated-route subgoal (%.2f, %.2f) "
        "lost live connectivity; holding its direction for up to 8 s.",
        committed_online_subgoal_->x(), committed_online_subgoal_->y());
    } else if (std::isfinite(snapshot_stamp) && snapshot_stamp > 0.0) {
      failed_online_subgoals_.push_back({*committed_online_subgoal_, snapshot_stamp});
      if (failed_online_subgoals_.size() > 64U) {
        failed_online_subgoals_.erase(failed_online_subgoals_.begin());
      }
      RCLCPP_WARN(logger_, "[MincoPlanner] Exploration subgoal (%.2f, %.2f) "
        "%s; avoiding its 1.20 m area for 30 s.",
        committed_online_subgoal_->x(), committed_online_subgoal_->y(),
        lost_live_connectivity ? "lost live connectivity" : "stalled");
    }
    committed_online_subgoal_.reset();
    committed_online_from_history_ = false;
  };
  const auto complete_subgoal = [this]() {
    completed_online_subgoals_.push_back(*committed_online_subgoal_);
    if (completed_online_subgoals_.size() > 64U) {
      completed_online_subgoals_.erase(completed_online_subgoals_.begin());
    }
    committed_online_subgoal_.reset();
    committed_online_from_history_ = false;
  };
  if (committed_online_subgoal_) {
    const double remaining = (start_xy - *committed_online_subgoal_).norm();
    if (remaining <= subgoal_reached_distance) {
      complete_subgoal();
    } else if (std::isfinite(snapshot_stamp) && snapshot_stamp > 0.0) {
      if (snapshot_stamp < committed_online_progress_stamp_ ||
        remaining < committed_online_best_distance_ - 0.20) {
        committed_online_best_distance_ = remaining;
        committed_online_progress_stamp_ = snapshot_stamp;
      } else if (snapshot_stamp - committed_online_progress_stamp_ > 8.0) {
        RCLCPP_WARN(logger_, "[MincoPlanner] Exploration subgoal (%.2f, %.2f) "
          "made no 0.20 m progress for 8 s; trying another observed frontier.",
          committed_online_subgoal_->x(), committed_online_subgoal_->y());
        reject_subgoal(false);
      }
    }
  }
  const auto holds_history_direction = [this, &start_xy](const Eigen::Vector2d & candidate) {
    if (!suspended_history_subgoal_) return true;
    const Eigen::Vector2d forward = *suspended_history_subgoal_ - start_xy;
    const Eigen::Vector2d step = candidate - start_xy;
    return forward.norm() <= 1e-6 || step.norm() <= 1e-6 ||
      step.dot(forward) >= 0.10 * step.norm() * forward.norm();
  };
  const Eigen::Vector2d goal_dir =
    (Eigen::Vector2d(goal_rog.pose.position.x, goal_rog.pose.position.y) - start_xy);
  Eigen::Vector2d goal_dir_norm = Eigen::Vector2d::Zero();
  if (goal_dir.norm() > 1e-6) {
    goal_dir_norm = goal_dir.normalized();
  }

  size_t best_idx = map_size;
  double best_score = std::numeric_limits<double>::infinity();
  const int offsets[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};

  while (!open.empty()) {
    if (cancel_checker && cancel_checker()) {
      return false;
    }
    const QueueNode current = open.top();
    open.pop();
    if (closed[current.idx]) {
      continue;
    }
    closed[current.idx] = 1U;

    const unsigned int cx = static_cast<unsigned int>(current.idx % nx);
    const unsigned int cy = static_cast<unsigned int>(current.idx / nx);
    // Advance through observed free space even when the sensor's free region
    // does not reach the rolling window boundary. Unknown cells are obstacles
    // in the online profile, including in the distance field.
    double wx = 0.0, wy = 0.0;
    query->mapToWorld(cx, cy, wx, wy);
    const Eigen::Vector2d candidate(wx, wy);
    const Eigen::Vector2d goal_xy(goal_rog.pose.position.x, goal_rog.pose.position.y);
    const double remaining = (candidate - goal_xy).norm();
    double score = remaining + 0.15 * current.cost;
    if (mode_context.explorationPreferGoalDirection() && goal_dir_norm.norm() > 1e-6) {
      const auto direction = candidate - start_xy;
      if (direction.norm() > 1e-6) score += 0.2 * (1.0 - direction.normalized().dot(goal_dir_norm));
    }
    const double minimum_advance = std::max(0.5, 2.0 * tolerance_);
    if (!recently_completed(candidate) && holds_history_direction(candidate) &&
      (candidate - start_xy).norm() >= minimum_advance &&
      remaining < goal_dir.norm() - 0.1 && score < best_score) {
      const auto clearance = query->query({wx, wy, 0.0});
      if (clearance.ok && std::isfinite(clearance.distance) &&
        clearance.distance >= online_subgoal_clearance_) {
        best_score = score;
        best_idx = current.idx;
      }
    }

    for (const auto & offset : offsets) {
      const int nx_i = static_cast<int>(cx) + offset[0];
      const int ny_i = static_cast<int>(cy) + offset[1];
      if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<int>(nx) || ny_i >= static_cast<int>(ny)) {
        continue;
      }
      const unsigned int nbx = static_cast<unsigned int>(nx_i);
      const unsigned int nby = static_cast<unsigned int>(ny_i);
      const size_t nb_idx = index_of(nbx, nby);
      if (closed[nb_idx] || !traversable(nb_idx)) {
        continue;
      }
      if (!online_edge(cx, cy, nbx, nby)) continue;
      const bool diagonal = std::abs(offset[0]) + std::abs(offset[1]) == 2;
      if (diagonal && (!traversable(index_of(nbx, cy)) ||
        !traversable(index_of(cx, nby)))) continue;
      const double step = (diagonal ? 1.41421356237 : 1.0) * resolution;
      const double next_dist = current.cost + step;
      if (next_dist < dist[nb_idx]) {
        dist[nb_idx] = next_dist;
        parent[nb_idx] = static_cast<int>(current.idx);
        open.push(QueueNode{nb_idx, next_dist});
      }
    }
  }

  // The session map remembers corridors after they leave the rolling window.
  // Use a known-free route to the mission goal to choose the next waypoint;
  // only the live search above may authorize the segment sent to the robot.
  // In particular, a short Euclidean step toward a previously seen dead end
  // must not outrank the beginning of a longer corridor around the wall.
  size_t remembered_candidate_idx = map_size;
  if (remembered && use_smac_ && smac_) {
    unsigned int hsx = 0U, hsy = 0U, hgx = 0U, hgy = 0U;
    if (remembered->worldToMap(start_xy.x(), start_xy.y(), hsx, hsy) &&
        remembered->worldToMap(goal_position.x, goal_position.y, hgx, hgy) &&
        remembered->isFree(hsx, hsy) && remembered->isFree(hgx, hgy)) {
      const double clearance = smac_->minimumClearance();
      constexpr double endpoint_shift = 0.6;
      if (projectStartToSearchableCell(remembered, hsx, hsy, clearance, endpoint_shift) &&
          projectStartToSearchableCell(remembered, hgx, hgy, clearance, endpoint_shift)) {
        smac_->setMap(remembered);
        smac_->setESDFQuery(remembered);
        smac_->setEdgeFilter([remembered](unsigned int ax, unsigned int ay,
            unsigned int bx, unsigned int by) {
          double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
          remembered->mapToWorld(ax, ay, x1, y1);
          remembered->mapToWorld(bx, by, x2, y2);
          return remembered->terrainTransition(x1, y1, x2, y2);
        });
        mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector route;
        if (smac_->createPath(hsx, hsy, hgx, hgy, route, cancel_checker)) {
          // SMAC returns goal-to-start. The first live-reachable point is the
          // farthest safe lookahead near the remembered route. The cumulative
          // and live clearance fields can differ, so permit a small lateral
          // offset instead of requiring the same grid center in both maps.
          constexpr double route_corridor = 0.45;
          const int corridor_cells = static_cast<int>(std::ceil(route_corridor / resolution));
          for (const auto & cell : route) {
            double wx = 0.0, wy = 0.0;
            remembered->mapToWorld(static_cast<unsigned int>(cell.x),
              static_cast<unsigned int>(cell.y), wx, wy);
            unsigned int lx = 0U, ly = 0U;
            if (!query->worldToMap(wx, wy, lx, ly)) continue;
            double nearest = std::numeric_limits<double>::infinity();
            size_t route_idx = map_size;
            for (int dy = -corridor_cells; dy <= corridor_cells; ++dy) {
              for (int dx = -corridor_cells; dx <= corridor_cells; ++dx) {
                const int cx = static_cast<int>(lx) + dx;
                const int cy = static_cast<int>(ly) + dy;
                if (cx < 0 || cy < 0 || cx >= static_cast<int>(nx) ||
                    cy >= static_cast<int>(ny)) continue;
                const size_t live_idx = index_of(static_cast<unsigned int>(cx),
                  static_cast<unsigned int>(cy));
                if (!closed[live_idx]) continue;
                double candidate_x = 0.0, candidate_y = 0.0;
                query->mapToWorld(static_cast<unsigned int>(cx),
                  static_cast<unsigned int>(cy), candidate_x, candidate_y);
                const Eigen::Vector2d candidate(candidate_x, candidate_y);
                const double offset = std::hypot(candidate_x - wx, candidate_y - wy);
                if (offset > route_corridor || offset >= nearest ||
                    (candidate - start_xy).norm() < std::max(0.5, 2.0 * tolerance_) ||
                    recently_completed(candidate)) continue;
                const auto field = query->query({candidate_x, candidate_y, 0.0});
                if (!field.ok || !std::isfinite(field.distance) ||
                    field.distance < online_subgoal_clearance_) continue;
                nearest = offset;
                route_idx = live_idx;
              }
            }
            if (route_idx < map_size) {
              best_idx = route_idx;
              remembered_candidate_idx = route_idx;
              break;
            }
          }
        }
      }
    }
  }

  // Preserve the current observed waypoint while it is still connected to
  // the robot in this map snapshot. A better-scoring point on the other side
  // of a wall must not reverse the robot on every rolling-map update.
  if (committed_online_subgoal_) {
    unsigned int cx = 0U, cy = 0U;
    if (query->worldToMap(committed_online_subgoal_->x(),
        committed_online_subgoal_->y(), cx, cy)) {
      const size_t committed_idx = index_of(cx, cy);
      const auto field = query->query({committed_online_subgoal_->x(),
        committed_online_subgoal_->y(), 0.0});
      if (closed[committed_idx] && field.ok && std::isfinite(field.distance) &&
        field.distance >= online_subgoal_clearance_) {
        best_idx = committed_idx;
      } else {
        reject_subgoal(true);
      }
    } else {
      reject_subgoal(true);
    }
  }

  // A previously valid accumulated route can disappear for a few snapshots.
  // The ordinary Euclidean candidate was scored before the commitment was
  // checked, so reject an immediate reversal after a live-map invalidation.
  if (best_idx < map_size && best_idx != remembered_candidate_idx) {
    double candidate_x = 0.0, candidate_y = 0.0;
    query->mapToWorld(static_cast<unsigned int>(best_idx % nx),
      static_cast<unsigned int>(best_idx / nx), candidate_x, candidate_y);
    const Eigen::Vector2d candidate(candidate_x, candidate_y);
    if (recently_completed(candidate) || !holds_history_direction(candidate)) {
      best_idx = map_size;
    }
  }

  if (best_idx >= map_size) {
    // A wall can hide the destination while every safe first step around it
    // temporarily increases the Euclidean goal distance. In that case move
    // toward an observed, reachable standoff from an unknown boundary. The
    // unknown cells guide exploration; neither the route nor its endpoint may
    // enter them. Propagate through known free cells so a frontier on the far
    // side of a wall does not attract a candidate through that wall.
    constexpr double frontier_standoff = 0.85;
    std::vector<double> frontier_dist(map_size, std::numeric_limits<double>::infinity());
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> frontier_open;
    // In the online launch UNKNOWN is deliberately assigned cost 254, the
    // same cost as a wall. Use the projection's cell type to distinguish them.
    const bool have_live_types = semantic && semantic->types.size() == map_size;
    const auto is_unknown = [&](size_t idx) {
      const bool live_unknown = have_live_types ?
        semantic->types[idx] == static_cast<uint8_t>(rog_map::CellType::UNKNOWN) :
        costs[idx] == 255U;
      if (!live_unknown || !remembered) return live_unknown;
      double wx = 0.0, wy = 0.0;
      query->mapToWorld(static_cast<unsigned int>(idx % nx),
        static_cast<unsigned int>(idx / nx), wx, wy);
      unsigned int hx = 0U, hy = 0U;
      return !remembered->worldToMap(wx, wy, hx, hy) ||
        remembered->value(hx, hy) == 255U;
    };
    for (size_t idx = 0; idx < map_size; ++idx) {
      if (!smacTraversableCost(costs[idx], false)) continue;
      const unsigned int x = static_cast<unsigned int>(idx % nx);
      const unsigned int y = static_cast<unsigned int>(idx / nx);
      if (!online_edge(x, y, x, y)) continue;
      for (size_t direction = 0; direction < 4U; ++direction) {
        const auto & offset = offsets[direction];
        const int near_x = static_cast<int>(x) + offset[0];
        const int near_y = static_cast<int>(y) + offset[1];
        if (near_x < 0 || near_y < 0 || near_x >= static_cast<int>(nx) ||
          near_y >= static_cast<int>(ny)) continue;
        if (is_unknown(index_of(static_cast<unsigned int>(near_x),
            static_cast<unsigned int>(near_y)))) {
          frontier_dist[idx] = 0.0;
          frontier_open.push(QueueNode{idx, 0.0});
          break;
        }
      }
    }
    while (!frontier_open.empty()) {
      if (cancel_checker && cancel_checker()) return false;
      const QueueNode current = frontier_open.top();
      frontier_open.pop();
      if (current.cost > frontier_dist[current.idx] ||
        current.cost > frontier_standoff) continue;
      const unsigned int cx = static_cast<unsigned int>(current.idx % nx);
      const unsigned int cy = static_cast<unsigned int>(current.idx / nx);
      for (const auto & offset : offsets) {
        const int next_x = static_cast<int>(cx) + offset[0];
        const int next_y = static_cast<int>(cy) + offset[1];
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(nx) ||
          next_y >= static_cast<int>(ny)) continue;
        const unsigned int x = static_cast<unsigned int>(next_x);
        const unsigned int y = static_cast<unsigned int>(next_y);
        const size_t idx = index_of(x, y);
        if (!smacTraversableCost(costs[idx], false) ||
          !online_edge(cx, cy, x, y)) continue;
        const bool diagonal = std::abs(offset[0]) + std::abs(offset[1]) == 2;
        if (diagonal && (!smacTraversableCost(costs[index_of(x, cy)], false) ||
          !smacTraversableCost(costs[index_of(cx, y)], false))) continue;
        const double next_dist = current.cost +
          (diagonal ? 1.41421356237 : 1.0) * resolution;
        if (next_dist < frontier_dist[idx] && next_dist <= frontier_standoff) {
          frontier_dist[idx] = next_dist;
          frontier_open.push(QueueNode{idx, next_dist});
        }
      }
    }

    double best_frontier_score = std::numeric_limits<double>::infinity();
    const double minimum_advance = std::max(0.5, 2.0 * tolerance_);
    for (size_t idx = 0; idx < map_size; ++idx) {
      if (!closed[idx] || frontier_dist[idx] > frontier_standoff) continue;
      double wx = 0.0, wy = 0.0;
      query->mapToWorld(static_cast<unsigned int>(idx % nx),
        static_cast<unsigned int>(idx / nx), wx, wy);
      const Eigen::Vector2d candidate(wx, wy);
      if ((candidate - start_xy).norm() < minimum_advance) continue;
      if (recently_completed(candidate)) continue;
      if (!holds_history_direction(candidate)) continue;
      const auto clearance = query->query({wx, wy, 0.0});
      if (!clearance.ok || !std::isfinite(clearance.distance) ||
        clearance.distance < online_subgoal_clearance_) continue;
      const double score = frontier_dist[idx] +
        0.25 * (candidate - Eigen::Vector2d(
          goal_rog.pose.position.x, goal_rog.pose.position.y)).norm() +
        0.15 * dist[idx];
      if (score < best_frontier_score) {
        best_frontier_score = score;
        best_idx = idx;
      }
    }
    if (best_idx >= map_size) {
      if (suspended_history_subgoal_) {
        RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
          "[MincoPlanner] Accumulated-route direction hold has no live-safe forward "
          "subgoal; stopping this refresh rather than reversing toward a stale frontier.");
      } else {
        RCLCPP_ERROR(logger_, "[MincoPlanner] Online search has no reachable observed subgoal "
          "with safe goal progress or an explorable frontier "
          "(route_clearance=%.3f subgoal_clearance=%.3f).",
          route_clearance, online_subgoal_clearance_);
      }
      return false;
    }
    double frontier_x = 0.0, frontier_y = 0.0;
    query->mapToWorld(static_cast<unsigned int>(best_idx % nx),
      static_cast<unsigned int>(best_idx / nx), frontier_x, frontier_y);
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Goal progress is blocked; advancing to observed frontier "
      "standoff (%.2f, %.2f), %.2f m from unknown, to reveal a route.",
      frontier_x, frontier_y, frontier_dist[best_idx]);
  }

  std::vector<size_t> reversed;
  for (size_t idx = best_idx;;) {
    reversed.push_back(idx);
    if (idx == updated_start_idx) {
      break;
    }
    const int p = parent[idx];
    if (p < 0) {
      RCLCPP_ERROR(logger_, "[MincoPlanner] Online subgoal path reconstruction failed.");
      return false;
    }
    idx = static_cast<size_t>(p);
  }

  latest_global_path.clear();
  latest_global_path.reserve(reversed.size());
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    const unsigned int x = static_cast<unsigned int>(*it % nx);
    const unsigned int y = static_cast<unsigned int>(*it / nx);
    double wx = 0.0;
    double wy = 0.0;
    query->mapToWorld(x, y, wx, wy);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = mode_context.outputFrame();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    latest_global_path.push_back(pose);
  }
  if (latest_global_path.size() < 2U) return false;
  double subgoal_x = 0.0, subgoal_y = 0.0;
  query->mapToWorld(static_cast<unsigned int>(best_idx % nx),
    static_cast<unsigned int>(best_idx / nx), subgoal_x, subgoal_y);
  const Eigen::Vector2d selected(subgoal_x, subgoal_y);
  const bool new_subgoal = !committed_online_subgoal_ ||
    (selected - *committed_online_subgoal_).norm() > query->resolution();
  if (new_subgoal) {
    RCLCPP_INFO(logger_, "[MincoPlanner] Committing observed exploration subgoal "
      "(%.2f, %.2f) until reached or blocked%s.", subgoal_x, subgoal_y,
      best_idx == remembered_candidate_idx ? " along the accumulated-map route" : "");
    committed_online_best_distance_ = (selected - start_xy).norm();
    committed_online_progress_stamp_ = query->snapshotStampSeconds();
  }
  if (new_subgoal) {
    committed_online_from_history_ = best_idx == remembered_candidate_idx;
  }
  if (best_idx == remembered_candidate_idx) suspended_history_subgoal_.reset();
  committed_online_subgoal_ = selected;
  return true;
}

bool GlobalPathSearcher::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const PlannerModeContext & mode_context,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  std::lock_guard<std::mutex> lock(search_mutex_);
  std::vector<geometry_msgs::msg::PoseStamped> ignored_path;
  return makePlanOnQuery(start,
    goal,
    mode_context.globalQuery(),
    mode_context.outputFrame(),
    "ROGMap",
    tolerance,
    cancel_checker,
    plan,
    ignored_path);
}

bool GlobalPathSearcher::makePlanOnQuery(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & output_frame,
  const std::string & failure_source,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
  mas2027_nav_executor::smac::SmacPlanner2DSimple::EdgeFilter edge_filter,
  bool relaxed_search)
{
  if (relaxed_search) {
    RCLCPP_WARN(logger_,
      "[MincoPlanner] %s primary global search failed; retrying with relaxed "
      "minimum clearance %.3f m. Local trajectory gates remain unchanged.",
      failure_source.c_str(),
      smac_ ? smac_->minimumClearance() : 0.0);
  } else {
    last_goal_shifted_ = false;
  }
  bool goal_shifted = false;
  plan.poses.clear();
  plan.header.stamp = rclcpp::Clock().now();
  plan.header.frame_id = output_frame;
  if (!query) {
    RCLCPP_ERROR(
      logger_, "[MincoPlanner] %s query is not available for global planning.", failure_source.c_str());
    return false;
  }

  double wx = start.position.x;
  double wy = start.position.y;
  unsigned int mx_start = 0;
  unsigned int my_start = 0;
  if (!query->worldToMap(wx, wy, mx_start, my_start)) {
    RCLCPP_ERROR(logger_,
      "%s worldToMap failed for start world coordinates (%.2f, %.2f). "
      "map size=%ux%u origin=(%.3f, %.3f) resolution=%.3f frame=%s",
      failure_source.c_str(),
      wx,
      wy,
      query->sizeX(),
      query->sizeY(),
      query->originX(),
      query->originY(),
      query->resolution(),
      output_frame.c_str());
    return false;
  }

  const unsigned int raw_mx_start = mx_start;
  const unsigned int raw_my_start = my_start;
  if (!projectStartToFreeCell(query, mx_start, my_start)) {
    RCLCPP_ERROR(logger_,
      "%s failed to project start cell to a traversable free cell near world coordinates (%.2f, %.2f)",
      failure_source.c_str(),
      wx,
      wy);
    logCellDiagnostics(logger_, query, failure_source, "start(raw)", raw_mx_start, raw_my_start, allow_unknown_);
    return false;
  }

  // The cost-projected start can still be inside SMAC's ESDF exclusion band.
  // Move it to a nearby searchable cell before invoking SMAC; otherwise the
  // search logs a misleading "open set exhausted" with iterations=1.
  const double start_clearance_required =
    (use_smac_ && smac_) ? static_cast<double>(smac_->minimumClearance()) : 0.0;
  constexpr double kStartSearchShiftMax = 0.6;  // metres; local planner keeps the real pose
  const unsigned int cost_projected_mx_start = mx_start;
  const unsigned int cost_projected_my_start = my_start;
  if (start_clearance_required > 0.0 &&
      !projectStartToSearchableCell(
        query, mx_start, my_start, start_clearance_required, kStartSearchShiftMax)) {
    RCLCPP_WARN(logger_,
      "%s start cell=(%u,%u) has insufficient search clearance at %.3f m and no "
      "searchable cell exists within %.2f m; leaving the start to the local escape path.",
      failure_source.c_str(),
      mx_start,
      my_start,
      start_clearance_required,
      kStartSearchShiftMax);
    mx_start = cost_projected_mx_start;
    my_start = cost_projected_my_start;
  } else if (mx_start != cost_projected_mx_start || my_start != cost_projected_my_start) {
    double raw_x = 0.0;
    double raw_y = 0.0;
    double used_x = 0.0;
    double used_y = 0.0;
    query->mapToWorld(cost_projected_mx_start, cost_projected_my_start, raw_x, raw_y);
    query->mapToWorld(mx_start, my_start, used_x, used_y);
    RCLCPP_WARN(logger_,
      "%s start cell=(%u,%u) is below search clearance %.3f m; using nearby "
      "start cell=(%u,%u) shifted %.3f m.",
      failure_source.c_str(),
      cost_projected_mx_start,
      cost_projected_my_start,
      start_clearance_required,
      mx_start,
      my_start,
      std::hypot(used_x - raw_x, used_y - raw_y));
  }

  wx = goal.position.x;
  wy = goal.position.y;
  unsigned int mx_goal = 0;
  unsigned int my_goal = 0;
  if (!query->worldToMap(wx, wy, mx_goal, my_goal)) {
    RCLCPP_ERROR(logger_,
      "%s worldToMap failed for goal world coordinates (%.2f, %.2f). "
      "map size=%ux%u origin=(%.3f, %.3f) resolution=%.3f frame=%s",
      failure_source.c_str(),
      wx,
      wy,
      query->sizeX(),
      query->sizeY(),
      query->originX(),
      query->originY(),
      query->resolution(),
      output_frame.c_str());
    return false;
  }
  // If the requested goal cell is blocked or has insufficient search clearance, keep
  // the same search/validation chain but move the endpoint to a nearby cell that the
  // search can actually accept. First retreat along the start->goal line; if the
  // whole line is blocked, search a ring around the goal on the robot side. This is
  // still an approach endpoint, not a relaxed safety gate.
  geometry_msgs::msg::Pose search_goal = goal;
  const double goal_clearance_required = (use_smac_ && smac_) ? smac_->minimumClearance() : 0.0;
  if (goal_clearance_required > 0.0) {
    const unsigned char requested_goal_cost = query->value(mx_goal, my_goal);
    // SMAC samples ESDF at the goal cell center. Use that same position and
    // float comparison here; querying the original pose can approve a cell
    // that SMAC immediately rejects, especially near an ESDF grid boundary.
    double goal_cell_x = 0.0;
    double goal_cell_y = 0.0;
    query->mapToWorld(mx_goal, my_goal, goal_cell_x, goal_cell_y);
    const auto requested_goal_field = query->query({goal_cell_x, goal_cell_y, 0.0});
    const bool requested_goal_ok = smacTraversableCost(requested_goal_cost, allow_unknown_) &&
      requested_goal_field.ok && std::isfinite(requested_goal_field.distance) &&
      static_cast<float>(requested_goal_field.distance) >=
        static_cast<float>(goal_clearance_required);
    if (!requested_goal_ok) {
      geometry_msgs::msg::Pose line_approach;
      double line_shorten = std::numeric_limits<double>::infinity();
      const bool line_ok = findGoalApproach(
        query, start, goal, goal_clearance_required, allow_unknown_,
        goal_approach_max_shorten_, line_approach, line_shorten);
      geometry_msgs::msg::Pose radial_approach;
      double radial_shorten = std::numeric_limits<double>::infinity();
      bool radial_used = false;
      const bool radial_ok = findGoalEjection(
        query, start, goal, goal_clearance_required, allow_unknown_,
        goal_approach_max_shorten_, radial_approach, radial_shorten, radial_used);
      // Prefer the closer safe endpoint. The line approach preserves the
      // requested direction; radial ejection is the fallback for goals inside
      // a wall or behind a blocked strip.
      const bool use_radial = radial_used && radial_ok &&
        (!line_ok || radial_shorten < line_shorten);
      const bool have_approach = use_radial || line_ok;
      if (have_approach) {
        const geometry_msgs::msg::Pose & approach =
          use_radial ? radial_approach : line_approach;
        const double shorten_distance = use_radial ? radial_shorten : line_shorten;
        unsigned int ax = 0U;
        unsigned int ay = 0U;
        if (query->worldToMap(
            approach.position.x, approach.position.y, ax, ay)) {
          search_goal = approach;
          mx_goal = ax;
          my_goal = ay;
          goal_shifted = true;
          RCLCPP_WARN(logger_,
            "[MincoPlanner] %s requested goal (%.2f,%.2f) cost=%u(%s) clearance=%.3f "
            "is not traversable at %.3f m; using %s point (%.2f,%.2f), %.2f m from goal.",
            failure_source.c_str(),
            goal.position.x,
            goal.position.y,
            static_cast<unsigned int>(requested_goal_cost),
            cellCostLabel(requested_goal_cost),
            requested_goal_field.ok ? requested_goal_field.distance :
              std::numeric_limits<double>::quiet_NaN(),
            goal_clearance_required,
            use_radial ? "ejected" : "approach",
            search_goal.position.x,
            search_goal.position.y,
            shorten_distance);
        }
      }
    }
  }

  unsigned int nx = query->sizeX();
  unsigned int ny = query->sizeY();
  const unsigned char start_cost = query->value(mx_start, my_start);
  const unsigned char goal_cost = query->value(mx_goal, my_goal);
  RCLCPP_INFO(logger_,
    "[MincoPlanner] %s global search input: planner=%s frame=%s map=%ux%u origin=(%.3f,%.3f) "
    "res=%.3f tolerance=%.3f allow_unknown=%s start_world=(%.3f,%.3f) start_cell=(%u,%u)->(%u,%u) "
    "start_cost=%u(%s) goal_world=(%.3f,%.3f) goal_cell=(%u,%u) goal_cost=%u(%s)",
    failure_source.c_str(),
    (use_smac_ && smac_) ? "SMAC2D" : "Astar",
    output_frame.c_str(),
    nx,
    ny,
    query->originX(),
    query->originY(),
    query->resolution(),
    tolerance,
    allow_unknown_ ? "true" : "false",
    start.position.x,
    start.position.y,
    raw_mx_start,
    raw_my_start,
    mx_start,
    my_start,
    static_cast<unsigned int>(start_cost),
    cellCostLabel(start_cost),
    search_goal.position.x,
    search_goal.position.y,
    mx_goal,
    my_goal,
    static_cast<unsigned int>(goal_cost),
    cellCostLabel(goal_cost));

  if (use_smac_ && smac_) {
    // SMAC 2D 主搜索：query() 即 planningConstraints() 烘焙的二维有符号距离场（ESDF），
    // 量纲与旧工程 ROGMap ESDF 一致，smac_2d.use_esdf_cost 势场偏置可复用；两者同系，无需换算层。
    smac_->setMap(query);
    smac_->setESDFQuery(query);
    smac_->setEdgeFilter(edge_filter);

    mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector smac_path;
    const bool smac_success =
      smac_->createPath(mx_start, my_start, mx_goal, my_goal, smac_path, cancel_checker);

    if (!smac_success || smac_path.size() < 2) {
      RCLCPP_ERROR(logger_,
        "%s SMAC 2D failed to find path (success=%s, path_size=%zu). See endpoint diagnostics below.",
        failure_source.c_str(),
        smac_success ? "true" : "false",
        smac_path.size());
      if (raw_mx_start != mx_start || raw_my_start != my_start) {
        logCellDiagnostics(
          logger_, query, failure_source, "start(raw-before-projection)", raw_mx_start,
          raw_my_start, allow_unknown_, smac_->minimumClearance());
      }
      logCellDiagnostics(logger_, query, failure_source, "start(used)", mx_start, my_start,
        allow_unknown_, smac_->minimumClearance());
      logCellDiagnostics(logger_, query, failure_source, "goal", mx_goal, my_goal,
        allow_unknown_, smac_->minimumClearance());
      const double primary_clearance = smac_->minimumClearance();
      const double relaxed_clearance = relaxed_minimum_clearance_;
      if (!relaxed_search && relaxed_clearance > 0.0 &&
          relaxed_clearance + 1.0e-9 < primary_clearance) {
        smac_->setMinimumClearance(relaxed_clearance);
        const bool relaxed_ok = makePlanOnQuery(start, goal, query, output_frame,
          failure_source, tolerance, cancel_checker, plan, latest_global_path,
          edge_filter, true);
        smac_->setMinimumClearance(primary_clearance);
        if (relaxed_ok) {
          RCLCPP_WARN(logger_,
            "[MincoPlanner] %s relaxed global search succeeded at %.3f m "
            "(primary %.3f m); the local narrow-corridor gate must still accept the path.",
            failure_source.c_str(), relaxed_clearance, primary_clearance);
          return true;
        }
        RCLCPP_ERROR(logger_,
          "[MincoPlanner] %s relaxed global search also failed at %.3f m "
          "(primary %.3f m).",
          failure_source.c_str(), relaxed_clearance, primary_clearance);
      }
      return false;
    }

    latest_global_path.clear();
    latest_global_path.reserve(smac_path.size());
    plan.poses.reserve(smac_path.size());

    // SMAC 输出是 goal -> start，调用方要求 start -> goal。
    for (auto it = smac_path.rbegin(); it != smac_path.rend(); ++it) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double path_wx = 0.0;
      double path_wy = 0.0;
      query->mapToWorld(
        static_cast<unsigned int>(it->x), static_cast<unsigned int>(it->y), path_wx, path_wy);

      pose.pose.position.x = path_wx;
      pose.pose.position.y = path_wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path.push_back(pose);
      plan.poses.push_back(pose);
    }
  } else {
    if (!astar_) {
      return false;
    }
    astar_->setSize(nx, ny);
    astar_->setStart(static_cast<int>(mx_start), static_cast<int>(my_start));
    astar_->setGoal(static_cast<int>(mx_goal), static_cast<int>(my_goal));
    astar_->setupNavFn(true);
    std::vector<unsigned char> query_costmap_copy;
    const size_t map_size = static_cast<size_t>(nx) * static_cast<size_t>(ny);
    if (!query->copyValues(query_costmap_copy) || query_costmap_copy.size() != map_size) {
      RCLCPP_ERROR(logger_,
        "%s Astar failed to copy map values: copied=%zu expected=%zu",
        failure_source.c_str(),
        query_costmap_copy.size(),
        map_size);
      return false;
    }
    astar_->setCostmap(query_costmap_copy.data(), true, allow_unknown_);

    // lab3 is 770x347=267190 cells; the old `int(nx*ny)*9999` overflowed INT_MAX
    // (~2.15e9) to a negative budget, so the while loop never ran and even a 1.2 m
    // free-space goal failed with "Astar failed to find path" in ~1 ms.
    const int64_t map_cells = static_cast<int64_t>(nx) * static_cast<int64_t>(ny);
    int64_t max_total_cycles = map_cells;
    const int cycles_per_step = static_cast<int>(
      std::max(map_cells / 20, static_cast<int64_t>(nx) + static_cast<int64_t>(ny)));
    while (max_total_cycles > 0) {
      if (cancel_checker && cancel_checker()) {
        return false;
      }
      if (!astar_->propNavFnAstar(cycles_per_step, cancel_checker)) {
        break;
      }
      max_total_cycles -= cycles_per_step;
    }

    if (!astar_->calcPath(nx * ny / 2) || astar_->getPathLen() < 2) {
      RCLCPP_ERROR(logger_, "%s Astar failed to find path. See endpoint diagnostics below.", failure_source.c_str());
      if (raw_mx_start != mx_start || raw_my_start != my_start) {
        logCellDiagnostics(
          logger_, query, failure_source, "start(raw-before-projection)", raw_mx_start,
          raw_my_start, allow_unknown_);
      }
      logCellDiagnostics(logger_, query, failure_source, "start(used)", mx_start, my_start,
        allow_unknown_);
      logCellDiagnostics(logger_, query, failure_source, "goal", mx_goal, my_goal,
        allow_unknown_);
      return false;
    }

    float * path_x = astar_->getPathX();
    float * path_y = astar_->getPathY();
    const int len = astar_->getPathLen();

    latest_global_path.clear();
    latest_global_path.reserve(len);
    plan.poses.reserve(len);

    for (int i = 0; i < len; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = plan.header;

      double path_wx = 0.0;
      double path_wy = 0.0;
      query->mapToWorld(
        static_cast<unsigned int>(path_x[i]), static_cast<unsigned int>(path_y[i]), path_wx, path_wy);

      pose.pose.position.x = path_wx;
      pose.pose.position.y = path_wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      latest_global_path.push_back(pose);
      plan.poses.push_back(pose);
    }
  }

  // The legacy Astar path cannot filter graph edges during expansion. Keep its
  // result fail-closed if it crossed a restricted slope boundary.
  if (edge_filter) {
    for (size_t i = 1; i < latest_global_path.size(); ++i) {
      const auto & a = latest_global_path[i - 1].pose.position;
      const auto & b = latest_global_path[i].pose.position;
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      const int steps = std::max(1, static_cast<int>(std::ceil(
        length / (0.5 * query->resolution()))));
      unsigned int previous_x = 0, previous_y = 0;
      if (!query->worldToMap(a.x, a.y, previous_x, previous_y)) return false;
      for (int step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / steps;
        unsigned int x = 0, y = 0;
        if (!query->worldToMap(a.x + ratio * (b.x - a.x),
            a.y + ratio * (b.y - a.y), x, y)) return false;
        if ((x != previous_x || y != previous_y) &&
            !edge_filter(previous_x, previous_y, x, y)) return false;
        previous_x = x; previous_y = y;
      }
    }
  }

  if (!plan.poses.empty()) {
    plan.poses.back().pose.position.x = search_goal.position.x;
    plan.poses.back().pose.position.y = search_goal.position.y;
    plan.poses.back().pose.position.z = search_goal.position.z;
    plan.poses.back().pose.orientation = search_goal.orientation;
    latest_global_path.back().pose.position.x = search_goal.position.x;
    latest_global_path.back().pose.position.y = search_goal.position.y;
    latest_global_path.back().pose.position.z = search_goal.position.z;
    latest_global_path.back().pose.orientation = search_goal.orientation;
  }

  const bool success = !plan.poses.empty();
  if (success) {
    last_goal_shifted_ = goal_shifted;
  }
  return success;
}

}  // namespace minco_planner
