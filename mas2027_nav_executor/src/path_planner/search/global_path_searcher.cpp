#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"

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

void logCellDiagnostics(const rclcpp::Logger & logger,
  const std::shared_ptr<rog_map::MapQueryInterface> & query,
  const std::string & source,
  const std::string & label,
  unsigned int mx,
  unsigned int my,
  bool allow_unknown)
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

  int valid = 0;
  int unknown = 0;
  int free = 0;
  int blocked = 0;
  int smac_traversable = 0;
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
        ++smac_traversable;
      }
    }
  }

  RCLCPP_WARN(logger,
    "[MincoPlanner] %s %s diag: cell=(%u,%u) world=(%.3f,%.3f) cost=%u(%s) query_free=%s "
    "smac_traversable=%s allow_unknown=%s 5x5(valid=%d invalid=%d free=%d unknown=%d blocked=%d "
    "smac_traversable=%d)",
    source.c_str(),
    label.c_str(),
    mx,
    my,
    wx,
    wy,
    static_cast<unsigned int>(cost),
    cellCostLabel(cost),
    query->isFree(mx, my) ? "true" : "false",
    smacTraversableCost(cost, allow_unknown) ? "true" : "false",
    allow_unknown ? "true" : "false",
    valid,
    invalid,
    free,
    unknown,
    blocked,
    smac_traversable);
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

bool GlobalPathSearcher::plan(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
  return planExploration(start, goal, mode_context, latest_global_path);
}

bool GlobalPathSearcher::planExploration(const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const PlannerModeContext & mode_context,
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
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
      if (!makePlanOnQuery(start_map.pose, goal_map.pose, terrain_query,
            mode_context.outputFrame(), "Terrain", tolerance_, cancel_checker, terrain_plan,
            latest_global_path)) {
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
  bool goal_traversable = false;
  if (goal_inside) {
    const unsigned char goal_cost = query->value(gx, gy);
    goal_traversable = goal_cost == kUnknownCost
                         ? !mode_context.explorationUnknownAsOccupied()
                         : goal_cost < kInscribedCost;
  }
  if (goal_traversable) {
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
          latest_global_path)) {
      return true;
    }
    RCLCPP_WARN(logger_,
      "[MincoPlanner] Exploration goal is inside ROGMap and traversable but unreachable; "
      "fallback to a reachable observed subgoal.");
  }

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
  const auto traversable = [&costs, allow_unknown](size_t idx) {
    const unsigned char cost = costs[idx];
    if (cost == kUnknownCost) {
      return allow_unknown;
    }
    return cost < kInscribedCost;
  };

  const size_t start_idx = index_of(sx, sy);
  if (!traversable(start_idx)) {
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
  if (!traversable(updated_start_idx)) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] EXPLORATION start cell is not traversable in ROGMap.");
    return false;
  }
  dist[updated_start_idx] = 0.0;
  open.push(QueueNode{updated_start_idx, 0.0});

  const Eigen::Vector2d start_xy(start_rog.pose.position.x, start_rog.pose.position.y);
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
    if ((candidate - start_xy).norm() >= minimum_advance &&
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

  if (best_idx >= map_size) {
    RCLCPP_ERROR(logger_, "[MincoPlanner] Online search has no reachable observed subgoal with goal progress and clearance.");
    return false;
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
  return latest_global_path.size() >= 2U;
}

bool GlobalPathSearcher::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  const PlannerModeContext & mode_context,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
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
  std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path)
{
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
    goal.position.x,
    goal.position.y,
    mx_goal,
    my_goal,
    static_cast<unsigned int>(goal_cost),
    cellCostLabel(goal_cost));

  if (use_smac_ && smac_) {
    // SMAC 2D 主搜索：query() 即 planningConstraints() 烘焙的二维有符号距离场（ESDF），
    // 量纲与旧工程 ROGMap ESDF 一致，smac_2d.use_esdf_cost 势场偏置可复用；两者同系，无需换算层。
    smac_->setMap(query);
    smac_->setESDFQuery(query);

    mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector smac_path;
    const bool smac_success =
      smac_->createPath(mx_start, my_start, mx_goal, my_goal, smac_path, cancel_checker);

    if (!smac_success || smac_path.size() < 2) {
      // SMAC 失败即失败，不退回 Astar：否则一次失败会产生两条搜索路径与两套日志，无法判断来源。
      RCLCPP_ERROR(logger_,
        "%s SMAC 2D failed to find path (success=%s, path_size=%zu). See endpoint diagnostics below.",
        failure_source.c_str(),
        smac_success ? "true" : "false",
        smac_path.size());
      if (raw_mx_start != mx_start || raw_my_start != my_start) {
        logCellDiagnostics(
          logger_, query, failure_source, "start(raw-before-projection)", raw_mx_start, raw_my_start, allow_unknown_);
      }
      logCellDiagnostics(logger_, query, failure_source, "start(used)", mx_start, my_start, allow_unknown_);
      logCellDiagnostics(logger_, query, failure_source, "goal", mx_goal, my_goal, allow_unknown_);
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
          logger_, query, failure_source, "start(raw-before-projection)", raw_mx_start, raw_my_start, allow_unknown_);
      }
      logCellDiagnostics(logger_, query, failure_source, "start(used)", mx_start, my_start, allow_unknown_);
      logCellDiagnostics(logger_, query, failure_source, "goal", mx_goal, my_goal, allow_unknown_);
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

  if (!plan.poses.empty()) {
    plan.poses.back().pose.position.x = goal.position.x;
    plan.poses.back().pose.position.y = goal.position.y;
    plan.poses.back().pose.position.z = goal.position.z;
    plan.poses.back().pose.orientation = goal.orientation;
    latest_global_path.back().pose.position.x = goal.position.x;
    latest_global_path.back().pose.position.y = goal.position.y;
    latest_global_path.back().pose.position.z = goal.position.z;
    latest_global_path.back().pose.orientation = goal.orientation;
  }

  return !plan.poses.empty();
}

}  // namespace minco_planner
