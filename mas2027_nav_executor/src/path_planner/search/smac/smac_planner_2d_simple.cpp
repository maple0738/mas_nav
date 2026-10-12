// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>

#include <Eigen/Core>

namespace mas2027_nav_executor {
namespace smac {

SmacPlanner2DSimple::SmacPlanner2DSimple()
: allow_unknown_(true), max_iterations_(5000000), tolerance_(0.125), size_x_(0), size_y_(0),
  motion_model_(MotionModel::TWOD)
{
  search_info_.cost_penalty = 2.0;
}

SmacPlanner2DSimple::~SmacPlanner2DSimple()
{
}

void SmacPlanner2DSimple::configure(rclcpp::Logger logger)
{
  logger_ = logger;
}

void SmacPlanner2DSimple::setESDFParameters(
  bool enable, double weight, double decay, double max_cost)
{
  use_esdf_cost_ = enable;
  esdf_weight_ = static_cast<float>(weight);
  esdf_decay_ = static_cast<float>(decay);
  esdf_max_cost_ = static_cast<float>(max_cost);

  if (esdf_decay_ <= 1e-3f) {
    esdf_decay_ = 1e-3f;
  }
  invalidateESDFCache();
}

void SmacPlanner2DSimple::setMinimumClearance(double clearance)
{
  minimum_clearance_ = static_cast<float>(std::max(0.0, clearance));
  invalidateESDFCache();
}

void SmacPlanner2DSimple::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  map_ = map;
  if (!map_) {
    return;
  }
  costmap_origin_x_ = map_->originX();
  costmap_origin_y_ = map_->originY();
  costmap_resolution_ = map_->resolution();
  size_x_ = map_->sizeX();
  size_y_ = map_->sizeY();
  ensureSearchBuffers();
  // 注意：这里**不能**把 planning_id_ 归零：visited_/closed_/parent_ 复用上次搜索的标记与父指针，
  // 归零后 ++ 又得到 1、与更早搜索撞号，失败判定失效、静默回退陈旧路径；地图变化无需清空 closed_。
}

void SmacPlanner2DSimple::setESDFQuery(
  const std::shared_ptr<rog_map::MapQueryInterface> & query)
{
  if (esdf_query_ == query) {
    return;
  }
  esdf_query_ = query;
  invalidateESDFCache();
}

void SmacPlanner2DSimple::invalidateESDFCache()
{
  std::fill(esdf_cost_cache_id_.begin(), esdf_cost_cache_id_.end(), 0u);
}

void SmacPlanner2DSimple::setParameters(bool allow_unknown, int max_iterations, float tolerance)
{
  allow_unknown_ = allow_unknown;
  max_iterations_ = max_iterations;
  tolerance_ = tolerance;
}

void SmacPlanner2DSimple::ensureSearchBuffers()
{
  grid_size_ = static_cast<uint64_t>(size_x_) * static_cast<uint64_t>(size_y_);
  const size_t size = static_cast<size_t>(grid_size_);
  if (size == 0u) {
    return;
  }

  if (g_score_.size() != size) {
    g_score_.assign(size, std::numeric_limits<float>::infinity());
    parent_.assign(size, -1);
    visited_.assign(size, 0u);
    closed_.assign(size, 0u);

    esdf_cost_cache_.assign(size, 0.0f);
    esdf_distance_cache_.assign(size, -std::numeric_limits<float>::infinity());
    esdf_cost_cache_id_.assign(size, 0u);
  }
}

void SmacPlanner2DSimple::logFailure(const std::string & reason,
  unsigned int start_x,
  unsigned int start_y,
  unsigned int goal_x,
  unsigned int goal_y,
  int iterations) const
{
  RCLCPP_WARN(logger_,
    "[SMAC 2D] %s | start=(%u,%u) goal=(%u,%u) size=%ux%u origin=(%.3f,%.3f) res=%.3f "
    "allow_unknown=%s tolerance=%.3f minimum_clearance=%.3f max_iterations=%d iterations=%d",
    reason.c_str(),
    start_x,
    start_y,
    goal_x,
    goal_y,
    size_x_,
    size_y_,
    costmap_origin_x_,
    costmap_origin_y_,
    costmap_resolution_,
    allow_unknown_ ? "true" : "false",
    tolerance_,
    minimum_clearance_,
    max_iterations_,
    iterations);
}

float SmacPlanner2DSimple::getESDFPotentialCost(unsigned int mx, unsigned int my)
{
  if (!use_esdf_cost_ || !map_ || !esdf_query_) {
    return 0.0f;
  }

  const float distance = getESDFDistance(mx, my);
  if (!std::isfinite(distance)) {
    return 0.0f;
  }
  const size_t idx = static_cast<size_t>(my) * size_x_ + mx;
  return esdf_cost_cache_[idx];
}

float SmacPlanner2DSimple::getESDFDistance(unsigned int mx, unsigned int my)
{
  if (!map_ || !esdf_query_) {
    return -std::numeric_limits<float>::infinity();
  }

  if (mx >= size_x_ || my >= size_y_) {
    return -std::numeric_limits<float>::infinity();
  }

  const uint64_t index =
    static_cast<uint64_t>(my) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(mx);

  const size_t idx = static_cast<size_t>(index);
  if (esdf_cost_cache_id_[idx] == planning_id_) {
    return esdf_distance_cache_[idx];
  }

  // Convert through the prior search map; the ESDF query handles any further frame conversion.
  double wx = 0.0;
  double wy = 0.0;
  map_->mapToWorld(mx, my, wx, wy);

  const auto result = esdf_query_->query(Eigen::Vector3d(wx, wy, 0.0));
  if (!result.ok || !std::isfinite(result.distance)) {
    esdf_cost_cache_[idx] = 0.0f;
    esdf_distance_cache_[idx] = -std::numeric_limits<float>::infinity();
    esdf_cost_cache_id_[idx] = planning_id_;
    return esdf_distance_cache_[idx];
  }
  esdf_distance_cache_[idx] = static_cast<float>(result.distance);
  const double dist = std::max(0.0, result.distance);
  const float potential = esdf_weight_ *
    static_cast<float>(std::exp(-dist / static_cast<double>(esdf_decay_)));
  esdf_cost_cache_[idx] = esdf_max_cost_ > 0.0f ? std::min(potential, esdf_max_cost_) : potential;
  esdf_cost_cache_id_[idx] = planning_id_;
  return esdf_distance_cache_[idx];
}

float SmacPlanner2DSimple::evaluateInflationCost(unsigned char cell_cost)
{
  if (cell_cost == UNKNOWN_COST) {
    return 1.0f;
  }

  const float normalized_cost =
    static_cast<float>(cell_cost) / static_cast<float>(MAX_NON_OBSTACLE_COST);

  if (cell_cost >= INSCRIBED_COST) {
    return 50.0f;
  }

  if (cell_cost > 128U) {
    return 1.0f + 20.0f * (normalized_cost * normalized_cost * normalized_cost);
  }

  return 1.0f + search_info_.cost_penalty * (normalized_cost * normalized_cost);
}

bool SmacPlanner2DSimple::createPath(const unsigned int & start_x,
  const unsigned int & start_y,
  const unsigned int & goal_x,
  const unsigned int & goal_y,
  CoordinateVector & path,
  std::function<bool()> cancel_checker)
{
  path.clear();

  if (!map_) {
    logFailure("map query is null", start_x, start_y, goal_x, goal_y);
    return false;
  }

  // Refresh cached metadata (avoid per-cell getters in the search loop).
  costmap_origin_x_ = map_->originX();
  costmap_origin_y_ = map_->originY();
  costmap_resolution_ = map_->resolution();
  size_x_ = map_->sizeX();
  size_y_ = map_->sizeY();
  ensureSearchBuffers();

  if (size_x_ == 0u || size_y_ == 0u) {
    logFailure("map query has zero size", start_x, start_y, goal_x, goal_y);
    return false;
  }

  if (start_x >= size_x_ || start_y >= size_y_ || goal_x >= size_x_ || goal_y >= size_y_) {
    logFailure("start or goal is outside map bounds", start_x, start_y, goal_x, goal_y);
    return false;
  }

  ++planning_id_;
  if (planning_id_ == 0u) {
    std::fill(visited_.begin(), visited_.end(), 0u);
    std::fill(closed_.begin(), closed_.end(), 0u);
    std::fill(esdf_cost_cache_id_.begin(), esdf_cost_cache_id_.end(), 0u);
    planning_id_ = 1u;
  }

  const uint64_t start_index =
    static_cast<uint64_t>(start_y) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(start_x);
  const uint64_t goal_index =
    static_cast<uint64_t>(goal_y) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(goal_x);

  const size_t map_size = static_cast<size_t>(size_x_) * static_cast<size_t>(size_y_);
  std::vector<unsigned char> map_charmap;
  if (!map_->copyValues(map_charmap) || map_charmap.size() != map_size) {
    logFailure("failed to copy map values or copied map size mismatched",
      start_x,
      start_y,
      goal_x,
      goal_y);
    return false;
  }
  const auto * charmap = map_charmap.data();
  if (!charmap) {
    logFailure("copied map data is null", start_x, start_y, goal_x, goal_y);
    return false;
  }

  const auto is_traversable = [this, charmap](const uint64_t index) -> bool {
    const unsigned char cost = charmap[static_cast<size_t>(index)];
    if ((cost == UNKNOWN_COST && !allow_unknown_) ||
        (cost != UNKNOWN_COST && cost >= INSCRIBED_COST)) {
      return false;
    }
    if (minimum_clearance_ <= 0.0f) {
      return true;
    }
    const auto x = static_cast<unsigned int>(index % size_x_);
    const auto y = static_cast<unsigned int>(index / size_x_);
    return getESDFDistance(x, y) >= minimum_clearance_;
  };

  if (!is_traversable(goal_index)) {
    const unsigned char goal_cost = charmap[static_cast<size_t>(goal_index)];
    const bool cost_rejected = (goal_cost == UNKNOWN_COST && !allow_unknown_) ||
      (goal_cost != UNKNOWN_COST && goal_cost >= INSCRIBED_COST);
    const float distance = cost_rejected ? -std::numeric_limits<float>::infinity() :
      getESDFDistance(goal_x, goal_y);
    const std::string reason = cost_rejected ? "goal rejected by grid cost" :
      (std::isfinite(distance) ? "goal rejected by ESDF clearance" :
      "goal rejected: ESDF query unavailable or nonfinite");
    logFailure(reason + ", cost=" + std::to_string(static_cast<unsigned int>(goal_cost)) +
      ", clearance=" + (cost_rejected ? std::string("not_checked") : std::to_string(distance)),
      start_x,
      start_y,
      goal_x,
      goal_y);
    return false;
  }

  const float step_cardinal = static_cast<float>(costmap_resolution_);
  const float step_diagonal = step_cardinal * 1.41421356237f;
  const float D = step_cardinal;
  const float D2 = step_diagonal;

  const auto heuristic = [D, D2, goal_x, goal_y](const unsigned int x, const unsigned int y) {
    const unsigned int dx = (x > goal_x) ? (x - goal_x) : (goal_x - x);
    const unsigned int dy = (y > goal_y) ? (y - goal_y) : (goal_y - y);
    const unsigned int min_d = (dx < dy) ? dx : dy;
    return D * static_cast<float>(dx + dy) + (D2 - 2.0f * D) * static_cast<float>(min_d);
  };

  const float tol_cells =
    (costmap_resolution_ > 1e-9) ? static_cast<float>(tolerance_ / costmap_resolution_) : tolerance_;
  const float tol_sq = tol_cells * tol_cells;

  // Min-heap open list
  std::priority_queue<NodeMin, std::vector<NodeMin>, std::greater<NodeMin>> open;

  visited_[static_cast<size_t>(start_index)] = planning_id_;
  closed_[static_cast<size_t>(start_index)] = 0u;
  g_score_[static_cast<size_t>(start_index)] = 0.0f;
  parent_[static_cast<size_t>(start_index)] = -1;
  open.push(NodeMin{start_index, heuristic(start_x, start_y)});

  uint64_t goal_reached_index = goal_index;
  bool goal_reached = false;
  int iterations = 0;

  while (!open.empty() && iterations < max_iterations_) {
    if (cancel_checker && cancel_checker()) {
      logFailure("planning canceled", start_x, start_y, goal_x, goal_y, iterations);
      return false;
    }

    const NodeMin current = open.top();
    open.pop();

    const uint64_t current_index = current.index;
    const size_t current_i = static_cast<size_t>(current_index);
    if (closed_[current_i] == planning_id_) {
      continue;
    }
    closed_[current_i] = planning_id_;
    ++iterations;

    const uint64_t cy = current_index / static_cast<uint64_t>(size_x_);
    const uint64_t cx = current_index - cy * static_cast<uint64_t>(size_x_);
    const unsigned int cx_u = static_cast<unsigned int>(cx);
    const unsigned int cy_u = static_cast<unsigned int>(cy);

    const float dx = static_cast<float>((cx_u > goal_x) ? (cx_u - goal_x) : (goal_x - cx_u));
    const float dy = static_cast<float>((cy_u > goal_y) ? (cy_u - goal_y) : (goal_y - cy_u));
    if (dx * dx + dy * dy <= tol_sq) {
      goal_reached = true;
      goal_reached_index = current_index;
      break;
    }

    const float g_current = g_score_[current_i];

    // 8-connected neighbors: (dx,dy,cost)
    // No division/mod in the inner neighbor expansion loop.
    const int offsets[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}};

    for (int k = 0; k < 8; ++k) {
      const int nx_i = static_cast<int>(cx_u) + offsets[k][0];
      const int ny_i = static_cast<int>(cy_u) + offsets[k][1];
      if (nx_i < 0 || ny_i < 0) {
        continue;
      }
      const unsigned int nx = static_cast<unsigned int>(nx_i);
      const unsigned int ny = static_cast<unsigned int>(ny_i);
      if (nx >= size_x_ || ny >= size_y_) {
        continue;
      }

      const uint64_t neighbor_index =
        static_cast<uint64_t>(ny) * static_cast<uint64_t>(size_x_) + static_cast<uint64_t>(nx);
      const size_t neighbor_i = static_cast<size_t>(neighbor_index);
      if (closed_[neighbor_i] == planning_id_) {
        continue;
      }

      if (!is_traversable(neighbor_index)) {
        continue;
      }
      if (edge_filter_ && !edge_filter_(cx_u, cy_u, nx, ny)) {
        continue;
      }

      const bool diagonal = (k >= 4);
      if (diagonal) {
        const uint64_t side_x = cy * static_cast<uint64_t>(size_x_) + nx;
        const uint64_t side_y = static_cast<uint64_t>(ny) * size_x_ + cx;
        if (!is_traversable(side_x) || !is_traversable(side_y)) {
          continue;
        }
      }
      const float step_cost = diagonal ? step_diagonal : step_cardinal;

      const unsigned char cell_cost = charmap[neighbor_i];
      const float traversal_factor = evaluateInflationCost(cell_cost);

      float tentative_g = g_current + step_cost * traversal_factor;
      if (use_esdf_cost_) {
        tentative_g += step_cost * getESDFPotentialCost(nx, ny);
      }

      if (visited_[neighbor_i] != planning_id_ || tentative_g < g_score_[neighbor_i]) {
        visited_[neighbor_i] = planning_id_;
        g_score_[neighbor_i] = tentative_g;
        parent_[neighbor_i] = static_cast<int>(current_index);

        const float f = tentative_g + heuristic(nx, ny);
        open.push(NodeMin{neighbor_index, f});
      }
    }
  }

  if (!goal_reached && closed_[static_cast<size_t>(goal_index)] != planning_id_) {
    logFailure(iterations >= max_iterations_ ? "max iterations reached before goal" : "open set exhausted before goal",
      start_x,
      start_y,
      goal_x,
      goal_y,
      iterations);
    return false;
  }

  // Backtrace (goal -> start). Caller expects reversed ordering.
  uint64_t index = goal_reached ? goal_reached_index : goal_index;
  for (;;) {
    const uint64_t y = index / static_cast<uint64_t>(size_x_);
    const uint64_t x = index - y * static_cast<uint64_t>(size_x_);
    path.emplace_back(static_cast<float>(x), static_cast<float>(y));

    if (index == start_index) {
      break;
    }

    const int p = parent_[static_cast<size_t>(index)];
    if (p < 0) {
      logFailure("path backtrace failed: missing parent", start_x, start_y, goal_x, goal_y, iterations);
      return false;
    }
    index = static_cast<uint64_t>(p);
  }

  if (path.size() < 2u) {
    if (goal_reached) {
      // Start is already inside tolerance. Callers require >= 2 poses; emit
      // goal then start so the reversed path is start -> goal.
      path.clear();
      path.emplace_back(static_cast<float>(goal_x), static_cast<float>(goal_y));
      path.emplace_back(static_cast<float>(start_x), static_cast<float>(start_y));
      return true;
    }
    logFailure("path has fewer than 2 poses", start_x, start_y, goal_x, goal_y, iterations);
    return false;
  }

  return true;
}

}  // namespace smac
}  // namespace mas2027_nav_executor
