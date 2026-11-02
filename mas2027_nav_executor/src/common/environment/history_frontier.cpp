#include "mas2027_nav_executor/common/environment/history_frontier.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace mas2027_nav_executor {
namespace {

constexpr uint8_t kFree = 0U;
constexpr uint8_t kUnknown = 255U;

struct Node
{
  size_t index{0};
  double cost{0.0};
  bool operator>(const Node & other) const { return cost > other.cost; }
};

}  // namespace

std::optional<HistoryFrontierRoute> findHistoryFrontierRoute(
  const OnlineGlobalMapSnapshot & map, const HistoryFrontierQuery & request)
{
  const int width = static_cast<int>(map.sizeX());
  const int height = static_cast<int>(map.sizeY());
  const double resolution = map.resolution();
  if (width <= 0 || height <= 0 || !(resolution > 0.0) ||
    !request.start.allFinite() || !request.goal.allFinite()) return std::nullopt;
  const size_t count = static_cast<size_t>(width) * height;
  const auto index = [width](int x, int y) { return static_cast<size_t>(y) * width + x; };
  const auto inside = [width, height](int x, int y) {
    return x >= 0 && y >= 0 && x < width && y < height;
  };
  const auto center = [&map](int x, int y) {
    double wx = 0.0, wy = 0.0;
    map.mapToWorld(static_cast<unsigned int>(x), static_cast<unsigned int>(y), wx, wy);
    return Eigen::Vector2d(wx, wy);
  };
  const auto clearance = [&map](const Eigen::Vector2d & p) {
    const auto q = map.query({p.x(), p.y(), 0.0});
    return q.ok && std::isfinite(q.distance) ? q.distance :
      -std::numeric_limits<double>::infinity();
  };
  // Prefer a standoff away from drops, without deleting every frontier in a
  // narrow platform. Traversability/terrain gates still decide hard safety.
  const auto height_penalty = [&map, &request](const Eigen::Vector2d & p) {
    if (!(request.height_clearance > 0.0)) return 0.0;
    const auto q = map.queryHeight({p.x(), p.y(), 0.0});
    if (!q.ok || !std::isfinite(q.distance)) return 0.0;
    return 4.0 * std::max(0.0, request.height_clearance - q.distance);
  };
  std::vector<int8_t> traversable_cache(count, -1);
  const auto traversable = [&](int x, int y) {
    const size_t i = index(x, y);
    if (traversable_cache[i] < 0) {
      traversable_cache[i] = map.value(x, y) == kFree &&
        clearance(center(x, y)) >= request.route_clearance ? 1 : 0;
    }
    return traversable_cache[i] != 0;
  };
  const auto direction = makeDirectedTerrainSampler(width,height,resolution,map.originX(),map.originY(),
    [&map](double x,double y) { return map.sampleTerrain(x,y); },request.directed_terrain,
    [&map](int x,int y) { return map.isFree(x,y); });
  const auto transition = [&](int ax, int ay, int bx, int by) {
    const auto a = center(ax, ay), b = center(bx, by);
    return map.terrainTransition(a.x(), a.y(), b.x(), b.y()) &&
      directed_terrain_edge_allowed(direction, a,b,request.directed_terrain,0.5*resolution);
  };
  constexpr int kOffsets[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  // Start: the robot's cell, or the nearest searchable cell within reach.
  unsigned int ux = 0U, uy = 0U;
  if (!map.worldToMap(request.start.x(), request.start.y(), ux, uy)) return std::nullopt;
  int sx = static_cast<int>(ux), sy = static_cast<int>(uy);
  if (!traversable(sx, sy)) {
    const int radius = static_cast<int>(std::ceil(request.start_projection / resolution));
    double best = std::numeric_limits<double>::infinity();
    int bx = -1, by = -1;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const int x = sx + dx, y = sy + dy;
        const double d = std::hypot(dx, dy) * resolution;
        if (!inside(x, y) || d > request.start_projection || d >= best || !traversable(x, y)) {
          continue;
        }
        best = d; bx = x; by = y;
      }
    }
    if (bx < 0) return std::nullopt;
    sx = bx; sy = by;
  }

  // Shortest routes from the start through remembered, cleared free cells.
  std::vector<double> distance(count, std::numeric_limits<double>::infinity());
  std::vector<int64_t> parent(count, -1);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  distance[index(sx, sy)] = 0.0;
  open.push({index(sx, sy), 0.0});
  while (!open.empty()) {
    const Node current = open.top();
    open.pop();
    if (current.cost > distance[current.index]) continue;
    const int cx = static_cast<int>(current.index % width);
    const int cy = static_cast<int>(current.index / width);
    for (const auto & offset : kOffsets) {
      const int x = cx + offset[0], y = cy + offset[1];
      if (!inside(x, y) || !traversable(x, y)) continue;
      const bool diagonal = offset[0] != 0 && offset[1] != 0;
      if (diagonal && (!traversable(x, cy) || !traversable(cx, y))) continue;
      const double next = current.cost + (diagonal ? std::sqrt(2.0) : 1.0) * resolution;
      if (next >= distance[index(x, y)] || !transition(cx, cy, x, y)) continue;
      distance[index(x, y)] = next;
      parent[index(x, y)] = static_cast<int64_t>(current.index);
      open.push({index(x, y), next});
    }
  }

  size_t target = count;
  if (request.fixed_target) {
    unsigned int tx = 0U, ty = 0U;
    if (!map.worldToMap(request.fixed_target->x(), request.fixed_target->y(), tx, ty) ||
      !std::isfinite(distance[index(static_cast<int>(tx), static_cast<int>(ty))])) {
      return std::nullopt;
    }
    target = index(static_cast<int>(tx), static_cast<int>(ty));
  } else {
    // Standoff from never-observed space, propagated only through free cells
    // so a frontier behind a wall does not attract a target through it.
    std::vector<double> frontier(count, std::numeric_limits<double>::infinity());
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> seeds;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (map.value(x, y) != kFree) continue;
        for (int k = 0; k < 4; ++k) {
          const int nx = x + kOffsets[k][0], ny = y + kOffsets[k][1];
          if (!inside(nx, ny) || map.value(nx, ny) == kUnknown) {
            frontier[index(x, y)] = 0.0;
            seeds.push({index(x, y), 0.0});
            break;
          }
        }
      }
    }
    while (!seeds.empty()) {
      const Node current = seeds.top();
      seeds.pop();
      if (current.cost > frontier[current.index]) continue;
      const int cx = static_cast<int>(current.index % width);
      const int cy = static_cast<int>(current.index / width);
      for (const auto & offset : kOffsets) {
        const int x = cx + offset[0], y = cy + offset[1];
        if (!inside(x, y) || map.value(x, y) != kFree) continue;
        const bool diagonal = offset[0] != 0 && offset[1] != 0;
        const double next = current.cost + (diagonal ? std::sqrt(2.0) : 1.0) * resolution;
        if (next > request.frontier_standoff || next >= frontier[index(x, y)] ||
          !transition(cx, cy, x, y)) continue;
        frontier[index(x, y)] = next;
        seeds.push({index(x, y), next});
      }
    }
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < count; ++i) {
      if (!std::isfinite(distance[i]) || frontier[i] > request.frontier_standoff) continue;
      const auto p = center(static_cast<int>(i % width), static_cast<int>(i / width));
      if ((p - request.start).norm() < request.minimum_advance) continue;
      if (request.excluded && request.excluded(p)) continue;
      if (clearance(p) < request.target_clearance) continue;
      // Route length is real cost; goal distance keeps the search purposeful.
      const double score = 0.5 * distance[i] + (p - request.goal).norm() + height_penalty(p);
      if (score < best_score) {
        best_score = score;
        target = i;
      }
    }
    if (target >= count) return std::nullopt;
  }

  HistoryFrontierRoute route;
  route.target = center(static_cast<int>(target % width), static_cast<int>(target / width));
  for (int64_t i = static_cast<int64_t>(target); i >= 0; i = parent[static_cast<size_t>(i)]) {
    route.target_to_start.push_back(
      center(static_cast<int>(i % width), static_cast<int>(i / width)));
    if (static_cast<size_t>(i) == index(sx, sy)) break;
  }
  return route;
}

}  // namespace mas2027_nav_executor
