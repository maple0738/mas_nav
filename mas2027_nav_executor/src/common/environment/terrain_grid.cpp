#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <cstring>

#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"

namespace mas2027_nav_executor {
namespace {
constexpr int kOccupiedCost = 95;
}

void TerrainGrid::updateCost(const nav_msgs::msg::OccupancyGrid & grid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cost_ = grid;
  refresh();
  revision_.fetch_add(1, std::memory_order_release);
}

void TerrainGrid::updateLabels(const sensor_msgs::msg::Image & image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  labels_ = image;
  refresh();
  revision_.fetch_add(1, std::memory_order_release);
}

void TerrainGrid::updateDirections(const sensor_msgs::msg::Image & image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  directions_ = image;
  refresh();
  revision_.fetch_add(1, std::memory_order_release);
}

void TerrainGrid::refresh()
{
  snapshot_.reset();
  const auto width = cost_.info.width;
  const auto height = cost_.info.height;
  if (width == 0 || height == 0 || !std::isfinite(cost_.info.resolution) ||
    cost_.info.resolution <= 0.0F ||
    cost_.data.size() != static_cast<size_t>(width) * height ||
    labels_.width != width || labels_.height != height ||
    labels_.encoding != "mono8" || labels_.step < width ||
    labels_.data.size() < static_cast<size_t>(labels_.step) * height ||
    cost_.header.frame_id.empty() || cost_.header.frame_id != labels_.header.frame_id ||
    std::abs(cost_.info.origin.orientation.x) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.y) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.z) > 1e-6 ||
    std::abs(cost_.info.origin.orientation.w - 1.0) > 1e-6) {
    return;
  }
  const bool direction_valid = directions_.width == width && directions_.height == height &&
    directions_.header.frame_id == cost_.header.frame_id &&
    directions_.encoding == "32FC2" && !directions_.is_bigendian &&
    directions_.step >= width * 2U * sizeof(float) &&
    directions_.data.size() >= static_cast<size_t>(directions_.step) * height;
  auto next = std::make_shared<Snapshot>(Snapshot{
    cost_, labels_, direction_valid ? directions_ : sensor_msgs::msg::Image{}});
  const size_t count = static_cast<size_t>(width) * height;
  std::vector<uint8_t> labels(count);
  std::vector<float> uphill_x(count, 0.0F), uphill_y(count, 0.0F);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const size_t index = static_cast<size_t>(y) * width + x;
      labels[index] = labels_.data[static_cast<size_t>(y) * labels_.step + x];
      if (direction_valid) {
        const size_t byte = static_cast<size_t>(y) * directions_.step +
          static_cast<size_t>(x) * 2U * sizeof(float);
        std::memcpy(&uphill_x[index], directions_.data.data() + byte, sizeof(float));
        std::memcpy(&uphill_y[index], directions_.data.data() + byte + sizeof(float), sizeof(float));
      }
    }
  }
  next->slope_side_mask = buildSlopeSideMask(
    static_cast<int>(width), static_cast<int>(height), labels, uphill_x, uphill_y);
  snapshot_ = std::move(next);
}

std::shared_ptr<const TerrainGrid::Snapshot> TerrainGrid::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

std::optional<nav_msgs::msg::OccupancyGrid> TerrainGrid::planningConstraints() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!snapshot_) return std::nullopt;

  nav_msgs::msg::OccupancyGrid result = snapshot_->cost;
  for (size_t index = 0; index < result.data.size(); ++index) {
    const int8_t cost = snapshot_->cost.data[index];
    result.data[index] = cost < 0 || cost >= kOccupiedCost ||
      snapshot_->slope_side_mask[index] ? 100 : 0;
  }
  return result;
}

bool TerrainGrid::Snapshot::cell(const Eigen::Vector2d & point, int & x, int & y) const
{
  if (!point.allFinite()) return false;
  const double res = cost.info.resolution;
  const double fx = (point.x() - cost.info.origin.position.x) / res;
  const double fy = (point.y() - cost.info.origin.position.y) / res;
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0 ||
    fx >= cost.info.width || fy >= cost.info.height) return false;
  x = static_cast<int>(std::floor(fx));
  y = static_cast<int>(std::floor(fy));
  return true;
}

Eigen::Vector2d TerrainGrid::Snapshot::center(int x, int y) const
{
  const double res = cost.info.resolution;
  return {cost.info.origin.position.x + (x + 0.5) * res,
    cost.info.origin.position.y + (y + 0.5) * res};
}

bool TerrainGrid::Snapshot::freeCell(int x, int y) const
{
  const size_t index = static_cast<size_t>(y) * cost.info.width + x;
  return cost.data[index] >= 0 && cost.data[index] < kOccupiedCost &&
    (slope_side_mask.empty() || slope_side_mask[index] == 0U);
}

bool TerrainGrid::Snapshot::contains(const Eigen::Vector2d & point) const
{
  int x = 0, y = 0;
  return cell(point, x, y);
}

bool TerrainGrid::Snapshot::traversable(const Eigen::Vector2d & point) const
{
  int x = 0, y = 0;
  if (!cell(point, x, y)) return false;
  const size_t index = static_cast<size_t>(y) * cost.info.width + x;
  return freeCell(x, y);
}

std::optional<uint8_t> TerrainGrid::Snapshot::terrainLabelAt(const Eigen::Vector2d & point) const
{
  int x = 0, y = 0;
  if (!cell(point, x, y)) return std::nullopt;
  return labels.data[static_cast<size_t>(y) * labels.step + static_cast<size_t>(x)];
}

bool TerrainGrid::Snapshot::transition(
  const Eigen::Vector2d & from, const Eigen::Vector2d & to) const
{
  const Eigen::Vector2d delta = to - from;
  const double length = delta.norm();
  if (!std::isfinite(length) || length < 1e-9) return traversable(from);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * cost.info.resolution))));
  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector2d point = from + delta * (static_cast<double>(i) / steps);
    int x = 0, y = 0;
    if (!cell(point, x, y) || !freeCell(x, y)) return false;
  }
  return slopeTransition(from, to);
}

bool TerrainGrid::Snapshot::slopeTransition(
  const Eigen::Vector2d & from, const Eigen::Vector2d & to) const
{
  return terrainSideTransitionAllowed(static_cast<int>(cost.info.width),
    static_cast<int>(cost.info.height), cost.info.resolution,
    cost.info.origin.position.x, cost.info.origin.position.y, slope_side_mask,
    from.x(), from.y(), to.x(), to.y(), true);
}

bool TerrainGrid::Snapshot::search(const Eigen::Vector2d & start,
  const Eigen::Vector2d & goal, std::vector<Eigen::Vector2d> & path) const
{
  path.clear();
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!cell(start, sx, sy) || !cell(goal, gx, gy) ||
    !traversable(start) || !traversable(goal)) return false;
  if (sx == gx && sy == gy) {
    if (!transition(start, goal)) return false;
    path = {start, goal};
    return true;
  }
  const int width = static_cast<int>(cost.info.width);
  const int height = static_cast<int>(cost.info.height);
  const int count = width * height;
  const auto index = [width](int x, int y) { return y * width + x; };
  const int source = index(sx, sy), target = index(gx, gy);
  const auto edge_free = [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return transition(a, b);
  };
  const auto heuristic = [gx, gy](int x, int y) {
    const int dx = std::abs(x - gx), dy = std::abs(y - gy);
    return static_cast<double>(std::max(dx, dy)) + 0.4142135623730951 * std::min(dx, dy);
  };
  using Candidate = std::pair<double, int>;
  std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> open;
  std::vector<double> g(count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(count, -1);
  std::vector<uint8_t> closed(count, 0);
  g[source] = 0.0;
  open.emplace(heuristic(sx, sy), source);
  constexpr std::array<std::pair<int, int>, 8> neighbors{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  while (!open.empty()) {
    const int current = open.top().second;
    open.pop();
    if (closed[current]) continue;
    if (current == target) break;
    closed[current] = 1;
    const int x = current % width, y = current / width;
    for (const auto & [dx, dy] : neighbors) {
      const int nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
      const int next = index(nx, ny);
      if (closed[next] || !edge_free(center(x, y), center(nx, ny))) continue;
      // No diagonal corner cutting, even if the center-to-center ray misses the corner cells.
      if (dx != 0 && dy != 0 &&
        (!edge_free(center(x, y), center(x + dx, y)) ||
        !edge_free(center(x, y), center(x, y + dy)))) continue;
      const double step = std::hypot(dx, dy);
      const double cost_penalty = std::max(0, static_cast<int>(cost.data[next])) / 100.0;
      const double next_g = g[current] + step * (1.0 + cost_penalty);
      if (next_g >= g[next]) continue;
      g[next] = next_g;
      parent[next] = current;
      open.emplace(next_g + heuristic(nx, ny), next);
    }
  }
  if (parent[target] < 0) return false;
  for (int cursor = target; cursor != source; cursor = parent[cursor]) {
    if (cursor < 0) return false;
    path.push_back(center(cursor % width, cursor / width));
  }
  path.push_back(center(sx, sy));
  std::reverse(path.begin(), path.end());
  if (!edge_free(start, path.front()) || !edge_free(path.back(), goal)) {
    path.clear();
    return false;
  }
  path.front() = start;
  path.back() = goal;
  return true;
}

}  // namespace mas2027_nav_executor
