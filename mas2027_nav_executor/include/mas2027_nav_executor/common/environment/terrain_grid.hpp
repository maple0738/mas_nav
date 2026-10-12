#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mas2027_nav_executor {

// Immutable snapshots keep the planning thread independent of map callbacks.
class TerrainGrid final
{
public:
  void updateCost(const nav_msgs::msg::OccupancyGrid & grid);
  void updateLabels(const sensor_msgs::msg::Image & image);
  void updateDirections(const sensor_msgs::msg::Image & image);
  uint64_t revision() const noexcept { return revision_.load(std::memory_order_acquire); }

  struct Snapshot
  {
    nav_msgs::msg::OccupancyGrid cost;
    sensor_msgs::msg::Image labels;
    sensor_msgs::msg::Image directions;
    std::vector<uint8_t> slope_side_mask;

    bool contains(const Eigen::Vector2d & point) const;
    bool traversable(const Eigen::Vector2d & point) const;
    std::optional<uint8_t> terrainLabelAt(const Eigen::Vector2d & point) const;
    bool transition(const Eigen::Vector2d & from, const Eigen::Vector2d & to) const;
    bool slopeTransition(const Eigen::Vector2d & from, const Eigen::Vector2d & to) const;
    bool search(const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
      std::vector<Eigen::Vector2d> & path) const;

  private:
    bool cell(const Eigen::Vector2d & point, int & x, int & y) const;
    Eigen::Vector2d center(int x, int y) const;
    bool freeCell(int x, int y) const;
  };

  std::shared_ptr<const Snapshot> snapshot() const;

  // RViz-facing map of the hard static occupancy constraints used by the planner.
  // 100: blocked; 0: traversable.
  std::optional<nav_msgs::msg::OccupancyGrid> planningConstraints() const;

private:
  void refresh();
  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid cost_;
  sensor_msgs::msg::Image labels_;
  sensor_msgs::msg::Image directions_;
  std::shared_ptr<const Snapshot> snapshot_;
  std::atomic<uint64_t> revision_{0};
};

}  // namespace mas2027_nav_executor
