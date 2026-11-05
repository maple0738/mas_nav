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
#include "rog_map/terrain_direction.hpp"

namespace mas2027_nav_executor {

// Immutable snapshots keep the planning thread independent of map callbacks.
class TerrainGrid final
{
public:
  // Repeated static content preserves the snapshot and revision, regardless of timestamps.
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
    std::vector<uint8_t> terrain_labels;
    std::vector<float> uphill_x, uphill_y;
    rog_map::TerrainDirectionSample sampleTerrain(double x, double y) const
    { return rog_map::sampleTerrainDirection(cost.info.width, cost.info.height,
        cost.info.resolution, cost.info.origin.position.x, cost.info.origin.position.y,
        terrain_labels, uphill_x, uphill_y, x, y); }

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
