#pragma once

#include <memory>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rog_map/map_query_interface.hpp"
namespace rog_map { struct MapSnapshot; }

namespace mas2027_nav_executor {

struct OnlineGlobalMapConfig
{
  double resolution{0.1};
  size_t max_cells{2000000};
  int occupied_increment{3};
  int free_decrement{1};
  int evidence_limit{5};
  int occupied_confirmations{2};
  double confirmation_max_gap_s{1.5};
  int terrain_confirmations{3};
  int terrain_clear_confirmations{10};
};

// Each instance is immutable. Pin one for an entire search so an expansion
// cannot change the origin between worldToMap(), value() and mapToWorld().
class OnlineGlobalMapSnapshot final : public rog_map::MapQueryInterface
{
public:
  const nav_msgs::msg::OccupancyGrid & grid() const { return grid_; }
  nav_msgs::msg::OccupancyGrid terrainGrid() const;
  sensor_msgs::msg::Image uphillImage() const;
  bool terrainTransition(double from_x, double from_y, double to_x, double to_y) const;
  const std::vector<uint8_t> & slopeSideMask() const { return slope_side_mask_; }
  uint64_t revision() const { return revision_; }
  bool worldToMap(double x, double y, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & x, double & y) const override;
  unsigned int sizeX() const override { return grid_.info.width; }
  unsigned int sizeY() const override { return grid_.info.height; }
  double resolution() const override { return resolution_; }
  double originX() const override { return grid_.info.origin.position.x; }
  double originY() const override { return grid_.info.origin.position.y; }
  double snapshotStampSeconds() const override { return stamp_; }
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override { return nullptr; }
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & position) const override;
  bool evaluate(const Eigen::Vector3d & position, double & distance,
    Eigen::Vector3d & gradient) const override;

private:
  friend class OnlineGlobalMap;
  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<uint8_t> terrain_labels_;
  std::vector<float> uphill_x_;
  std::vector<float> uphill_y_;
  std::vector<uint8_t> slope_side_mask_;
  std::shared_ptr<const std::vector<double>> distances_;
  uint64_t revision_{0};
  double resolution_{0.1};
  double stamp_{0.0};
};

// Session map in the same odom frame as ROGMap. Unknown local cells never erase
// history. Occupancy candidates win within a coarse cell and need confirmation;
// freeing a confirmed obstacle needs full coverage
// and repeated free observations from distinct source snapshots.
class OnlineGlobalMap final
{
public:
  enum class UpdateResult { UPDATED, NO_NEW_OBSERVATION, INVALID_INPUT, CELL_LIMIT };
  explicit OnlineGlobalMap(const OnlineGlobalMapConfig & config, std::string frame);
  UpdateResult update(const rog_map::MapSnapshot & source);
  std::shared_ptr<OnlineGlobalMapSnapshot> snapshot() const;

private:
  struct Cell {
    int8_t evidence{0};
    bool observed{false};
    uint8_t pending_hits{0};
    double last_pending_stamp{0.0};
    uint8_t terrain_label{255U};
    float uphill_x{0.0f};
    float uphill_y{0.0f};
    uint8_t pending_terrain_label{255U};
    uint8_t terrain_hits{0};
    uint8_t ordinary_hits{0};
    uint64_t last_terrain_observation_sequence{0};
  };
  OnlineGlobalMapConfig config_;
  std::string frame_;
  mutable std::mutex mutex_;
  std::mutex update_mutex_;
  std::vector<Cell> cells_;
  int origin_x_{0};
  int origin_y_{0};
  int width_{0};
  int height_{0};
  uint64_t last_source_sequence_{0};
  double last_source_stamp_{0.0};
  std::shared_ptr<OnlineGlobalMapSnapshot> snapshot_;
};

}  // namespace mas2027_nav_executor
