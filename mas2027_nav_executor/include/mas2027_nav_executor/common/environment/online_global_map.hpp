#pragma once

#include <memory>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rog_map/map_query_interface.hpp"
#include "rog_map/terrain_direction.hpp"
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

// World-fixed extent of an observed undulating strip. History and its pinned
// snapshots share this geometry; only history owns its confirmation lifetime.
struct WaveCorridor
{
  Eigen::Vector2d start{Eigen::Vector2d::Zero()};
  Eigen::Vector2d direction{Eigen::Vector2d::UnitX()};
  double length{0.0};
  double half_width{0.0};
  bool contains(double x, double y) const;
};

// Each instance is immutable. Pin one for an entire search so an expansion
// cannot change the origin between worldToMap(), value() and mapToWorld().
class OnlineGlobalMapSnapshot final : public rog_map::MapQueryInterface
{
public:
  rog_map::TerrainDirectionSample sampleTerrain(double x, double y) const
  { return rog_map::sampleTerrainDirection(sizeX(), sizeY(), resolution_, originX(), originY(),
      terrain_labels_, uphill_x_, uphill_y_, x, y); }
  const nav_msgs::msg::OccupancyGrid & grid() const { return grid_; }
  nav_msgs::msg::OccupancyGrid terrainGrid() const;
  nav_msgs::msg::OccupancyGrid heightGrid() const;
  uint8_t terrainLabel(unsigned int mx, unsigned int my) const
  {
    if (mx >= sizeX() || my >= sizeY()) return 255U;
    return terrain_labels_[static_cast<size_t>(my) * sizeX() + mx];
  }
  sensor_msgs::msg::Image uphillImage() const;
  bool terrainTransition(double from_x, double from_y, double to_x, double to_y,
    double height_clearance = 0.0,
    std::optional<Eigen::Vector2d> clearance_origin = std::nullopt) const;
  // True when a confirmed repeated-wave label is within radius of the point.
  // Used to exempt only height-discontinuity evidence, never occupancy.
  bool nearWave(double x, double y, double radius) const;
  rog_map::QueryResult queryHeight(const Eigen::Vector3d & position) const;
  const std::vector<uint8_t> & slopeSideMask() const { return slope_side_mask_; }
  const std::vector<uint8_t> & heightDiscontinuityMask() const { return height_discontinuity_mask_; }
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
  rog_map::QueryResult queryField(const Eigen::Vector3d & position,
    const std::shared_ptr<const std::vector<double>> & field) const;
  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<uint8_t> terrain_labels_;
  std::vector<float> uphill_x_;
  std::vector<float> uphill_y_;
  std::vector<uint8_t> slope_side_mask_;
  std::vector<uint8_t> height_discontinuity_mask_;
  std::optional<WaveCorridor> wave_corridor_;
  std::shared_ptr<const std::vector<double>> distances_;
  std::shared_ptr<const std::vector<double>> height_distances_;
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
  // Travel direction used to project a newly confirmed wave label into a
  // fixed-length corridor. Without it a cross-road label strip would be
  // interpreted as a corridor along its short axis.
  void setWaveTravelDirection(const Eigen::Vector2d & direction);

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
    bool height_discontinuity{false};
    uint8_t height_clear_hits{0};
    double last_height_clear_stamp{0.0};
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
  mutable std::mutex wave_direction_mutex_;
  Eigen::Vector2d wave_travel_direction_{Eigen::Vector2d::UnitX()};
  bool wave_travel_direction_valid_{false};
  std::optional<WaveCorridor> wave_corridor_;
  double wave_latch_stamp_{0.0};
  uint64_t last_source_sequence_{0};
  double last_source_stamp_{0.0};
  std::shared_ptr<OnlineGlobalMapSnapshot> snapshot_;
};

}  // namespace mas2027_nav_executor
