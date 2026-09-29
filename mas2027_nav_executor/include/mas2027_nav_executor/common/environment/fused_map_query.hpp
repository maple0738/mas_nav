#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "minco_core/map_query_interface.hpp"

namespace mas2027_nav_executor {

// Presents the rolling ROGMap geometry while enforcing both ROGMap and the
// static terrain distance field. Query positions are expressed in the dynamic
// map frame (odom); the supplied transform maps them into the static map frame.
class FusedMapQuery final : public rog_map::MapQueryInterface
{
public:
  FusedMapQuery(
    std::shared_ptr<rog_map::MapQueryInterface> dynamic_query,
    std::shared_ptr<rog_map::MapQueryInterface> static_query);

  // Set the latest planar static_frame <- dynamic_frame transform.
  void setStaticTransform(double translation_x, double translation_y, double yaw);
  void clearStaticTransform();
  bool hasStaticTransform() const;

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;

  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  double snapshotStampSeconds() const override;

  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  struct Transform2D
  {
    double translation_x{0.0};
    double translation_y{0.0};
    double cosine{1.0};
    double sine{0.0};
    bool valid{false};
  };

  Transform2D transformSnapshot() const;
  Eigen::Vector3d toStatic(
    const Eigen::Vector3d & point, const Transform2D & transform) const;
  Eigen::Vector3d gradientToDynamic(
    const Eigen::Vector3d & gradient, const Transform2D & transform) const;
  bool staticCellForDynamicCell(
    unsigned int mx, unsigned int my, unsigned int & static_mx, unsigned int & static_my) const;

  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  std::shared_ptr<rog_map::MapQueryInterface> static_query_;
  mutable std::mutex transform_mutex_;
  Transform2D transform_;

  static constexpr uint8_t kLethalCost = 254U;
  static constexpr uint8_t kInscribedCost = 253U;
};

}  // namespace mas2027_nav_executor
