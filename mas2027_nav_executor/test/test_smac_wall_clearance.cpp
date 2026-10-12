#include "mas2027_nav_executor/path_planner/search/smac/smac_planner_2d_simple.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace {

class GridQuery final : public rog_map::MapQueryInterface
{
public:
  GridQuery(unsigned int width, unsigned int height, double resolution)
  : width_(width), height_(height), resolution_(resolution), cells_(width * height, 0U) {}

  void block(unsigned int x, unsigned int y) { cells_[y * width_ + x] = 254U; }
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (wx < 0.0 || wy < 0.0 || wx >= width_ * resolution_ || wy >= height_ * resolution_) return false;
    mx = static_cast<unsigned int>(wx / resolution_);
    my = static_cast<unsigned int>(wy / resolution_);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (mx + 0.5) * resolution_;
    wy = (my + 0.5) * resolution_;
  }
  unsigned int sizeX() const override { return width_; }
  unsigned int sizeY() const override { return height_; }
  double resolution() const override { return resolution_; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int mx, unsigned int my) const override { return cells_[my * width_ + mx]; }
  const unsigned char * values() const override { return cells_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override { return mx < width_ && my < height_; }
  bool isFree(unsigned int mx, unsigned int my) const override { return isValid(mx, my) && value(mx, my) == 0U; }
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override
  {
    unsigned int mx = 0, my = 0;
    if (!worldToMap(pos.x(), pos.y(), mx, my)) return false;
    dist = std::numeric_limits<double>::infinity();
    for (unsigned int y = 0; y < height_; ++y) {
      for (unsigned int x = 0; x < width_; ++x) {
        if (value(x, y) != 254U) continue;
        dist = std::min(dist, resolution_ * std::hypot(
          static_cast<double>(mx) - x, static_cast<double>(my) - y));
      }
    }
    grad.setZero();
    return std::isfinite(dist);
  }

private:
  unsigned int width_;
  unsigned int height_;
  double resolution_;
  std::vector<unsigned char> cells_;
};

TEST(SmacWallClearance, CannotCrossBlockedDiagonalCorner)
{
  auto map = std::make_shared<GridQuery>(3U, 3U, 0.05);
  map->block(1U, 0U);
  map->block(0U, 1U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple planner;
  planner.setMap(map);
  planner.setParameters(false, 1000, 0.0F);
  mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector path;
  EXPECT_FALSE(planner.createPath(0U, 0U, 2U, 2U, path));
}

TEST(SmacWallClearance, EdgeFilterRoutesThroughPermittedSlopeEntrance)
{
  auto map = std::make_shared<GridQuery>(7U, 7U, 0.1);
  mas2027_nav_executor::smac::SmacPlanner2DSimple planner;
  planner.setMap(map);
  planner.setParameters(false, 1000, 0.0F);
  planner.setEdgeFilter([](unsigned int x, unsigned int y,
      unsigned int next_x, unsigned int next_y) {
    const bool crosses = (x <= 2U && next_x >= 3U) || (x >= 3U && next_x <= 2U);
    return !crosses || (y == 3U && next_y == 3U);
  });
  mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(1U, 1U, 5U, 1U, path));
  bool used_entrance = false;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    if ((a.x <= 2.0F && b.x >= 3.0F) || (a.x >= 3.0F && b.x <= 2.0F)) {
      EXPECT_EQ(a.y, 3.0F);
      EXPECT_EQ(b.y, 3.0F);
      used_entrance = true;
    }
  }
  EXPECT_TRUE(used_entrance);
}

TEST(SmacWallClearance, ShortWallEndKeepsConfiguredClearance)
{
  auto map = std::make_shared<GridQuery>(45U, 35U, 0.05);
  for (unsigned int y = 16U; y <= 18U; ++y) {
    for (unsigned int x = 5U; x <= 27U; ++x) map->block(x, y);
  }
  mas2027_nav_executor::smac::SmacPlanner2DSimple planner;
  planner.setMap(map);
  planner.setESDFQuery(map);
  planner.setESDFParameters(true, 2.0, 0.8, 5.0);
  planner.setMinimumClearance(0.25);
  planner.setParameters(false, 100000, 0.0F);
  mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(22U, 6U, 22U, 29U, path));
  bool passed_short_end = false;
  for (const auto & cell : path) {
    double wx = 0.0, wy = 0.0;
    map->mapToWorld(static_cast<unsigned int>(cell.x), static_cast<unsigned int>(cell.y), wx, wy);
    double clearance = 0.0;
    Eigen::Vector3d gradient;
    ASSERT_TRUE(map->evaluate(Eigen::Vector3d(wx, wy, 0.0), clearance, gradient));
    EXPECT_GE(clearance + 1e-6, 0.25);
    if (cell.y >= 16.0F && cell.y <= 18.0F && cell.x > 27.0F) passed_short_end = true;
  }
  EXPECT_TRUE(passed_short_end);
}

TEST(SmacWallClearance, FreeGoalStillRequiresValidDistanceFieldAndClearance)
{
  auto map = std::make_shared<GridQuery>(20U, 20U, 0.05);
  map->block(10U, 10U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple planner;
  planner.setMap(map);
  planner.setMinimumClearance(0.27);
  planner.setParameters(false, 10000, 0.0F);
  mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector path;
  // Missing ESDF must fail closed even though every requested endpoint is free.
  EXPECT_FALSE(planner.createPath(1U, 1U, 18U, 18U, path));
  planner.setESDFQuery(map);
  EXPECT_FALSE(planner.createPath(1U, 1U, 10U, 11U, path));
  EXPECT_TRUE(path.empty());
  EXPECT_TRUE(planner.createPath(1U, 1U, 18U, 18U, path));
}

}  // namespace
