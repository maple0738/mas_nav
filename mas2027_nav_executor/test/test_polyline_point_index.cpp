#include "mas2027_nav_executor/common/geometry/polyline_point_index.hpp"
#include <gtest/gtest.h>
#include <random>

namespace mas2027_nav_executor {
namespace {
PolylinePointIndex::Nearest brute(const std::vector<Eigen::Vector2d> &points,
                                  double x, double y) {
  PolylinePointIndex::Nearest result;
  for (size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i].x() - x, dy = points[i].y() - y;
    const double distance = dx * dx + dy * dy;
    if (distance < result.squared_distance) {
      result.index = i;
      result.squared_distance = distance;
    }
  }
  return result;
}
TEST(PolylinePointIndex,
     MatchesGlobalSearchForCrossingsJumpsDuplicatesAndInvalidCoordinates) {
  std::mt19937 random(19241);
  std::uniform_real_distribution<double> value(-10., 10.);
  PolylinePointIndex index;
  for (int count : {0, 1, 15, 16, 17, 64, 1025}) {
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < count; ++i)
      points.emplace_back(value(random), value(random));
    if (count > 16) {
      points[2] = {0., 0.};
      points[16] = points[2];
    }
    if (count > 64) {
      points[19].x() = std::numeric_limits<double>::quiet_NaN();
      points[21].y() = std::numeric_limits<double>::infinity();
      points[24] = {1e300, -1e300};
    }
    index.build(points.size(), [&](size_t i) { return points[i]; });
    for (int query = 0; query < 200; ++query) {
      const double x = query == 0 ? 0. : value(random),
                   y = query == 0 ? 0. : value(random);
      const auto expected = brute(points, x, y);
      const auto actual = index.nearest(
          x, y, query % 2 ? points.size() + 5 : points.size() / 2);
      EXPECT_EQ(actual.index, expected.index) << count << ", " << query;
      EXPECT_DOUBLE_EQ(actual.squared_distance, expected.squared_distance);
    }
  }
}
TEST(PolylinePointIndex,
     PreservesEarliestTieAcrossBlocksAndHandlesAllNonfiniteDistances) {
  std::vector<Eigen::Vector2d> points(65, {1., 0.});
  points[1] = {-1., 0.};
  PolylinePointIndex index;
  index.build(points.size(), [&](size_t i) { return points[i]; });
  EXPECT_EQ(index.nearest(0, 0, 64).index, 0U);
  points.assign(33, {std::numeric_limits<double>::quiet_NaN(), 0.});
  index.build(points.size(), [&](size_t i) { return points[i]; });
  EXPECT_FALSE(index.nearest(0, 0).valid());
  EXPECT_EQ(index.nearest(0, 0).index, 0U);
  EXPECT_FALSE(
      index.nearest(std::numeric_limits<double>::infinity(), 0).valid());
}
} // namespace
} // namespace mas2027_nav_executor
