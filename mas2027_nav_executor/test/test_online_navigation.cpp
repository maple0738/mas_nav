#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "rog_map/query_adapter.hpp"

#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "mas2027_nav_executor/path_planner/search/route_commitment.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/local_path_processor.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/minco_optimizer.hpp"
#include "minco_core/components/planner_mode_context.hpp"

namespace {

class OnlineQuery final : public rog_map::MapQueryInterface
{
public:
  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.1; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  double snapshotStampSeconds() const override { return stamp; }
  bool worldToMap(double x, double y, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= 4 || y >= 4) {
      return false;
    }
    mx = static_cast<unsigned int>(x / resolution());
    my = static_cast<unsigned int>(y / resolution());
    return true;
  }
  void mapToWorld(unsigned int x, unsigned int y, double & wx, double & wy) const override
  { wx = (x + 0.5) * resolution(); wy = (y + 0.5) * resolution(); }
  uint8_t value(unsigned int x, unsigned int y) const override
  { return isValid(x, y) ? cells[y * sizeX() + x] : 254U; }
  const unsigned char * values() const override { return cells.data(); }
  bool isValid(unsigned int x, unsigned int y) const override
  { return x < sizeX() && y < sizeY(); }
  bool isFree(unsigned int x, unsigned int y) const override
  { return isValid(x, y) && value(x, y) < 253U; }
  bool evaluate(const Eigen::Vector3d & p, double & distance, Eigen::Vector3d & grad) const override
  {
    unsigned int x, y;
    if (!field_valid || !worldToMap(p.x(), p.y(), x, y)) return false;
    if (!first_pass_complete) {
      if (!first_pass_x.empty() && p.x() < first_pass_x.back() - 0.25) {
        first_pass_complete = true;
      } else {
        first_pass_x.push_back(p.x());
      }
    }
    // The observed region ends before the rolling window edge. Its unknown
    // border contributes to the ESDF, just as in the online projection profile.
    distance = std::min(clearance, observed_end - p.x());
    if (!cell_clearances.empty()) distance = std::min(distance, cell_clearances[y * sizeX() + x]);
    grad = {-1.0, 0.0, 0.0};
    if (diagonal_midpoint_hazard &&
        std::hypot(p.x() - 1.30, p.y() - 2.30) < 0.03) {
      distance = 0.225;
    }
    if (endpoint_cell_center_hazard &&
        std::hypot(p.x() - 1.55, p.y() - 2.55) < 0.015) {
      distance = 0.225;
    }
    if (narrow_hazard && p.x() > 0.70 && p.x() < 0.90) {
      distance = 0.20 + std::abs(p.y() - 2.05);
      grad = {0.0, p.y() >= 2.05 ? 1.0 : -1.0, 0.0};
    }
    if (corridor_walls && p.x() > 0.50 && p.x() < 1.70) {
      const double south = std::abs(p.y() - (2.0 - corridor_half_width));
      const double north = std::abs(p.y() - (2.0 + corridor_half_width));
      distance = corridor_single_wall ? south : std::min(south, north);
      grad = {0.0, corridor_single_wall || south < north ? 1.0 : -1.0, 0.0};
    }
    return true;
  }
  std::vector<unsigned char> cells = std::vector<unsigned char>(1600, 0U);
  std::vector<double> cell_clearances;
  double stamp{42.0};
  double clearance{1.0};
  double observed_end{4.0};
  bool field_valid{true};
  bool narrow_hazard{false};
  bool diagonal_midpoint_hazard{false};
  bool endpoint_cell_center_hazard{false};
  bool corridor_walls{false};
  bool corridor_single_wall{false};
  double corridor_half_width{0.25};
  mutable std::vector<double> first_pass_x;
  mutable bool first_pass_complete{false};
};

geometry_msgs::msg::PoseStamped pose(double x, double y = 2.05)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "odom";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

struct Search
{
  explicit Search(const std::shared_ptr<rog_map::MapQueryInterface> & query,
    bool use_smac = false) : astar(40, 40)
  {
    minco_planner::PlannerModeParams params;
    params.planner_mode = "EXPLORATION";
    params.rog_frame = "odom";
    params.exploration_unknown_as_occupied = true;
    context.configure(params, query, nullptr, rclcpp::get_logger("online_test"));
    smac.setParameters(false, 100000, 0.15F);
    smac.setMinimumClearance(0.27);
    search.configure(nullptr, &astar, use_smac ? &smac : nullptr, use_smac, false, 0.15,
      rclcpp::get_logger("online_test"));
    search.setQuery(query);
    search.setOnlineSubgoalClearance(0.33);
  }
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  minco_planner::Astar astar;
  minco_planner::PlannerModeContext context;
  minco_planner::GlobalPathSearcher search;
};

std::shared_ptr<OnlineQuery> wallWithUnknownEnds()
{
  auto q = std::make_shared<OnlineQuery>();
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 18; x < 40; ++x) {
      q->cells[y * 40 + x] = x <= 22 && y >= 8 && y <= 31 ? 254U : 255U;
    }
  }
  q->cell_clearances.assign(1600, 1.0);
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 0; x < 18; ++x) {
      for (unsigned int blocked_y = 0; blocked_y < 40; ++blocked_y) {
        for (unsigned int blocked_x = 18; blocked_x < 40; ++blocked_x) {
          q->cell_clearances[y * 40 + x] = std::min(
            q->cell_clearances[y * 40 + x],
            0.1 * std::hypot(static_cast<double>(x) - blocked_x,
              static_cast<double>(y) - blocked_y));
        }
      }
    }
  }
  return q;
}

std::shared_ptr<mas2027_nav_executor::OnlineGlobalMap> rememberedNorthRoute()
{
  auto remembered = std::make_shared<mas2027_nav_executor::OnlineGlobalMap>(
    mas2027_nav_executor::OnlineGlobalMapConfig{}, "odom");
  rog_map::MapSnapshot observation;
  observation.sequence = 1U;
  observation.stamp = 42.0;
  observation.width = 80;
  observation.height = 40;
  observation.resolution = 0.1;
  observation.types.assign(3200, 1U);
  observation.values.assign(3200, 0U);
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 80; ++x) {
      if ((x >= 18 && x <= 22 && y >= 8 && y <= 31) ||
          (x >= 35 && x <= 45 && y < 26)) {
        observation.types[y * 80 + x] = 3U;
        observation.values[y * 80 + x] = 254U;
      }
    }
  }
  remembered->update(observation);
  observation.sequence = 2U;
  observation.stamp = 42.2;
  remembered->update(observation);
  return remembered;
}

class OnlineNavigation : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(OnlineNavigation, PlansToObservedGoalWithoutStaticMapOrTF)
{
  auto q = std::make_shared<OnlineQuery>();
  Search s(q);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(s.search.plan(pose(0.55), pose(2.55), s.context, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_NEAR(path.back().pose.position.x, 2.55, 0.1);
  EXPECT_EQ(path.back().header.frame_id, "odom");
}

TEST_F(OnlineNavigation, ExploresSideOfWallWhenEverySafeStepInitiallyMovesAwayFromGoal)
{
  auto q = wallWithUnknownEnds();
  // The goal is outside the live window. The wall hides what is behind it,
  // while its upper and lower ends border unexplored space.
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  const auto start = pose(1.45, 2.05);
  const auto goal = pose(5.0, 2.05);
  ASSERT_TRUE(s.search.plan(start, goal, s.context, path));
  ASSERT_GE(path.size(), 2U);
  const auto & end = path.back().pose.position;
  EXPECT_LT(std::min(end.y, 4.0 - end.y), 1.4);
  EXPECT_GE(std::hypot(end.x - goal.pose.position.x,
    end.y - goal.pose.position.y), 3.15);
  for (const auto & point : path) {
    unsigned int x = 0, y = 0;
    ASSERT_TRUE(q->worldToMap(point.pose.position.x, point.pose.position.y, x, y));
    EXPECT_EQ(q->value(x, y), 0U);
    EXPECT_GE(q->query({point.pose.position.x, point.pose.position.y, 0.0}).distance, 0.27);
  }
  EXPECT_FALSE(s.search.lastGoalShifted());
}

TEST_F(OnlineNavigation, KeepsReachableExplorationSubgoalAcrossRollingReplans)
{
  auto q = wallWithUnknownEnds();
  Search s(q, true);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> first, second;
  ASSERT_TRUE(s.search.plan(pose(1.45, 1.65), goal, s.context, first));
  ASSERT_TRUE(s.search.plan(pose(1.45, 2.35), goal, s.context, second));
  ASSERT_GE(first.size(), 2U);
  ASSERT_GE(second.size(), 2U);
  EXPECT_NEAR(first.back().pose.position.x, second.back().pose.position.x, 0.11);
  EXPECT_NEAR(first.back().pose.position.y, second.back().pose.position.y, 0.11);

  Search uncommitted(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> opposite;
  ASSERT_TRUE(uncommitted.search.plan(pose(1.45, 2.35), goal,
    uncommitted.context, opposite));
  ASSERT_FALSE(opposite.empty());
  EXPECT_GT(std::abs(opposite.back().pose.position.y - first.back().pose.position.y), 1.0);

  // A new mission goal must release the earlier direction immediately.
  const auto changed_goal = pose(5.0, 2.25);
  std::vector<geometry_msgs::msg::PoseStamped> changed;
  ASSERT_TRUE(s.search.plan(pose(1.45, 2.35), changed_goal, s.context, changed));
  Search fresh(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> expected;
  ASSERT_TRUE(fresh.search.plan(pose(1.45, 2.35), changed_goal, fresh.context, expected));
  ASSERT_FALSE(expected.empty());
  ASSERT_FALSE(changed.empty());
  EXPECT_NEAR(changed.back().pose.position.x, expected.back().pose.position.x, 0.11);
  EXPECT_NEAR(changed.back().pose.position.y, expected.back().pose.position.y, 0.11);
}

TEST_F(OnlineNavigation, CompletedExplorationSubgoalIsNotImmediatelyRevisited)
{
  auto q = wallWithUnknownEnds();
  Search s(q, true);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> first, next;
  ASSERT_TRUE(s.search.plan(pose(1.45, 1.65), goal, s.context, first));
  ASSERT_GE(first.size(), 2U);
  const auto reached = first.back();
  ASSERT_TRUE(s.search.plan(reached, goal, s.context, next));
  ASSERT_GE(next.size(), 2U);
  EXPECT_GE(std::hypot(next.back().pose.position.x - reached.pose.position.x,
    next.back().pose.position.y - reached.pose.position.y), 0.80);
}

TEST_F(OnlineNavigation, StalledExplorationSubgoalReleasesItsDirection)
{
  auto q = wallWithUnknownEnds();
  Search s(q, true);
  const auto start = pose(1.45, 1.65);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> first, next;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, first));
  ASSERT_GE(first.size(), 2U);
  q->stamp += 9.0;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, next));
  ASSERT_GE(next.size(), 2U);
  EXPECT_GE(std::hypot(next.back().pose.position.x - first.back().pose.position.x,
    next.back().pose.position.y - first.back().pose.position.y), 0.80);
}

TEST_F(OnlineNavigation, BlockedExplorationSubgoalIsReplannedOnTheLiveMap)
{
  auto q = wallWithUnknownEnds();
  Search s(q, true);
  const auto start = pose(1.45, 1.65);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> first, next;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, first));
  ASSERT_GE(first.size(), 2U);
  unsigned int mx = 0U, my = 0U;
  ASSERT_TRUE(q->worldToMap(first.back().pose.position.x,
    first.back().pose.position.y, mx, my));
  q->cells[my * q->sizeX() + mx] = 254U;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, next));
  ASSERT_GE(next.size(), 2U);
  EXPECT_GT(std::hypot(next.back().pose.position.x - first.back().pose.position.x,
    next.back().pose.position.y - first.back().pose.position.y), 0.05);
  for (const auto & point : next) {
    ASSERT_TRUE(q->worldToMap(point.pose.position.x, point.pose.position.y, mx, my));
    EXPECT_TRUE(q->isFree(mx, my));
  }
}

TEST_F(OnlineNavigation, PreviouslyObservedSpaceIsNotAnExplorationFrontier)
{
  auto q = wallWithUnknownEnds();
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  const auto start = pose(1.45, 2.05);
  const auto goal = pose(5.0, 2.05);
  ASSERT_TRUE(s.search.plan(start, goal, s.context, path));

  // The live rolling window has forgotten the region behind the wall, while
  // the session map confirms that it was already observed. Revisit no longer
  // counts as information gain for a frontier candidate.
  auto remembered = std::make_shared<mas2027_nav_executor::OnlineGlobalMap>(
    mas2027_nav_executor::OnlineGlobalMapConfig{}, "odom");
  rog_map::MapSnapshot previous;
  previous.sequence = 1U;
  previous.stamp = 42.0;
  previous.width = previous.height = 40;
  previous.resolution = 0.1;
  previous.types.assign(1600, 1U);
  previous.values.assign(1600, 0U);
  ASSERT_EQ(remembered->update(previous),
    mas2027_nav_executor::OnlineGlobalMap::UpdateResult::UPDATED);
  Search with_history(q, true);
  with_history.search.setOnlineTerrainMap(remembered);
  path.clear();
  EXPECT_FALSE(with_history.search.plan(start, goal, with_history.context, path));
}

TEST_F(OnlineNavigation, RememberedRouteAvoidsLocallyAttractiveDeadEnd)
{
  auto live = wallWithUnknownEnds();
  const auto start = pose(1.45, 1.65);
  const auto goal = pose(5.0, 2.05);
  Search without_history(live, true);
  std::vector<geometry_msgs::msg::PoseStamped> unguided;
  ASSERT_TRUE(without_history.search.plan(start, goal, without_history.context, unguided));
  ASSERT_LT(unguided.back().pose.position.y, 1.5);

  // Both sides of the near wall look promising inside the live window. The
  // southern corridor is known to loop back behind a second wall; the route
  // around the north side remains connected to the goal in the session map.
  auto remembered = rememberedNorthRoute();

  const auto history = remembered->snapshot();
  ASSERT_TRUE(history);
  unsigned int hsx = 0U, hsy = 0U, hgx = 0U, hgy = 0U;
  ASSERT_TRUE(history->worldToMap(start.pose.position.x, start.pose.position.y, hsx, hsy));
  ASSERT_TRUE(history->worldToMap(goal.pose.position.x, goal.pose.position.y, hgx, hgy));
  ASSERT_TRUE(history->isFree(hsx, hsy));
  ASSERT_TRUE(history->isFree(hgx, hgy));
  EXPECT_GT(history->query({start.pose.position.x, start.pose.position.y, 0.0}).distance,
    0.27);
  EXPECT_GT(history->query({goal.pose.position.x, goal.pose.position.y, 0.0}).distance,
    0.27);
  mas2027_nav_executor::smac::SmacPlanner2DSimple route_search;
  route_search.configure(rclcpp::get_logger("history_test"));
  route_search.setParameters(false, 100000, 0.15F);
  route_search.setMinimumClearance(0.27);
  route_search.setMap(history);
  route_search.setESDFQuery(history);
  mas2027_nav_executor::smac::SmacPlanner2DSimple::CoordinateVector route;
  ASSERT_TRUE(route_search.createPath(hsx, hsy, hgx, hgy, route));
  ASSERT_FALSE(route.empty());

  Search guided(live, true);
  guided.search.setOnlineTerrainMap(remembered);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(guided.search.plan(start, goal, guided.context, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_GT(path.back().pose.position.y, 2.5);
  for (const auto & point : path) {
    unsigned int x = 0U, y = 0U;
    ASSERT_TRUE(live->worldToMap(point.pose.position.x, point.pose.position.y, x, y));
    EXPECT_EQ(live->value(x, y), 0U);
    EXPECT_GE(live->query({point.pose.position.x, point.pose.position.y, 0.0}).distance,
      0.27);
  }

  unsigned int blocked_x = 0U, blocked_y = 0U;
  ASSERT_TRUE(live->worldToMap(path.back().pose.position.x,
    path.back().pose.position.y, blocked_x, blocked_y));
  live->cells[blocked_y * live->sizeX() + blocked_x] = 254U;
  std::vector<geometry_msgs::msg::PoseStamped> replanned;
  ASSERT_TRUE(guided.search.plan(start, goal, guided.context, replanned));
  ASSERT_GE(replanned.size(), 2U);
  for (const auto & point : replanned) {
    unsigned int x = 0U, y = 0U;
    ASSERT_TRUE(live->worldToMap(point.pose.position.x, point.pose.position.y, x, y));
    EXPECT_EQ(live->value(x, y), 0U);
  }
}

TEST_F(OnlineNavigation, FailedExplorationAreaIsNotImmediatelySelectedAgain)
{
  auto live = wallWithUnknownEnds();
  Search s(live, true);
  const auto start = pose(1.45, 1.65);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> first, next;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, first));
  ASSERT_LT(first.back().pose.position.y, 1.5);
  unsigned int x = 0U, y = 0U;
  ASSERT_TRUE(live->worldToMap(first.back().pose.position.x,
    first.back().pose.position.y, x, y));
  live->cells[y * live->sizeX() + x] = 254U;
  live->stamp += 1.0;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, next));
  ASSERT_GE(next.size(), 2U);
  EXPECT_GT(next.back().pose.position.y, 2.5);

  // Once the exclusion expires and the other side is blocked, the original
  // area may be tried again if the live map now shows it as traversable.
  live->cells[y * live->sizeX() + x] = 0U;
  for (unsigned int north_y = 26U; north_y < 40U; ++north_y) {
    for (unsigned int north_x = 0U; north_x < 18U; ++north_x) {
      live->cells[north_y * live->sizeX() + north_x] = 254U;
    }
  }
  live->stamp += 31.0;
  std::vector<geometry_msgs::msg::PoseStamped> retried;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, retried));
  ASSERT_GE(retried.size(), 2U);
  EXPECT_LT(retried.back().pose.position.y, 1.5);
}

TEST_F(OnlineNavigation, BriefLossOfRememberedRouteDoesNotReverseDirection)
{
  auto live = wallWithUnknownEnds();
  Search s(live, true);
  s.search.setOnlineTerrainMap(rememberedNorthRoute());
  const auto start = pose(1.45, 1.65);
  const auto goal = pose(5.0, 2.05);
  std::vector<geometry_msgs::msg::PoseStamped> guided, retry;
  ASSERT_TRUE(s.search.plan(start, goal, s.context, guided));
  ASSERT_GT(guided.back().pose.position.y, 2.5);

  // The remembered route disappears for a short interval while a new live
  // obstacle blocks its next waypoint. The only currently safe frontier is
  // in the opposite direction; wait before reversing across the same area.
  s.search.setOnlineTerrainMap(nullptr);
  for (unsigned int y = 20U; y < 40U; ++y) {
    for (unsigned int x = 0U; x < 18U; ++x) {
      live->cells[y * live->sizeX() + x] = 254U;
    }
  }
  live->stamp += 1.0;
  EXPECT_FALSE(s.search.plan(start, goal, s.context, retry));
  live->stamp += 9.0;
  retry.clear();
  ASSERT_TRUE(s.search.plan(start, goal, s.context, retry));
  ASSERT_GE(retry.size(), 2U);
  EXPECT_LT(retry.back().pose.position.y, 1.5);
}

TEST_F(OnlineNavigation, DoesNotExploreTowardWallWithoutUnknownSpace)
{
  auto q = std::make_shared<OnlineQuery>();
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 18; x < 40; ++x) q->cells[y * 40 + x] = 254U;
  }
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(s.search.plan(pose(1.75, 2.05), pose(5.0, 2.05), s.context, path));
}

TEST_F(OnlineNavigation, ExploresUnknownCellsEncodedAsOccupiedInLiveProjection)
{
  auto snapshot = std::make_shared<rog_map::MapSnapshot>();
  snapshot->width = 40; snapshot->height = 40; snapshot->resolution = 0.1;
  snapshot->stamp = snapshot->field_stamp = 42.0;
  snapshot->values.assign(1600, 0U);
  snapshot->types.assign(1600, 1U);
  snapshot->distances.assign(1600, 1.0);
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 0; x < 40; ++x) {
      const size_t idx = y * 40 + x;
      if (x >= 18) {
        const bool wall = x <= 22 && y >= 8 && y <= 31;
        snapshot->types[idx] = wall ? 3U : 0U;
        snapshot->values[idx] = 254U;  // Both wall and unknown in online mode.
        snapshot->distances[idx] = -0.05;
      } else {
        snapshot->distances[idx] = std::min(1.0, 0.1 * (18 - x));
      }
    }
  }
  auto q = std::make_shared<rog_map::QueryAdapter>();
  q->update(snapshot, nullptr);
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(s.search.plan(pose(1.45, 2.05), pose(5.0, 2.05), s.context, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_LT(std::min(path.back().pose.position.y,
    4.0 - path.back().pose.position.y), 1.4);
  for (const auto & point : path) {
    unsigned int x = 0, y = 0;
    ASSERT_TRUE(q->worldToMap(point.pose.position.x, point.pose.position.y, x, y));
    EXPECT_EQ(snapshot->types[y * 40 + x], 1U);
    EXPECT_GE(q->query({point.pose.position.x, point.pose.position.y, 0.0}).distance, 0.27);
  }
}

TEST_F(OnlineNavigation, KeepsSafeCommittedRouteAcrossReverseMapDetour)
{
  const auto start = pose(1.0, 1.0);
  const std::vector<geometry_msgs::msg::PoseStamped> committed{
    pose(0.7, 1.0), start, pose(1.4, 1.0), pose(1.8, 1.0), pose(2.2, 1.0)};
  const std::vector<geometry_msgs::msg::PoseStamped> reverse{
    start, pose(0.6, 1.0), pose(0.6, 1.8), pose(2.2, 1.8), pose(2.2, 1.0)};
  const auto safe = [](double, double) { return true; };
  const auto kept = minco_planner::committedRouteForReversal(
    start, committed, reverse, 0.05, safe);
  ASSERT_TRUE(kept);
  ASSERT_GE(kept->size(), 2U);
  EXPECT_GT((*kept)[1].pose.position.x, start.pose.position.x);
  EXPECT_FALSE(minco_planner::committedRouteForReversal(
    start, committed, reverse, 0.05,
    [](double x, double) { return x < 1.5; }));
}

TEST_F(OnlineNavigation, AllowsShorterReverseRouteAndSameDirectionRefresh)
{
  const auto start = pose(1.0, 1.0);
  const std::vector<geometry_msgs::msg::PoseStamped> long_committed{
    start, pose(1.4, 1.0), pose(1.4, 2.0), pose(2.2, 2.0), pose(2.2, 1.0)};
  const std::vector<geometry_msgs::msg::PoseStamped> shorter_reverse{
    start, pose(0.6, 1.0), pose(2.2, 1.0)};
  EXPECT_FALSE(minco_planner::committedRouteForReversal(
    start, long_committed, shorter_reverse, 0.05,
    [](double, double) { return true; }));
  const std::vector<geometry_msgs::msg::PoseStamped> forward{
    start, pose(1.5, 1.0), pose(2.2, 1.0)};
  EXPECT_FALSE(minco_planner::committedRouteForReversal(
    start, long_committed, forward, 0.05,
    [](double, double) { return true; }));
}

TEST_F(OnlineNavigation, MincoSamplesNarrowOnlineHazardDuringOptimization)
{
  auto q = std::make_shared<OnlineQuery>();
  q->narrow_hazard = true;
  minco_planner::MincoOptimizer::Config config;
  config.max_vel = 2.0;
  config.max_acc = 2.0;
  config.integral_res = 2;  // Deliberately sparse configured baseline.
  config.print_optimizer_log = false;
  config.penaltyWeights = Eigen::VectorXd::Zero(5);
  config.penaltyWeights(0) = 1.0;
  config.magnitudeBounds.resize(3);
  config.magnitudeBounds << 0.3, 2.0, 2.0;
  minco_planner::MincoOptimizer optimizer(config);
  optimizer.setMap(q);

  Eigen::Matrix3d start = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d end = Eigen::Matrix3d::Zero();
  start.col(0) << 0.55, 2.05, 0.0;
  end.col(0) << 1.55, 2.05, 0.0;
  start.col(1).x() = 1.0;
  end.col(1).x() = 1.0;
  std::vector<Eigen::Vector3d> waypoints{start.col(0), end.col(0)};
  Eigen::VectorXd local_vmax(1);
  local_vmax << 1.0;
  traj_opt::Trajectory trajectory;
  optimizer.optimize(waypoints, start, end, local_vmax, trajectory);

  // The initial polynomial crosses a 0.2 m wide online feature between the
  // old three cost samples. The optimizer must see it before hard validation.
  ASSERT_GE(q->first_pass_x.size(), 10U);
  EXPECT_TRUE(std::any_of(q->first_pass_x.begin(), q->first_pass_x.end(),
    [](double x) { return x > 0.70 && x < 0.90; }));
}

TEST_F(OnlineNavigation, TightFreeSeedUsesWiderDetourWhenAvailable)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  for (unsigned int x = 8; x <= 12; ++x) {
    q->cell_clearances[20 * 40 + x] = 0.24;
  }
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.55), pose(1.55), pose(2.55)};

  const auto seed = processor.buildSeed(path, pose(0.55), s.context);
  ASSERT_TRUE(seed.valid);
  ASSERT_TRUE(seed.dense_reject.valid);
  EXPECT_TRUE(seed.dense_reject.clearance_only);
  ASSERT_TRUE(seed.used_clearance_detour);
  ASSERT_GE(seed.sparse_waypoints.size(), 2U);
  for (size_t i = 1; i < seed.sparse_waypoints.size(); ++i) {
    const auto &from = seed.sparse_waypoints[i - 1];
    const auto &to = seed.sparse_waypoints[i];
    const int count = std::max(1, static_cast<int>(std::ceil((to - from).head<2>().norm() / 0.02)));
    for (int j = 0; j <= count; ++j) {
      const auto point = from + (static_cast<double>(j) / count) * (to - from);
      if ((point - Eigen::Vector3d(0.55, 2.05, 0.0)).head<2>().norm() > 0.28) {
        EXPECT_GT(q->query(point).distance, 0.33);
      }
    }
  }

  processor.setClearanceDetourEnabled(false);
  const auto disabled_seed = processor.buildSeed(path, pose(0.55), s.context);
  EXPECT_TRUE(disabled_seed.valid);
  EXPECT_FALSE(disabled_seed.used_clearance_detour);
}

TEST_F(OnlineNavigation, CentersOnlyTwoSidedNarrowReferenceBeforeMinco)
{
  auto q = std::make_shared<OnlineQuery>();
  q->corridor_walls = true;
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  processor.setNarrowPassage(true, 0.24);
  processor.setClearanceDetourEnabled(false);
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.55, 1.98), pose(0.75, 1.98), pose(1.55, 1.98), pose(2.55, 2.00)};

  const auto seed = processor.buildSeed(path, pose(0.55, 1.98), s.context);
  ASSERT_TRUE(seed.valid);
  ASSERT_TRUE(seed.used_narrow_centering);
  bool reached_center = false;
  for (const auto &point : seed.dense_path) {
    if (point.x() > 1.0 && point.x() < 1.5 && point.y() >= 1.995) {
      reached_center = true;
      EXPECT_GE(q->query(point).distance, 0.24);
    }
  }
  EXPECT_TRUE(reached_center);
  EXPECT_NEAR(seed.dense_path.back().y(), 2.00, 1.0e-6);

  q->corridor_single_wall = true;
  const auto one_wall = processor.buildSeed(path, pose(0.55, 1.98), s.context);
  EXPECT_FALSE(one_wall.used_narrow_centering);

  q->corridor_single_wall = false;
  q->corridor_half_width = 0.35;
  const auto open_two_sides = processor.buildSeed(path, pose(0.55, 1.98), s.context);
  EXPECT_FALSE(open_two_sides.used_narrow_centering);
}

TEST_F(OnlineNavigation, RepairsADenseReferenceThatCutsInsideTheNarrowFloor)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  // Both path vertices are clear. Their straight chord crosses a 0.24 m
  // band, and the adjacent rows allow a floor-aware local detour.
  for (unsigned int x = 12; x <= 19; ++x) {
    q->cell_clearances[20 * 40 + x] = 0.24;
  }
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.42, 0.47,
                      rclcpp::get_logger("online_test"));
  processor.setNarrowPassage(true, 0.335);
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(1.05), pose(2.55)};

  const auto seed = processor.buildSeed(path, pose(1.05), s.context);
  ASSERT_TRUE(seed.valid);
  EXPECT_TRUE(seed.used_narrow_clearance_repair);
  ASSERT_GE(seed.dense_path.size(), 2U);
  bool detoured = false;
  for (const auto & point : seed.dense_path) {
    EXPECT_GT(q->query(point).distance + 0.001, 0.335);
    detoured = detoured || std::abs(point.y() - 2.05) > 0.05;
  }
  EXPECT_TRUE(detoured);
}

TEST_F(OnlineNavigation, FloorAwareSearchRoutesAroundUnsafeGridEdges)
{
  auto q = std::make_shared<OnlineQuery>();
  q->diagonal_midpoint_hazard = true;
  q->clearance = 0.25;
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  processor.setNarrowPassage(true, 0.24);
  processor.setClearanceDetourEnabled(false);
  // Cell centers on the diagonal are safe, but the edge between (1.25,2.25)
  // and (1.35,2.35) crosses a low-clearance pocket. A cell-only A* returns
  // that shortest route, then rejects it at the final segment check.
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(1.05, 2.05), pose(1.55, 2.55)};
  const auto seed = processor.buildSeed(path, pose(1.05, 2.05), s.context);
  ASSERT_TRUE(seed.valid);
  ASSERT_TRUE(seed.used_narrow_clearance_repair);
  for (size_t i = 1U; i < seed.dense_path.size(); ++i) {
    const auto &from = seed.dense_path[i - 1U];
    const auto &to = seed.dense_path[i];
    const int count = std::max(1, static_cast<int>(
      std::ceil((to - from).head<2>().norm() / 0.025)));
    for (int j = 0; j <= count; ++j) {
      const auto point = from + (static_cast<double>(j) / count) * (to - from);
      EXPECT_GE(q->query(point).distance + 1.0e-6, 0.24);
    }
  }
}

TEST_F(OnlineNavigation, FloorAwareSearchUsesTheActualLocalEndpoint)
{
  auto q = std::make_shared<OnlineQuery>();
  q->diagonal_midpoint_hazard = true;
  q->endpoint_cell_center_hazard = true;
  q->clearance = 0.25;
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  processor.setNarrowPassage(true, 0.24);
  processor.setClearanceDetourEnabled(false);
  // The endpoint itself is safe, while the center of its grid cell is not.
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(1.05, 2.05), pose(1.52, 2.52)};
  const auto seed = processor.buildSeed(path, pose(1.05, 2.05), s.context);
  ASSERT_TRUE(seed.valid);
  ASSERT_TRUE(seed.used_narrow_clearance_repair);
  EXPECT_NEAR(seed.dense_path.back().x(), 1.52, 1.0e-6);
  EXPECT_NEAR(seed.dense_path.back().y(), 2.52, 1.0e-6);
}

TEST_F(OnlineNavigation, FloorAwareRepairAcceptsCellsBetweenFloorAndFullThreshold)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 0.25);
  for (unsigned int x = 8; x <= 12; ++x) {
    q->cell_clearances[20 * 40 + x] = 0.23;
  }
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  processor.setNarrowPassage(true, 0.24);
  processor.setClearanceDetourEnabled(false);
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.55), pose(1.55), pose(2.55)};

  const auto seed = processor.buildSeed(path, pose(0.55), s.context);
  ASSERT_TRUE(seed.valid);
  EXPECT_TRUE(seed.used_narrow_clearance_repair);
  ASSERT_GE(seed.dense_path.size(), 2U);
  EXPECT_TRUE(std::any_of(seed.dense_path.begin(), seed.dense_path.end(),
    [](const Eigen::Vector3d & point) { return point.y() < 2.0; }));

  q->cell_clearances[20 * 40 + 5] = 0.23;
  const auto tight_start_seed = processor.buildSeed(path, pose(0.55), s.context);
  ASSERT_TRUE(tight_start_seed.valid);
  EXPECT_TRUE(tight_start_seed.used_narrow_clearance_repair);
}

TEST_F(OnlineNavigation, TightSeedKeepsOriginalRouteWhenWiderDetourIsImpossible)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  q->cell_clearances[20 * 40 + 25] = 0.24;
  Search s(q);
  minco_planner::LocalPathProcessor processor;
  processor.configure(3.0, 2.0, 2.0, 0.15, 0.28, 0.33,
                      rclcpp::get_logger("online_test"));
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.55), pose(1.55), pose(2.55)};

  const auto seed = processor.buildSeed(path, pose(0.55), s.context);
  ASSERT_TRUE(seed.valid);
  EXPECT_FALSE(seed.used_clearance_detour);
  EXPECT_NEAR(seed.dense_path.back().x(), 2.55, 1e-6);
}

TEST_F(OnlineNavigation, RemoteGoalUsesObservedSubgoalBeforeWindowBoundary)
{
  auto q = std::make_shared<OnlineQuery>();
  q->observed_end = 2.5;
  for (unsigned int y = 0; y < 40; ++y) {
    for (unsigned int x = 25; x < 40; ++x) q->cells[y * 40 + x] = 254U;
  }
  Search s(q);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
  EXPECT_GT(path.back().pose.position.x, 1.5);
  EXPECT_LE(path.back().pose.position.x, 2.17);
  for (const auto & p : path) {
    unsigned int x, y;
    ASSERT_TRUE(q->worldToMap(p.pose.position.x, p.pose.position.y, x, y));
    EXPECT_TRUE(q->isFree(x, y));
  }
}

TEST_F(OnlineNavigation, CornerCannotConnectDisconnectedFreeRegions)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cells.assign(1600, 254U);
  q->cells[20 * 40 + 5] = 0U;
  for (unsigned int x = 6; x < 35; ++x) q->cells[21 * 40 + x] = 0U;
  Search s(q);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
}

TEST_F(OnlineNavigation, SubgoalRequiresValidDistanceFieldAndClearance)
{
  auto q = std::make_shared<OnlineQuery>();
  Search s(q);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  q->clearance = 0.1;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
  q->clearance = 1.0;
  q->field_valid = false;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
}

TEST_F(OnlineNavigation, DisconnectedObservedGoalReportsFailureWithoutSmac)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cells.assign(1600, 254U);
  q->cells[20 * 40 + 5] = 0U;
  q->cells[20 * 40 + 25] = 0U;
  Search s(q);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(2.55), s.context, path));
}

TEST_F(OnlineNavigation, SubgoalRouteDetoursAroundLowClearanceFreeCells)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  // Grid costs are all free, but a low-clearance band crosses the straight route.
  for (unsigned int y = 10; y < 30; ++y) q->cell_clearances[y * 40 + 8] = 0.1;
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
  EXPECT_GT(path.back().pose.position.x, 1.5);
  for (const auto & p : path) {
    const auto field = q->query({p.pose.position.x, p.pose.position.y, 0.0});
    ASSERT_TRUE(field.ok);
    EXPECT_GE(field.distance, 0.27);
  }
}

TEST_F(OnlineNavigation, SubgoalCannotCrossLowClearanceBarrierToReachSafeEndpoint)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  for (unsigned int y = 0; y < 40; ++y) q->cell_clearances[y * 40 + 6] = 0.1;
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
}

TEST_F(OnlineNavigation, SubgoalCannotCutDiagonalAcrossLowClearanceSideCells)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  for (const auto & cell : std::vector<std::pair<unsigned int, unsigned int>>{
    {4, 20}, {6, 20}, {5, 19}, {5, 21}}) {
    q->cell_clearances[cell.second * 40 + cell.first] = 0.1;
  }
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
}

TEST_F(OnlineNavigation, ObservedLowClearanceGoalUsesSafeSubgoal)
{
  auto q = std::make_shared<OnlineQuery>();
  q->cell_clearances.assign(1600, 1.0);
  q->cell_clearances[20 * 40 + 25] = 0.1;
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(s.search.plan(pose(0.55), pose(2.55), s.context, path));
  for (const auto & p : path) {
    const auto field = q->query({p.pose.position.x, p.pose.position.y, 0.0});
    ASSERT_TRUE(field.ok);
    EXPECT_GE(field.distance, 0.27);
  }
}

TEST_F(OnlineNavigation, SubgoalRouteRejectsInvalidFieldAndCanLeaveTightStart)
{
  auto q = std::make_shared<OnlineQuery>();
  Search s(q, true);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  q->field_valid = false;
  EXPECT_FALSE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
  q->field_valid = true;
  q->cell_clearances.assign(1600, 1.0);
  q->cell_clearances[20 * 40 + 5] = 0.1;
  ASSERT_TRUE(s.search.plan(pose(0.55), pose(12.0), s.context, path));
  EXPECT_GT(path.back().pose.position.x, 1.5);
  // Only the initial occupied position is exempt, as in the direct SMAC search.
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & p = path[i].pose.position;
    EXPECT_GE(q->query({p.x, p.y, 0.0}).distance, 0.27);
  }
}

mas2027_nav_executor::ExecutorStatus command(
  const std::shared_ptr<OnlineQuery> & q, const std::shared_ptr<mas2027_nav_executor::TerrainGrid> & terrain = nullptr)
{
  minco_controller::State state{};
  state.x = 0.55;
  state.y = 2.05;
  minco_controller::Control control{};
  control.vx = 1.0;
  return mas2027_nav_executor::checkCommandSafety(terrain, q, nullptr, "odom",
    0.28, false, 0.24, false, 0.5, 0.05, state, control,
    rclcpp::Time(42, 0, RCL_SYSTEM_TIME));
}

TEST_F(OnlineNavigation, CommandUsesLiveClearanceWithoutStaticMapOrTF)
{
  auto q = std::make_shared<OnlineQuery>();
  EXPECT_EQ(command(q), mas2027_nav_executor::ExecutorStatus::PUBLISHED);
  q->observed_end = 0.85;
  EXPECT_EQ(command(q), mas2027_nav_executor::ExecutorStatus::DYNAMIC_BLOCKED);
}

TEST_F(OnlineNavigation, StaleAndInvalidOnlineMapsBrake)
{
  auto q = std::make_shared<OnlineQuery>();
  q->stamp = 40.0;
  EXPECT_EQ(command(q), mas2027_nav_executor::ExecutorStatus::DYNAMIC_BLOCKED);
  q->stamp = 42.0;
  q->field_valid = false;
  EXPECT_EQ(command(q), mas2027_nav_executor::ExecutorStatus::DYNAMIC_BLOCKED);
}

TEST_F(OnlineNavigation, PriorModeStillRequiresStaticSnapshot)
{
  EXPECT_EQ(command(std::make_shared<OnlineQuery>(),
    std::make_shared<mas2027_nav_executor::TerrainGrid>()),
    mas2027_nav_executor::ExecutorStatus::TERRAIN_BLOCKED);
}

}  // namespace
