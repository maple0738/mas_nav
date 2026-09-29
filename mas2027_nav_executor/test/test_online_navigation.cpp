#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"
#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
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
    // The observed region ends before the rolling window edge. Its unknown
    // border contributes to the ESDF, just as in the online projection profile.
    distance = std::min(clearance, observed_end - p.x());
    if (!cell_clearances.empty()) distance = std::min(distance, cell_clearances[y * sizeX() + x]);
    grad = {-1.0, 0.0, 0.0};
    return true;
  }
  std::vector<unsigned char> cells = std::vector<unsigned char>(1600, 0U);
  std::vector<double> cell_clearances;
  double stamp{42.0};
  double clearance{1.0};
  double observed_end{4.0};
  bool field_valid{true};
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
  explicit Search(const std::shared_ptr<OnlineQuery> & query, bool use_smac = false) : astar(40, 40)
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
    0.28, 0.5, 0.05, state, control, rclcpp::Time(42, 0, RCL_SYSTEM_TIME));
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
