#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"
#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
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
    if (narrow_hazard && p.x() > 0.70 && p.x() < 0.90) {
      distance = 0.20 + std::abs(p.y() - 2.05);
      grad = {0.0, p.y() >= 2.05 ? 1.0 : -1.0, 0.0};
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
