#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "mas2027_nav_executor/path_planner/search/global_path_searcher.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "minco_core/components/planner_mode_context.hpp"
#include "rog_map/query_adapter.hpp"

namespace {

class GoalCellQuery final : public rog_map::MapQueryInterface
{
public:
  explicit GoalCellQuery(bool low_start = false) : low_start_(low_start) {}

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 2.0 || wy >= 2.0) {
      return false;
    }
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }

  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0U; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < sizeX() && my < sizeY(); }
  bool isFree(unsigned int mx, unsigned int my) const override
  { return isValid(mx, my); }

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    // The requested pose (0.752, 0.525) looks safe, while the cell center
    // (0.775, 0.525), where SMAC samples, is below the search threshold.
    result.distance = std::abs(pos.x() - 0.775) < 1e-6 &&
      std::abs(pos.y() - 0.525) < 1e-6 ? 0.22 : 0.40;
    if (low_start_ && std::hypot(pos.x() - 0.525, pos.y() - 0.525) < 0.04) {
      result.distance = 0.20;
    }
    return result;
  }

  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  bool low_start_{false};
  std::vector<unsigned char> costs_ = std::vector<unsigned char>(40U * 40U, 0U);
};

// A long, 0.25 m-clearance corridor. The primary 0.27 m SMAC search must fail;
// the relaxed 0.25 m retry is expected to find the route.
class NarrowCorridorQuery final : public rog_map::MapQueryInterface
{
public:
  NarrowCorridorQuery()
  {
    costs_.assign(40U * 40U, 254U);
    for (unsigned int my = 0; my < 40U; ++my) {
      for (unsigned int mx = 0; mx < 40U; ++mx) {
        double wx = 0.0;
        double wy = 0.0;
        mapToWorld(mx, my, wx, wy);
        if (wx >= 0.25 && wx <= 1.75 && std::abs(wy - 1.0) <= 0.15) {
          costs_[static_cast<size_t>(my) * 40U + mx] = 0U;
        }
      }
    }
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 2.0 || wy >= 2.0) return false;
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }
  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int mx, unsigned int my) const override
  { return costs_[static_cast<size_t>(my) * 40U + mx]; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < 40U && my < 40U; }
  bool isFree(unsigned int mx, unsigned int my) const override
  { return isValid(mx, my) && value(mx, my) < 100U; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    const bool inside_corridor = worldToMap(pos.x(), pos.y(), mx, my) &&
      isFree(mx, my);
    result.distance = inside_corridor ? 0.25 : 0.0;
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  std::vector<unsigned char> costs_;
};

// A wall-like blocked strip crosses the direct start→goal line, but free cells
// exist around the goal. The line approach cannot escape within max_shorten,
// so this exercises the radial goal ejection.
class BlockedGoalQuery final : public rog_map::MapQueryInterface
{
public:
  BlockedGoalQuery()
  {
    costs_.assign(60U * 60U, 0U);
    for (unsigned int my = 0U; my < 60U; ++my) {
      for (unsigned int mx = 0U; mx < 60U; ++mx) {
        double wx = 0.0;
        double wy = 0.0;
        mapToWorld(mx, my, wx, wy);
        if (wx >= 1.2 && wx <= 2.55 && wy >= 1.45 && wy <= 1.55) {
          costs_[static_cast<size_t>(my) * 60U + mx] = 254U;
        }
      }
    }
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 3.0 || wy >= 3.0) return false;
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }
  unsigned int sizeX() const override { return 60U; }
  unsigned int sizeY() const override { return 60U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int mx, unsigned int my) const override
  { return costs_[static_cast<size_t>(my) * 60U + mx]; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < 60U && my < 60U; }
  bool isFree(unsigned int mx, unsigned int my) const override
  { return isValid(mx, my) && value(mx, my) < 100U; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    result.distance = worldToMap(pos.x(), pos.y(), mx, my) && isFree(mx, my) ? 0.5 : 0.0;
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  std::vector<unsigned char> costs_;
};

class AheadNarrowQuery final : public rog_map::MapQueryInterface
{
public:
  AheadNarrowQuery(double open = 0.30, double narrow = 0.25, double off_route = 0.10)
  : open_(open), narrow_(narrow), off_route_(off_route) {}

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 2.0 || wy >= 2.0) return false;
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }
  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0U; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < 40U && my < 40U; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    // The route starts in 0.30 m open space and becomes 0.25 m wide after
    // x=0.2. A trajectory 0.3 m off the reference path sits in a 0.10 m gap.
    if (pos.x() >= 0.2 && std::abs(pos.y() - 1.0) <= 0.08) {
      result.distance = narrow_;
    } else if (pos.x() >= 0.2 && pos.y() >= 1.2) {
      result.distance = off_route_;
    } else {
      result.distance = open_;
    }
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  double open_{0.30};
  double narrow_{0.25};
  double off_route_{0.10};
  std::vector<unsigned char> costs_ = std::vector<unsigned char>(40U * 40U, 0U);
};


// Clearance is 0.255 m on the longitudinal approach and 0.25 m once the
// corridor is truly narrow. The full gate is 0.26 m, so the approach points
// can only pass when they are still associated with the planned narrow route.
class LongitudinalNarrowQuery final : public rog_map::MapQueryInterface
{
public:
  LongitudinalNarrowQuery(double clearance_from, double narrow_from)
  : clearance_from_(clearance_from), narrow_from_(narrow_from) {}

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 2.0 || wy >= 2.0) return false;
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }
  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int, unsigned int) const override { return 0U; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < 40U && my < 40U; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    if (std::abs(pos.y() - 1.03) <= 0.02) {
      // The executed trajectory runs 3 cm off the reference. It is still in
      // the same corridor, but its clearance is just below the full gate.
      if (pos.x() >= narrow_from_) {
        result.distance = 0.25;
      } else if (pos.x() >= clearance_from_) {
        result.distance = 0.255;
      } else {
        result.distance = 0.27;
      }
    } else if (std::abs(pos.y() - 1.0) <= 0.02) {
      if (pos.x() >= narrow_from_) {
        result.distance = 0.25;
      } else {
        result.distance = 0.27;
      }
    } else {
      result.distance = 0.10;
    }
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  double clearance_from_{0.0};
  double narrow_from_{1.0};
  std::vector<unsigned char> costs_ = std::vector<unsigned char>(40U * 40U, 0U);
};

// Start area is free; everything else, including the goal and its surrounding
// approach ring, is blocked. This must be a terminal goal rejection, not an
// invitation to explore toward the wall.
class BlockedGoalNoEjectionQuery final : public rog_map::MapQueryInterface
{
public:
  BlockedGoalNoEjectionQuery()
  {
    costs_.assign(40U * 40U, 254U);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        costs_[static_cast<size_t>(10 + dy) * 40U + 10 + dx] = 0U;
      }
    }
  }
  void clearObstacle() { costs_.assign(40U * 40U, 0U); }
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0 ||
      wx >= 2.0 || wy >= 2.0) return false;
    mx = static_cast<unsigned int>(wx / 0.05);
    my = static_cast<unsigned int>(wy / 0.05);
    return true;
  }
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * 0.05;
    wy = (static_cast<double>(my) + 0.5) * 0.05;
  }
  unsigned int sizeX() const override { return 40U; }
  unsigned int sizeY() const override { return 40U; }
  double resolution() const override { return 0.05; }
  double originX() const override { return 0.0; }
  double originY() const override { return 0.0; }
  uint8_t value(unsigned int mx, unsigned int my) const override
  { return costs_[static_cast<size_t>(my) * 40U + mx]; }
  const unsigned char * values() const override { return costs_.data(); }
  bool isValid(unsigned int mx, unsigned int my) const override
  { return mx < 40U && my < 40U; }
  bool isFree(unsigned int mx, unsigned int my) const override
  { return isValid(mx, my) && value(mx, my) < 100U; }
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    result.distance = worldToMap(pos.x(), pos.y(), mx, my) && isFree(mx, my) ? 0.5 : -0.5;
    return result;
  }
  bool evaluate(const Eigen::Vector3d & pos, double & dist,
    Eigen::Vector3d & grad) const override
  {
    dist = query(pos).distance;
    grad.setZero();
    return true;
  }

private:
  std::vector<unsigned char> costs_;
};

TEST(GlobalGoalCellClearance, PathAwareNarrowGateAllowsOnlyPlannedCorridor)
{
  rclcpp::init(0, nullptr);
  auto query = std::make_shared<AheadNarrowQuery>();
  minco_planner::TrajectorySafetyChecker checker;
  checker.configure(0.05, rclcpp::get_logger("test_path_aware_narrow_gate"));
  checker.setQuery(query);
  checker.setNarrowPassage(true, 0.24);

  // Straight polynomial: x=t, y=1, duration 1 s.
  Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
  coeff(0, 4) = 1.0;
  coeff(1, 5) = 1.0;
  const traj_opt::Trajectory trajectory({1.0}, {coeff});
  std::vector<Eigen::Vector3d> reference_path;
  for (int i = 0; i <= 20; ++i) {
    reference_path.emplace_back(0.05 * i, 1.0, 0.0);
  }

  EXPECT_TRUE(checker.checkTrajectory(trajectory, 0.26, 0.28, reference_path));
  // A single live-map outlier on an older reference must not revoke the
  // narrow allowance from nearby safe samples. The trajectory is still
  // checked against the current map at every point.
  auto noisy_reference = reference_path;
  noisy_reference.emplace_back(0.5, 1.3, 0.0);
  EXPECT_TRUE(checker.checkTrajectory(trajectory, 0.26, 0.28, noisy_reference));
  // Without the reference route the same trajectory is rejected at 0.25 m.
  EXPECT_FALSE(checker.checkTrajectory(trajectory, 0.26, 0.28));

  // A trajectory 0.3 m off the route must not inherit the corridor floor.
  Eigen::MatrixXd offset_coeff = coeff;
  offset_coeff(1, 5) = 1.3;
  const traj_opt::Trajectory offset_trajectory({1.0}, {offset_coeff});
  EXPECT_FALSE(checker.checkTrajectory(offset_trajectory, 0.26, 0.28, reference_path));
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, NarrowGateIncludesTheApproachToTheFirstTightSample)
{
  rclcpp::init(0, nullptr);
  Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
  coeff(0, 4) = 1.4;
  coeff(1, 5) = 1.03;
  const traj_opt::Trajectory trajectory({1.0}, {coeff});
  std::vector<Eigen::Vector3d> reference_path;
  for (int i = 0; i <= 28; ++i) {
    reference_path.emplace_back(0.05 * i, 1.0, 0.0);
  }

  // The low-clearance band starts 0.40 m before the first tight sample, so the
  // approach samples belong to the same planned narrow corridor.
  auto near_query = std::make_shared<LongitudinalNarrowQuery>(0.80, 1.20);
  minco_planner::TrajectorySafetyChecker near_checker;
  near_checker.configure(0.05, rclcpp::get_logger("test_narrow_approach_gate"));
  near_checker.setQuery(near_query);
  near_checker.setNarrowPassage(true, 0.24);
  EXPECT_TRUE(near_checker.checkTrajectory(trajectory, 0.26, 0.28, reference_path));

  // Starting the same 0.255 m band 0.60 m before the first tight sample is too
  // far to inherit its floor, so the full 0.26 m gate must still reject it.
  auto far_query = std::make_shared<LongitudinalNarrowQuery>(0.60, 1.20);
  minco_planner::TrajectorySafetyChecker far_checker;
  far_checker.configure(0.05, rclcpp::get_logger("test_narrow_approach_far_gate"));
  far_checker.setQuery(far_query);
  far_checker.setNarrowPassage(true, 0.24);
  EXPECT_FALSE(far_checker.checkTrajectory(trajectory, 0.26, 0.28, reference_path));

  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, AcceptsClearanceExactlyAtThreshold)
{
  rclcpp::init(0, nullptr);
  auto query = std::make_shared<AheadNarrowQuery>(0.26, 0.26, 0.26);
  minco_planner::TrajectorySafetyChecker checker;
  checker.configure(0.05, rclcpp::get_logger("test_exact_clearance"));
  checker.setQuery(query);
  checker.setNarrowPassage(false, 0.0);

  Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
  coeff(0, 4) = 1.0;
  coeff(1, 5) = 1.0;
  const traj_opt::Trajectory trajectory({1.0}, {coeff});
  EXPECT_TRUE(checker.checkTrajectory(trajectory, 0.26, 0.28));
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, UsesCellCenterBeforeCallingSmac)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_global_goal_cell_clearance");
  auto query = std::make_shared<GoalCellQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, query, nullptr, logger);

  minco_planner::Astar astar(40U, 40U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.5);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 0.525;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 0.752;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(searcher.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  for (const auto & pose : path) {
    EXPECT_GE(query->query({pose.pose.position.x, pose.pose.position.y, 0.0}).distance, 0.27);
  }
  EXPECT_LT(path.back().pose.position.x, 0.775);
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, EjectsStartFromLowClearanceCell)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_global_start_clearance");
  auto query = std::make_shared<GoalCellQuery>(true);
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, query, nullptr, logger);

  minco_planner::Astar astar(40U, 40U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.0);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 0.525;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 1.5;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(searcher.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  const auto & first = path.front().pose.position;
  EXPECT_GE(query->query({first.x, first.y, 0.0}).distance, 0.27);
  EXPECT_GT(std::hypot(first.x - start.pose.position.x, first.y - start.pose.position.y), 0.0);
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, RelaxedSearchUsesNarrowCorridor)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_relaxed_narrow_search");
  auto query = std::make_shared<NarrowCorridorQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, query, nullptr, logger);

  minco_planner::Astar astar(40U, 40U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.5);
  searcher.setRelaxedMinimumClearance(0.25);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 1.025;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 1.525;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(searcher.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  for (const auto & pose : path) {
    EXPECT_GE(query->query({pose.pose.position.x, pose.pose.position.y, 0.0}).distance, 0.25);
  }
  EXPECT_NEAR(path.back().pose.position.x, goal.pose.position.x, 0.05);
  EXPECT_NEAR(smac.minimumClearance(), 0.27, 1.0e-6);
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, EjectsGoalAroundBlockedDirectLine)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_goal_ejection");
  auto query = std::make_shared<BlockedGoalQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, query, nullptr, logger);

  minco_planner::Astar astar(60U, 60U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(1.0);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 1.525;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 2.525;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(searcher.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  const auto & last = path.back().pose.position;
  EXPECT_GT(std::hypot(last.x - goal.pose.position.x, last.y - goal.pose.position.y), 0.02);
  EXPECT_LE(std::hypot(last.x - goal.pose.position.x, last.y - goal.pose.position.y), 1.01);
  EXPECT_TRUE(query->isFree(
    static_cast<unsigned int>(last.x / 0.05),
    static_cast<unsigned int>(last.y / 0.05)));
  EXPECT_TRUE(searcher.lastGoalShifted());
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, RetriesTransientGoalAndRejectsOnlyConfirmedObstacle)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_blocked_goal_no_ejection");
  auto query = std::make_shared<BlockedGoalNoEjectionQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, query, nullptr, logger);

  minco_planner::Astar astar(40U, 40U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.5);
  searcher.setRelaxedMinimumClearance(0.25);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 0.525;
  start.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped goal = start;
  goal.pose.position.x = 1.30;
  goal.pose.position.y = 1.30;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  EXPECT_FALSE(searcher.plan(start, goal, context, path));
  EXPECT_FALSE(searcher.lastGoalInvalid());
  EXPECT_TRUE(searcher.lastGoalBlockedRetryable());

  mas2027_nav_executor::OnlineGlobalMapConfig map_config;
  map_config.resolution = 0.05;
  map_config.occupied_confirmations = 2;
  auto remembered = std::make_shared<mas2027_nav_executor::OnlineGlobalMap>(map_config, "odom");
  searcher.setOnlineTerrainMap(remembered);
  rog_map::MapSnapshot snapshot;
  snapshot.width = snapshot.height = 40;
  snapshot.resolution = 0.05;
  snapshot.values.resize(1600U);
  snapshot.types.resize(1600U);
  for (unsigned int y = 0; y < 40U; ++y) {
    for (unsigned int x = 0; x < 40U; ++x) {
      const size_t index = static_cast<size_t>(y) * 40U + x;
      snapshot.values[index] = query->value(x, y);
      snapshot.types[index] = snapshot.values[index] >= 253U ? 3U : 1U;
    }
  }
  snapshot.sequence = 1;
  snapshot.stamp = 10.0;
  ASSERT_EQ(remembered->update(snapshot),
    mas2027_nav_executor::OnlineGlobalMap::UpdateResult::UPDATED);
  EXPECT_FALSE(searcher.plan(start, goal, context, path));
  EXPECT_TRUE(searcher.lastGoalBlockedRetryable());
  EXPECT_FALSE(searcher.lastGoalInvalid());

  snapshot.sequence = 2;
  snapshot.stamp = 10.5;
  ASSERT_EQ(remembered->update(snapshot),
    mas2027_nav_executor::OnlineGlobalMap::UpdateResult::UPDATED);
  EXPECT_FALSE(searcher.plan(start, goal, context, path));
  EXPECT_TRUE(searcher.lastGoalInvalid());
  EXPECT_FALSE(searcher.lastGoalBlockedRetryable());

  // The same obstacle at the live-window edge is not terminally invalid:
  // ejection candidates outside the current window have not been observed.
  goal.pose.position.x = 1.525;
  goal.pose.position.y = 1.525;
  (void)searcher.plan(start, goal, context, path);
  EXPECT_FALSE(searcher.lastGoalInvalid());
  EXPECT_FALSE(searcher.lastGoalBlockedRetryable());
  EXPECT_FALSE(searcher.lastGoalShifted());

  // A later live observation clears the temporary obstacle. Planning can
  // resume toward the same mission goal even while B1 still holds old evidence.
  query->clearObstacle();
  goal.pose.position.x = 1.30;
  goal.pose.position.y = 1.30;
  EXPECT_TRUE(searcher.plan(start, goal, context, path));
  EXPECT_FALSE(searcher.lastGoalInvalid());
  EXPECT_FALSE(searcher.lastGoalBlockedRetryable());
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, AdmissionRejectsOnlyConfirmedInteriorObstacle)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_goal_admission");
  auto live = std::make_shared<BlockedGoalNoEjectionQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, live, nullptr, logger);

  minco_planner::Astar astar(40U, 40U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.5);
  searcher.setRelaxedMinimumClearance(0.25);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 0.525;
  start.pose.orientation.w = 1.0;
  auto goal = start;
  goal.pose.position.x = 1.30;
  goal.pose.position.y = 1.30;
  // Without persistent confirmation, live occupancy may be transient.
  EXPECT_FALSE(searcher.goalKnownInvalid(start, goal, context));

  mas2027_nav_executor::OnlineGlobalMapConfig map_config;
  map_config.resolution = 0.05;
  map_config.occupied_confirmations = 1;
  auto remembered = std::make_shared<mas2027_nav_executor::OnlineGlobalMap>(map_config, "odom");
  rog_map::MapSnapshot snapshot;
  snapshot.sequence = 1;
  snapshot.stamp = 10.0;
  snapshot.width = snapshot.height = 40;
  snapshot.resolution = 0.05;
  snapshot.values.resize(1600U);
  snapshot.types.resize(1600U);
  for (unsigned int y = 0; y < 40U; ++y) {
    for (unsigned int x = 0; x < 40U; ++x) {
      const size_t index = static_cast<size_t>(y) * 40U + x;
      snapshot.values[index] = live->value(x, y);
      snapshot.types[index] = snapshot.values[index] >= 253U ? 3U : 1U;
    }
  }
  ASSERT_EQ(remembered->update(snapshot),
    mas2027_nav_executor::OnlineGlobalMap::UpdateResult::UPDATED);
  const auto remembered_snapshot = remembered->snapshot();
  ASSERT_TRUE(remembered_snapshot);
  unsigned int hx = 0U, hy = 0U;
  ASSERT_TRUE(remembered_snapshot->worldToMap(
    goal.pose.position.x, goal.pose.position.y, hx, hy));
  EXPECT_EQ(remembered_snapshot->value(hx, hy), 254U);
  EXPECT_GT(live->sizeX() * live->resolution() - goal.pose.position.x + live->originX(),
    0.5 + live->resolution());
  searcher.setOnlineTerrainMap(remembered);
  EXPECT_TRUE(searcher.goalKnownInvalid(start, goal, context));

  // A live-window edge cannot establish that no ejection exists outside it.
  goal.pose.position.x = 1.525;
  goal.pose.position.y = 1.525;
  EXPECT_FALSE(searcher.goalKnownInvalid(start, goal, context));
  // A goal outside the rolling window is not an occupied goal either.
  goal.pose.position.x = 3.0;
  EXPECT_FALSE(searcher.goalKnownInvalid(start, goal, context));
  rclcpp::shutdown();
}

TEST(GlobalGoalCellClearance, RememberedFreeRemoteGoalDoesNotBecomeEjectedEndpoint)
{
  rclcpp::init(0, nullptr);
  const auto logger = rclcpp::get_logger("test_remembered_free_goal");
  auto live = std::make_shared<BlockedGoalQuery>();
  minco_planner::PlannerModeContext context;
  minco_planner::PlannerModeParams params;
  params.rog_frame = "odom";
  context.configure(params, live, nullptr, logger);

  mas2027_nav_executor::OnlineGlobalMapConfig map_config;
  map_config.resolution = 0.05;
  auto remembered = std::make_shared<mas2027_nav_executor::OnlineGlobalMap>(map_config, "odom");
  rog_map::MapSnapshot snapshot;
  snapshot.sequence = 1;
  snapshot.stamp = 10.0;
  snapshot.width = snapshot.height = 60;
  snapshot.resolution = 0.05;
  snapshot.values.assign(3600U, 0U);
  snapshot.types.assign(3600U, 1U);
  ASSERT_EQ(remembered->update(snapshot),
    mas2027_nav_executor::OnlineGlobalMap::UpdateResult::UPDATED);

  minco_planner::Astar astar(60U, 60U);
  mas2027_nav_executor::smac::SmacPlanner2DSimple smac;
  smac.configure(logger);
  smac.setParameters(false, 1000000, 0.1F);
  smac.setMinimumClearance(0.27);
  minco_planner::GlobalPathSearcher searcher;
  searcher.configure(nullptr, &astar, &smac, true, false, 0.1, logger);
  searcher.setGoalApproachMaxShorten(0.5);
  searcher.setOnlineTerrainMap(remembered);

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "odom";
  start.pose.position.x = 0.525;
  start.pose.position.y = 1.525;
  start.pose.orientation.w = 1.0;
  auto goal = start;
  goal.pose.position.x = 2.025;
  std::vector<geometry_msgs::msg::PoseStamped> path;
  ASSERT_TRUE(searcher.plan(start, goal, context, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_FALSE(searcher.lastGoalShifted());
  const auto endpoint = path.back().pose.position;
  unsigned int ex = 0U, ey = 0U;
  ASSERT_TRUE(live->worldToMap(endpoint.x, endpoint.y, ex, ey));
  EXPECT_TRUE(live->isFree(ex, ey));
  rclcpp::shutdown();
}

}  // namespace
