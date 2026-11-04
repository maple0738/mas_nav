#include "mas2027_nav_executor/path_executor/path_executor.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/trajectory_safety_checker.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rcutils/logging.h>

namespace {
class Query final : public rog_map::MapQueryInterface {
public:
  bool narrow{false};
  mutable uint64_t queries{0};
  bool worldToMap(double, double, unsigned int &x,
                  unsigned int &y) const override {
    x = y = 0;
    return true;
  }
  void mapToWorld(unsigned int, unsigned int, double &x,
                  double &y) const override {
    x = y = 0;
  }
  unsigned int sizeX() const override { return 1; }
  unsigned int sizeY() const override { return 1; }
  double resolution() const override { return .05; }
  double originX() const override { return 0; }
  double originY() const override { return 0; }
  double snapshotStampSeconds() const override { return 10.; }
  uint8_t value(unsigned int, unsigned int) const override { return 0; }
  const unsigned char *values() const override {
    static unsigned char v = 0;
    return &v;
  }
  bool isValid(unsigned int, unsigned int) const override { return true; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d &p) const override {
    ++queries;
    rog_map::QueryResult q;
    q.ok = true;
    q.status = rog_map::QueryStatus::OK;
    q.distance = narrow ? (p.x() > 2. && p.x() < 2.25 ? .25 : .8) : 10.;
    q.gradient.setZero();
    return q;
  }
  bool evaluate(const Eigen::Vector3d &p, double &d,
                Eigen::Vector3d &g) const override {
    const auto q = query(p);
    d = q.distance;
    g = q.gradient;
    return q.ok;
  }
};
using Clock = std::chrono::steady_clock;
void report(const std::string &name, std::vector<double> times) {
  std::sort(times.begin(), times.end());
  std::cout << name << ',' << std::fixed << std::setprecision(3)
            << (times[(times.size() - 1) / 2] + times[times.size() / 2]) / 2
            << ','
            << times[static_cast<size_t>(std::ceil(.95 * times.size())) - 1]
            << '\n';
}
} // namespace
int main(int argc, char **argv) {
  rclcpp::init(0, nullptr);
  if (rcutils_logging_set_logger_level(
          "trajectory_benchmark", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK)
    return 4;
  std::ofstream states;
  if (argc > 1) {
    states.open(argv[1]);
    if (!states)
      return 1;
  }
  states << std::setprecision(17);
  const auto record = [&](const std::string &scene, const std::string &key,
                          double value) {
    states << scene << ',' << key << ',' << value << '\n';
  };
  std::cout << "scene,median_us,p95_us\n";
  for (int count : {64, 1024, 4096}) {
    auto map = std::make_shared<Query>();
    minco_controller::MPCConfig config;
    config.horizon = 40;
    config.use_acc_constraints = true;
    config.vx_min = config.vy_min = -3.;
    config.vx_max = config.vy_max = 3.;
    mas2027_nav_executor::PathExecutorParams params;
    params.planner_frequency = 20;
    params.trajectory_timeout_s = .6;
    params.odom_timeout_s = .5;
    params.rog_map_timeout_s = .5;
    params.rog_map_clearance = .28;
    params.odom_frame = "odom";
    params.deadzone_speed = .02;
    mas2027_nav_executor::PathExecutor executor(config, params, nullptr,
                                                nullptr, map);
    auto trajectory = std::make_shared<interfaces::msg::MpcPositionCommand>();
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    const rclcpp::Time now(10, 0, RCL_ROS_TIME);
    trajectory->header.stamp = now;
    trajectory->header.frame_id = "odom";
    odom->header.stamp = now;
    odom->pose.pose.orientation.w = 1.;
    for (int i = 0; i < count; ++i) {
      const double t = i * .05;
      interfaces::msg::PositionCommand p;
      p.position.x = t;
      p.position.y = .1 * std::sin(t);
      p.velocity.x = 1.;
      p.velocity.y = .1 * std::cos(t);
      p.acceleration.y = -.1 * std::sin(t);
      p.jerk.y = -.1 * std::cos(t);
      trajectory->cmds.push_back(p);
    }
    mas2027_nav_executor::ExecutorInput input;
    input.trajectory = trajectory;
    input.odom = odom;
    input.stamp = now;
    input.allow_motion = true;
    input.follow_gimbal = true;
    std::vector<double> times;
    const std::string scene = "control_" + std::to_string(count);
    for (int i = 0; i < 25; ++i) {
      odom->pose.pose.position.x = .02 * i;
      map->queries = 0;
      const auto start = Clock::now();
      const auto out = executor.computeCommand(input);
      const double us =
          std::chrono::duration<double, std::micro>(Clock::now() - start)
              .count();
      if (out.status != mas2027_nav_executor::ExecutorStatus::PUBLISHED)
        return 2;
      if (i >= 5)
        times.push_back(us);
      const std::string prefix = std::to_string(i) + "_";
      record(scene, prefix + "vx", out.command.linear.x);
      record(scene, prefix + "vy", out.command.linear.y);
      record(scene, prefix + "omega", out.command.angular.z);
      record(scene, prefix + "queries", map->queries);
      record(scene, prefix + "remaining", out.goal_approach_remaining_m);
      record(scene, prefix + "allowed", out.goal_approach_allowed_speed);
      record(scene, prefix + "capped", out.goal_approach_capped);
    }
    report(scene, times);
  }
  for (int count : {64, 1024, 4096}) {
    auto map = std::make_shared<Query>();
    map->narrow = true;
    minco_planner::TrajectorySafetyChecker checker;
    checker.configure(.01, rclcpp::get_logger("trajectory_benchmark"));
    checker.setQuery(map);
    checker.setNarrowPassage(true, .24);
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
    coeff(0, 4) = 1.;
    const traj_opt::Trajectory trajectory({5.}, {coeff});
    std::vector<Eigen::Vector3d> reference;
    for (int i = 0; i < count; ++i)
      reference.emplace_back(5. * i / (count - 1), 0., 0.);
    std::vector<double> times;
    const std::string scene = "safety_" + std::to_string(count);
    for (int i = 0; i < 15; ++i) {
      map->queries = 0;
      const auto start = Clock::now();
      const bool safe =
          checker.checkTrajectory(trajectory, .26, .28, reference);
      const double us =
          std::chrono::duration<double, std::micro>(Clock::now() - start)
              .count();
      if (!safe)
        return 3;
      if (i >= 3)
        times.push_back(us);
      record(scene, std::to_string(i) + "_safe", safe);
      record(scene, std::to_string(i) + "_queries", map->queries);
    }
    report(scene, times);
  }
  rclcpp::shutdown();
}
