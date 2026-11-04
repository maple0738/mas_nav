#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/minco_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

namespace {
class FreeMap final : public rog_map::MapQueryInterface {
public:
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
  uint8_t value(unsigned int, unsigned int) const override { return 0; }
  const unsigned char *values() const override {
    static unsigned char v = 0;
    return &v;
  }
  bool isValid(unsigned int, unsigned int) const override { return true; }
  bool isFree(unsigned int, unsigned int) const override { return true; }
  rog_map::QueryResult query(const Eigen::Vector3d &) const override {
    rog_map::QueryResult q;
    q.ok = true;
    q.status = rog_map::QueryStatus::OK;
    q.distance = .8;
    q.gradient.setZero();
    return q;
  }
  bool evaluate(const Eigen::Vector3d &, double &d,
                Eigen::Vector3d &g) const override {
    d = .8;
    g.setZero();
    return true;
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

// End-to-end solver timings with deterministic output samples for A/B checking.
int main(int argc, char **argv) {
  std::ofstream results;
  if (argc > 1) {
    results.open(argv[1]);
    if (!results)
      return 1;
  }
  results << std::setprecision(17);
  const auto record = [&](const std::string &scene, const std::string &key,
                          double value) {
    results << scene << ',' << key << ',' << value << '\n';
  };
  std::cout << "scene,median_us,p95_us\n";
  for (int scenario = 0; scenario < 4; ++scenario) {
    minco_controller::MPCConfig cfg;
    cfg.horizon = scenario < 2 ? 10 : 40;
    cfg.use_acc_constraints = scenario % 2;
    cfg.vx_max = cfg.vy_max = 3;
    cfg.vx_min = cfg.vy_min = -3;
    minco_controller::MpcSolver solver(cfg);
    std::vector<minco_controller::ReferencePoint> refs(cfg.horizon);
    for (int k = 0; k < cfg.horizon; ++k) {
      const double t = (k + 1) * cfg.dt;
      refs[k].pos = {.4 + t, .2 + .15 * std::sin(t)};
      refs[k].vel = {1, .15 * std::cos(t)};
      refs[k].yaw = -3.12 + .1 * t;
      refs[k].yaw_rate = .1;
    }
    const std::string scene = "mpc_n" + std::to_string(cfg.horizon) +
                              (cfg.use_acc_constraints ? "_acc" : "_box");
    minco_controller::State state;
    state.x = .4;
    state.y = .2;
    state.yaw = 3.13;
    minco_controller::Control control;
    std::vector<minco_controller::State> prediction;
    std::vector<double> times;
    for (int i = 0; i < 35; ++i) {
      solver.setConfig(cfg);
      solver.setLastControl({.2, -.1, .05});
      const auto start = Clock::now();
      if (!solver.solve(state, refs, control, &prediction))
        return 2;
      const double us =
          std::chrono::duration<double, std::micro>(Clock::now() - start)
              .count();
      if (i >= 5)
        times.push_back(us);
    }
    report(scene, times);
    record(scene, "vx", control.vx);
    record(scene, "vy", control.vy);
    record(scene, "omega", control.omega);
    for (size_t k = 0; k < prediction.size(); ++k) {
      record(scene, "x" + std::to_string(k), prediction[k].x);
      record(scene, "y" + std::to_string(k), prediction[k].y);
      record(scene, "yaw" + std::to_string(k), prediction[k].yaw);
    }
  }
  for (int scenario = 0; scenario < 4; ++scenario) {
    const int pieces = scenario == 0 ? 2 : 6;
    minco_planner::MincoOptimizer::Config cfg;
    cfg.max_vel = 1;
    cfg.max_acc = 2;
    cfg.rho = 100;
    cfg.integral_res = 8;
    cfg.opt_accuracy = 1e-3;
    cfg.print_optimizer_log = false;
    cfg.magnitudeBounds.resize(3);
    cfg.magnitudeBounds << .25, 1., 2.;
    cfg.penaltyWeights.resize(5);
    cfg.penaltyWeights << 100., 100., 100., 10., 10.;
    cfg.runup.enable = scenario >= 2;
    cfg.runup.distance = .5;
    cfg.runup.minimum_alignment_cosine = .8;
    minco_planner::MincoOptimizer optimizer(cfg);
    optimizer.setMap(std::make_shared<FreeMap>());
    if (cfg.runup.enable)
      optimizer.setTerrainSampler([](double x, double) {
        rog_map::TerrainDirectionSample s;
        s.uphill = Eigen::Vector2d::UnitX();
        s.body = 1. / (1. + std::exp(-5 * (x - 1.5)));
        s.body_gradient.x() = 5 * s.body * (1 - s.body);
        return s;
      });
    std::vector<Eigen::Vector3d> points;
    for (int k = 0; k <= pieces; ++k)
      points.emplace_back(k * .8, k == pieces ? 0. : .10 * std::sin(k), 0.);
    Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = start;
    end.col(0) = points.back();
    const auto speeds = super_utils::VecDf::Constant(pieces, 1.);
    traj_opt::Trajectory trajectory;
    double cost = 0.;
    std::vector<double> times;
    const std::string scene = scenario == 0   ? "minco_2_cold"
                              : scenario == 1 ? "minco_6_cold"
                              : scenario == 2 ? "minco_6_runup"
                                              : "minco_6_two_stage";
    for (int i = 0; i < 13; ++i) {
      optimizer.setInitPsAndTs({}, {});
      const auto begin = Clock::now();
      cost = scenario == 3
                 ? optimizer.optimizeTwoStage(points, start, end, speeds,
                                              trajectory)
                 : optimizer.optimize(points, start, end, speeds, trajectory);
      const double us =
          std::chrono::duration<double, std::micro>(Clock::now() - begin)
              .count();
      if (!std::isfinite(cost) || trajectory.empty()) {
        std::cerr << "Failed scene " << scene << '\n';
        return 3;
      }
      if (i >= 3)
        times.push_back(us);
    }
    report(scene, times);
    record(scene, "cost", cost);
    record(scene, "iterations", optimizer.lastIterationCount());
    record(scene, "return_code", optimizer.lastReturnCode());
    record(scene, "duration", trajectory.getTotalDuration());
    for (int k = 0; k <= 40; ++k) {
      const double t = trajectory.getTotalDuration() * k / 40.;
      for (int order = 0; order < 3; ++order) {
        const Eigen::Vector3d v = order == 0   ? trajectory.getPos(t)
                                  : order == 1 ? trajectory.getVel(t)
                                               : trajectory.getAcc(t);
        for (int axis = 0; axis < 3; ++axis)
          record(scene,
                 std::to_string(order) + "_" + std::to_string(k) + "_" +
                     std::to_string(axis),
                 v[axis]);
      }
    }
  }
}
