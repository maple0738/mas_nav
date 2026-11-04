#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace minco_controller {
struct DenseQP {
  Eigen::MatrixXd H, A;
  Eigen::VectorXd g, lb, ub, lbA, ubA;
};
struct MpcSolverTestAccess {
  static bool build(MpcSolver &solver, const State &state,
                    const std::vector<ReferencePoint> &ref, DenseQP &qp) {
    return solver.buildCondensedQP(state, ref, qp.H, qp.g, qp.lb, qp.ub, qp.A,
                                   qp.lbA, qp.ubA);
  }
};
namespace {
// Independent dense formulation of X_k=x0+dt*sum_{j<=k} u_j.
DenseQP denseReference(const MPCConfig &cfg, const State &state,
                       const std::vector<ReferencePoint> &ref,
                       const Eigen::Vector3d &anchor) {
  const int n = 3 * cfg.horizon;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, n), Q = B;
  Eigen::VectorXd error(n), commands(n), r(n);
  double yaw = state.yaw;
  for (int i = 0; i < cfg.horizon; ++i) {
    for (int j = 0; j <= i; ++j)
      B.block<3, 3>(3 * i, 3 * j) = cfg.dt * Eigen::Matrix3d::Identity();
    yaw += std::remainder(ref[i].yaw - yaw, 2 * std::acos(-1.));
    error.segment<3>(3 * i) = Eigen::Vector3d(
        state.x - ref[i].pos.x(), state.y - ref[i].pos.y(), state.yaw - yaw);
    commands.segment<3>(3 * i) =
        Eigen::Vector3d(ref[i].vel.x(), ref[i].vel.y(), ref[i].yaw_rate);
    r.segment<3>(3 * i) = cfg.R;
    const auto &v = ref[i].vel;
    const double heading =
        v.norm() > .02 ? std::atan2(v.y(), v.x()) : ref[i].yaw;
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading),
        std::cos(heading);
    Q.block<2, 2>(3 * i, 3 * i) =
        rotation * Eigen::Vector2d(cfg.q_along, cfg.q_cross).asDiagonal() *
        rotation.transpose();
    Q(3 * i + 2, 3 * i + 2) = cfg.Q.z();
  }
  DenseQP result;
  result.H = 2 * (B.transpose() * Q * B);
  result.H.diagonal().array() += 2 * r.array() + cfg.eps_regularization;
  result.g = 2 * (B.transpose() * Q * error - r.cwiseProduct(commands));
  result.lb.resize(n);
  result.ub.resize(n);
  for (int i = 0; i < cfg.horizon; ++i) {
    result.lb.segment<3>(3 * i) =
        Eigen::Vector3d(cfg.vx_min, cfg.vy_min, cfg.omega_min);
    result.ub.segment<3>(3 * i) =
        Eigen::Vector3d(cfg.vx_max, cfg.vy_max, cfg.omega_max);
  }
  if (cfg.use_acc_constraints) {
    result.A = Eigen::MatrixXd::Identity(n, n);
    result.lbA.resize(n);
    result.ubA.resize(n);
    for (int i = 0; i < cfg.horizon; ++i) {
      if (i)
        result.A.block<3, 3>(3 * i, 3 * (i - 1)) = -Eigen::Matrix3d::Identity();
      result.lbA.segment<3>(3 * i) =
          cfg.dt * Eigen::Vector3d(cfg.ax_min, cfg.ay_min, cfg.alpha_min);
      result.ubA.segment<3>(3 * i) =
          cfg.dt * Eigen::Vector3d(cfg.ax_max, cfg.ay_max, cfg.alpha_max);
    }
    result.lbA.head<3>() += anchor;
    result.ubA.head<3>() += anchor;
  } else {
    result.A.resize(0, 0);
    result.lbA.resize(0);
    result.ubA.resize(0);
  }
  return result;
}

TEST(MpcCondensing,
     MatchesDenseObjectiveAndAllBoundsAcrossConfigurationChanges) {
  MPCConfig cfg;
  MpcSolver solver(cfg);
  const Eigen::Vector3d anchor(.23, -.12, .08);
  State state;
  state.x = .7;
  state.y = -.3;
  state.yaw = 3.13;
  for (int n : {1, 10, 40, 4, 20})
    for (bool acc : {false, true}) {
      cfg.horizon = n;
      cfg.dt = n == 4 ? .1 : .05;
      cfg.use_acc_constraints = acc;
      cfg.q_along = n == 4 ? 6. : 3.;
      cfg.q_cross = n == 4 ? 2. : 12.;
      cfg.R = {.8, 1.5, .7};
      cfg.ax_min = -1.4;
      cfg.ax_max = 3.1;
      cfg.ay_min = -2.2;
      cfg.ay_max = 4.;
      solver.setConfig(cfg);
      solver.setLastControl(anchor);
      std::vector<ReferencePoint> ref(n);
      for (int i = 0; i < n; ++i) {
        ref[i].pos = {.2 + .03 * i, .7 - .01 * i};
        ref[i].vel = i % 3 == 0
                         ? Eigen::Vector2d::Zero()
                         : Eigen::Vector2d(std::cos(i * .3), std::sin(i * .3));
        ref[i].yaw = -3.12 + .2 * i;
        ref[i].yaw_rate = .1 * std::cos(i);
      }
      DenseQP actual;
      ASSERT_TRUE(MpcSolverTestAccess::build(solver, state, ref, actual));
      const auto expected = denseReference(cfg, state, ref, anchor);
      EXPECT_LE((actual.H - expected.H).cwiseAbs().maxCoeff(), 1e-10) << n;
      EXPECT_LE((actual.g - expected.g).cwiseAbs().maxCoeff(), 1e-10) << n;
      EXPECT_TRUE(actual.lb.isApprox(expected.lb, 1e-14));
      EXPECT_TRUE(actual.ub.isApprox(expected.ub, 1e-14));
      EXPECT_EQ(actual.A.rows(), expected.A.rows());
      if (acc) {
        EXPECT_TRUE(actual.A.isApprox(expected.A, 1e-14));
        EXPECT_TRUE(actual.lbA.isApprox(expected.lbA, 1e-14));
        EXPECT_TRUE(actual.ubA.isApprox(expected.ubA, 1e-14));
      }
    }
}
TEST(MpcCondensing,
     ReusedWorkspaceMatchesFreshSolvesAfterConfigChangesAndFailure) {
  MPCConfig cfg;
  MpcSolver reused(cfg);
  State state;
  state.x = .2;
  state.y = -.1;
  state.yaw = 3.13;
  const Eigen::Vector3d anchor(.1, -.05, .03);
  for (int n : {10, 40, 1, 5, 20})
    for (bool acc : {true, false}) {
      cfg.horizon = n;
      cfg.use_acc_constraints = acc;
      cfg.dt = n == 5 ? .1 : .05;
      cfg.q_along = n == 5 ? 8. : 3.;
      cfg.q_cross = n == 5 ? 1. : 12.;
      std::vector<ReferencePoint> refs(n);
      for (int i = 0; i < n; ++i) {
        refs[i].pos = {.2 + .01 * i, -.1 + .005 * i};
        refs[i].vel = {.2, .1};
        refs[i].yaw = -3.12;
        refs[i].yaw_rate = .02;
      }
      reused.setConfig(cfg);
      reused.setLastControl(anchor);
      MpcSolver fresh(cfg);
      fresh.setLastControl(anchor);
      Control actual, expected;
      std::vector<State> a, b;
      ASSERT_TRUE(reused.solve(state, refs, actual, &a));
      ASSERT_TRUE(fresh.solve(state, refs, expected, &b));
      EXPECT_NEAR(actual.vx, expected.vx, 1e-11);
      EXPECT_NEAR(actual.vy, expected.vy, 1e-11);
      EXPECT_NEAR(actual.omega, expected.omega, 1e-11);
      ASSERT_EQ(a.size(), b.size());
      for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_NEAR(a[i].x, b[i].x, 1e-11);
        EXPECT_NEAR(a[i].y, b[i].y, 1e-11);
        EXPECT_NEAR(a[i].yaw, b[i].yaw, 1e-11);
      }
      cfg.q_along = -1.;
      reused.setConfig(cfg);
      EXPECT_FALSE(reused.solve(state, refs, actual));
    }
}
} // namespace
} // namespace minco_controller
