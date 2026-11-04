#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <qpOASES.hpp>

namespace minco_controller {

MpcSolver::MpcSolver(const MPCConfig & config) : config_(config)
{
}

void MpcSolver::setConfig(const MPCConfig & config)
{
  config_ = config;
}

bool MpcSolver::buildCondensedQP(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Eigen::MatrixXd & H,
  Eigen::VectorXd & g,
  Eigen::VectorXd & lb,
  Eigen::VectorXd & ub,
  Eigen::MatrixXd & A_con,
  Eigen::VectorXd & lbA,
  Eigen::VectorXd & ubA)
{
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }
  if (!std::isfinite(config_.q_along) || config_.q_along <= 0.0 ||
    !std::isfinite(config_.q_cross) || config_.q_cross <= 0.0) {
    return false;
  }

  // For the global velocity integrator, A=I and B=dt*I. Therefore
  // H_ij = 2*dt^2*sum_{k>=max(i,j)} Q_k + 2*delta_ij*R + eps*I,
  // g_i  = 2*dt*sum_{k>=i} Q_k*(x0-xref_k) - 2*R*uref_i.
  // Suffix sums avoid generic matrix powers and cubic dense products while
  // retaining the same rotated along/cross cost and yaw branch handling.
  const int nU = 3 * N;
  reference_errors_.resize(3, N);
  state_weights_.resize(N);
  double previous_yaw = curr.yaw;
  for (int i = 0; i < N; ++i) {
    previous_yaw += std::atan2(
      std::sin(ref_traj[i].yaw - previous_yaw),
      std::cos(ref_traj[i].yaw - previous_yaw));
    reference_errors_.col(i) = Eigen::Vector3d(
      curr.x - ref_traj[i].pos.x(), curr.y - ref_traj[i].pos.y(), curr.yaw - previous_yaw);
    const Eigen::Vector2d tangent = ref_traj[i].vel;
    const double tangent_norm = tangent.norm();
    const bool has_useful_tangent = std::isfinite(tangent_norm) && tangent_norm > 0.02;
    const double c = has_useful_tangent ? tangent.x() / tangent_norm : std::cos(ref_traj[i].yaw);
    const double s = has_useful_tangent ? tangent.y() / tangent_norm : std::sin(ref_traj[i].yaw);
    auto & Q = state_weights_[i];
    Q.setZero();
    Q(0, 0) = config_.q_along * c * c + config_.q_cross * s * s;
    Q(0, 1) = Q(1, 0) = (config_.q_along - config_.q_cross) * c * s;
    Q(1, 1) = config_.q_along * s * s + config_.q_cross * c * c;
    Q(2, 2) = config_.Q.z();
  }
  H.resize(nU, nU);
  g.resize(nU);
  Eigen::Matrix3d suffix_weight = Eigen::Matrix3d::Zero();
  Eigen::Vector3d suffix_error = Eigen::Vector3d::Zero();
  for (int i = N - 1; i >= 0; --i) {
    suffix_weight += state_weights_[i];
    suffix_error.noalias() += state_weights_[i] * reference_errors_.col(i);
    const Eigen::Matrix3d block = (2.0 * config_.dt * config_.dt) * suffix_weight;
    for (int j = 0; j <= i; ++j) {
      H.block<3, 3>(3 * i, 3 * j) = block;
      if (j != i) H.block<3, 3>(3 * j, 3 * i) = block.transpose();
    }
    H.block<3, 3>(3 * i, 3 * i).diagonal().array() +=
      2.0 * config_.R.array() + config_.eps_regularization;
    const Eigen::Vector3d command(ref_traj[i].vel.x(), ref_traj[i].vel.y(), ref_traj[i].yaw_rate);
    g.segment<3>(3 * i) = 2.0 * (config_.dt * suffix_error - config_.R.cwiseProduct(command));
  }

  // 变量边界（速度约束，map/global 系）
  // Use -1e12 instead of -inf: qpOASES defines INFTY=1e20 internally.
  // IEEE -inf causes NaN in active-set arithmetic.
  lb = Eigen::VectorXd::Constant(nU, -1.0e12);
  ub = Eigen::VectorXd::Constant(nU, 1.0e12);
  for (int i = 0; i < N; ++i) {
    lb(i * 3 + 0) = config_.vx_min;
    ub(i * 3 + 0) = config_.vx_max;
    lb(i * 3 + 1) = config_.vy_min;
    ub(i * 3 + 1) = config_.vy_max;
    lb(i * 3 + 2) = config_.omega_min;
    ub(i * 3 + 2) = config_.omega_max;
  }

  // 加速度约束（可选）：a_min <= (v_k - v_{k-1})/dt <= a_max
  if (!config_.use_acc_constraints) {
    A_con.resize(0, 0);
    lbA.resize(0);
    ubA.resize(0);
    return true;
  }

  const int nC = 3 * N;  // 对每个 k 对每个轴各一个差分约束：v_k - v_{k-1}
  A_con = Eigen::MatrixXd::Zero(nC, nU);
  lbA = Eigen::VectorXd::Zero(nC);
  ubA = Eigen::VectorXd::Zero(nC);

  const double last_vx = last_u_global_.x();
  const double last_vy = last_u_global_.y();
  const double last_w = last_u_global_.z();

  // 约束形式：lbA <= A_con * U <= ubA
  // 这里 A_con * U 直接表示 (v_k - v_{k-1})，再用 bounds 限制到 [a_min*dt, a_max*dt]
  for (int k = 0; k < N; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      const int row = k * 3 + axis;
      const int col_curr = k * 3 + axis;
      A_con(row, col_curr) = 1.0;

      if (k == 0) {
        // v0 - v(-1)
        // v(-1) 使用当前时刻真实全局速度作为已知常数，转移到 bounds：
        // a_min*dt <= v0 - v_curr_global <= a_max*dt
        const double last = (axis == 0 ? last_vx : (axis == 1 ? last_vy : last_w));
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.dt +
          last;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.dt +
          last;
      } else {
        const int col_prev = (k - 1) * 3 + axis;
        A_con(row, col_prev) = -1.0;
        lbA(row) =
          (axis == 0 ? config_.ax_min : (axis == 1 ? config_.ay_min : config_.alpha_min)) * config_.dt;
        ubA(row) =
          (axis == 0 ? config_.ax_max : (axis == 1 ? config_.ay_max : config_.alpha_max)) * config_.dt;
      }
    }
  }

  return true;
}

bool MpcSolver::solve(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Control & out_u,
  std::vector<State> * out_pred)
{
  const int N = config_.horizon;
  if (N <= 0) {
    return false;
  }
  if (static_cast<int>(ref_traj.size()) < N) {
    return false;
  }

  auto & H = hessian_;
  auto & g = gradient_;
  auto & lb = lower_;
  auto & ub = upper_;
  auto & A_con = constraints_;
  auto & lbA = constraint_lower_;
  auto & ubA = constraint_upper_;

  if (!buildCondensedQP(curr, ref_traj, H, g, lb, ub, A_con, lbA, ubA)) {
    return false;
  }

  const int nV = static_cast<int>(g.size());
  const int nC = static_cast<int>(lbA.size());

  auto & H_qp = qp_hessian_;
  H_qp.resize(static_cast<size_t>(nV) * static_cast<size_t>(nV));
  for (int i = 0; i < nV; ++i) {
    for (int j = 0; j < nV; ++j) {
      H_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
        static_cast<qpOASES::real_t>(H(i, j));
    }
  }

  auto & g_qp = qp_gradient_;
  g_qp.resize(static_cast<size_t>(nV));
  auto & lb_qp = qp_lower_;
  lb_qp.resize(static_cast<size_t>(nV));
  auto & ub_qp = qp_upper_;
  ub_qp.resize(static_cast<size_t>(nV));
  for (int i = 0; i < nV; ++i) {
    g_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(g(i));
    lb_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lb(i));
    ub_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ub(i));
  }

  auto & A_qp = qp_constraints_;
  auto & lbA_qp = qp_constraint_lower_;
  auto & ubA_qp = qp_constraint_upper_;

  const qpOASES::real_t * A_ptr = nullptr;
  const qpOASES::real_t * lbA_ptr = nullptr;
  const qpOASES::real_t * ubA_ptr = nullptr;

  if (nC > 0) {
    A_qp.resize(static_cast<size_t>(nC) * static_cast<size_t>(nV));
    for (int i = 0; i < nC; ++i) {
      for (int j = 0; j < nV; ++j) {
        A_qp[static_cast<size_t>(i) * static_cast<size_t>(nV) + static_cast<size_t>(j)] =
          static_cast<qpOASES::real_t>(A_con(i, j));
      }
    }

    lbA_qp.resize(static_cast<size_t>(nC));
    ubA_qp.resize(static_cast<size_t>(nC));
    for (int i = 0; i < nC; ++i) {
      lbA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(lbA(i));
      ubA_qp[static_cast<size_t>(i)] = static_cast<qpOASES::real_t>(ubA(i));
    }

    A_ptr = A_qp.data();
    lbA_ptr = lbA_qp.data();
    ubA_ptr = ubA_qp.data();
  }

  qpOASES::QProblem qp(nV, nC);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);

  qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(config_.max_working_set_recalculations);
  const qpOASES::returnValue ret =
    qp.init(H_qp.data(), g_qp.data(), A_ptr, lb_qp.data(), ub_qp.data(), lbA_ptr, ubA_ptr, nWSR);

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      std::cout << "[MpcSolver] qpOASES Max NWSR Reached!"
                << std::endl;
    } else if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
      std::cout << "[MpcSolver] qpOASES Infeasible!" << std::endl;
    } else {
      std::cout << "[MpcSolver] qpOASES Error Code: " << static_cast<int>(ret) << std::endl;
    }
    return false;
  }

  auto & xOpt = qp_solution_;
  xOpt.assign(static_cast<size_t>(nV), 0.0);
  qp.getPrimalSolution(xOpt.data());

  // 如果需要输出预测轨迹
  if (out_pred) {
    out_pred->clear();
    out_pred->reserve(N + 1);
    out_pred->push_back(curr);

    State s = curr;
    for (int i = 0; i < N; ++i) {
      const double ux = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 0)]);
      const double uy = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 1)]);
      const double uw = static_cast<double>(xOpt[static_cast<size_t>(3 * i + 2)]);

      // Same global integrator used by the condensed objective above.
      s.x += config_.dt * ux;
      s.y += config_.dt * uy;
      s.yaw += config_.dt * uw;
      out_pred->push_back(s);
    }
  }

  // 取第一个控制量 u0 (global)
  const Eigen::Vector3d u0_global(
    static_cast<double>(xOpt[0]), static_cast<double>(xOpt[1]), static_cast<double>(xOpt[2]));

  // 记录上一帧全局速度，用于平滑加速度
  last_u_global_ = u0_global;

  // 直接输出全局速度
  out_u.vx = u0_global.x();
  out_u.vy = u0_global.y();
  out_u.omega = u0_global.z();

  return true;
}

}  // namespace minco_controller
