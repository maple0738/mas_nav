#pragma once

#include <vector>

#include <Eigen/Core>
#include <qpOASES.hpp>

#include "mas2027_nav_executor/path_executor/mpc/mpc_types.hpp"

namespace minco_controller {

// 纯 C++ MPC 求解器：不依赖 ROS，仅依赖 Eigen 与 qpOASES。
class MpcSolver
{
public:
  explicit MpcSolver(const MPCConfig & config);

  void setConfig(const MPCConfig & config);

  // @brief 求解一次 MPC。
  // @param curr 当前状态 (map)
  // @param ref_traj 参考轨迹 (horizon 个点，若不足由插件层补齐)
  // @param out_u 输出控制 (map)
  // @param out_pred (可选) 输出预测轨迹
  // @return 是否求解成功
  bool solve(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Control & out_u,
    std::vector<State> * out_pred = nullptr);

  Eigen::Vector3d getLastControl() const { return last_u_global_; }
  void resetLastControl() { last_u_global_.setZero(); }
  /// 加速度约束的锚点取实测车速，切勿清零。
  ///
  /// 约束为 a_min·dt <= u_0 - anchor <= a_max·dt；若打断一拍就清零，恢复首拍会被限成
  /// |u_0| <= a_max·dt = 0.2 m/s，车还在跑却被硬刹，门控抖动时即 MPC 反复冷启动。
  /// 取实测车速不放松速度上限与任何净空判据，只让约束与物理一致（真实加速度 dv_actual/dt）。
  void setLastControl(const Eigen::Vector3d & control)
  {
    last_u_global_ = control;
  }

private:
  friend struct MpcSolverTestAccess;

  // Fixed global integrator: [x,y,yaw]_{k+1} = state_k + dt*[vx,vy,omega]_k.
  // The condensed objective and predicted poses use this same model.
  // 组装 MPC 的 condensed QP：0.5*U^T H U + g^T U
  // 其中 U 堆叠了 horizon 个控制量 [vx, vy, omega] (map/global)
  bool buildCondensedQP(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Eigen::MatrixXd & H,
    Eigen::VectorXd & g,
    Eigen::VectorXd & lb,
    Eigen::VectorXd & ub,
    Eigen::MatrixXd & A_con,
    Eigen::VectorXd & lbA,
    Eigen::VectorXd & ubA);

private:
  MPCConfig config_;

  // Only storage is reused. Every QP coefficient/bound is rebuilt for each
  // reference/configuration; qpOASES still starts cold on every solve.
  Eigen::MatrixXd hessian_, constraints_;
  Eigen::VectorXd gradient_, lower_, upper_, constraint_lower_, constraint_upper_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> reference_errors_;
  std::vector<Eigen::Matrix3d> state_weights_;
  std::vector<qpOASES::real_t> qp_hessian_, qp_gradient_, qp_lower_, qp_upper_;
  std::vector<qpOASES::real_t> qp_constraints_, qp_constraint_lower_, qp_constraint_upper_, qp_solution_;

  // 用于加速度约束的上一时刻 global 控制量
  Eigen::Vector3d last_u_global_{0.0, 0.0, 0.0};
};

}  // namespace minco_controller
