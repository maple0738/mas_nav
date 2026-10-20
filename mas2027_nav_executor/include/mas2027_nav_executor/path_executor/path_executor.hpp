#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mas2027_nav_executor/common/control/goal_approach.hpp"
#include "mas2027_nav_executor/path_executor/monitoring/command_safety.hpp"
#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include "mas2027_nav_executor/path_executor/state/execution_state.hpp"
#include "tf2_ros/buffer.h"
#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "rog_map/map_query_interface.hpp"

namespace mas2027_nav_executor {

struct PathExecutorParams
{
  double planner_frequency{};
  double trajectory_timeout_s{};
  double odom_timeout_s{};
  double control_delay_s{};
  double deadzone_speed{};
  bool output_in_body_frame{};
  std::string odom_frame;
  double rog_map_clearance{};
  /// Narrow-corridor continuation: activates when the measured current
  /// clearance is below the full effective threshold but above this floor, or
  /// when the committed trajectory enters such a band within a short
  /// lookahead. The latter uses a braking envelope to reach narrow_passage_speed
  /// at the corridor mouth; it never grants the floor if the current pose is
  /// already below it.
  bool narrow_passage_enable{false};
  double narrow_passage_clearance{0.0};
  double narrow_passage_speed{0.5};
  /// Additional clearance for stopping when moving toward an obstacle.
  ClearanceBrakingParams clearance_braking{};
  /// 几何窄道底盘对齐：不依赖语义 label。只有轨迹两侧法向探测都显示
  /// 障碍靠近时才按切线对齐；单侧贴墙仍由独立的 ESDF 净空门处理。
  /// 实际是否允许反向由 gimbal_follow 参数决定。
  bool narrow_passage_alignment_enable{false};
  double narrow_passage_alignment_lookahead{2.0};
  double narrow_passage_alignment_hold_distance{2.0};
  double swept_radius{0.302};
  double rotation_margin{0.05};
  /// Resample the MPC reference after a first solve when the predicted world
  /// velocity diverges from the route tangent or the route enters a narrow band.
  /// The angle and narrow-band branches have independent enable flags so field
  /// A/B runs can isolate the two behaviours.
  bool secondary_reference_enable{false};
  bool secondary_reference_angle_enable{true};
  bool secondary_reference_narrow_enable{true};
  double secondary_reference_angle_deg{20.0};
  /// Hard lower bound on every resampled time scale; narrow_time_scale cannot
  /// push the reference below this.
  double secondary_reference_min_time_scale{0.35};
  double secondary_reference_narrow_time_scale{0.70};
  /// ROGMap 最后一次完成更新允许的最大年龄（秒）。
  double rog_map_timeout_s{};
  /// 目标接近减速带：用实测刹车能力给"沿已提交轨迹的剩余弧长 → 允许速度"
  /// 封顶。规划侧 seed 也做同一件事；这里兜住"规划来不及、轨迹已过期但仍在
  /// 缓冲窗口内"的情况。`deceleration <= 0` 表示关闭。
  GoalApproachParams goal_approach{};
};

// Owns the MPC and the whole trajectory-tracking step: staleness check, frame
// alignment, reference sampling, solve, and body-frame velocity conversion.
class PathExecutor final
{
public:
  PathExecutor(
    const minco_controller::MPCConfig & config,
    const PathExecutorParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<TerrainGrid> terrain,
    std::shared_ptr<rog_map::MapQueryInterface> rog_query);

  ExecutorOutput computeCommand(const ExecutorInput & input);

  /// 几何窄道对齐的航向误差（度）：沿已提交轨迹向前扫描至
  /// `narrow_passage_alignment_lookahead`，找到第一个净空落在
  /// `[narrow_passage_clearance, aligned-body clearance)` 且左右两侧均有
  /// 障碍靠近的轨迹点，返回该处
  /// 轨迹切线相对 `current_yaw` 的角度误差。进入后至少保持指定的实际
  /// 行驶里程；之后实测位置恢复宽路且近前方没有窄点时退出。
  /// 短暂地图波动时沿用最后切线。
  std::optional<double> narrowPassageHeadingErrorDegrees(
    const interfaces::msg::MpcPositionCommand & trajectory,
    double current_x, double current_y, double current_yaw);

  void resetNarrowPassageAlignment();
  /// 每个有效里程计周期更新对齐模式内的累计行驶距离。
  void updateNarrowPassageAlignmentPose(double x, double y);
  /// 已到达目标时，满 2 m 的区间可以结束；不足 2 m 的区间跨目标保留。
  bool releaseNarrowPassageAlignmentAtGoal();
  bool narrowPassageAlignmentActive() const { return narrow_alignment_active_; }
  double narrowPassageAlignmentTraveled() const {
    return narrow_alignment_active_ ? narrow_alignment_traveled_m_ : last_alignment_release_m_;
  }

  /// 最近一次 computeCommand 里命令门的判据归因（哪一条把指令否决掉的）。
  /// 只读快照，供 nav_executor_node 打日志用；reason=="none" 表示上一次未被否决。
  const CommandSafetyDetail & lastSafetyDetail() const { return last_safety_detail_; }

private:
  bool transformedCommands(
    const interfaces::msg::MpcPositionCommand & trajectory,
    std::vector<interfaces::msg::PositionCommand> & commands) const;

  bool buildReference(
    const interfaces::msg::MpcPositionCommand & trajectory,
    const minco_controller::State & current,
    std::vector<minco_controller::ReferencePoint> & reference,
    bool follow_gimbal, bool reverse_heading,
    size_t * nearest_index = nullptr,
    std::vector<interfaces::msg::PositionCommand> * transformed = nullptr) const;

  /// 按剩余弧长给参考速度封顶（只改速度，不改位置/航向）。
  /// 返回是否真的封了顶，并带回最紧的一处剩余距离与允许速度。
  bool applyGoalApproachCap(
    std::vector<minco_controller::ReferencePoint> & reference,
    const std::vector<interfaces::msg::PositionCommand> & commands,
    double * remaining_out = nullptr,
    double * allowed_out = nullptr) const;

  bool applyClearanceSpeedCap(
    std::vector<minco_controller::ReferencePoint> & reference,
    double base_clearance) const;

  bool sampleReference(
    const std::vector<interfaces::msg::PositionCommand> & commands,
    size_t nearest_index, const std::vector<double> & time_scales,
    std::vector<minco_controller::ReferencePoint> & reference,
    bool follow_gimbal, bool reverse_heading) const;

  double rotationClearanceRequirement() const;
  double alignedBodyClearanceRequirement() const;

  minco_controller::MPCConfig config_;
  PathExecutorParams params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<TerrainGrid> terrain_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_;
  std::unique_ptr<minco_controller::MpcSolver> solver_;
  CommandSafetyDetail last_safety_detail_{};
  bool narrow_alignment_active_{false};
  Eigen::Vector2d last_narrow_tangent_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_alignment_pose_{Eigen::Vector2d::Zero()};
  double narrow_alignment_traveled_m_{0.0};
  double last_alignment_release_m_{0.0};
  bool narrow_alignment_rearm_near_only_{false};
  void releaseNarrowPassageAlignment();
};

}  // namespace mas2027_nav_executor
