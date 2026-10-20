#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "interfaces/msg/mpc_position_command.hpp"
#include "interfaces/msg/chassis_command.hpp"
#include "interfaces/msg/region_status.hpp"
#include "mas2027_nav_executor/common/environment/region_control.hpp"
#include "mas2027_nav_executor/common/control/gimbal_follow_adapter.hpp"
#include "mas2027_nav_executor/path_executor/mpc/mpc_solver.hpp"
#include "mas2027_nav_executor/path_planner/path_planner.hpp"
#include "mas2027_nav_executor/path_executor/path_executor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace mas2027_nav_executor {
using minco_controller::MPCConfig;
namespace {

// Observation-only yaw-rate command outside gimbal-following mode. The CDC1
// serial protocol does not carry w; the lower board owns the physical spin.
constexpr double kNonFollowYawRateRadS = 2.0;

std_msgs::msg::ColorRGBA velocity_color(
  const double velocity, const double minimum, const double maximum)
{
  const double normalized = std::isfinite(velocity) && maximum > minimum ?
    std::clamp((velocity - minimum) / (maximum - minimum), 0.0, 1.0) : 0.0;
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 3.0), 0.0, 1.0));
  color.g = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 2.0), 0.0, 1.0));
  color.b = static_cast<float>(std::clamp(1.5 - std::abs(4.0 * normalized - 1.0), 0.0, 1.0));
  color.a = 1.0F;
  return color;
}

}  // namespace

// 导航 ROS 总装配层：接收地图、轨迹、里程计和目标，驱动规划/执行模块，并发布控制结果。
// 全局搜索与 MINCO 由 PathPlanner 管理，MPC 跟踪由 PathExecutor 管理。
class NavExecutorNode final : public rclcpp::Node
{
public:
  NavExecutorNode()
  : Node("nav_executor")
  {
    rcl_interfaces::msg::ParameterDescriptor map_source_descriptor;
    map_source_descriptor.read_only = true;
    map_source_descriptor.description = "Map source selected at startup: prior or online";
    const auto map_source = declare_parameter<std::string>(
      "node.map_source", "prior", map_source_descriptor);
    if (map_source != "prior" && map_source != "online") {
      throw std::invalid_argument("node.map_source must be prior or online");
    }
    const bool online_only = map_source == "online";
    RCLCPP_INFO(get_logger(), "Navigation map source: %s", map_source.c_str());
    // ── 参数层：读取 ROS 话题名、坐标系和控制周期。
    control_rate_hz_ = declare_parameter<double>("node.control_rate_hz");
    planner_frequency_ = declare_parameter<double>("node.planner_frequency");
    odom_topic_ = declare_parameter<std::string>("node.topics.odom_sub");
    trajectory_topic_ = declare_parameter<std::string>("node.topics.trajectory_sub");
    odom_frame_ = declare_parameter<std::string>("node.frames.odom");
    output_topic_ = declare_parameter<std::string>("node.topics.cmd_vel_pub");
    chassis_cmd_topic_ = declare_parameter<std::string>(
      "node.topics.chassis_cmd_pub", "/nav_executor/chassis_cmd");
    region_status_topic_ = declare_parameter<std::string>(
      "node.topics.region_status_pub", "/nav_executor/region_status");
    cmd_spin_topic_ = declare_parameter<std::string>("node.topics.spin_cmd_sub");
    goal_topic_ = declare_parameter<std::string>("node.topics.goal_sub");
    minco_path_topic_ = declare_parameter<std::string>("node.topics.minco_path_pub");
    minco_trajectory_topic_ = declare_parameter<std::string>("node.topics.minco_trajectory_pub");
    planning_constraints_topic_ = declare_parameter<std::string>(
      "node.topics.planning_constraints_pub", "/planning_constraints");
    planning_constraints_marker_topic_ = declare_parameter<std::string>(
      "node.topics.planning_constraints_marker_pub", "/planning_constraints_markers");
    const auto cost_map_topic = declare_parameter<std::string>("node.topics.terrain_cost_sub", "/cost_map");
    const auto label_map_topic = declare_parameter<std::string>(
      "node.topics.terrain_label_sub", "/terrain_label_map");
    const auto uphill_map_topic = declare_parameter<std::string>(
      "node.topics.terrain_uphill_sub", "/terrain_uphill_map");
    const auto online_label_topic = declare_parameter<std::string>(
      "node.topics.online_terrain_label_sub", "/nav_executor/online_terrain_map");
    persistent_online_terrain_ = online_label_topic == "/nav_executor/online_terrain_map";
    online_label_timeout_s_ = declare_parameter<double>(
      "node.online_terrain_label_timeout_s", 0.8);
    if (!std::isfinite(online_label_timeout_s_) || online_label_timeout_s_ <= 0.0) {
      throw std::invalid_argument("node.online_terrain_label_timeout_s must be positive");
    }
    // ── 区域策略层：把 label 映射到 mode，并读取每类区域的速度、加速度和阶段距离。
    const int normal_mode = declare_parameter<int>("region_control.normal_mode", 4);
    if (normal_mode < 0 || normal_mode > 255) {
      throw std::invalid_argument("region_control.normal_mode must be in [0,255]");
    }
    const double normal_max_speed = declare_parameter<double>(
      "region_control.normal_max_speed", 3.0);
    const double normal_max_acceleration = declare_parameter<double>(
      "region_control.normal_max_acceleration", 4.0);
    if (!std::isfinite(normal_max_speed) || normal_max_speed <= 0.0 ||
      !std::isfinite(normal_max_acceleration) || normal_max_acceleration <= 0.0) {
      throw std::invalid_argument("region_control normal motion limits must be finite and positive");
    }
    const double speed_blend_rate = declare_parameter<double>(
      "region_control.speed_blend_rate", 2.0);
    const double acceleration_blend_rate = declare_parameter<double>(
      "region_control.acceleration_blend_rate", 4.0);
    if (!std::isfinite(speed_blend_rate) || speed_blend_rate <= 0.0 ||
      !std::isfinite(acceleration_blend_rate) || acceleration_blend_rate <= 0.0) {
      throw std::invalid_argument("region_control blend rates must be finite and positive");
    }
    const std::array<std::pair<const char *, uint8_t>, 3> region_names{{
      {"slope", 5}, {"tunnel", 6}, {"undulating", 7}}};
    for (size_t i = 0; i < region_names.size(); ++i) {
      const std::string prefix = std::string("region_control.") + region_names[i].first + ".";
      RegionRule & rule = region_rules_[i];
      rule.label = region_names[i].second;
      rule.mode = declare_parameter<int>(prefix + "mode", rule.label);
      if (rule.mode != rule.label) {
        throw std::invalid_argument(prefix + "mode must equal terrain label " +
          std::to_string(rule.label));
      }
      rule.max_speed = declare_parameter<double>(prefix + "max_speed", 1.0);
      rule.max_acceleration = declare_parameter<double>(prefix + "max_acceleration", 2.0);
      rule.prepare_distance = declare_parameter<double>(prefix + "prepare_distance", 1.0);
      rule.activation_distance = declare_parameter<double>(prefix + "activation_distance", 0.8);
      rule.commit_distance = declare_parameter<double>(prefix + "commit_distance", 0.2);
      rule.release_distance = declare_parameter<double>(prefix + "release_distance", 0.5);
      if (rule.mode < -1 || rule.mode > 255 ||
        !std::isfinite(rule.max_speed) || rule.max_speed <= 0.0 ||
        !std::isfinite(rule.max_acceleration) || rule.max_acceleration <= 0.0 ||
        !std::isfinite(rule.prepare_distance) || rule.prepare_distance < 0.0 ||
        !std::isfinite(rule.activation_distance) || rule.activation_distance < 0.0 ||
        rule.activation_distance > rule.prepare_distance ||
        !std::isfinite(rule.commit_distance) || rule.commit_distance < 0.0 ||
        rule.commit_distance > rule.activation_distance ||
        !std::isfinite(rule.release_distance) || rule.release_distance < 0.0) {
        throw std::invalid_argument(prefix + " has an invalid mode or speed/extent parameter");
      }
    }
    // ── 可视化配置层：轨迹速度配色、全局路径显示频率与样式。
    velocity_color_min_ = declare_parameter<double>("node.visualization.velocity_color_min");
    velocity_color_max_ = declare_parameter<double>("node.visualization.velocity_color_max");
    // 全局搜索折线（SMAC 2D / Astar 的输出），专门给 RViz 看。
    // MINCO 轨迹另发 minco_path_pub，避免再用“global_path”指代局部优化轨迹。
    global_plan_topic_ = declare_parameter<std::string>(
      "node.topics.global_plan_pub", "/nav_executor/global_plan");
    global_plan_marker_topic_ = declare_parameter<std::string>(
      "node.topics.global_plan_marker_pub", "/nav_executor/debug/global_plan");
    global_plan_publish_hz_ = declare_parameter<double>("node.visualization.global_plan_publish_hz", 5.0);
    global_plan_line_width_ = declare_parameter<double>("node.visualization.global_plan_line_width", 0.15);
    global_plan_color_r_ = declare_parameter<double>("node.visualization.global_plan_color_r", 0.0);
    global_plan_color_g_ = declare_parameter<double>("node.visualization.global_plan_color_g", 1.0);
    global_plan_color_b_ = declare_parameter<double>("node.visualization.global_plan_color_b", 1.0);
    if (!(std::isfinite(global_plan_publish_hz_) && global_plan_publish_hz_ > 0.0)) {
      global_plan_publish_hz_ = 5.0;
    }
    if (!(std::isfinite(global_plan_line_width_) && global_plan_line_width_ > 0.0)) {
      global_plan_line_width_ = 0.15;
    }

    // ── 执行参数层：数据超时、安全净空、输出坐标系等 PathExecutor 参数。
    PathExecutorParams executor_params;
    executor_params.planner_frequency = planner_frequency_;
    executor_params.trajectory_timeout_s = declare_parameter<double>("node.trajectory_timeout_s");
    executor_params.odom_timeout_s = declare_parameter<double>("node.odom_timeout_s", 0.5);
    // 与规划器发布前校验共用同一个硬净空，避免 node / planner 两份值漂移。
    executor_params.rog_map_clearance =
      declare_parameter<double>("planner.minco_optimizer.collision_dist", 0.28);
    // ROGMap field.inflation_radius=0: every clearance is from base_link center.
    // Corridor shaping may use a smaller radius; validate the hard clearance
    // against the separately measured swept envelope of the square chassis.
    const double corridor_radius = declare_parameter<double>("planner.corridor.robot_radius", 0.28);
    const double swept_radius = declare_parameter<double>("planner.corridor.swept_radius", 0.302);
    const double rotation_margin = declare_parameter<double>("planner.corridor.extra_margin", 0.05);
    executor_params.swept_radius = swept_radius;
    executor_params.rotation_margin = rotation_margin;
    const bool allow_undersized_clearance = declare_parameter<bool>(
      "planner.corridor.allow_undersized_clearance", false);
    executor_params.narrow_passage_enable =
      declare_parameter<bool>("planner.minco_optimizer.narrow_passage_enable", true);
    executor_params.narrow_passage_clearance =
      declare_parameter<double>("planner.minco_optimizer.narrow_passage_clearance", 0.24);
    executor_params.narrow_passage_speed =
      declare_parameter<double>("planner.minco_optimizer.narrow_passage_speed", 0.5);
    executor_params.clearance_braking.deceleration =
      declare_parameter<double>("planner.minco_optimizer.clearance_braking.deceleration", 1.2);
    executor_params.clearance_braking.reaction_time =
      declare_parameter<double>("planner.minco_optimizer.clearance_braking.reaction_time", 0.20);
    executor_params.narrow_passage_alignment_enable =
      declare_parameter<bool>("planner.minco_optimizer.narrow_passage_alignment_enable", true);
    executor_params.narrow_passage_alignment_lookahead =
      declare_parameter<double>("planner.minco_optimizer.narrow_passage_alignment_lookahead", 2.0);
    executor_params.narrow_passage_alignment_hold_distance =
      declare_parameter<double>("planner.minco_optimizer.narrow_passage_alignment_hold_distance", 2.0);
    // 目标接近减速带：与规划侧共用同一组 YAML 键，保证"种子限速"和"参考限速"
    // 用的是同一个实测刹车能力；deceleration<=0 表示关闭。
    executor_params.goal_approach.deceleration =
      declare_parameter<double>("planner.minco_optimizer.goal_approach.deceleration", 1.2);
    executor_params.goal_approach.stop_margin =
      declare_parameter<double>("planner.minco_optimizer.goal_approach.stop_margin", 0.30);
    executor_params.goal_approach.min_speed =
      declare_parameter<double>("planner.minco_optimizer.goal_approach.min_speed", 0.15);
    executor_params.goal_approach.reaction_time =
      declare_parameter<double>("planner.minco_optimizer.goal_approach.reaction_time", 0.20);
    goal_approach_deceleration_ = executor_params.goal_approach.deceleration;
    executor_params.rog_map_timeout_s = declare_parameter<double>("node.rog_map_timeout_s", 0.5);
    executor_params.control_delay_s = declare_parameter<double>("mpc.control_delay_compensation");
    executor_params.secondary_reference_enable =
      declare_parameter<bool>("mpc.secondary_reference.enable", false);
    executor_params.secondary_reference_angle_enable =
      declare_parameter<bool>("mpc.secondary_reference.angle_enable", true);
    executor_params.secondary_reference_narrow_enable =
      declare_parameter<bool>("mpc.secondary_reference.narrow_enable", true);
    executor_params.secondary_reference_angle_deg =
      declare_parameter<double>("mpc.secondary_reference.angle_threshold_deg", 20.0);
    executor_params.secondary_reference_min_time_scale =
      declare_parameter<double>("mpc.secondary_reference.min_time_scale", 0.35);
    executor_params.secondary_reference_narrow_time_scale =
      declare_parameter<double>("mpc.secondary_reference.narrow_time_scale", 0.70);
    executor_params.deadzone_speed = declare_parameter<double>("mpc.deadzone_speed_threshold");
    executor_params.output_in_body_frame = declare_parameter<bool>("node.output_in_body_frame");
    executor_params.odom_frame = odom_frame_;
    alignment_odom_timeout_s_ = executor_params.odom_timeout_s;
    alignment_trajectory_timeout_s_ = executor_params.trajectory_timeout_s;
    GimbalFollowParams follow_params;
    follow_params.allow_reverse = declare_parameter<bool>("gimbal_follow.allow_reverse", true);
    follow_params.reverse_switch_hysteresis_deg = declare_parameter<double>(
      "gimbal_follow.reverse_switch_hysteresis_deg", 10.0);
    follow_params.ready_tolerance_deg = declare_parameter<double>("gimbal_follow.ready_tolerance_deg", 5.0);
    follow_params.ready_release_deg = declare_parameter<double>("gimbal_follow.ready_release_deg", 8.0);
    follow_params.settle_time_s = declare_parameter<double>("gimbal_follow.settle_time_s", 0.3);
    follow_params.heading_error_gain_s = declare_parameter<double>(
      "gimbal_follow.heading_error_gain_s", 1.0);
    follow_params.heading_deadband_deg = declare_parameter<double>(
      "gimbal_follow.heading_deadband_deg", 0.5);
    follow_params.max_heading_rate_deg_s = declare_parameter<double>(
      "gimbal_follow.max_heading_rate_deg_s", 120.0);
    gimbal_hold_enabled_ = declare_parameter<bool>(
      "gimbal_follow.hold_last_alignment", true);
    gimbal_follow_adapter_ = std::make_unique<GimbalFollowAdapter>(follow_params);

    // ── MPC 参数层：采样周期、预测时域、运动约束与代价权重。
    MPCConfig config;
    config.dt = declare_parameter<double>("mpc.dt");
    config.lookahead_time = declare_parameter<double>("mpc.lookahead_time");
    config.horizon = std::max(1, static_cast<int>(std::ceil(config.lookahead_time / config.dt)));
    config.vx_min = declare_parameter<double>("mpc.constraints.velocity.x.min");
    config.vx_max = declare_parameter<double>("mpc.constraints.velocity.x.max");
    config.vy_min = declare_parameter<double>("mpc.constraints.velocity.y.min");
    config.vy_max = declare_parameter<double>("mpc.constraints.velocity.y.max");
    config.omega_min = declare_parameter<double>("mpc.constraints.velocity.omega.min");
    config.omega_max = declare_parameter<double>("mpc.constraints.velocity.omega.max");
    config.use_acc_constraints = declare_parameter<bool>("mpc.constraints.acceleration.enable");
    config.ax_min = declare_parameter<double>("mpc.constraints.acceleration.x.min");
    config.ax_max = declare_parameter<double>("mpc.constraints.acceleration.x.max");
    config.ay_min = declare_parameter<double>("mpc.constraints.acceleration.y.min");
    config.ay_max = declare_parameter<double>("mpc.constraints.acceleration.y.max");
    config.alpha_min = declare_parameter<double>("mpc.constraints.acceleration.omega.min");
    config.alpha_max = declare_parameter<double>("mpc.constraints.acceleration.omega.max");

    const auto q = declare_parameter<std::vector<double>>("mpc.weights.state");
    const auto r = declare_parameter<std::vector<double>>("mpc.weights.command");
    if (q.size() != 3U || r.size() != 3U) {
      throw std::invalid_argument("mpc.weights.state and command must each contain 3 values");
    }
    config.Q = Eigen::Vector3d(q[0], q[1], q[2]);
    config.R = Eigen::Vector3d(r[0], r[1], r[2]);
    // State x/y remain supported as the legacy fallback when the explicit
    // along/cross keys are absent. The current YAML sets both explicitly so the
    // effective 3.0/12.0 behaviour is preserved and tunable.
    config.q_along = declare_parameter<double>("mpc.weights.along", q[0]);
    config.q_cross = declare_parameter<double>("mpc.weights.cross", q[1]);
    if (!std::isfinite(config.q_along) || config.q_along <= 0.0 ||
      !std::isfinite(config.q_cross) || config.q_cross <= 0.0) {
      throw std::invalid_argument("mpc.weights.along/cross must be finite and positive");
    }

    // ── 区域执行层：普通能力作为基线；特殊区域命中后平滑切换 mode 和运动限制。
    region_controller_ = std::make_unique<RegionController>(
      static_cast<uint8_t>(normal_mode), normal_max_speed, normal_max_acceleration,
      speed_blend_rate, acceleration_blend_rate);

    if (executor_params.narrow_passage_enable &&
      std::isfinite(swept_radius) && swept_radius > 0.0 &&
      std::isfinite(executor_params.narrow_passage_clearance) &&
      executor_params.narrow_passage_clearance <
        swept_radius / std::sqrt(2.0) + rotation_margin &&
      !allow_undersized_clearance) {
      throw std::invalid_argument(
        "planner.minco_optimizer.narrow_passage_clearance=" +
        std::to_string(executor_params.narrow_passage_clearance) +
        " m must be >= aligned square half-width=" +
        std::to_string(swept_radius / std::sqrt(2.0)) + " m + clearance margin=" +
        std::to_string(rotation_margin) + " m");
    }
    if (control_rate_hz_ <= 0.0 || planner_frequency_ <= 0.0 || config.dt <= 0.0 ||
      !std::isfinite(executor_params.trajectory_timeout_s) || executor_params.trajectory_timeout_s <= 0.0 ||
      !std::isfinite(executor_params.odom_timeout_s) || executor_params.odom_timeout_s <= 0.0 ||
      !std::isfinite(executor_params.rog_map_timeout_s) || executor_params.rog_map_timeout_s <= 0.0 ||
      !std::isfinite(executor_params.rog_map_clearance) || executor_params.rog_map_clearance < 0.0 ||
      !std::isfinite(corridor_radius) || corridor_radius <= 0.0 ||
      !std::isfinite(swept_radius) || swept_radius <= 0.0 ||
      !std::isfinite(rotation_margin) || rotation_margin < 0.0 ||
      swept_radius < corridor_radius ||
      (!allow_undersized_clearance &&
       effectiveClearanceThreshold(executor_params.rog_map_clearance) <
         swept_radius + rotation_margin) ||
      !std::isfinite(executor_params.narrow_passage_speed) ||
      executor_params.narrow_passage_speed <= 0.0 ||
      !std::isfinite(executor_params.clearance_braking.deceleration) ||
      !std::isfinite(executor_params.clearance_braking.reaction_time) ||
      executor_params.clearance_braking.reaction_time < 0.0 ||
      (executor_params.narrow_passage_enable &&
       (!std::isfinite(executor_params.narrow_passage_clearance) ||
        executor_params.narrow_passage_clearance <= 0.0 ||
        executor_params.narrow_passage_clearance >= executor_params.rog_map_clearance)) ||
      (executor_params.narrow_passage_alignment_enable &&
       (!std::isfinite(executor_params.narrow_passage_alignment_lookahead) ||
        executor_params.narrow_passage_alignment_lookahead <= 0.0 ||
        !std::isfinite(executor_params.narrow_passage_alignment_hold_distance) ||
        executor_params.narrow_passage_alignment_hold_distance < 0.0)) ||
      (executor_params.goal_approach.deceleration > 0.0 &&
       !executor_params.goal_approach.valid()) ||
      (executor_params.secondary_reference_enable &&
       (!std::isfinite(executor_params.secondary_reference_angle_deg) ||
        executor_params.secondary_reference_angle_deg <= 0.0 ||
        executor_params.secondary_reference_angle_deg >= 90.0 ||
        !std::isfinite(executor_params.secondary_reference_min_time_scale) ||
        executor_params.secondary_reference_min_time_scale <= 0.0 ||
        executor_params.secondary_reference_min_time_scale > 1.0 ||
        !std::isfinite(executor_params.secondary_reference_narrow_time_scale) ||
        executor_params.secondary_reference_narrow_time_scale <= 0.0 ||
        executor_params.secondary_reference_narrow_time_scale > 1.0))) {
      throw std::invalid_argument(
        "Invalid control timing, clearance, body radius, braking, or secondary reference parameters; "
        "effective clearance must be at least swept radius + rotation margin unless "
        "planner.corridor.allow_undersized_clearance is true");
    }
    if (executor_params.secondary_reference_enable &&
      executor_params.secondary_reference_narrow_time_scale <
      executor_params.secondary_reference_min_time_scale) {
      throw std::invalid_argument(
        "mpc.secondary_reference.narrow_time_scale must be >= min_time_scale");
    }
    if (allow_undersized_clearance) {
      RCLCPP_WARN(get_logger(),
        "Clearance experiment: effective=%.3f m narrow=%.3f m swept_radius=%.3f m "
        "(effective-radius=%+.3f m narrow-radius=%+.3f m); "
        "aligned square half-width=%.3f m",
        effectiveClearanceThreshold(executor_params.rog_map_clearance),
        executor_params.narrow_passage_clearance, swept_radius,
        effectiveClearanceThreshold(executor_params.rog_map_clearance) - swept_radius,
        executor_params.narrow_passage_clearance - swept_radius,
        swept_radius / std::sqrt(2.0));
    }
    if (!std::isfinite(velocity_color_min_) || !std::isfinite(velocity_color_max_) ||
      velocity_color_min_ >= velocity_color_max_) {
      throw std::invalid_argument("visualization velocity color range must be finite and ordered");
    }
    RCLCPP_INFO(get_logger(),
      "Clearance policy: corridor_radius=%.3f m swept_radius=%.3f m regular=%.2f m narrow=%.3f m "
      "brake_decel=%.2f m/s^2 reaction=%.2f s geometric_alignment=%s align_hold=%.2f m",
      corridor_radius, swept_radius,
      effectiveClearanceThreshold(executor_params.rog_map_clearance),
      executor_params.narrow_passage_enable ? executor_params.narrow_passage_clearance :
      effectiveClearanceThreshold(executor_params.rog_map_clearance),
      executor_params.clearance_braking.deceleration,
      executor_params.clearance_braking.reaction_time,
      executor_params.narrow_passage_alignment_enable ? "on" : "off",
      executor_params.narrow_passage_alignment_hold_distance);

    // ── 地图与规划层：TerrainGrid 管静态语义地图；PathPlanner 管全局搜索、MINCO 和在线 ROGMap。
    if (!online_only) terrain_grid_ = std::make_shared<TerrainGrid>();
    path_planner_ = std::make_shared<PathPlanner>(
      terrain_grid_, executor_params.odom_timeout_s, odom_frame_,
      executor_params.rog_map_timeout_s);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    // ── 跟踪执行层：PathExecutor 用 MPC 跟踪轨迹，并在发布前执行命令安全检查。
    path_executor_ = std::make_unique<PathExecutor>(
      config, executor_params, tf_buffer_, terrain_grid_, path_planner_->mapQuery());

    // ── 静态地图输入层：代价图与标签图构成 TerrainGrid 快照；变化后刷新规划约束可视化。
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    if (terrain_grid_) {
      terrain_cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        cost_map_topic, map_qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          terrain_grid_->updateCost(*msg);
          publish_planning_constraints();
          refresh_region_plan();
        });
      terrain_label_sub_ = create_subscription<sensor_msgs::msg::Image>(
        label_map_topic, map_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          terrain_grid_->updateLabels(*msg);
          refresh_region_plan();
        });
      terrain_uphill_sub_ = create_subscription<sensor_msgs::msg::Image>(
        uphill_map_topic, map_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          terrain_grid_->updateDirections(*msg);
          publish_planning_constraints();
        });
    }
    online_terrain_label_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      online_label_topic, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          online_terrain_labels_ = msg;
          online_terrain_labels_received_s_ = now().seconds();
        }
        refresh_region_plan();
      });

    // ── 轨迹输入与区域标注层：对 MINCO 最终轨迹采样地图 label，生成与轨迹绑定的 RegionPlan。
    trajectory_sub_ = create_subscription<interfaces::msg::MpcPositionCommand>(
      trajectory_topic_, rclcpp::QoS(1),
      [this](const interfaces::msg::MpcPositionCommand::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          trajectory_ = msg;
        }
        refresh_region_plan();
        bool annotation_valid;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          annotation_valid = static_cast<bool>(region_plan_);
        }
        if (!annotation_valid) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Region annotation unavailable; trajectory will be held");
        }
        publish_trajectory_visualization(*msg);
      });

    // ── 机器人状态输入层：缓存 odometry；自转前馈和目标分别交给控制与规划入口。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        odom_ = msg;
      });
    spin_sub_ = create_subscription<example_interfaces::msg::Float32>(
      cmd_spin_topic_, rclcpp::QoS(1),
      [this](const example_interfaces::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spin_speed_ = std::isfinite(msg->data) ? msg->data : 0.0;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) { accept_goal(goal); });

    // ── 实时命令输出层：/cmd_vel 用于 ROS 速度接口；ChassisCommand 将 vx/vy/mode 同周期交给底盘桥。
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(1));
    chassis_command_pub_ = create_publisher<interfaces::msg::ChassisCommand>(
      chassis_cmd_topic_, rclcpp::QoS(1));
    region_status_pub_ = create_publisher<interfaces::msg::RegionStatus>(
      region_status_topic_, rclcpp::QoS(1));

    // ── 规划诊断输出层：发布 MINCO 轨迹、全局搜索路线、规划约束和区域控制状态。
    minco_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      minco_path_topic_, rclcpp::QoS(1).transient_local());
    global_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(1).transient_local());
    global_plan_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      // depth 要 > 1：transient_local 的 durability 缓存只保留最后 depth 条，而折线与
      // 终点球是两条独立消息，depth=1 时后打开 RViz 只能拿到终点球、看不到折线。
      global_plan_marker_topic_, rclcpp::QoS(10).transient_local());
    minco_trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      minco_trajectory_topic_, rclcpp::QoS(1).transient_local());
    planning_constraints_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      planning_constraints_topic_, rclcpp::QoS(1).reliable().transient_local());
    planning_constraints_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      planning_constraints_marker_topic_, rclcpp::QoS(1).reliable().transient_local());

    // ── 调度层：固定频率计算控制命令；较低频率重发全局路线供后打开的 RViz 显示。
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&NavExecutorNode::control_tick, this));
    global_plan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / global_plan_publish_hz_),
      std::bind(&NavExecutorNode::publish_global_plan, this));
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr planner_node() const
  {
    return path_planner_->node();
  }

  void shutdown_planner()
  {
    path_planner_.reset();
  }

private:
  // 地图可视化层：把 TerrainGrid 约束栅格和受限/禁止点转换成 OccupancyGrid 与 Marker。
  void publish_planning_constraints()
  {
    if (!terrain_grid_) return;
    auto constraints = terrain_grid_->planningConstraints();
    if (!constraints) return;
    constraints->header.stamp = now();
    planning_constraints_pub_->publish(*constraints);

    visualization_msgs::msg::Marker markers;
    markers.header = constraints->header;
    markers.ns = "planning_constraints";
    markers.id = 0;
    markers.type = visualization_msgs::msg::Marker::POINTS;
    markers.action = visualization_msgs::msg::Marker::ADD;
    markers.pose.orientation.w = 1.0;
    markers.scale.x = std::max(0.04F, constraints->info.resolution * 0.9F);
    markers.scale.y = markers.scale.x;
    for (size_t index = 0; index < constraints->data.size(); ++index) {
      const int8_t value = constraints->data[index];
      if (value <= 0) continue;
      geometry_msgs::msg::Point point;
      point.x = constraints->info.origin.position.x +
        (static_cast<double>(index % constraints->info.width) + 0.5) * constraints->info.resolution;
      point.y = constraints->info.origin.position.y +
        (static_cast<double>(index / constraints->info.width) + 0.5) * constraints->info.resolution;
      point.z = 0.06;
      std_msgs::msg::ColorRGBA color;
      color.a = 1.0F;
      if (value >= 100) {
        color.r = 1.0F; color.g = 0.05F; color.b = 0.05F;
      } else {
        color.r = 1.0F; color.g = 1.0F; color.b = 0.0F;
      }
      markers.points.push_back(point);
      markers.colors.push_back(color);
    }
    planning_constraints_marker_pub_->publish(markers);
  }

  // 局部轨迹可视化层：把 MINCO 命令点转换为 Path，并按速度生成着色轨迹 Marker。
  void publish_trajectory_visualization(const interfaces::msg::MpcPositionCommand & trajectory)
  {
    if (trajectory.cmds.empty()) return;

    nav_msgs::msg::Path path;
    path.header = trajectory.header;
    if (path.header.frame_id.empty()) path.header.frame_id = odom_frame_;
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "minco_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.07;
    path.poses.reserve(trajectory.cmds.size());
    marker.points.reserve(trajectory.cmds.size());
    marker.colors.reserve(trajectory.cmds.size());
    for (const auto & command : trajectory.cmds) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = command.position;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
        0.0, 0.0, std::sin(command.yaw * 0.5), std::cos(command.yaw * 0.5)));
      path.poses.push_back(pose);
      marker.points.push_back(command.position);
      marker.colors.push_back(
        velocity_color(command.vel_norm, velocity_color_min_, velocity_color_max_));
    }
    minco_path_pub_->publish(path);
    minco_trajectory_pub_->publish(marker);
  }

  // 全局路线可视化层：显示全局搜索器（SMAC 2D / Astar，odom 系）的折线。与 MINCO 轨迹分开显示：轨迹是
  // 「车准备怎么走」，这条线是「搜索给出的拓扑引导」，两者对不上时一眼就能看出来。
  void publish_global_plan()
  {
    if (!path_planner_ || !global_plan_pub_ || !global_plan_marker_pub_) return;

    std::vector<geometry_msgs::msg::PoseStamped> plan;
    const bool has_plan = path_planner_->copyLatestGlobalPath(plan) && plan.size() >= 2U;

    if (!has_plan) {
      // 目标被 invalidate / 搜索失败时清掉 RViz 上的旧线，否则会看着像还有一条可用路径。
      if (!global_plan_published_) return;
      global_plan_published_ = false;
      nav_msgs::msg::Path empty;
      empty.header.frame_id = odom_frame_;
      empty.header.stamp = now();
      global_plan_pub_->publish(empty);
      for (int id = 0; id <= 1; ++id) {
        visualization_msgs::msg::Marker clear;
        clear.header = empty.header;
        clear.ns = "global_plan";
        clear.id = id;
        clear.action = visualization_msgs::msg::Marker::DELETE;
        global_plan_marker_pub_->publish(clear);
      }
      return;
    }

    const auto & last = plan.back();

    nav_msgs::msg::Path path;
    path.header.frame_id = plan.front().header.frame_id.empty()
      ? odom_frame_ : plan.front().header.frame_id;
    path.header.stamp = now();
    path.poses = plan;

    // 抬高 3 cm，避免与 z=0 的代价图/规划约束栅格打架闪面。
    const double z_offset = 0.03;
    visualization_msgs::msg::Marker line;
    line.header = path.header;
    line.ns = "global_plan";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.pose.position.z = z_offset;
    line.scale.x = global_plan_line_width_;
    line.color.r = static_cast<float>(std::clamp(global_plan_color_r_, 0.0, 1.0));
    line.color.g = static_cast<float>(std::clamp(global_plan_color_g_, 0.0, 1.0));
    line.color.b = static_cast<float>(std::clamp(global_plan_color_b_, 0.0, 1.0));
    line.color.a = 1.0F;
    line.points.reserve(plan.size());
    for (const auto & pose : plan) {
      line.points.push_back(pose.pose.position);
    }

    // 终点球：让「全局搜索最后停在哪」也一目了然（到点容差内可能不是精确目标点）。
    visualization_msgs::msg::Marker goal_dot = line;
    goal_dot.id = 1;
    goal_dot.type = visualization_msgs::msg::Marker::SPHERE;
    goal_dot.points.clear();
    goal_dot.pose.position = last.pose.position;
    goal_dot.pose.position.z += z_offset;
    goal_dot.scale.x = global_plan_line_width_ * 2.5;
    goal_dot.scale.y = global_plan_line_width_ * 2.5;
    goal_dot.scale.z = global_plan_line_width_ * 2.5;

    global_plan_pub_->publish(path);
    global_plan_marker_pub_->publish(line);
    global_plan_marker_pub_->publish(goal_dot);
    // 每个 tick 都重发（不是只在变化时发）：RViz 常在本节点之后才打开，靠 transient_local
    // 缓存定期重发才能看到完整折线，也避免 Marker 被后续消息挤出；5 Hz 的开销可忽略。
    global_plan_published_ = true;
  }

  // 任务接入层：检查目标消息基本有效性，再交给 PathPlanner 检查地图、定位、TF 和在线地图。
  void accept_goal(const geometry_msgs::msg::PoseStamped::SharedPtr & goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal on %s", goal_topic_.c_str());
    if (!goal || !std::isfinite(goal->pose.position.x) ||
      !std::isfinite(goal->pose.position.y) || !std::isfinite(goal->pose.position.z)) {
      RCLCPP_WARN(get_logger(), "Ignoring an invalid navigation goal");
      return;
    }
    if (!path_planner_->acceptGoal(*goal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring goal until required maps, TF and odometry are ready (%s)", odom_topic_.c_str());
    }
  }

  bool online_labels_fresh(
    const nav_msgs::msg::OccupancyGrid & labels, double received_s, double current_s) const
  {
    if (persistent_online_terrain_) {
      return !labels.header.frame_id.empty() && labels.info.width > 0 &&
        labels.info.height > 0 &&
        labels.data.size() == static_cast<size_t>(labels.info.width) * labels.info.height;
    }
    const double stamp_s = rclcpp::Time(labels.header.stamp).seconds();
    const double stamp_age = current_s - stamp_s;
    const double receive_age = current_s - received_s;
    return !labels.header.frame_id.empty() &&
      std::isfinite(stamp_age) && stamp_age >= -0.05 &&
      stamp_age <= online_label_timeout_s_ &&
      std::isfinite(receive_age) && receive_age >= 0.0 &&
      receive_age <= online_label_timeout_s_;
  }

  void refresh_region_plan()
  {
    interfaces::msg::MpcPositionCommand::SharedPtr trajectory;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr online_labels;
    double received_s;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      trajectory = trajectory_;
      online_labels = online_terrain_labels_;
      received_s = online_terrain_labels_received_s_;
    }
    if (!trajectory) return;
    const double current_s = now().seconds();
    const bool online_current = online_labels &&
      online_labels_fresh(*online_labels, received_s, current_s);
    bool online_applied = online_current;
    std::shared_ptr<const RegionPlan> plan;
    if (const auto terrain = terrain_grid_ ? terrain_grid_->snapshot() : nullptr) {
      auto annotated = annotateRegions(*trajectory, *terrain, *tf_buffer_,
        region_rules_, odom_frame_, online_current ? online_labels.get() : nullptr);
      if (!annotated && online_current) {
        // Conflicting online regions can create overlapping mode windows.
        // Keep the known static annotation for this update.
        annotated = annotateRegions(*trajectory, *terrain, *tf_buffer_,
          region_rules_, odom_frame_);
        online_applied = false;
      }
      if (annotated) {
        plan = std::make_shared<RegionPlan>(std::move(*annotated));
      }
    } else if (!terrain_grid_) {
      auto annotated = annotateOnlineRegions(*trajectory, *tf_buffer_, region_rules_,
        odom_frame_, online_current ? online_labels.get() : nullptr);
      if (!annotated && online_current) {
        // Conflicting online mode windows must not stop ordinary driving. Retry
        // without semantic labels so the trajectory gets a normal, segment-free
        // plan; the next replan will try the live labels again.
        annotated = annotateOnlineRegions(*trajectory, *tf_buffer_, region_rules_,
          odom_frame_, nullptr);
        online_applied = false;
        if (annotated) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Conflicting online terrain labels; using normal region profile for this trajectory");
        }
      }
      if (annotated) plan = std::make_shared<RegionPlan>(std::move(*annotated));
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (trajectory_ == trajectory && online_terrain_labels_ == online_labels) {
        region_plan_ = std::move(plan);
        online_plan_active_ = online_applied;
      }
    }
  }

  // 实时控制层：快照输入 → 判断轨迹许可 → 更新区域进度/限速 → MPC 与安全门 → 发布命令和状态。
  void control_tick()
  {
    bool online_expired;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      online_expired = online_plan_active_ &&
        (!online_terrain_labels_ || !online_labels_fresh(
          *online_terrain_labels_, online_terrain_labels_received_s_, now().seconds()));
    }
    if (online_expired) refresh_region_plan();
    ExecutorInput input;
    std::shared_ptr<const RegionPlan> region_plan;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      input.trajectory = trajectory_;
      input.odom = odom_;
      input.spin_speed = spin_speed_;
      region_plan = region_plan_;
    }
    input.stamp = now();
    input.allow_motion = input.trajectory &&
      region_plan && path_planner_->acceptsTrajectory(rclcpp::Time(input.trajectory->header.stamp));

    region_controller_->setPlan(region_plan);
    if (input.odom) {
      region_controller_->update({input.odom->pose.pose.position.x,
        input.odom->pose.pose.position.y});
    }
    region_controller_->tickProfile(1.0 / control_rate_hz_);
    input.region_speed_limit = region_controller_->speedLimit();
    input.region_acceleration_limit = region_controller_->accelerationLimit();
    if (input.odom &&
      std::abs((input.stamp - rclcpp::Time(input.odom->header.stamp)).seconds()) <=
      alignment_odom_timeout_s_ &&
      std::isfinite(input.odom->pose.pose.position.x) &&
      std::isfinite(input.odom->pose.pose.position.y)) {
      path_executor_->updateNarrowPassageAlignmentPose(
        input.odom->pose.pose.position.x, input.odom->pose.pose.position.y);
    }
    if (!path_planner_->hasActiveGoal() &&
        path_executor_->releaseNarrowPassageAlignmentAtGoal()) {
      RCLCPP_INFO(get_logger(),
        "Narrow alignment completed at goal after %.2f m of odometry travel",
        path_executor_->narrowPassageAlignmentTraveled());
    }

    std::optional<double> heading_error;
    bool semantic_alignment = false;
    // Follow a valid road heading even while the MPC or collision gate brakes translation.
    // The lower board interprets yaw as a position offset from auto_yaw_offset, not a live LIO error.
    if (input.allow_motion && input.odom && input.trajectory && !input.trajectory->cmds.empty() &&
      std::abs((input.stamp - rclcpp::Time(input.odom->header.stamp)).seconds()) <= alignment_odom_timeout_s_ &&
      std::abs((input.stamp - rclcpp::Time(input.trajectory->header.stamp)).seconds()) <= alignment_trajectory_timeout_s_) {
      const auto & q = input.odom->pose.pose.orientation;
      const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (std::isfinite(norm_squared) && norm_squared > 1e-12 &&
        std::isfinite(input.odom->pose.pose.position.x) &&
        std::isfinite(input.odom->pose.pose.position.y)) {
        const double lio_yaw = tf2::getYaw(q);
        // Once the square has entered a geometric narrow section, keep its
        // latched corridor heading even if a terrain label also requests a
        // semantic heading. Switching targets inside the slot would rotate
        // the corners and repeatedly revoke heading readiness.
        if (path_executor_->narrowPassageAlignmentActive()) {
          heading_error = path_executor_->narrowPassageHeadingErrorDegrees(
            *input.trajectory, input.odom->pose.pose.position.x,
            input.odom->pose.pose.position.y, lio_yaw);
        }
        if (!heading_error) {
          heading_error = region_controller_->trajectoryHeadingErrorDegrees(lio_yaw);
          semantic_alignment = heading_error.has_value();
          if (!heading_error) {
            // 5.5.5.4.4: align only where both sides of a low-clearance route
            // are constrained, before the body reaches the corridor mouth.
            heading_error = path_executor_->narrowPassageHeadingErrorDegrees(
              *input.trajectory, input.odom->pose.pose.position.x,
              input.odom->pose.pose.position.y, lio_yaw);
          }
        }
      }
    }
    bool odom_pose_current = false;
    if (input.odom &&
      std::abs((input.stamp - rclcpp::Time(input.odom->header.stamp)).seconds()) <=
      alignment_odom_timeout_s_) {
      const auto & q = input.odom->pose.pose.orientation;
      const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      odom_pose_current = std::isfinite(norm_squared) && norm_squared > 1e-12 &&
        std::isfinite(input.odom->pose.pose.position.x) &&
        std::isfinite(input.odom->pose.pose.position.y);
    }
    // A planner retry gap must not hand the chassis back to its free-spin mode.
    // The lower board keeps the gimbal target relative to auto_yaw_offset; if we
    // clear follow and ref=0 between trajectories, the chassis spins while the
    // next trajectory is searched and the next translation command can end up in
    // the opposite direction. Hold the last valid angle with zero translation
    // until the same active goal yields another trajectory (or is finished).
    const bool hold_gimbal_alignment = !heading_error &&
      gimbal_follow_adapter_->hasCommand() &&
      (path_executor_->narrowPassageAlignmentActive() ||
       (gimbal_hold_enabled_ && input.trajectory && !input.allow_motion &&
        odom_pose_current && path_planner_->hasActiveGoal()));
    const auto follow = hold_gimbal_alignment ?
      gimbal_follow_adapter_->hold() :
      gimbal_follow_adapter_->updateHeading(heading_error, 1.0 / control_rate_hz_);
    input.follow_gimbal = follow.follow_gimbal;
    input.reverse_follow_heading = follow.reverse_heading;
    // The aligned square can translate through a slot narrower than its
    // rotating envelope. Wait for measured heading before entering it.
    input.hold_translation = follow.follow_gimbal && !follow.heading_ready &&
      (semantic_alignment || path_executor_->narrowPassageAlignmentActive());
    if (heading_error) {
      input.alignment_heading_odom = tf2::getYaw(input.odom->pose.pose.orientation) +
        follow.heading_error_deg * (3.14159265358979323846 / 180.0);
    }
    ExecutorOutput output = path_executor_->computeCommand(input);
    if (output.secondary_reference_applied) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Secondary MPC reference: max_angle=%.1f deg min_scale=%.2f narrow=%s solve=%s %.2f ms",
        output.secondary_reference_max_angle_deg,
        output.secondary_reference_min_time_scale,
        output.secondary_reference_narrow ? "true" : "false",
        output.secondary_reference_solve_ok ? "ok" : "fallback",
        output.secondary_reference_solve_ms);
    }
    if (output.goal_approach_capped) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Goal-approach speed cap: remaining=%.2f m allowed=%.2f m/s (decel=%.2f m/s^2)",
        output.goal_approach_remaining_m,
        output.goal_approach_allowed_speed,
        goal_approach_deceleration_);
    }
    if (output.clearance_braking_capped) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Clearance braking cap: ESDF=%.2f m allowed=%.2f m/s",
        output.clearance_braking_distance_m,
        output.clearance_braking_allowed_speed);
    }
    const double mpc_omega = output.status == ExecutorStatus::PUBLISHED ? output.command.angular.z : 0.0;
    const double ref_angle_deg = hold_gimbal_alignment ?
      gimbal_follow_adapter_->commandDegrees() :
      gimbal_follow_adapter_->integrateMpcYawRate(mpc_omega, 1.0 / control_rate_hz_);
    if (input.hold_translation) {
      output.command.linear.x = 0.0;
      output.command.linear.y = 0.0;
    }
    // When following (or holding the last alignment), the lower PID owns
    // rotation; otherwise publish +2 rad/s for observation only. The bridge
    // does not receive a yaw rate.
    if (hold_gimbal_alignment) {
      output.command.linear.x = 0.0;
      output.command.linear.y = 0.0;
      output.command.angular.z = 0.0;
    } else if (!follow.follow_gimbal) {
      output.command.angular.z = kNonFollowYawRateRadS;
    }
    command_pub_->publish(output.command);
    interfaces::msg::ChassisCommand chassis_command;
    chassis_command.header.stamp = input.stamp;
    chassis_command.header.frame_id = "base_link";
    chassis_command.vx = static_cast<float>(output.command.linear.x);
    chassis_command.vy = static_cast<float>(output.command.linear.y);
    chassis_command.mode = region_controller_->mode();
    chassis_command.small_yaw_ref = static_cast<float>(ref_angle_deg);
    chassis_command.follow_gimbal = follow.follow_gimbal;
    if (!has_follow_gimbal_state_ || chassis_command.follow_gimbal != last_follow_gimbal_) {
      RCLCPP_INFO(get_logger(),
        "Chassis alignment transition: follow=%s geometric=%s traveled=%.2f m "
        "hold=%s heading=%s allow_motion=%s active_goal=%s odom_current=%s",
        chassis_command.follow_gimbal ? "true" : "false",
        path_executor_->narrowPassageAlignmentActive() ? "true" : "false",
        path_executor_->narrowPassageAlignmentTraveled(),
        hold_gimbal_alignment ? "true" : "false",
        heading_error ? "true" : "false",
        input.allow_motion ? "true" : "false",
        path_planner_->hasActiveGoal() ? "true" : "false",
        odom_pose_current ? "true" : "false");
      last_follow_gimbal_ = chassis_command.follow_gimbal;
      has_follow_gimbal_state_ = true;
    }
    chassis_command_pub_->publish(chassis_command);
    interfaces::msg::RegionStatus region_status;
    region_status.header.stamp = input.stamp;
    region_status.header.frame_id = odom_frame_;
    region_status.trajectory_id = region_plan ? region_plan->trajectory_id : 0;
    region_status.terrain_label = region_controller_->label().value_or(0);
    region_status.mode = region_controller_->mode();
    region_status.phase = static_cast<uint8_t>(region_controller_->phase());
    region_status.path_progress = static_cast<float>(region_controller_->progress());
    region_status.speed_limit = static_cast<float>(region_controller_->speedLimit());
    region_status.gimbal_heading_error_deg = static_cast<float>(follow.heading_error_deg);
    region_status.gimbal_heading_ready = follow.heading_ready;
    region_status_pub_->publish(region_status);
    if (!has_region_status_ || region_status.terrain_label != last_region_label_ ||
      region_status.mode != last_region_mode_) {
      const double x = input.odom ? input.odom->pose.pose.position.x : 0.0;
      const double y = input.odom ? input.odom->pose.pose.position.y : 0.0;
      RCLCPP_INFO(get_logger(),
        "Region status transition: label=%u mode=%u phase=%u online=%s "
        "position=(%.2f, %.2f) progress=%.2f m speed_limit=%.2f m/s",
        static_cast<unsigned int>(region_status.terrain_label),
        static_cast<unsigned int>(region_status.mode),
        static_cast<unsigned int>(region_status.phase),
        online_plan_active_ ? "true" : "false", x, y,
        static_cast<double>(region_status.path_progress),
        static_cast<double>(region_status.speed_limit));
      last_region_label_ = region_status.terrain_label;
      last_region_mode_ = region_status.mode;
      has_region_status_ = true;
    }
    if (follow.follow_gimbal && std::abs(follow.heading_error_deg) > 5.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Gimbal axis not aligned: error=%.1f deg ref=%.1f deg reverse=%s",
        follow.heading_error_deg, ref_angle_deg, follow.reverse_heading ? "true" : "false");
    } else if (hold_gimbal_alignment) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Holding gimbal alignment ref=%.1f deg with zero translation while the "
        "active goal waits for a new trajectory", ref_angle_deg);
    }
    if (output.status == ExecutorStatus::REFERENCE_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Braking: trajectory cannot be expressed in %s", odom_frame_.c_str());
    } else if (output.status == ExecutorStatus::SOLVER_FAILED) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Braking: MPC solve failed");
    } else if (output.status == ExecutorStatus::TERRAIN_BLOCKED ||
               output.status == ExecutorStatus::DYNAMIC_BLOCKED) {
      // 动态层缺帧 / 地形层 transition 拒绝 / TF 拿不到 / 净空或动态走廊违规会归到同一
      // 状态，故打出命令门给出的判据名与实测值，便于区分具体成因。
      const auto & d = path_executor_->lastSafetyDetail();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Braking: %s (reason=%s value=%.3f threshold=%.3f)",
        output.status == ExecutorStatus::TERRAIN_BLOCKED ?
          "terrain layer or map transform unavailable, or next command violates terrain" :
          "current dynamic obstacle intersects the MPC reference horizon",
        d.reason, d.value, d.threshold);
    }
  }

  // 算法组件层：规划器、轨迹执行器、静态地形快照和区域状态机。
  std::unique_ptr<PathExecutor> path_executor_;
  std::shared_ptr<TerrainGrid> terrain_grid_;
  std::array<RegionRule, 3> region_rules_{};
  std::unique_ptr<RegionController> region_controller_;
  std::unique_ptr<GimbalFollowAdapter> gimbal_follow_adapter_;
  bool gimbal_hold_enabled_{true};
  bool last_follow_gimbal_{false};
  bool has_follow_gimbal_state_{false};
  double alignment_odom_timeout_s_{};
  double alignment_trajectory_timeout_s_{};
  /// 目标接近减速带用的实测刹车能力（<=0 表示关闭），仅用于节流日志。
  double goal_approach_deceleration_{0.0};
  std::shared_ptr<const RegionPlan> region_plan_;
  double online_label_timeout_s_{0.8};
  bool online_plan_active_{false};
  double online_terrain_labels_received_s_{0.0};
  uint8_t last_region_label_{255U};
  uint8_t last_region_mode_{255U};
  bool has_region_status_{false};
  bool persistent_online_terrain_{false};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr online_terrain_labels_;
  // 地图输入接口层：静态代价和 label 地图；动态 ROGMap 由 PathPlanner 内部维护。
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_cost_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr terrain_label_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr terrain_uphill_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr online_terrain_label_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<PathPlanner> path_planner_;
  // 共享运行状态层：订阅回调更新缓存；控制周期加锁复制后独立计算。
  std::mutex data_mutex_;
  interfaces::msg::MpcPositionCommand::SharedPtr trajectory_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  rclcpp::Subscription<interfaces::msg::MpcPositionCommand>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  // 实时控制输出接口层：常规速度、带 mode 的底盘命令及区域阶段诊断。
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<interfaces::msg::ChassisCommand>::SharedPtr chassis_command_pub_;
  rclcpp::Publisher<interfaces::msg::RegionStatus>::SharedPtr region_status_pub_;
  // 可视化输出接口层：局部/全局路径、优化轨迹和地图规划约束。
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr minco_path_pub_;
  // 全局搜索折线；MINCO 局部优化轨迹由 minco_path_pub_ 发布。
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_plan_marker_pub_;
  rclcpp::TimerBase::SharedPtr global_plan_timer_;
  bool global_plan_published_{false};
  double global_plan_publish_hz_{5.0};
  double global_plan_line_width_{0.15};
  double global_plan_color_r_{0.0};
  double global_plan_color_g_{1.0};
  double global_plan_color_b_{1.0};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr minco_trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_constraints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planning_constraints_marker_pub_;
  // 调度与可视化参数层：控制定时器、全局路线重发频率和显示样式。
  rclcpp::TimerBase::SharedPtr timer_;
  double control_rate_hz_{};
  double planner_frequency_{};
  double spin_speed_{};
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string odom_frame_;
  std::string output_topic_;
  std::string chassis_cmd_topic_;
  std::string region_status_topic_;
  std::string minco_path_topic_;
  std::string global_plan_topic_;
  std::string global_plan_marker_topic_;
  std::string minco_trajectory_topic_;
  std::string planning_constraints_topic_;
  std::string planning_constraints_marker_topic_;
  std::string cmd_spin_topic_;
  std::string goal_topic_;
  double velocity_color_min_{};
  double velocity_color_max_{};
};

}  // namespace mas2027_nav_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mas2027_nav_executor::NavExecutorNode>();

  // ROS 运行容器层：同一 executor 同时调度接口节点与 PathPlanner 内部生命周期节点。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->planner_node()->get_node_base_interface());
  executor.spin();

  // 先释放规划器及其 ROGMap/MINCO 资源，再关闭 ROS。
  node->shutdown_planner();
  rclcpp::shutdown();
  return 0;
}
