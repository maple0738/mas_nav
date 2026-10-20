// Corresponding header
#include "mas2027_nav_executor/path_planner/trajectory/minco_planner.hpp"

#include "mas2027_nav_executor/common/environment/fused_map_query.hpp"
#include "mas2027_nav_executor/common/environment/terrain_map_query.hpp"
#include "mas2027_nav_executor/common/environment/clearance_gate.hpp"
#include "mas2027_nav_executor/common/environment/terrain_semantics.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "mas2027_nav_executor/path_planner/search/route_commitment.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/arc_length_speed_profile.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/replan_continuity.hpp"
#include "rog_map/query_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include "tf2/utils.h"

namespace minco_planner {

namespace {

// 地形门否决日志节流：前 N 条逐条打印、之后每 M 次采样一条，不按时间节流以免失败密集时丢光现场。
constexpr uint64_t kTerrainRejectLogFirstN = 10;
constexpr uint64_t kTerrainRejectLogEveryN = 50;

bool onlineSlopeTransition(const rog_map::MapSnapshot & map,
  const std::vector<uint8_t> & sides,
  const Eigen::Vector3d & from, const Eigen::Vector3d & to)
{
  return mas2027_nav_executor::terrainSideTransitionAllowed(map.width, map.height,
    map.resolution, map.origin_x, map.origin_y, sides,
    from.x(), from.y(), to.x(), to.y(), false);
}

void declareParameterIfMissing(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, default_value);
  }
}

}  // namespace

using namespace color_text;

MincoPlanner::MincoPlanner() : tf_(nullptr)
{
}

MincoPlanner::~MincoPlanner()
{
  planner_perf_monitor_.close();
}

void MincoPlanner::configureMincoPerfLogging(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & prefix)
{
  const std::string default_minco_csv_path = "/tmp/minco_perf_detailed.csv";

  declareParameterIfMissing(
    node, prefix + "performance.enable", rclcpp::ParameterValue(true));
  declareParameterIfMissing(
    node, prefix + "performance.print_enable", rclcpp::ParameterValue(true));
  declareParameterIfMissing(
    node, prefix + "performance.detailed_csv_enable", rclcpp::ParameterValue(false));
  declareParameterIfMissing(
    node, prefix + "performance.odom_sub_debug_enable", rclcpp::ParameterValue(true));
  declareParameterIfMissing(
    node, prefix + "performance.print_period_sec", rclcpp::ParameterValue(1.0));
  declareParameterIfMissing(
    node, prefix + "performance.csv_flush_every_n", rclcpp::ParameterValue(30));
  declareParameterIfMissing(
    node, prefix + "performance.minco_csv_path", rclcpp::ParameterValue(default_minco_csv_path));
  declareParameterIfMissing(
    node, prefix + "performance.run_id", rclcpp::ParameterValue(""));
  declareParameterIfMissing(
    node, prefix + "performance.scenario", rclcpp::ParameterValue(""));
  declareParameterIfMissing(
    node, prefix + "performance.variant", rclcpp::ParameterValue(""));

  bool performance_enable = true;
  bool print_enable = true;
  bool detailed_csv_enable = false;
  bool odom_sub_debug_enable = true;
  double print_period_sec = 1.0;
  int csv_flush_every_n = 30;
  std::string minco_csv_path = default_minco_csv_path;
  std::string run_id;
  std::string scenario;
  std::string variant;
  node->get_parameter(prefix + "performance.enable", performance_enable);
  node->get_parameter(prefix + "performance.print_enable", print_enable);
  node->get_parameter(prefix + "performance.detailed_csv_enable", detailed_csv_enable);
  node->get_parameter(prefix + "performance.odom_sub_debug_enable", odom_sub_debug_enable);
  node->get_parameter(prefix + "performance.print_period_sec", print_period_sec);
  node->get_parameter(prefix + "performance.csv_flush_every_n", csv_flush_every_n);
  node->get_parameter(prefix + "performance.minco_csv_path", minco_csv_path);
  node->get_parameter(prefix + "performance.run_id", run_id);
  node->get_parameter(prefix + "performance.scenario", scenario);
  node->get_parameter(prefix + "performance.variant", variant);

  PlannerPerformanceConfig perf_cfg;
  perf_cfg.enable = performance_enable;
  perf_cfg.print_enable = print_enable;
  perf_cfg.detailed_csv_enable = detailed_csv_enable;
  perf_cfg.odom_sub_debug_enable = odom_sub_debug_enable;
  perf_cfg.detailed_csv_path = minco_csv_path;
  perf_cfg.run_id = run_id;
  perf_cfg.scenario = scenario;
  perf_cfg.variant = variant;
  perf_cfg.print_period_sec = print_period_sec;
  perf_cfg.csv_flush_every_n = csv_flush_every_n;

  planner_perf_monitor_.configure(perf_cfg, logger_);
}

bool MincoPlanner::ensureMapAvailable()
{
  if (rog_query_raw_ || map_) {
    return true;
  }

  auto node = node_.lock();
  if (node) {
    RCLCPP_ERROR_THROTTLE(logger_,
      *node->get_clock(),
      1000,
      "[MincoPlanner] MapQueryInterface unavailable: ROGMap was not injected.");
  } else {
    RCLCPP_ERROR(logger_, "[MincoPlanner] MapQueryInterface unavailable.");
  }
  return false;
}

void MincoPlanner::rebuildModeDependentQueries()
{
  if (!mode_context_) {
    return;
  }

  mode_context_->rebuildQueries(rog_query_raw_, tf_, logger_);
  map_ = mode_context_->dynamicQuery();

  if (global_path_searcher_) {
    global_path_searcher_->setQuery(mode_context_->globalQuery());
  }
  if (astar_planner_) {
    astar_planner_->setMap(mode_context_->globalQuery());
  }
  if (smac_planner_) {
    smac_planner_->setMap(mode_context_->globalQuery());
    smac_planner_->setESDFQuery(mode_context_->dynamicQuery());
  }
  if (minco_optimizer_) {
    minco_optimizer_->setMap(mode_context_->dynamicQuery());
  }
  if (corridor_gen_) {
    corridor_gen_->setMap(mode_context_->dynamicQuery());
  }
  if (safety_checker_) {
    safety_checker_->setQuery(mode_context_->dynamicQuery());
  }

  rebuildFusedQuery();

  RCLCPP_INFO(logger_, "[MincoPlanner] Rebuilt ROGMap queries.");
}

void MincoPlanner::initPlannerMode(
  const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame)
{
  mode_params_.planner_mode = planner_mode_param;
  mode_params_.map_frame = map_frame.empty() ? "map" : map_frame;
  mode_params_.rog_frame = rog_frame.empty() ? "camera_init" : rog_frame;
  mode_params_.exploration_boundary_margin = exploration_boundary_margin_;
  mode_params_.exploration_boundary_sample_step = exploration_boundary_sample_step_;
  mode_params_.exploration_unknown_as_occupied = exploration_unknown_as_occupied_;
  mode_params_.exploration_prefer_goal_direction = exploration_prefer_goal_direction_;

  mode_context_ = std::make_unique<PlannerModeContext>();
  mode_context_->configure(mode_params_, rog_query_raw_, tf_, logger_);

  planning_frame_ = mode_context_->planningFrame();
  output_frame_ = mode_context_->outputFrame();
  map_frame_ = mode_context_->mapFrame();
  rog_frame_ = mode_context_->rogFrame();
  map_ = mode_context_->dynamicQuery();

  // 打印实际配置的 planner_mode，避免硬编码名字误导排障；全局搜索器名字见 configure()。
  RCLCPP_INFO(logger_,
    "[MincoPlanner] planner_mode=%s",
    mode_params_.planner_mode.c_str());
  RCLCPP_INFO(logger_,
    "[MincoPlanner] planning_frame=%s output_frame=%s map_query_frame=%s",
    planning_frame_.c_str(),
    output_frame_.c_str(),
    rog_frame_.c_str());
}

// -----------------------------------------------------------------------------
// 2) Lifecycle management
// -----------------------------------------------------------------------------

void MincoPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;

  auto node = parent.lock();
  logger_ = node->get_logger();

  const std::string prefix = name_ + ".";
  configureMincoPerfLogging(node, prefix);

  // --- General config --------------------------------------------------------

  std::string planner_mode_param = "EXPLORATION";
  declareParameterIfMissing(
    node, prefix + "planner_mode", rclcpp::ParameterValue(planner_mode_param));
  node->get_parameter(prefix + "planner_mode", planner_mode_param);

  std::string configured_map_frame = "map";
  std::string configured_rog_frame = "camera_init";
  declareParameterIfMissing(
    node, prefix + "frames.map_frame", rclcpp::ParameterValue(configured_map_frame));
  declareParameterIfMissing(
    node, prefix + "frames.rog_frame", rclcpp::ParameterValue(configured_rog_frame));
  node->get_parameter(prefix + "frames.map_frame", configured_map_frame);
  node->get_parameter(prefix + "frames.rog_frame", configured_rog_frame);

  declareParameterIfMissing(
    node, prefix + "exploration.boundary_margin", rclcpp::ParameterValue(0.8));
  node->get_parameter(prefix + "exploration.boundary_margin", exploration_boundary_margin_);

  declareParameterIfMissing(
    node, prefix + "exploration.boundary_sample_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(prefix + "exploration.boundary_sample_step", exploration_boundary_sample_step_);

  declareParameterIfMissing(
    node, prefix + "exploration.unknown_as_occupied", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.unknown_as_occupied", exploration_unknown_as_occupied_);

  declareParameterIfMissing(
    node, prefix + "exploration.prefer_goal_direction", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "exploration.prefer_goal_direction", exploration_prefer_goal_direction_);

  std::string configured_global_frame = "map";
  
  declareParameterIfMissing(
    node, prefix + "global_frame", rclcpp::ParameterValue(configured_global_frame));
  node->get_parameter(prefix + "global_frame", configured_global_frame);

  if (!ensureMapAvailable()) {
    throw std::runtime_error("TerrainMapQuery must be injected before MincoPlanner::configure");
  }
  initPlannerMode(planner_mode_param, configured_map_frame, configured_rog_frame);

  declareParameterIfMissing(node, prefix + "tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(prefix + "tolerance", tolerance_);

  declareParameterIfMissing(
    node, prefix + "allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "allow_unknown", allow_unknown_);

  // 全局主搜索选择：对齐 mas_nav_2027 sentry1.yaml 的 use_smac: true，false 时退回 Astar 波前。
  declareParameterIfMissing(node, prefix + "use_smac", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "use_smac", use_smac_);
  // 打印实际生效的全局搜索器，不要硬编码实现名，免得排障时去找一个已删除的搜索器。
  RCLCPP_INFO(logger_,
    "[MincoPlanner] global_search=%s dynamic_query=%s",
    use_smac_ ? "SMAC2D" : "Astar",
    "ROGMap");

  // SMAC 的 ESDF 势场软代价和最小净空；势场默认值对齐 mas_nav_2027 nav2_params.yaml。
  // ESDF 取自 TerrainMapQuery 烘焙的二维距离场（上限 3.0 m），量纲与旧工程 ROGMap 一致。
  declareParameterIfMissing(
    node, prefix + "smac_2d.use_esdf_cost", rclcpp::ParameterValue(true));
  declareParameterIfMissing(
    node, prefix + "smac_2d.esdf_weight", rclcpp::ParameterValue(1.0));
  declareParameterIfMissing(
    node, prefix + "smac_2d.esdf_decay", rclcpp::ParameterValue(0.8));
  declareParameterIfMissing(
    node, prefix + "smac_2d.esdf_max_cost", rclcpp::ParameterValue(0.5));
  declareParameterIfMissing(
    node, prefix + "smac_2d.minimum_clearance", rclcpp::ParameterValue(0.0));
  // 主搜索无解时的第二遍搜索净空。它只在窄道续行开启且低于主净空时生效；
  // 这只是给局部规划器一条可跟踪的参考线，执行侧仍按轨迹门和窄道地板验收。
  declareParameterIfMissing(
    node, prefix + "smac_2d.relaxed_minimum_clearance", rclcpp::ParameterValue(0.0));
  node->get_parameter(prefix + "smac_2d.use_esdf_cost", smac_use_esdf_cost_);
  node->get_parameter(prefix + "smac_2d.esdf_weight", smac_esdf_weight_);
  node->get_parameter(prefix + "smac_2d.esdf_decay", smac_esdf_decay_);
  node->get_parameter(prefix + "smac_2d.esdf_max_cost", smac_esdf_max_cost_);
  node->get_parameter(prefix + "smac_2d.minimum_clearance", smac_minimum_clearance_);
  node->get_parameter(
    prefix + "smac_2d.relaxed_minimum_clearance", smac_relaxed_minimum_clearance_);
  if (!std::isfinite(smac_minimum_clearance_) || smac_minimum_clearance_ < 0.0) {
    throw std::invalid_argument("smac_2d.minimum_clearance must be finite and non-negative");
  }
  if (!std::isfinite(smac_relaxed_minimum_clearance_) ||
      smac_relaxed_minimum_clearance_ < 0.0) {
    throw std::invalid_argument(
      "smac_2d.relaxed_minimum_clearance must be finite and non-negative");
  }

  declareParameterIfMissing(
    node, prefix + "lidar_offset_x", rclcpp::ParameterValue(0.0));
  declareParameterIfMissing(
    node, prefix + "lidar_offset_y", rclcpp::ParameterValue(-0.2));
  node->get_parameter(prefix + "lidar_offset_x", lidar_offset_x_);
  node->get_parameter(prefix + "lidar_offset_y", lidar_offset_y_);

  // Odometry topic
  std::string odom_topic = "/odom";
  declareParameterIfMissing(
    node, prefix + "odom_topic", rclcpp::ParameterValue(odom_topic));
  node->get_parameter(prefix + "odom_topic", odom_topic);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.lookahead_dist", rclcpp::ParameterValue(5.0));
  node->get_parameter(prefix + "minco_optimizer.lookahead_dist", lookahead_dist_);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.traj_goal_tolerance", rclcpp::ParameterValue(0.15));
  node->get_parameter(prefix + "minco_optimizer.traj_goal_tolerance", traj_goal_tolerance_);

  double goal_approach_max_shorten = 1.5;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.goal_approach_max_shorten",
    rclcpp::ParameterValue(goal_approach_max_shorten));
  node->get_parameter(
    prefix + "minco_optimizer.goal_approach_max_shorten", goal_approach_max_shorten);
  if (!std::isfinite(goal_approach_max_shorten) || goal_approach_max_shorten < 0.0) {
    goal_approach_max_shorten = 1.5;
  }

  // --- Optimizer config ------------------------------------------------------

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.safe_dist", rclcpp::ParameterValue(0.3));
  node->get_parameter(prefix + "minco_optimizer.safe_dist", minco_config.safe_dist);

  double collision_dist = minco_config.safe_dist;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.collision_dist", rclcpp::ParameterValue(collision_dist));
  node->get_parameter(prefix + "minco_optimizer.collision_dist", collision_dist);
  collision_dist_ = collision_dist;

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.narrow_passage_enable", rclcpp::ParameterValue(true));
  node->get_parameter(
    prefix + "minco_optimizer.narrow_passage_enable", narrow_passage_enable_);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.narrow_passage_clearance", rclcpp::ParameterValue(0.24));
  node->get_parameter(
    prefix + "minco_optimizer.narrow_passage_clearance", narrow_passage_clearance_);
  if (!std::isfinite(narrow_passage_clearance_) || narrow_passage_clearance_ <= 0.0 ||
      narrow_passage_clearance_ >= collision_dist_) {
    narrow_passage_enable_ = false;
    narrow_passage_clearance_ = 0.0;
  }

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.replan_period_s", rclcpp::ParameterValue(0.2));
  node->get_parameter(prefix + "minco_optimizer.replan_period_s", force_replan_period_sec_);
  if (!(std::isfinite(force_replan_period_sec_) && force_replan_period_sec_ > 0.0)) {
    force_replan_period_sec_ = 0.2;
  }

  // 目标接近减速带：deceleration 取底盘实测刹车能力（1.4–2.6 m/s²），而不是
  // minco_optimizer.max_acceleration（5.0）。种子与局部速度上限都按它封顶，
  // 否则参考线会在离目标 0.3 m 处仍有 ~1.1 m/s，实车必然过冲。
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.goal_approach.deceleration",
    rclcpp::ParameterValue(goal_approach_.deceleration));
  node->get_parameter(
    prefix + "minco_optimizer.goal_approach.deceleration", goal_approach_.deceleration);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.goal_approach.stop_margin",
    rclcpp::ParameterValue(goal_approach_.stop_margin));
  node->get_parameter(
    prefix + "minco_optimizer.goal_approach.stop_margin", goal_approach_.stop_margin);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.goal_approach.min_speed",
    rclcpp::ParameterValue(goal_approach_.min_speed));
  node->get_parameter(
    prefix + "minco_optimizer.goal_approach.min_speed", goal_approach_.min_speed);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.goal_approach.reaction_time",
    rclcpp::ParameterValue(goal_approach_.reaction_time));
  node->get_parameter(
    prefix + "minco_optimizer.goal_approach.reaction_time", goal_approach_.reaction_time);
  if (!goal_approach_.valid()) {
    RCLCPP_WARN(node->get_logger(),
      "[MincoPlanner] Invalid goal_approach parameters (deceleration=%.3f stop_margin=%.3f "
      "min_speed=%.3f); goal-approach deceleration disabled.",
      goal_approach_.deceleration, goal_approach_.stop_margin, goal_approach_.min_speed);
    goal_approach_.deceleration = 0.0;
  }
  if (goal_approach_.deceleration > 1e-3) {
    RCLCPP_INFO(node->get_logger(),
      "[MincoPlanner] Goal-approach deceleration: decel=%.2f m/s^2 reaction=%.2f s "
      "stop_margin=%.2f m min_speed=%.2f m/s (measured chassis braking)",
      goal_approach_.deceleration, goal_approach_.reaction_time,
      goal_approach_.stop_margin, goal_approach_.min_speed);
  }

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.replan_continuity_time_s",
    rclcpp::ParameterValue(0.3));
  node->get_parameter(
    prefix + "minco_optimizer.replan_continuity_time_s", replan_continuity_time_s_);
  if (!std::isfinite(replan_continuity_time_s_) || replan_continuity_time_s_ < 0.0) {
    replan_continuity_time_s_ = 0.3;
  }

  // 失败原因日志：每种原因前 N 次逐条打印，每 M 次失败另打一条分类汇总；N<=0 退回 2 s 节流。
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.failure_log_first_n",
    rclcpp::ParameterValue(static_cast<int64_t>(10)));
  node->get_parameter(prefix + "minco_optimizer.failure_log_first_n", failure_log_first_n_);
  // 首 N 次之后的采样间隔：每 failure_log_every_n_ 次该原因打印一条，1 为全打，<=0 退回 2 s 节流。
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.failure_log_every_n",
    rclcpp::ParameterValue(static_cast<int64_t>(25)));
  node->get_parameter(prefix + "minco_optimizer.failure_log_every_n", failure_log_every_n_);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.failure_summary_every",
    rclcpp::ParameterValue(static_cast<int64_t>(50)));
  node->get_parameter(prefix + "minco_optimizer.failure_summary_every", failure_summary_every_);

  // 第四层兜底：正常路径 / 局部绕行 / 完整停车前缀三层都失败时，退化成"不比当前净空更差"的短前缀
  // （creep）把车挪出贴死状态；长度上限固定为一个车体半径，因此不需要改动任何安全阈值。
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stuck_escape.enable", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.stuck_escape.enable", escape_enable_);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stuck_escape.min_length", rclcpp::ParameterValue(0.08));
  node->get_parameter(prefix + "minco_optimizer.stuck_escape.min_length", escape_min_length_);
  if (!(std::isfinite(escape_min_length_) && escape_min_length_ > 0.0)) {
    escape_min_length_ = 0.08;
  }
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stuck_escape.buffer", rclcpp::ParameterValue(0.05));
  node->get_parameter(prefix + "minco_optimizer.stuck_escape.buffer", escape_buffer_);
  if (!(std::isfinite(escape_buffer_) && escape_buffer_ >= 0.0)) {
    escape_buffer_ = 0.05;
  }

  // 种子门只按"占据 / 地形 / 查询失效"否决；折线净空不足不否决种子，净空由优化后的轨迹级门把关。

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.max_velocity", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.max_velocity", minco_config.max_vel);
  declareParameterIfMissing(
    node, "region_control.normal_max_speed", rclcpp::ParameterValue(3.0));
  node->get_parameter("region_control.normal_max_speed", normal_speed_cap_);
  if (!std::isfinite(normal_speed_cap_) || normal_speed_cap_ <= 0.0) {
    throw std::invalid_argument("region_control.normal_max_speed must be finite and positive");
  }
  region_speed_caps_.fill(std::numeric_limits<double>::infinity());
  region_prepare_distances_.fill(0.0);
  region_release_distances_.fill(0.0);
  for (const auto & entry : std::array<std::pair<const char *, size_t>, 3>{{
      {"slope", 5}, {"tunnel", 6}, {"undulating", 7}}}) {
    const std::string key = std::string("region_control.") + entry.first + ".max_speed";
    declareParameterIfMissing(node, key, rclcpp::ParameterValue(1.0));
    node->get_parameter(key, region_speed_caps_[entry.second]);
    if (!std::isfinite(region_speed_caps_[entry.second]) || region_speed_caps_[entry.second] <= 0.0) {
      throw std::invalid_argument(key + " must be finite and positive");
    }
    const std::string prepare_key = std::string("region_control.") + entry.first + ".prepare_distance";
    const std::string release_key = std::string("region_control.") + entry.first + ".release_distance";
    declareParameterIfMissing(node, prepare_key, rclcpp::ParameterValue(1.0));
    declareParameterIfMissing(node, release_key, rclcpp::ParameterValue(0.5));
    node->get_parameter(prepare_key, region_prepare_distances_[entry.second]);
    node->get_parameter(release_key, region_release_distances_[entry.second]);
    if (!std::isfinite(region_prepare_distances_[entry.second]) ||
      region_prepare_distances_[entry.second] < 0.0 ||
      !std::isfinite(region_release_distances_[entry.second]) ||
      region_release_distances_[entry.second] < 0.0) {
      throw std::invalid_argument("region_control distances must be finite and nonnegative");
    }
  }

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.max_acceleration", rclcpp::ParameterValue(4.0));
  node->get_parameter(prefix + "minco_optimizer.max_acceleration", minco_config.max_acc);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.turn_angle_deadzone", rclcpp::ParameterValue(0.174));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_deadzone", minco_config.turn_angle_deadzone);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.turn_angle_saturation", rclcpp::ParameterValue(1.57));
  node->get_parameter(prefix + "minco_optimizer.turn_angle_saturation", minco_config.turn_angle_saturation);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.min_turn_vel", rclcpp::ParameterValue(1.0));
  node->get_parameter(prefix + "minco_optimizer.min_turn_vel", minco_config.min_turn_vel);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.decay_power", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "minco_optimizer.decay_power", minco_config.decay_power);

  double max_yaw_dot = 3.14;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.max_yaw_dot", rclcpp::ParameterValue(max_yaw_dot));
  node->get_parameter(prefix + "minco_optimizer.max_yaw_dot", max_yaw_dot);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.enable_yaw_opt", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.enable_yaw_opt", use_yaw_opt_);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_time", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time", minco_config.rho);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.smooth_eps", rclcpp::ParameterValue(0.01));
  node->get_parameter(prefix + "minco_optimizer.smooth_eps", minco_config.smooth_eps);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.integral_res", rclcpp::ParameterValue(16));
  node->get_parameter(prefix + "minco_optimizer.integral_res", minco_config.integral_res);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.opt_accuracy", rclcpp::ParameterValue(1.0e-4));
  node->get_parameter(prefix + "minco_optimizer.opt_accuracy", minco_config.opt_accuracy);

  declareParameterIfMissing(
    node, prefix + "minco_optimizer.print_optimizer_log", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.print_optimizer_log", minco_config.print_optimizer_log);

  // USTC 5.5.4.2: two-stage MINCO. PRE keeps the trajectory shape stable;
  // FINELY uses direction-aware ESDF gradients and reuses PRE as its seed.
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.two_stage_enable", rclcpp::ParameterValue(true));
  node->get_parameter(prefix + "minco_optimizer.two_stage_enable", minco_config.two_stage_enable);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stage1_time_balance_weight", rclcpp::ParameterValue(10.0));
  node->get_parameter(
    prefix + "minco_optimizer.stage1_time_balance_weight", minco_config.stage1_time_balance_weight);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stage1_time_balance_ratio", rclcpp::ParameterValue(0.10));
  node->get_parameter(
    prefix + "minco_optimizer.stage1_time_balance_ratio", minco_config.stage1_time_balance_ratio);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.stage2_directional_gradient_enable", rclcpp::ParameterValue(true));
  node->get_parameter(
    prefix + "minco_optimizer.stage2_directional_gradient_enable",
    minco_config.stage2_directional_gradient_enable);
  minco_config.stage1_time_balance_weight = std::max(0.0, minco_config.stage1_time_balance_weight);
  minco_config.stage1_time_balance_ratio = std::max(0.0, minco_config.stage1_time_balance_ratio);

  double penalty_weight_pos = 0.0;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_pos", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_pos", penalty_weight_pos);

  double penalty_weight_vel = 0.0;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_vel", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_vel", penalty_weight_vel);

  double penalty_weight_acc = 0.0;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_acc", rclcpp::ParameterValue(10000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_acc", penalty_weight_acc);

  double penalty_weight_att = 0.0;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_att", rclcpp::ParameterValue(1000.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_att", penalty_weight_att);

  double penalty_weight_time_barrier = 0.0;
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.penalty_weight_time_barrier", rclcpp::ParameterValue(100.0));
  node->get_parameter(prefix + "minco_optimizer.penalty_weight_time_barrier", penalty_weight_time_barrier);

  minco_config.penaltyWeights.resize(5);
  minco_config.penaltyWeights(0) = penalty_weight_pos;
  minco_config.penaltyWeights(1) = penalty_weight_vel;
  minco_config.penaltyWeights(2) = penalty_weight_acc;
  minco_config.penaltyWeights(3) = penalty_weight_att;
  minco_config.penaltyWeights(4) = penalty_weight_time_barrier;

  minco_config.magnitudeBounds.resize(3);
  minco_config.magnitudeBounds(0) = minco_config.safe_dist;
  minco_config.magnitudeBounds(1) = minco_config.max_vel;
  minco_config.magnitudeBounds(2) = minco_config.max_acc;

  // --- Corridor config -------------------------------------------------------

  double corridor_robot_radius = 0.4;
  declareParameterIfMissing(
    node, prefix + "corridor.robot_radius", rclcpp::ParameterValue(corridor_robot_radius));
  node->get_parameter(prefix + "corridor.robot_radius", corridor_robot_radius);

  double corridor_extra_margin = 0.15;
  declareParameterIfMissing(
    node, prefix + "corridor.extra_margin", rclcpp::ParameterValue(corridor_extra_margin));
  node->get_parameter(prefix + "corridor.extra_margin", corridor_extra_margin);

  // --- Recovery server config -----------------------------------------------

  declareParameterIfMissing(
    node, prefix + "recovery_server.fail_threshold", rclcpp::ParameterValue(3));
  node->get_parameter(prefix + "recovery_server.fail_threshold", recovery_server_config_.fail_threshold);

  declareParameterIfMissing(
    node, prefix + "recovery_server.cooldown_sec", rclcpp::ParameterValue(2.0));
  node->get_parameter(prefix + "recovery_server.cooldown_sec", recovery_server_config_.cooldown_sec);

  declareParameterIfMissing(
    node, prefix + "recovery_server.recovery_window_sec", rclcpp::ParameterValue(3.0));
  node->get_parameter(
    prefix + "recovery_server.recovery_window_sec", recovery_server_config_.recovery_window_sec);

  declareParameterIfMissing(
    node, prefix + "recovery_server.escape_speed", rclcpp::ParameterValue(0.4));
  node->get_parameter(prefix + "recovery_server.escape_speed", recovery_server_config_.escape_speed);

  // --- Components / publishers / timers -------------------------------------

  const auto global_query = mode_context_ ? mode_context_->globalQuery() : nullptr;
  const auto dynamic_query = mode_context_ ? mode_context_->dynamicQuery() : nullptr;
  const unsigned int init_size_x = global_query ? global_query->sizeX() : 1U;
  const unsigned int init_size_y = global_query ? global_query->sizeY() : 1U;
  astar_planner_ = std::make_unique<Astar>(init_size_x, init_size_y);
  astar_planner_->setMap(global_query);

  // SMAC 2D 对齐旧工程：allow_unknown 同源、max_iterations 1000000、tolerance 用 tolerance_。
  if (use_smac_) {
    smac_planner_ = std::make_unique<mas2027_nav_executor::smac::SmacPlanner2DSimple>();
    smac_planner_->configure(logger_);
    smac_planner_->setParameters(allow_unknown_, 1000000, static_cast<float>(tolerance_));
    smac_planner_->setESDFParameters(
      smac_use_esdf_cost_, smac_esdf_weight_, smac_esdf_decay_, smac_esdf_max_cost_);
    smac_planner_->setMinimumClearance(smac_minimum_clearance_);
    smac_planner_->setMap(global_query);
    smac_planner_->setESDFQuery(dynamic_query);
    RCLCPP_INFO(logger_,
      "[MincoPlanner] SMAC 2D global search enabled: use_esdf_cost=%s weight=%.3f decay=%.3f "
      "max_cost=%.3f min_clearance=%.3f tolerance=%.3f",
      smac_use_esdf_cost_ ? "true" : "false",
      smac_esdf_weight_,
      smac_esdf_decay_,
      smac_esdf_max_cost_,
      smac_minimum_clearance_,
      tolerance_);
  } else {
    smac_planner_.reset();
    RCLCPP_INFO(logger_, "[MincoPlanner] SMAC 2D disabled (use_smac=false); using Astar global search.");
  }

  global_path_searcher_ = std::make_unique<GlobalPathSearcher>();
  // allow_unknown 必须与 mode context 的 exploration.unknown_as_occupied 同源，
  // 否则外层放行"未知格可通行"而降级规划、搜索内部却当障碍，目标在未观测区必然找不到路径。
  global_path_searcher_->configure(
    tf_, astar_planner_.get(), smac_planner_.get(), use_smac_,
    !exploration_unknown_as_occupied_, tolerance_, logger_);
  global_path_searcher_->setQuery(global_query);
  // The fallback endpoint should not be stricter than the route itself. Keep it
  // at the body safety radius; the final trajectory and command gates still
  // enforce their own hard threshold.
  global_path_searcher_->setOnlineSubgoalClearance(collision_dist_);
  global_path_searcher_->setGoalApproachMaxShorten(goal_approach_max_shorten);
  double relaxed_search_clearance = 0.0;
  if (narrow_passage_enable_ && smac_relaxed_minimum_clearance_ > 0.0 &&
      smac_relaxed_minimum_clearance_ < smac_minimum_clearance_) {
    // Never let the global retry choose a cell tighter than the local narrow
    // corridor floor; otherwise the planner returns a path that MINCO must reject.
    relaxed_search_clearance =
      std::max(smac_relaxed_minimum_clearance_, narrow_passage_clearance_);
    if (relaxed_search_clearance >= smac_minimum_clearance_) {
      relaxed_search_clearance = 0.0;
    }
  }
  global_path_searcher_->setRelaxedMinimumClearance(relaxed_search_clearance);
  if (relaxed_search_clearance > 0.0) {
    RCLCPP_INFO(logger_,
      "[MincoPlanner] Narrow global-search fallback enabled: primary=%.3f m relaxed=%.3f m "
      "local_floor=%.3f m",
      smac_minimum_clearance_, relaxed_search_clearance, narrow_passage_clearance_);
  }

  local_path_processor_ = std::make_unique<LocalPathProcessor>();
  local_path_processor_->configure(
    lookahead_dist_, minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_,
    collision_dist_, minco_config.safe_dist, logger_);
  local_path_processor_->setNarrowPassage(narrow_passage_enable_, narrow_passage_clearance_);
  declareParameterIfMissing(
    node, prefix + "minco_optimizer.clearance_detour_enable", rclcpp::ParameterValue(true));
  bool clearance_detour_enable = true;
  node->get_parameter(prefix + "minco_optimizer.clearance_detour_enable", clearance_detour_enable);
  local_path_processor_->setClearanceDetourEnabled(clearance_detour_enable);
  // 第四层兜底（短距离脱困前缀）的参数在此注入；stuck_escape.* 已在上面读取。
  local_path_processor_->setEscapeOptions(
    escape_enable_, escape_min_length_, escape_buffer_);

  safety_checker_ = std::make_unique<TrajectorySafetyChecker>();
  safety_checker_->configure(0.05, logger_);
  safety_checker_->setQuery(dynamic_query);
  safety_checker_->setNarrowPassage(narrow_passage_enable_, narrow_passage_clearance_);

  opt_path_pub_ = node->create_publisher<interfaces::msg::MpcPositionCommand>(
    "/opt_path", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 备份安全盒（SFC）可视化。QoS 与 minco_trajectory 话题一致，transient_local 便于 RViz 中途接入。
  safe_corridor_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/nav_executor/debug/safe_corridor", rclcpp::QoS(1).transient_local());

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      if (!msg) {
        return;
      }

      auto node = node_.lock();
      if (node) {
        planner_perf_monitor_.recordOdomCallback(node->now(), msg->header.stamp);
      }

      {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        latest_odom_ = *msg;
        has_latest_odom_ = true;
      }

    });

  visualizer_ = std::make_unique<Visualizer>();
  visualizer_->configure(parent, output_frame_);

  minco_optimizer_ = std::make_unique<MincoOptimizer>(minco_config);
  minco_optimizer_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);
  RCLCPP_INFO(logger_,
    "[MincoPlanner] MINCO two_stage=%s time_balance=%.2f ratio=%.2f directional_gradient=%s "
    "(cold/hot start remains active)",
    minco_config.two_stage_enable ? "true" : "false",
    minco_config.stage1_time_balance_weight,
    minco_config.stage1_time_balance_ratio,
    minco_config.stage2_directional_gradient_enable ? "true" : "false");

  corridor_gen_ = std::make_shared<SimpleCorridorGenerator>();
  corridor_gen_->setMap(mode_context_ ? mode_context_->dynamicQuery() : nullptr);
  corridor_gen_->setSafetyMargins(corridor_robot_radius, corridor_extra_margin);

  backup_opt_ = std::make_unique<traj_opt::BackupTrajOpt>();
  yaw_opt_ = std::make_unique<traj_opt::YawTrajOpt>(max_yaw_dot);

  recovery_server_ = std::make_shared<RecoverServer>();
  recovery_server_->configure(recovery_server_config_);

  on_set_parameters_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&MincoPlanner::onSetParameters, this, std::placeholders::_1));
}

void MincoPlanner::setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map)
{
  rog_query_raw_ = map;
  rebuildModeDependentQueries();
}

void MincoPlanner::cleanup()
{
  planner_perf_monitor_.close();

  on_set_parameters_callback_handle_.reset();

  recovery_server_.reset();

  if (visualizer_) {
    visualizer_->cleanup();
    visualizer_.reset();
  }

  astar_planner_.reset();
  smac_planner_.reset();
  global_path_searcher_.reset();
  local_path_processor_.reset();
  safety_checker_.reset();
  mode_context_.reset();
  minco_optimizer_.reset();
  corridor_gen_.reset();
  fused_query_.reset();
  terrain_query_.reset();
  backup_opt_.reset();
  yaw_opt_.reset();
  opt_path_pub_.reset();
  safe_corridor_pub_.reset();
  odom_sub_.reset();
  map_.reset();
  rog_query_raw_.reset();
}

rcl_interfaces::msg::SetParametersResult MincoPlanner::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::string planner_mode_param = name_ + ".planner_mode";
  const auto is_configure_time_mode_param = [this, &planner_mode_param](const std::string & param_name) {
    return param_name == planner_mode_param || param_name == name_ + ".frames.map_frame" ||
           param_name == name_ + ".frames.rog_frame" ||
           param_name == name_ + ".use_smac" ||
           param_name == name_ + ".exploration.boundary_margin" ||
           param_name == name_ + ".exploration.boundary_sample_step" ||
           param_name == name_ + ".exploration.unknown_as_occupied" ||
           param_name == name_ + ".exploration.prefer_goal_direction";
  };
  const std::string max_vel_param = name_ + ".minco_optimizer.max_velocity";
  const std::string max_acc_param = name_ + ".minco_optimizer.max_acceleration";
  const std::string penalty_pos_param = name_ + ".minco_optimizer.penalty_weight_pos";
  const std::string penalty_vel_param = name_ + ".minco_optimizer.penalty_weight_vel";
  const std::string penalty_acc_param = name_ + ".minco_optimizer.penalty_weight_acc";
  const std::string penalty_att_param = name_ + ".minco_optimizer.penalty_weight_att";
  const std::string penalty_time_barrier_param = name_ + ".minco_optimizer.penalty_weight_time_barrier";

  double next_max_vel = minco_config.max_vel;
  double next_max_acc = minco_config.max_acc;
  double next_penalty_pos =
    (minco_config.penaltyWeights.size() > 0) ? minco_config.penaltyWeights(0) : 1000.0;
  double next_penalty_vel =
    (minco_config.penaltyWeights.size() > 1) ? minco_config.penaltyWeights(1) : 1000.0;
  double next_penalty_acc =
    (minco_config.penaltyWeights.size() > 2) ? minco_config.penaltyWeights(2) : 10000.0;
  double next_penalty_att =
    (minco_config.penaltyWeights.size() > 3) ? minco_config.penaltyWeights(3) : 1000.0;
  double next_penalty_time_barrier =
    (minco_config.penaltyWeights.size() > 4) ? minco_config.penaltyWeights(4) : 100.0;
  bool optimizer_config_changed = false;

  bool next_smac_use_esdf_cost = smac_use_esdf_cost_;
  double next_smac_esdf_weight = smac_esdf_weight_;
  double next_smac_esdf_decay = smac_esdf_decay_;
  double next_smac_esdf_max_cost = smac_esdf_max_cost_;
  double next_smac_minimum_clearance = smac_minimum_clearance_;
  bool smac_config_changed = false;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();

    auto parse_numeric = [&](double & out) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        out = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        out = static_cast<double>(param.as_int());
        return true;
      }
      return false;
    };

    if (is_configure_time_mode_param(param_name)) {
      result.successful = false;
      result.reason =
        "Planner mode/frame parameters are configure-time only; restart planner_server to apply.";
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }

    if (param_name == max_vel_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_vel_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_vel = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == max_acc_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate <= 0.0) {
        result.successful = false;
        result.reason = "Parameter must be a positive number: " + max_acc_param;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }
      next_max_acc = candidate;
      optimizer_config_changed = true;
      continue;
    }

    if (param_name == penalty_pos_param || param_name == penalty_vel_param ||
        param_name == penalty_acc_param || param_name == penalty_att_param ||
        param_name == penalty_time_barrier_param) {
      double candidate = 0.0;
      if (!parse_numeric(candidate) || !std::isfinite(candidate) || candidate < 0.0) {
        result.successful = false;
        result.reason = "Penalty weight must be a non-negative number: " + param_name;
        RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
        return result;
      }

      if (param_name == penalty_pos_param) {
        next_penalty_pos = candidate;
      } else if (param_name == penalty_vel_param) {
        next_penalty_vel = candidate;
      } else if (param_name == penalty_acc_param) {
        next_penalty_acc = candidate;
      } else if (param_name == penalty_att_param) {
        next_penalty_att = candidate;
      } else {
        next_penalty_time_barrier = candidate;
      }

      optimizer_config_changed = true;
      continue;
    }

    // ESDF 势场和最小净空可在线调；use_smac 是结构开关，configure-time 已拒绝。
    if (param_name == name_ + ".smac_2d.use_esdf_cost" ||
        param_name == name_ + ".smac_2d.esdf_weight" ||
        param_name == name_ + ".smac_2d.esdf_decay" ||
        param_name == name_ + ".smac_2d.esdf_max_cost" ||
        param_name == name_ + ".smac_2d.minimum_clearance") {
      if (param_name == name_ + ".smac_2d.use_esdf_cost") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "Parameter must be a bool: " + param_name;
          RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
          return result;
        }
        next_smac_use_esdf_cost = param.as_bool();
      } else {
        double candidate = 0.0;
        if (!parse_numeric(candidate) || !std::isfinite(candidate)) {
          result.successful = false;
          result.reason = "Parameter must be a finite number: " + param_name;
          RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
          return result;
        }
        // decay 是 exp(-d/decay) 的尺度，必须为正；weight 与 max_cost 允许 0（等于关闭该偏置）。
        if (param_name == name_ + ".smac_2d.esdf_decay") {
          if (candidate <= 0.0) {
            result.successful = false;
            result.reason = "smac_2d.esdf_decay must be positive: " + param_name;
            RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
            return result;
          }
          next_smac_esdf_decay = candidate;
        } else if (candidate < 0.0) {
          result.successful = false;
          result.reason = "Parameter must be non-negative: " + param_name;
          RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
          return result;
        } else if (param_name == name_ + ".smac_2d.esdf_weight") {
          next_smac_esdf_weight = candidate;
        } else if (param_name == name_ + ".smac_2d.minimum_clearance") {
          next_smac_minimum_clearance = candidate;
        } else {
          next_smac_esdf_max_cost = candidate;
        }
      }
      smac_config_changed = true;
      continue;
    }

    // Other planner parameters are read only during configuration. Accepting
    // a runtime write here would change the reported value without changing
    // the safety checks or the planner that uses it.
    if (param_name.rfind(name_ + ".", 0) == 0) {
      result.successful = false;
      result.reason = "Parameter requires a planner restart: " + param_name;
      RCLCPP_ERROR(logger_, "[MincoPlanner] %s", result.reason.c_str());
      return result;
    }
  }

  if (smac_config_changed) {
    smac_use_esdf_cost_ = next_smac_use_esdf_cost;
    smac_esdf_weight_ = next_smac_esdf_weight;
    smac_esdf_decay_ = next_smac_esdf_decay;
    smac_esdf_max_cost_ = next_smac_esdf_max_cost;
    smac_minimum_clearance_ = next_smac_minimum_clearance;
    if (smac_planner_) {
      smac_planner_->setESDFParameters(
        smac_use_esdf_cost_, smac_esdf_weight_, smac_esdf_decay_, smac_esdf_max_cost_);
      smac_planner_->setMinimumClearance(smac_minimum_clearance_);
    }
    RCLCPP_INFO(logger_,
      "[MincoPlanner] SMAC 2D ESDF updated: use_esdf_cost=%s weight=%.3f decay=%.3f "
      "max_cost=%.3f min_clearance=%.3f",
      smac_use_esdf_cost_ ? "true" : "false",
      smac_esdf_weight_,
      smac_esdf_decay_,
      smac_esdf_max_cost_,
      smac_minimum_clearance_);
  }

  if (optimizer_config_changed) {
    minco_config.max_vel = next_max_vel;
    minco_config.max_acc = next_max_acc;

    minco_config.penaltyWeights.resize(5);
    minco_config.penaltyWeights(0) = next_penalty_pos;
    minco_config.penaltyWeights(1) = next_penalty_vel;
    minco_config.penaltyWeights(2) = next_penalty_acc;
    minco_config.penaltyWeights(3) = next_penalty_att;
    minco_config.penaltyWeights(4) = next_penalty_time_barrier;

    minco_config.magnitudeBounds.resize(3);
    minco_config.magnitudeBounds(0) = minco_config.safe_dist;
    minco_config.magnitudeBounds(1) = minco_config.max_vel;
    minco_config.magnitudeBounds(2) = minco_config.max_acc;
    if (minco_optimizer_) {
      minco_optimizer_->setConfig(minco_config);
    }
    if (local_path_processor_) {
      local_path_processor_->updateLimits(minco_config.max_vel, minco_config.max_acc, traj_goal_tolerance_);
    }

    RCLCPP_INFO(logger_,
      "[MincoPlanner] Updated optimizer params: vmax=%.3f, amax=%.3f, w=[%.3f %.3f %.3f %.3f %.3f]",
      minco_config.max_vel,
      minco_config.max_acc,
      minco_config.penaltyWeights(0),
      minco_config.penaltyWeights(1),
      minco_config.penaltyWeights(2),
      minco_config.penaltyWeights(3),
      minco_config.penaltyWeights(4));
  }

  return result;
}

// -----------------------------------------------------------------------------
// 3) Core business interface
// -----------------------------------------------------------------------------

bool MincoPlanner::goalKnownInvalid(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return global_path_searcher_ && mode_context_ &&
    global_path_searcher_->goalKnownInvalid(start, goal, *mode_context_);
}

bool MincoPlanner::PlanGlobalPath(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto search_start = record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  auto record_search_time = [&]() {
    if (!record_perf) {
      return;
    }
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - search_start).count();
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    last_global_search_time_ms_ = elapsed_ms;
    has_fresh_global_search_time_ = true;
  };

  if (!global_path_searcher_ || !mode_context_) {
    last_goal_invalid_ = false;
    last_goal_blocked_retryable_ = false;
    last_goal_shifted_ = false;
    return false;
  }
  std::vector<geometry_msgs::msg::PoseStamped> committed_path;
  bool committed_goal_shifted = false;
  if (usesOnlineMapOnly()) {
    std::lock_guard<std::mutex> path_lock(path_mutex_);
    committed_path = latest_global_path_;
    committed_goal_shifted = last_goal_shifted_;
  }
  std::vector<geometry_msgs::msg::PoseStamped> planned_path;
  // 全局搜索不再需要实测速度 / max_vel / max_acc：主搜索是地形图纯栅格 A*（对齐 SMAC2D）。
  if (!global_path_searcher_->plan(start, goal, *mode_context_, planned_path)) {
    // 区分“暂时搜索失败”与“目标已在已知障碍内且没有安全排出点”。后者必须
    // 让 TaskManager 立刻停住旧轨迹，不能再沿旧目标继续开。
    last_goal_invalid_ = global_path_searcher_->lastGoalInvalid();
    last_goal_blocked_retryable_ = global_path_searcher_->lastGoalBlockedRetryable();
    last_goal_shifted_ = false;
    record_search_time();
    return false;
  }
  last_goal_invalid_ = false;
  last_goal_blocked_retryable_ = false;
  last_goal_shifted_ = global_path_searcher_->lastGoalShifted();
  record_search_time();

  if (!committed_path.empty() && planned_path.size() >= 2U) {
    const auto query = mode_context_->globalQuery();
    if (query && query->resolution() > 0.0) {
      const double floor = narrow_passage_enable_ ? narrow_passage_clearance_ :
        mas2027_nav_executor::effectiveClearanceThreshold(collision_dist_);
      const auto valid_point = [query, floor](double x, double y) {
        unsigned int mx = 0U, my = 0U;
        if (!query->worldToMap(x, y, mx, my) || !query->isFree(mx, my)) {
          return false;
        }
        const auto result = query->query(Eigen::Vector3d(x, y, 0.0));
        return result.ok && std::isfinite(result.distance) &&
          result.distance + mas2027_nav_executor::kClearanceComparisonTolerance >= floor;
      };
      auto retained = committedRouteForReversal(
        start, committed_path, planned_path, query->resolution(), valid_point);
      if (retained) {
        planned_path = std::move(*retained);
        last_goal_shifted_ = committed_goal_shifted;
        RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
          "[MincoPlanner] Keeping live-map-safe committed route instead of reversing to a "
          "comparable or longer detour (floor=%.2f m)", floor);
      }
    }
  }

  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_ = std::move(planned_path);
  return latest_global_path_.size() >= 2U;
}

void MincoPlanner::setTerrainGrid(std::shared_ptr<mas2027_nav_executor::TerrainGrid> terrain)
{
  terrain_ = std::move(terrain);
  if (global_path_searcher_) global_path_searcher_->setTerrainGrid(terrain_);
  rebuildFusedQuery();
}

void MincoPlanner::setOnlineTerrainMap(
  std::shared_ptr<mas2027_nav_executor::OnlineGlobalMap> terrain)
{
  online_terrain_ = std::move(terrain);
  if (global_path_searcher_) global_path_searcher_->setOnlineTerrainMap(online_terrain_);
}

void MincoPlanner::rebuildFusedQuery()
{
  const auto dynamic_query = mode_context_ ? mode_context_->dynamicQuery() : rog_query_raw_;
  std::shared_ptr<rog_map::MapQueryInterface> local_query = dynamic_query;
  if (terrain_ && dynamic_query) {
    terrain_query_ = std::make_shared<mas2027_nav_executor::TerrainMapQuery>(terrain_);
    fused_query_ = std::make_shared<mas2027_nav_executor::FusedMapQuery>(
      dynamic_query, terrain_query_);
    local_query = fused_query_;
  } else {
    fused_query_.reset();
    terrain_query_.reset();
  }
  if (minco_optimizer_) minco_optimizer_->setMap(local_query);
  if (corridor_gen_) corridor_gen_->setMap(local_query);
  if (safety_checker_) safety_checker_->setQuery(local_query);
}

bool MincoPlanner::refreshFusedTransform()
{
  if (!fused_query_) {
    return true;
  }
  if (!terrain_ || !tf_) {
    fused_query_->clearStaticTransform();
    return false;
  }
  const auto terrain = terrain_->snapshot();
  if (!terrain) {
    fused_query_->clearStaticTransform();
    return false;
  }
  try {
    const auto transform = tf_->lookupTransform(
      terrain->cost.header.frame_id, output_frame_, tf2::TimePointZero);
    fused_query_->setStaticTransform(
      transform.transform.translation.x,
      transform.transform.translation.y,
      tf2::getYaw(transform.transform.rotation));
    return true;
  } catch (const tf2::TransformException &) {
    fused_query_->clearStaticTransform();
    return false;
  }
}

bool MincoPlanner::ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose)
{
  const bool record_perf = planner_perf_monitor_.detailedCsvEnabled();
  const auto replan_start = record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  std::optional<MincoPerfSample> perf;
  if (record_perf) {
    perf.emplace();
    perf->stamp_ros = rclcpp::Clock().now().seconds();
    perf->stamp_steady_ns = PlannerPerformanceMonitor::steadyNowNs();
    perf->planner_mode = mode_params_.planner_mode;
    std::lock_guard<std::mutex> perf_lock(perf_mutex_);
    if (has_fresh_global_search_time_) {
      perf->global_search_time_ms = last_global_search_time_ms_;
      has_fresh_global_search_time_ = false;
    }
  }
  auto finish = [&](bool success, const std::string & reason) {
    if (!success) {
      // 失败原因必须实车可读：不用 RCLCPP_WARN_THROTTLE，按时间节流会在失败密集时丢掉整片现场。
      ++replan_failure_total_;
      uint64_t & reason_count = replan_failure_counts_[reason];
      ++reason_count;
      const bool log_this =
        reason_count <= static_cast<uint64_t>(std::max<int64_t>(0, failure_log_first_n_)) ||
        (failure_log_every_n_ > 0 &&
         reason_count % static_cast<uint64_t>(failure_log_every_n_) == 0);
      if (log_this) {
        RCLCPP_WARN(logger_,
          "MINCO trajectory not published: %s (reason #%llu, total %llu)",
          reason.c_str(),
          static_cast<unsigned long long>(reason_count),
          static_cast<unsigned long long>(replan_failure_total_));
      }
      if (failure_summary_every_ > 0 &&
          replan_failure_total_ % failure_summary_every_ == 0) {
        std::string breakdown;
        for (const auto & entry : replan_failure_counts_) {
          if (!breakdown.empty()) breakdown += " ";
          breakdown += entry.first + "=" + std::to_string(entry.second);
        }
        RCLCPP_WARN(logger_, "MINCO failure summary: total=%llu [%s]",
          static_cast<unsigned long long>(replan_failure_total_), breakdown.c_str());
      }
    }
    if (perf) {
      perf->success = success;
      perf->failure_reason = success ? "NONE" : reason;
      perf->total_replan_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - replan_start).count();
      {
        std::lock_guard<std::mutex> perf_lock(perf_mutex_);
        if (last_minco_perf_stamp_ns_ > 0 && perf->stamp_steady_ns > last_minco_perf_stamp_ns_) {
          const double dt_sec =
            static_cast<double>(perf->stamp_steady_ns - last_minco_perf_stamp_ns_) * 1.0e-9;
          if (dt_sec > 1.0e-9) {
            perf->planner_hz = 1.0 / dt_sec;
          }
        }
        last_minco_perf_stamp_ns_ = perf->stamp_steady_ns;
      }
      planner_perf_monitor_.recordPlannerSample(*perf);
    }
    return success;
  };

  if (!minco_optimizer_ || !mode_context_ || !local_path_processor_) {
    return finish(false, "OPTIMIZER_FAILED");
  }

  // Snapshot the global goal for end-state logic.
  Eigen::Vector3d global_goal(0.0, 0.0, 0.0);
  double goal_yaw = 0.0;
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (latest_global_path_.empty()) {
      return finish(false, "OPTIMIZER_FAILED");
    }
    global_path_snapshot = latest_global_path_;
    global_goal.x() = latest_global_path_.back().pose.position.x;
    global_goal.y() = latest_global_path_.back().pose.position.y;
    global_goal.z() = 0.0;
    goal_yaw = utils::quaternionToYaw(latest_global_path_.back().pose.orientation);
  }

  std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)> terrain_segment_free;
  if (terrain_) {
    const auto terrain = terrain_->snapshot();
    if (!terrain || !tf_) return finish(false, "TERRAIN_MAP_UNAVAILABLE");
    try {
      const auto transform = tf_->lookupTransform(
        terrain->cost.header.frame_id, output_frame_, tf2::TimePointZero);
      const double yaw = tf2::getYaw(transform.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double tx = transform.transform.translation.x;
      const double ty = transform.transform.translation.y;
      if (fused_query_) {
        fused_query_->setStaticTransform(tx, ty, yaw);
      }
      terrain_segment_free = [terrain, c, s, tx, ty](
        const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
        const auto to_map = [c, s, tx, ty](const Eigen::Vector3d & p) {
          return Eigen::Vector2d(c * p.x() - s * p.y() + tx,
            s * p.x() + c * p.y() + ty);
        };
        return terrain->transition(to_map(a), to_map(b));
      };
    } catch (const tf2::TransformException &) {
      return finish(false, "TERRAIN_TF_UNAVAILABLE");
    }
  }
  if (!terrain_) {
    const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(map_);
    const auto semantic = adapter ? adapter->snapshot() : nullptr;
    const auto remembered = online_terrain_ ? online_terrain_->snapshot() : nullptr;
    const auto live_sides = semantic ? std::make_shared<const std::vector<uint8_t>>(
      mas2027_nav_executor::buildSlopeSideMask(semantic->width, semantic->height,
        semantic->terrain_labels, semantic->uphill_x, semantic->uphill_y)) : nullptr;
    if (semantic || remembered) {
      terrain_segment_free = [semantic, remembered, live_sides](const Eigen::Vector3d & a,
          const Eigen::Vector3d & b) {
        return (!semantic || onlineSlopeTransition(*semantic, *live_sides, a, b)) &&
          (!remembered || remembered->terrainTransition(a.x(), a.y(), b.x(), b.y()));
      };
    }
  }
  const LocalPathSeed seed = local_path_processor_->buildSeed(
    global_path_snapshot, current_pose, *mode_context_, terrain_segment_free);
  if (visualizer_) {
    if (!seed.dense_path.empty()) {
      visualizer_->updateLocalEndPoint(seed.dense_path.back(), seed.local_end_is_goal);
    } else {
      visualizer_->clearLocalEndPoint();
    }
  }
  if (!seed.valid) {
    // 局部种子无效 = 还没进优化器就失败，与 validateTrajectory 的 COLLISION 分开命名以便定位瓶颈。
    return finish(false,
                  seed.repair_rejected ? "LOCAL_SEED_REJECTED_AFTER_REPAIR"
                                       : "LOCAL_SEED_INVALID");
  }
  std::vector<Eigen::Vector3d> sparse_path = seed.sparse_waypoints;
  const bool local_end_is_goal = seed.local_end_is_goal;
  const bool stop_at_local_end = seed.stop_at_local_end;

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  // 4. Determine state (HOT/COLD).
  PlanningState state = PlanningState::COLD_START;
  traj_opt::Trajectory last_traj_snapshot;
  bool has_last_traj_snapshot = false;
  double last_traj_start_WT = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = determinePlanningState(current_pose.pose, sparse_path);
    if (has_last_traj_) {
      last_traj_snapshot = last_traj_;
      has_last_traj_snapshot = true;
      last_traj_start_WT = last_traj_.start_WT;
    }
  }

  if (state == PlanningState::EMERGENCY_STOP) {
    return finish(false, "RECOVERY_TRIGGERED");
  }

  // 5. Prepare start state.
  Eigen::Matrix3d start_state;
  vec_Vec3f shifted_waypoints;
  VecDf shifted_durations;
  bool has_shifted_seed = false;
  if (state == PlanningState::HOT_START) {
    const double now = rclcpp::Clock().now().seconds() + 0.005;  // small buffer
    const double t_dur = now - last_traj_start_WT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      prepareHotStart(current_pose.pose, t_dur, start_state);
    }

    // Extract remaining trajectory segment as shifted warm-start seed.
    if (has_last_traj_snapshot) {
      traj_opt::Trajectory remain;
      const double total = last_traj_snapshot.getTotalDuration();
      if (std::isfinite(t_dur) && t_dur > 0.0 && total > t_dur + 1e-3 &&
          last_traj_snapshot.getPartialTrajectoryByTime(t_dur, total, remain)) {
        shifted_waypoints = remain.getWaypoints();
        shifted_durations = remain.getDurations();
        has_shifted_seed = (!shifted_waypoints.empty() && shifted_durations.size() > 0);
      }
    }
  } else {
    prepareColdStart(current_pose.pose, start_state, sparse_path);
    // Avoid reusing stale warm-start guesses.
    minco_optimizer_->setInitPsAndTs(vec_Vec3f{}, VecDf{});
  }
  // P1b: If the robot is inside an obstacle (ESDF dist < 0), project the
  // start position to the nearest free space along the ESDF gradient.
  if (safety_checker_) {
    constexpr double kMargin = 0.05;
    Eigen::Vector3d start_pos = start_state.col(0);
    if (safety_checker_->projectOutOfObstacle(start_pos, kMargin)) {
      start_state.col(0) = start_pos;
    }
  }

  // 6. Generate backup trajectory (safety).
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);

  // 7. Prepare MINCO optimization.
  traj_opt::Trajectory opt_traj;
  Eigen::Matrix3d end_state;
  end_state.setZero();
  end_state.col(0) = sparse_path.back();

  // 滑窗末端不是终点，不能设零末速度，否则会在 ROGMap 边缘反复刹停；只在到终点或被封死时要求零速。
  const double dist_to_goal = (end_state.col(0) - global_goal).head<2>().norm();
  const double amax = std::max(0.0, minco_config.max_acc);
  const bool command_stop = local_end_is_goal || stop_at_local_end;
  const auto end_velocity_for = [&](const Eigen::Matrix3d & candidate_start) -> Eigen::Vector3d {
    if (command_stop) return Eigen::Vector3d::Zero();
    const double v_curr = std::max(0.0, candidate_start.col(1).head<2>().norm());
    Eigen::Vector3d tangent(1.0, 0.0, 0.0);
    if (sparse_path.size() >= 2) {
      tangent = sparse_path.back() - sparse_path[sparse_path.size() - 2];
      tangent.z() = 0.0;
      const double n = tangent.head<2>().norm();
      if (n > 0.1) {
        tangent /= n;
      } else {
        tangent = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
    }
    const double v_max_kinematic =
      std::sqrt(std::max(0.0, v_curr * v_curr + 2.0 * amax * dist_to_goal));
    double local_end_vmax = minco_config.max_vel;
    if (sparse_path.size() >= 3) {
      local_end_vmax = utils::LimitLocalVel(sparse_path,
        sparse_path.size() - 3,
        minco_config.max_vel,
        minco_config.turn_angle_deadzone,
        minco_config.turn_angle_saturation,
        minco_config.min_turn_vel,
        minco_config.decay_power);
    }
    const double v_cmd =
      std::min({minco_config.max_vel, v_max_kinematic, dist_to_goal, local_end_vmax});
    return tangent * v_cmd;
  };
  end_state.col(1) = end_velocity_for(start_state);

  // Remove near-start redundant points from sparse_path.
  while (sparse_path.size() > 2) {
    if ((sparse_path[1] - start_state.col(0)).norm() < 0.2) {
      sparse_path.erase(sparse_path.begin() + 1);
    } else {
      break;
    }
  }

  // 7.5 Initial guess Ps/Ts for optimizer (all cases).
  const int N = static_cast<int>(sparse_path.size()) - 1;
  const bool used_shifted_prefix = state == PlanningState::HOT_START &&
    has_shifted_seed && replan_continuity_time_s_ > 0.0 && N > 1 &&
    mas2027_nav_executor::selectReplanContinuityPrefix(
      sparse_path, shifted_waypoints, shifted_durations,
      replan_continuity_time_s_) > 0;
  VecDf local_vmaxs(N);
  if (N > 0) {
    vec_Vec3f init_ps;
    VecDf init_ts(N);
    PTAllocation(sparse_path,
      start_state,
      command_stop,
      state,
      has_shifted_seed,
      shifted_waypoints,
      shifted_durations,
      init_ps,
      init_ts,
      local_vmaxs);

    minco_optimizer_->setInitPsAndTs(init_ps, init_ts);
  }

  // 8. Optimize.
  const auto opt_start_steady =
    record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  if (perf) {
    perf->local_search_time_ms =
      std::chrono::duration<double, std::milli>(opt_start_steady - replan_start).count();
  }
  auto opt_start_time = rclcpp::Clock().now().seconds();
  double final_cost = minco_config.two_stage_enable ?
    minco_optimizer_->optimizeTwoStage(sparse_path, start_state, end_state, local_vmaxs, opt_traj) :
    minco_optimizer_->optimize(sparse_path, start_state, end_state, local_vmaxs, opt_traj);
  const int initial_optimizer_code = minco_optimizer_->lastReturnCode();
  Eigen::Vector3d measured_velocity = Eigen::Vector3d::Zero();
  if (!std::isfinite(final_cost) && state == PlanningState::HOT_START) {
    measured_velocity = getCurrentSpeed();
  }
  if (!std::isfinite(final_cost) && state == PlanningState::HOT_START &&
      measured_velocity.allFinite()) {
    // A shifted prefix may no longer match a rerouted path. More importantly,
    // the old trajectory's predicted velocity/acceleration can differ from
    // odometry after a collision gate or heading hold has stopped translation.
    // Retry once with this cycle's path and measured velocity. The candidate
    // still passes the complete trajectory validation before publication.
    Eigen::Matrix3d measured_start = start_state;
    measured_start.col(1) = measured_velocity;
    measured_start.col(2).setZero();
    Eigen::Matrix3d measured_end = end_state;
    measured_end.col(1) = end_velocity_for(measured_start);
    vec_Vec3f fresh_points;
    VecDf fresh_times(N);
    VecDf fresh_vmaxs(N);
    PTAllocation(sparse_path, measured_start, command_stop, PlanningState::COLD_START, false,
      vec_Vec3f{}, VecDf{}, fresh_points, fresh_times, fresh_vmaxs);
    minco_optimizer_->setInitPsAndTs(fresh_points, fresh_times);
    traj_opt::Trajectory retry_traj;
    const double retry_cost = minco_config.two_stage_enable ?
      minco_optimizer_->optimizeTwoStage(
        sparse_path, measured_start, measured_end, fresh_vmaxs, retry_traj) :
      minco_optimizer_->optimize(
        sparse_path, measured_start, measured_end, fresh_vmaxs, retry_traj);
    if (std::isfinite(retry_cost)) {
      final_cost = retry_cost;
      opt_traj = std::move(retry_traj);
      start_state = measured_start;
      end_state = measured_end;
      state = PlanningState::COLD_START;
      RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
        "[MincoPlanner] Measured-velocity path seed recovered a failed hot-start optimization "
        "(initial_code=%d shifted_prefix=%s)", initial_optimizer_code,
        used_shifted_prefix ? "true" : "false");
    }
  }
  if (perf) {
    perf->optimizer_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - opt_start_steady).count();
  }

  // const double max_allowed_cost = 6000.0;
  // if (!std::isfinite(final_cost) || final_cost > max_allowed_cost) {
  if (!std::isfinite(final_cost)) {
    // RCLCPP_WARN(logger_,
    //   "[MincoPlanner] Rejecting new trajectory! Cost (%.2f) exceeds limit (%.2f).",
    //   final_cost,
    //   max_allowed_cost);

    if (visualizer_) {
      visualizer_->clearCandidateTrajectory("OPTIMIZER_FAILED");
    }

    bool has_last_traj = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_last_traj = has_last_traj_;
    }

    if (has_last_traj && !isTrajectoryTimeExpired(rclcpp::Clock().now().seconds())) {
      RCLCPP_WARN(logger_,
        "[MincoPlanner] Optimization failed (initial_code=%d retry_code=%d); "
        "keeping the previous unexpired trajectory",
        initial_optimizer_code, minco_optimizer_->lastReturnCode());
    }
    return finish(false, "OPTIMIZER_FAILED");
  }

  auto opt_end_time = rclcpp::Clock().now().seconds();
  double opt_duration = opt_end_time - opt_start_time;
  std::cout << GREEN << "[MincoPlanner] Minco optimization time: "
            << opt_duration << " seconds, "
            << "cost: " << final_cost << RESET << std::endl;

  // 8.5 Quality gating (hard validation) before publishing.
  const bool validation_ok = validateTrajectory(opt_traj, end_state.col(0), seed.dense_path);
  if (visualizer_) {
    visualizer_->updateCandidateTrajectory(opt_traj,
      opt_duration,
      validation_ok,
      validation_ok ? "NONE" : last_validation_failure_reason_);
  }

  if (!validation_ok) {
    std::cout << RED << "[MincoPlanner] Trajectory validation failed! Rejecting." << RESET << std::endl;
    // 被拒的只是候选轨迹；旧轨迹若仍在执行，由执行前指令门逐周期检查。
    return finish(false, last_validation_failure_reason_);
  }

  double fallback_yaw = 0.0;
  if (start_state.col(1).head<2>().norm() > 1e-3) {
    fallback_yaw = std::atan2(start_state.col(1).y(), start_state.col(1).x());
  } else {
    fallback_yaw = getCurrentYawFromOdom();
  }

  // Slope-aware short-horizon yaw lock based on real odometry attitude.
  double pitch = 0.0;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (has_latest_odom_) {
      const auto & odom_q = latest_odom_.pose.pose.orientation;
      const tf2::Quaternion q(odom_q.x, odom_q.y, odom_q.z, odom_q.w);
      double roll = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }
  }

  constexpr double slope_threshold = 0.1;
  if (std::abs(pitch) > slope_threshold && sparse_path.size() >= 2) {
    goal_yaw = std::atan2(end_state.col(1).y(), end_state.col(1).x());
  } else if (sparse_path.size() >= 2) {
    const Eigen::Vector2d tail_dir = (sparse_path.back() - sparse_path[sparse_path.size() - 2]).head<2>();
    if (tail_dir.norm() > 0.1) {
      goal_yaw = std::atan2(tail_dir.y(), tail_dir.x());
    } else {
      goal_yaw = getCurrentYawFromOdom();
    }
  } else {
    goal_yaw = getCurrentYawFromOdom();
  }

  traj_opt::Trajectory yaw_traj;
  if (use_yaw_opt_) {
    const bool yaw_success =
      optimizeYaw(start_state, opt_traj, yaw_traj, state, current_pose.pose, goal_yaw);
    if (!yaw_success) {
      std::cout << YELLOW
                << "[MincoPlanner] Yaw optimization failed. Falling back to constant yaw trajectory."
                << RESET << std::endl;

      Eigen::MatrixXd cMat(3, 6);
      cMat.setZero();
      cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
      const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
      yaw_traj.clear();
      yaw_traj.emplace_back(yaw_dur, cMat);
      yaw_traj.start_WT = opt_traj.start_WT;
    }
  } else {
    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat(0, 5) = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
    const double yaw_dur = std::max(0.02, opt_traj.getTotalDuration());
    yaw_traj.clear();
    yaw_traj.emplace_back(yaw_dur, cMat);
    yaw_traj.start_WT = opt_traj.start_WT;
  }

  // 9. Publish and cache.
  const double t_step = 0.05;
  int steps = static_cast<int>(std::ceil(opt_traj.getTotalDuration() / t_step)) + 1;
  steps = std::max(2, steps);

  utils::publishOptimizedTrajectory(
    opt_traj, yaw_traj, opt_path_pub_, opt_trajectory_id_, header_msg, steps, t_step);

  if (visualizer_) {
    nav_msgs::msg::Path astar_path_msg;
    {
      std::lock_guard<std::mutex> path_lock(path_mutex_);
      astar_path_msg.header.stamp = rclcpp::Clock().now();
      astar_path_msg.header.frame_id = output_frame_;
      astar_path_msg.poses = latest_global_path_;
    }
    visualizer_->update(sparse_path, backup_traj, opt_traj, opt_duration, astar_path_msg);
  }

  last_traj_ = opt_traj;
  last_traj_.start_WT = rclcpp::Clock().now().seconds();
  has_last_traj_ = true;
  last_yaw_traj_ = yaw_traj;
  last_yaw_traj_.start_WT = last_traj_.start_WT;
  has_last_yaw_traj_ = true;

  return finish(true, "NONE");
}

void MincoPlanner::PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
  const Eigen::Matrix3d & start_state,
  bool goal_reached,
  PlanningState state,
  bool has_shifted_seed,
  const vec_Vec3f & shifted_waypoints,
  const VecDf & shifted_durations,
  vec_Vec3f & init_ps,
  VecDf & init_ts,
  VecDf & local_vmaxs) const
{
  const int N = static_cast<int>(sparse_path.size()) - 1;
  if (N <= 0) {
    init_ps.clear();
    init_ts.resize(0);
    local_vmaxs.resize(0);
    return;
  }

  const double global_vmax = std::max(0.0, minco_config.max_vel);
  const double amax = std::max(1e-3, minco_config.max_acc);
  const double kMinSegTime = 0.1;
  const double kBrakeSafety = 1.2;
  // 预留的刹车余量同样按实测刹车能力算：用 amax 会把余量算短，滚窗 subgoal
  // 之前来不及收速。
  const double brake_capability =
    (goal_approach_.deceleration > 1e-3) ? goal_approach_.deceleration : amax;
  const double max_brake_dist = (global_vmax * global_vmax) / (2.0 * brake_capability);

  local_vmaxs.resize(N);
  local_vmaxs.setConstant(global_vmax);
  init_ts.resize(N);

  init_ps.clear();
  init_ps.reserve(static_cast<size_t>(std::max(0, N - 1)));
  int copyPs = 0;
  if (state == PlanningState::HOT_START && has_shifted_seed &&
    replan_continuity_time_s_ > 0.0 && N > 1) {
    copyPs = mas2027_nav_executor::selectReplanContinuityPrefix(
      sparse_path, shifted_waypoints, shifted_durations, replan_continuity_time_s_);
  }
  for (int j = 0; j < copyPs; ++j) {
    init_ps.emplace_back(shifted_waypoints[static_cast<size_t>(j + 1)]);
  }
  for (int j = copyPs; j < (N - 1); ++j) {
    init_ps.emplace_back(sparse_path[static_cast<size_t>(j + 1)]);
  }

  std::vector<double> seg_len(static_cast<size_t>(N), 0.0);
  for (int i = 0; i < N; ++i) {
    const double dis =
      (sparse_path[static_cast<size_t>(i + 1)] - sparse_path[static_cast<size_t>(i)]).head<2>().norm();
    seg_len[static_cast<size_t>(i)] = (std::isfinite(dis) && dis > 0.0) ? dis : 0.0;
  }

  std::vector<double> remain_after(static_cast<size_t>(N), 0.0);
  for (int i = N - 2; i >= 0; --i) {
    remain_after[static_cast<size_t>(i)] =
      remain_after[static_cast<size_t>(i + 1)] + seg_len[static_cast<size_t>(i + 1)];
  }

  std::vector<double> local_vmax_vec(static_cast<size_t>(N), global_vmax);
  for (int i = 0; i < N - 1; ++i) {
    local_vmax_vec[static_cast<size_t>(i)] = utils::LimitLocalVel(sparse_path,
      i,
      global_vmax,
      minco_config.turn_angle_deadzone,
      minco_config.turn_angle_saturation,
      minco_config.min_turn_vel,
      minco_config.decay_power);
  }
  std::vector<bool> slope_window(static_cast<size_t>(N), false);
  std::vector<double> other_region_cap(
    static_cast<size_t>(N), std::numeric_limits<double>::infinity());
  if (terrain_ && tf_) {
    if (const auto terrain = terrain_->snapshot()) {
      try {
        const auto transform = tf_->lookupTransform(
          terrain->cost.header.frame_id, output_frame_, tf2::TimePointZero);
        const double yaw = tf2::getYaw(transform.transform.rotation);
        const double c = std::cos(yaw), s = std::sin(yaw);
        const Eigen::Vector2d offset(transform.transform.translation.x,
          transform.transform.translation.y);
        const double spacing = 0.5 * terrain->cost.info.resolution;
        std::vector<double> node_s(static_cast<size_t>(N + 1), 0.0);
        for (int i = 0; i < N; ++i) {
          node_s[static_cast<size_t>(i + 1)] = node_s[static_cast<size_t>(i)] +
            seg_len[static_cast<size_t>(i)];
        }
        for (int i = 0; i < N; ++i) {
          const Eigen::Vector2d a = sparse_path[static_cast<size_t>(i)].head<2>();
          const Eigen::Vector2d b = sparse_path[static_cast<size_t>(i + 1)].head<2>();
          const int samples = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
          for (int j = 0; j <= samples; ++j) {
            const Eigen::Vector2d p = a + (static_cast<double>(j) / samples) * (b - a);
            const auto label = terrain->terrainLabelAt(
              {c * p.x() - s * p.y() + offset.x(),
               s * p.x() + c * p.y() + offset.y()});
            if (!label) continue;
            if (*label < region_speed_caps_.size()) {
              const double sample_s = node_s[static_cast<size_t>(i)] +
                (static_cast<double>(j) / samples) * seg_len[static_cast<size_t>(i)];
              const double affected_start = sample_s - region_prepare_distances_[*label];
              const double affected_end = sample_s + region_release_distances_[*label];
              for (int k = 0; k < N; ++k) {
                if (node_s[static_cast<size_t>(k)] <= affected_end &&
                  node_s[static_cast<size_t>(k + 1)] >= affected_start) {
                  if (*label == 5U) {
                    slope_window[static_cast<size_t>(k)] = true;
                  } else {
                    other_region_cap[static_cast<size_t>(k)] = std::min(
                      other_region_cap[static_cast<size_t>(k)], region_speed_caps_[*label]);
                  }
                }
              }
            }
          }
        }
      } catch (const tf2::TransformException &) {
        // The final trajectory safety check still rejects an unavailable terrain transform.
      }
    }
  }
  for (int i = 0; i < N; ++i) {
    const size_t k = static_cast<size_t>(i);
    const double region_cap = std::min(
      slope_window[k] ? region_speed_caps_[5] : normal_speed_cap_, other_region_cap[k]);
    local_vmax_vec[k] = std::min(local_vmax_vec[k], region_cap);
  }
  // 目标接近减速带：按"从该段起点到局部终点的剩余弧长"封顶，用实测刹车
  // 能力而不是 max_acceleration。只在局部终点就是停车点时生效；滚窗
  // subgoal 不是停车点，不能把速度压成爬行。
  if (goal_reached && goal_approach_.deceleration > 1e-3) {
    for (int i = 0; i < N; ++i) {
      const double remaining = seg_len[static_cast<size_t>(i)] +
        remain_after[static_cast<size_t>(i)];
      const double allowed =
        mas2027_nav_executor::goalApproachSpeedLimit(remaining, goal_approach_);
      if (std::isfinite(allowed)) {
        local_vmax_vec[static_cast<size_t>(i)] =
          std::min(local_vmax_vec[static_cast<size_t>(i)], allowed);
      }
    }
    RCLCPP_INFO_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
      "[MincoPlanner] Goal-approach speed cap active: remaining %.2f m -> allowed %.2f m/s "
      "(decel=%.2f m/s^2, floor=%.2f m/s)",
      seg_len.front() + remain_after.front(),
      local_vmax_vec.front(), goal_approach_.deceleration, goal_approach_.min_speed);
  }
  utils::VelPropogation(seg_len, amax, local_vmax_vec);
  for (int i = 0; i < N; ++i) {
    local_vmaxs(i) = local_vmax_vec[static_cast<size_t>(i)];
  }

  double v_curr = start_state.col(1).head<2>().norm();
  if (!std::isfinite(v_curr) || v_curr < 0.0) {
    v_curr = 0.0;
  }

  // 反向刹车余量一律用真实刹车能力；`max_acceleration` 只留给加速段。
  const double brake_deceleration =
    (goal_approach_.deceleration > 1e-3) ? goal_approach_.deceleration : amax;

  std::vector<double> profile_speeds, profile_times;
  if (mas2027_nav_executor::buildArcLengthSpeedProfile(
      seg_len, local_vmax_vec, v_curr, goal_reached, amax, brake_deceleration,
      profile_speeds, profile_times)) {
    for (int i = 0; i < N; ++i) init_ts(i) = profile_times[static_cast<size_t>(i)];
  } else {
    // Preserve the prior conservative seed when the measured start velocity
    // cannot be connected to the available braking distance.
    const double local_goal_remain = goal_reached ? 0.0 : max_brake_dist;
    for (int i = 0; i < N; ++i) {
      const bool is_last = (i == N - 1);
      const double L = seg_len[static_cast<size_t>(i)];
      const double remain = remain_after[static_cast<size_t>(i)] + local_goal_remain;
      if (L <= 1e-6) {
        init_ts(i) = kMinSegTime;
        continue;
      }
      if (is_last && goal_reached) {
        // 停车段同样按真实刹车能力给时间，不能用 amax（5 m/s²）。
        const double t_stop = v_curr / brake_deceleration;
        const double t_dist = L / std::max(v_curr, 0.1);
        init_ts(i) = std::max({kMinSegTime, t_dist, kBrakeSafety * t_stop});
        v_curr = 0.0;
        continue;
      }
      const double local_vmax = local_vmax_vec[static_cast<size_t>(i)];
      const double v_next = utils::ComputeNextSpeed(v_curr, L, remain, amax, local_vmax);
      init_ts(i) = utils::ComputeSegmentTime(L, v_curr, v_next, local_vmax, amax, kMinSegTime);
      v_curr = v_next;
    }
  }

  if (state == PlanningState::HOT_START && has_shifted_seed && copyPs > 0) {
    const int oldN = std::min(copyPs, static_cast<int>(shifted_durations.size()));
    for (int i = 0; i < oldN; ++i) {
      const double t_seed = shifted_durations(i);
      if (std::isfinite(t_seed) && t_seed > 0.02) {
        init_ts(i) = std::max(init_ts(i), t_seed);
      }
    }
  }
}

bool MincoPlanner::makePlan(const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  double tolerance,
  std::function<bool()> cancel_checker,
  nav_msgs::msg::Path & plan)
{
  if (!global_path_searcher_ || !mode_context_) {
    return false;
  }
  if (!global_path_searcher_->makePlan(start, goal, *mode_context_, tolerance, cancel_checker, plan)) {
    return false;
  }
  std::lock_guard<std::mutex> path_lock(path_mutex_);
  latest_global_path_ = plan.poses;
  return true;
}

std::vector<Eigen::Vector3d> MincoPlanner::extractLocalPath(const Eigen::Vector3d & cur_pos)
{
  if (!local_path_processor_ || !mode_context_) {
    return {};
  }
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.pose.position.x = cur_pos.x();
  current_pose.pose.position.y = cur_pos.y();
  current_pose.pose.position.z = cur_pos.z();
  std::vector<geometry_msgs::msg::PoseStamped> global_path_snapshot;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_snapshot = latest_global_path_;
  }
  return local_path_processor_->buildSeed(global_path_snapshot, current_pose, *mode_context_).dense_path;
}

MincoPlanner::PlanningState MincoPlanner::determinePlanningState(
  const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path)
{
  if (!has_last_traj_) {
    return PlanningState::COLD_START;
  }

  double now = rclcpp::Clock().now().seconds() + 0.005;
  double t_dur = now - last_traj_.start_WT;
  if (t_dur <= 0.0 || t_dur >= last_traj_.getTotalDuration()) {
    std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Invalid time duration (t_dur=" << t_dur
              << "s)" << RESET << std::endl;
    return PlanningState::COLD_START;
  }

  Eigen::Vector3d current_pos(start_pose.position.x, start_pose.position.y, 0.0);
  Eigen::Vector3d pred_pos = last_traj_.getPos(t_dur);
  Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
  double tracking_error = (current_pos - pred_pos).norm();
  Eigen::Vector3d current_speed = getCurrentSpeed();
  double dynamic_error_threshold = 1.0 + 0.5 * current_speed.head<2>().norm();
  double vel_error = (current_speed - pred_vel).norm();
  // 跟踪 / 速度误差大时仍保持 HOT_START，避免 20 Hz 重复丢掉热启动种子；不安全轨迹仍由净空门拒绝。
  if (tracking_error > dynamic_error_threshold) {
    std::cout << YELLOW << "[MincoPlanner] Large tracking error (" << tracking_error
              << "m); keeping HOT_START for trajectory continuity." << RESET << std::endl;
    return PlanningState::HOT_START;
  }

  if (vel_error > 1.0) {
    std::cout << YELLOW << "[MincoPlanner] Large velocity error (" << vel_error
              << "m/s); keeping HOT_START for trajectory continuity." << RESET << std::endl;
    return PlanningState::HOT_START;
  }

  if (new_path.size() >= 2) {
    Eigen::Vector3d pred_vel = last_traj_.getVel(t_dur);
    if (pred_vel.norm() > 0.1) {
      Eigen::Vector3d path_dir = (new_path[1] - new_path[0]).normalized();
      Eigen::Vector3d vel_dir = pred_vel.normalized();
      double dot = vel_dir.dot(path_dir);

      if (dot < 0.5) {
        std::cout << YELLOW << "[MincoPlanner] Hot Start Rejected: Direction mismatch (dot=" << dot
                  << ", angle=" << std::acos(dot) * 180.0 / M_PI << " deg)" << RESET << std::endl;
        return PlanningState::COLD_START;
      }
    }
  }

  return PlanningState::HOT_START;
}

void MincoPlanner::prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
  Eigen::Matrix3d & start_state,
  const std::vector<Eigen::Vector3d> & sparse_path)
{
  start_state.setZero();
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  (void)sparse_path;
  Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(1) = real_speed;
}

void MincoPlanner::prepareHotStart(
  const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state)
{
  start_state.setZero();
  // start_state.col(0) = last_traj_.getPos(t_dur);
  start_state.col(0) = Eigen::Vector3d(start_pose.position.x, start_pose.position.y, 0.0);
  start_state.col(1) = last_traj_.getVel(t_dur);
  // Eigen::Vector3d real_speed = getCurrentSpeed();
  start_state.col(2) = last_traj_.getAcc(t_dur);
}

bool MincoPlanner::optimizeYaw(const Eigen::Matrix3d & start_state,
  const traj_opt::Trajectory & pos_traj,
  traj_opt::Trajectory & out_yaw_traj,
  PlanningState state,
  const geometry_msgs::msg::Pose & current_pose,
  double goal_yaw)
{
  (void)current_pose;

  if (!yaw_opt_) {
    return false;
  }

  const double pos_dur = pos_traj.getTotalDuration();
  if (!(std::isfinite(pos_dur) && pos_dur > 1e-6)) {
    return false;
  }

  Eigen::Vector4d init_yaw_state = Eigen::Vector4d::Zero();
  Eigen::Vector4d goal_yaw_state = Eigen::Vector4d::Zero();

  bool use_hot_seed = false;
  if (state == PlanningState::HOT_START) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_yaw_traj_ && has_last_traj_) {
      const double t_dur = nowSeconds() - last_traj_.start_WT;
      const double yaw_dur = last_yaw_traj_.getTotalDuration();
      if (std::isfinite(t_dur) && std::isfinite(yaw_dur) && yaw_dur > 1e-6 && t_dur >= 0.0) {
        const double sample_t = std::min(t_dur, yaw_dur);
        init_yaw_state(0) = last_yaw_traj_.getPos(sample_t)(0);
        init_yaw_state(1) = last_yaw_traj_.getVel(sample_t)(0);
        use_hot_seed = true;
      }
    }
  }

  if (!use_hot_seed) {
    if (start_state.col(1).head<2>().norm() > 0.1) {
      init_yaw_state(0) = std::atan2(start_state.col(1).y(), start_state.col(1).x());
    } else {
      init_yaw_state(0) = getCurrentYawFromOdom();
    }
    init_yaw_state(1) = 0.0;
  }

  if (!std::isfinite(goal_yaw)) {
    goal_yaw = init_yaw_state(0);
  }
  const double yaw_err =
    std::atan2(std::sin(goal_yaw - init_yaw_state(0)), std::cos(goal_yaw - init_yaw_state(0)));
  goal_yaw_state(0) = init_yaw_state(0) + yaw_err;
  goal_yaw_state(1) = 0.0;

  return yaw_opt_->optimize(init_yaw_state, goal_yaw_state, pos_traj, out_yaw_traj, 5, false, true);
}

// -----------------------------------------------------------------------------
// 5) Helpers / callbacks / getters
// -----------------------------------------------------------------------------

bool MincoPlanner::validateTrajectory(
  const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos,
  const std::vector<Eigen::Vector3d> & reference_path)
{
  last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
  constexpr double kDt = 0.05;
  constexpr double kSevereScale = 1.5;

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid duration." << RESET << std::endl;
    last_validation_failure_reason_ = "OPTIMIZER_FAILED";
    return false;
  }

  const double vmax = minco_config.max_vel;
  const double amax = minco_config.max_acc;
  if (!(std::isfinite(vmax) && std::isfinite(amax) && vmax > 1e-6 && amax > 1e-6)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: invalid vmax/amax config." << RESET
              << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double vmax_severe = kSevereScale * vmax;
  const double amax_severe = kSevereScale * amax;

  // 1) Dynamic feasibility (severe violation gate).
  for (double t = 0.0; t <= dur; t += kDt) {
    const Eigen::Vector3d v = traj.getVel(t);
    const Eigen::Vector3d a = traj.getAcc(t);
    if (!(v.allFinite() && a.allFinite())) {
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite v/a." << RESET << std::endl;
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
    if (v.norm() > vmax_severe || a.norm() > amax_severe) {
      // 打印 t/dur 分清成因：t=0 是起点自带大加速度，t>0 才是优化超限；dur 过小说明轨迹退化。
      std::cout << YELLOW << "[MincoPlanner] validateTrajectory: severe dynamics violation."
                << " t=" << t << "/" << dur
                << " |v|=" << v.norm() << " (limit=" << vmax_severe << ")"
                << ", |a|=" << a.norm() << " (limit=" << amax_severe << ")" << RESET << std::endl;
      last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
      return false;
    }
  }

  // 2) Goal reachability.
  const Eigen::Vector3d end_pos = traj.getPos(dur);
  if (!end_pos.allFinite()) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: non-finite end position." << RESET
              << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  const double goal_err = (end_pos - expected_end_pos).norm();
  if (!(std::isfinite(goal_err) && goal_err <= traj_goal_tolerance_)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: goal not reached. err=" << goal_err
              << " tol=" << traj_goal_tolerance_ << RESET << std::endl;
    last_validation_failure_reason_ = "KINEMATIC_VIOLATION";
    return false;
  }

  // 3) Collision safety.
  if (!checkCollision(traj, reference_path)) {
    std::cout << YELLOW << "[MincoPlanner] validateTrajectory: collision detected." << RESET << std::endl;
    last_validation_failure_reason_ = "COLLISION";
    return false;
  }

  // ROGMap checks live obstacles; the same candidate must also obey the static
  // terrain body before any trajectory is published to the /cmd_vel tracker.
  if (terrain_) {
    const auto terrain = terrain_->snapshot();
    if (!terrain || !tf_) {
      last_validation_failure_reason_ = "TERRAIN_MAP_UNAVAILABLE";
      return false;
    }
    try {
      const auto transform = tf_->lookupTransform(
        terrain->cost.header.frame_id, output_frame_, tf2::TimePointZero);
      const double yaw = tf2::getYaw(transform.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double tx = transform.transform.translation.x;
      const double ty = transform.transform.translation.y;
      const auto to_map = [c, s, tx, ty](const Eigen::Vector3d & p) {
        return Eigen::Vector2d(c * p.x() - s * p.y() + tx,
          s * p.x() + c * p.y() + ty);
      };
      // 地形否决点日志：只加日志、不改判据 / 阈值 / 行为，打印 map 系坐标、格号、cost 及前一点。
      const auto log_terrain_reject = [&](const char * stage,
        const Eigen::Vector2d & point_map, const Eigen::Vector3d & point_odom,
        const Eigen::Vector2d * prev_map) {
        const uint64_t seen = terrain_reject_log_count_;
        ++terrain_reject_log_count_;
        if (seen >= kTerrainRejectLogFirstN && (seen % kTerrainRejectLogEveryN) != 0) {
          return;
        }
        const auto & info = terrain->cost.info;
        const auto cell_of = [&info](const Eigen::Vector2d & p, int & mx, int & my) {
          mx = static_cast<int>(std::floor(
            (p.x() - info.origin.position.x) / info.resolution));
          my = static_cast<int>(std::floor(
            (p.y() - info.origin.position.y) / info.resolution));
        };
        const auto cost_of = [&info, &terrain](int mx, int my) {
          if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
            my >= static_cast<int>(info.height)) {
            return -1;
          }
          return static_cast<int>(terrain->cost.data[
            static_cast<size_t>(my) * static_cast<size_t>(info.width) +
            static_cast<size_t>(mx)]);
        };
        int mx = 0, my = 0, pmx = -1, pmy = -1;
        cell_of(point_map, mx, my);
        if (prev_map != nullptr) {
          cell_of(*prev_map, pmx, pmy);
        }
        // 再看 ROGMap：地形图说占据而 rog_clear 很大，则根因是"优化器只看 ROGMap"的结构，不是阈值。
        double rog_clear = std::numeric_limits<double>::quiet_NaN();
        int rog_value = -1;
        int rog_free = -1;
        const char * rog_status = "no_query";
        if (map_) {
          const auto rog_res = map_->query(point_odom);
          rog_status = rog_map::queryStatusName(rog_res.status);
          if (rog_res.ok && std::isfinite(rog_res.distance)) {
            rog_clear = rog_res.distance;
          }
          unsigned int rx = 0U, ry = 0U;
          if (map_->worldToMap(point_odom.x(), point_odom.y(), rx, ry)) {
            rog_value = static_cast<int>(map_->value(rx, ry));
            rog_free = map_->isFree(rx, ry) ? 1 : 0;
          }
        }
        RCLCPP_WARN(logger_,
          "[MincoPlanner] Terrain rejection #%llu: stage=%s map=(%.3f,%.3f) cell=(%d,%d) "
          "cost=%d prev_cell=(%d,%d) prev_cost=%d odom=(%.3f,%.3f) frame=%s yaw=%.3f "
          "| rog_clear=%.3f rog_free=%d rog_value=%d rog_status=%s",
          static_cast<unsigned long long>(seen + 1), stage,
          point_map.x(), point_map.y(), mx, my, cost_of(mx, my),
          pmx, pmy, cost_of(pmx, pmy),
          point_odom.x(), point_odom.y(),
          terrain->cost.header.frame_id.c_str(), yaw,
          rog_clear, rog_free, rog_value, rog_status);
      };
      const double duration = traj.getTotalDuration();
      const double dt = std::min(0.02,
        static_cast<double>(terrain->cost.info.resolution) /
        (2.0 * std::max(0.1, minco_config.max_vel)));
      const int intervals = std::max(1, static_cast<int>(std::ceil(duration / dt)));
      Eigen::Vector2d previous = to_map(traj.getPos(0.0));
      if (!terrain->traversable(previous)) {
        log_terrain_reject("start_static", previous, traj.getPos(0.0), nullptr);
        last_validation_failure_reason_ = "TERRAIN_COLLISION";
        return false;
      }
      for (int i = 1; i <= intervals; ++i) {
        const Eigen::Vector3d pos_odom = traj.getPos(duration * i / intervals);
        const Eigen::Vector2d current = to_map(pos_odom);
        if (!terrain->transition(previous, current)) {
          log_terrain_reject("edge_static", current, pos_odom, &previous);
          last_validation_failure_reason_ = "TERRAIN_COLLISION";
          return false;
        }
        previous = current;
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Terrain trajectory validation TF failed: %s", ex.what());
      last_validation_failure_reason_ = "TERRAIN_TF_UNAVAILABLE";
      return false;
    }
  }

  if (!terrain_) {
    const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(map_);
    const auto semantic = adapter ? adapter->snapshot() : nullptr;
    const auto remembered = online_terrain_ ? online_terrain_->snapshot() : nullptr;
    const auto live_sides = semantic ? mas2027_nav_executor::buildSlopeSideMask(
      semantic->width, semantic->height, semantic->terrain_labels,
      semantic->uphill_x, semantic->uphill_y) : std::vector<uint8_t>{};
    if (semantic || remembered) {
      const int intervals = std::max(1, static_cast<int>(std::ceil(
        dur * std::max(0.1, vmax) /
        (0.5 * (semantic ? semantic->resolution : remembered->resolution())))));
      Eigen::Vector3d previous = traj.getPos(0.0);
      for (int i = 1; i <= intervals; ++i) {
        const Eigen::Vector3d current = traj.getPos(dur * i / intervals);
        if ((semantic && !onlineSlopeTransition(*semantic, live_sides, previous, current)) ||
            (remembered && !remembered->terrainTransition(
              previous.x(), previous.y(), current.x(), current.y()))) {
          last_validation_failure_reason_ = "TERRAIN_SIDE_ENTRY";
          return false;
        }
        previous = current;
      }
    }
  }

  return true;
}

bool MincoPlanner::checkCollision(
  const traj_opt::Trajectory & traj,
  const std::vector<Eigen::Vector3d> & reference_path)
{
  if (!safety_checker_) {
    return false;
  }

  const double dur = traj.getTotalDuration();
  if (!(std::isfinite(dur) && dur > 1e-6)) {
    return true;
  }

  // 发布前唯一一道轨迹净空检查；执行侧会在每个控制周期再检查当前指令。
  return safety_checker_->checkTrajectory(
    traj,
    mas2027_nav_executor::effectiveClearanceThreshold(collision_dist_),
    collision_dist_,
    reference_path);
}

void MincoPlanner::publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose)
{
  const double now_s = nowSeconds();
  if (last_estop_s_ >= 0.0 && (now_s - last_estop_s_) < 0.1) {
    return;
  }
  last_estop_s_ = now_s;

  auto node = node_.lock();
  if (node) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[MincoPlanner] Publishing emergency stop: committed traj unsafe and replan failed.");
  }

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
  header_msg.stamp = rclcpp::Clock().now();

  Eigen::Matrix3d start_state;
  prepareColdStart(current_pose.pose, start_state, std::vector<Eigen::Vector3d>{});
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_last_traj_) {
      const double t_dur = now_s - last_traj_.start_WT;
      const double total = last_traj_.getTotalDuration();
      if (std::isfinite(t_dur) && std::isfinite(total) && t_dur >= 0.0 && t_dur <= total) {
        start_state.col(1) = last_traj_.getVel(t_dur);
        start_state.col(2) = last_traj_.getAcc(t_dur);
      }
    }
  }

  const double current_yaw = getCurrentYawFromOdom();
  refreshFusedTransform();
  traj_opt::Trajectory backup_traj = generateBackupTraj(start_state);
  backup_traj.start_WT = now_s;
  utils::publishBackupTrajectory(
    backup_traj, opt_path_pub_, opt_trajectory_id_, header_msg, 20, 0.1, current_yaw);
  // Do not replace last_traj_ with the brake piece: determinePlanningState / HOT_START
  // still inherit the committed velocity so the chassis does not drop into a crawl.
}

void MincoPlanner::publishSafeCorridorBox(const PolyhedronH & poly)
{
  if (!safe_corridor_pub_ || poly.rows() < 6 || poly.cols() < 4) {
    return;
  }

  // 盒子轴对齐，6 行依次为 x/y/z 上下界（n·p < d）：负法向给下界 d=-min，正法向给上界 d=max。
  const double x_min = -poly(0, 3);
  const double x_max = poly(1, 3);
  const double y_min = -poly(2, 3);
  const double y_max = poly(3, 3);
  const double z_min = -poly(4, 3);
  const double z_max = poly(5, 3);
  if (!(x_min <= x_max && y_min <= y_max && z_min <= z_max)) {
    return;
  }

  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker box;
  box.header.stamp = rclcpp::Clock().now();
  box.header.frame_id = output_frame_;
  box.ns = "safe_corridor";
  box.id = 0;
  box.type = visualization_msgs::msg::Marker::LINE_LIST;
  box.action = visualization_msgs::msg::Marker::ADD;
  box.pose.orientation.w = 1.0;
  box.scale.x = 0.03;  // 线宽
  box.color.r = 0.0f;
  box.color.g = 1.0f;
  box.color.b = 1.0f;
  box.color.a = 0.9f;

  const Eigen::Vector3d corner[8] = {
    {x_min, y_min, z_min}, {x_max, y_min, z_min}, {x_max, y_max, z_min}, {x_min, y_max, z_min},
    {x_min, y_min, z_max}, {x_max, y_min, z_max}, {x_max, y_max, z_max}, {x_min, y_max, z_max}};
  static const int kEdges[12][2] = {  // 12 条棱：底面 / 顶面 / 竖棱各 4 条
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  box.points.reserve(24);
  for (const auto & edge : kEdges) {
    for (const int index : edge) {
      geometry_msgs::msg::Point p;
      p.x = corner[index].x();
      p.y = corner[index].y();
      p.z = corner[index].z();
      box.points.push_back(p);
    }
  }
  arr.markers.push_back(box);

  // 半边长标注：与 corridor.robot_radius / corridor.extra_margin 对照可判断盒子为什么这么大。
  visualization_msgs::msg::Marker label;
  label.header = box.header;
  label.ns = "safe_corridor";
  label.id = 1;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.orientation.w = 1.0;
  label.pose.position.x = 0.5 * (x_min + x_max);
  label.pose.position.y = 0.5 * (y_min + y_max);
  label.pose.position.z = z_max + 0.3;
  label.scale.z = 0.35;
  label.color = box.color;
  const double half_size =
    0.5 * std::max({x_max - x_min, y_max - y_min, z_max - z_min});
  char text[64] = {};
  std::snprintf(text, sizeof(text), "SFC half=%.2f m", half_size);
  label.text = text;
  arr.markers.push_back(label);

  safe_corridor_pub_->publish(arr);
}

traj_opt::Trajectory MincoPlanner::generateBackupTraj(const Eigen::Matrix3d & start_state)
{
  auto make_stop_traj = [&start_state]() -> traj_opt::Trajectory {
    traj_opt::Trajectory stop_traj;
    const Eigen::Vector3d p = start_state.col(0);

    Eigen::MatrixXd cMat(3, 6);
    cMat.setZero();
    cMat.col(5) = p;

    // Two very short constant pieces ("2 points" semantics).
    stop_traj.emplace_back(0.2, cMat);
    stop_traj.emplace_back(0.2, cMat);
    return stop_traj;
  };

  if (!corridor_gen_ || !backup_opt_) {
    std::cout << RED << "[MincoPlanner] Backup optimizer not initialized!" << RESET << std::endl;
    return make_stop_traj();
  }

  // Step 1: Generate SFC (safe box).
  auto safe_poly = corridor_gen_->generateSafeBox(start_state.col(0), 1.0);

  // 调试可视化：把本次生成的安全盒发到 RViz，纯发布、不影响喂给备份优化器的约束。
  publishSafeCorridorBox(safe_poly);

  // Step 2: Setup backup optimizer.
  backup_opt_->setInitState(start_state);
  backup_opt_->setStopConstraints();
  backup_opt_->setPolygons({safe_poly});

  // Step 3: Optimize.
  traj_opt::Trajectory backup_traj;
  bool success = backup_opt_->optimize(backup_traj);

  // Step 4: Return.
  if (success) {
    return backup_traj;
  }

  std::cout << RED << "[MincoPlanner] Backup trajectory optimization failed, fallback to stop." << RESET
            << std::endl;
  return make_stop_traj();
}

bool MincoPlanner::hasGlobalPath() const
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  return latest_global_path_.size() >= 2U;
}

bool MincoPlanner::copyLatestGlobalPath(std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  out = latest_global_path_;
  return out.size() >= 2U;
}

bool MincoPlanner::copyLatestGlobalGoal(geometry_msgs::msg::PoseStamped & goal) const
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (latest_global_path_.empty()) {
    return false;
  }
  goal = latest_global_path_.back();
  return true;
}

void MincoPlanner::invalidateGlobalPath()
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_global_path_.clear();
  last_goal_shifted_ = false;
}

double MincoPlanner::nowSeconds() const
{
  return rclcpp::Clock().now().seconds();
}

bool MincoPlanner::getRobotPose(geometry_msgs::msg::PoseStamped & pose) const
{
  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    if (!has_latest_odom_) {
      return false;
    }
    odom = latest_odom_;
  }

  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  if (pose.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] odom frame is empty, treating it as %s.",
      planning_frame_.c_str());
  }
  pose.header.frame_id = planning_frame_;
  return true;
}

bool MincoPlanner::checkGoalReached(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  const double dx = current_pose.pose.position.x - goal.pose.position.x;
  const double dy = current_pose.pose.position.y - goal.pose.position.y;
  const double dist = std::hypot(dx, dy);
  return std::isfinite(dist) && dist <= traj_goal_tolerance_;
}

bool MincoPlanner::checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (latest_global_path_.empty()) {
    return false;
  }

  const auto & goal = latest_global_path_.back().pose.position;
  const double dx = current_pose.pose.position.x - goal.x;
  const double dy = current_pose.pose.position.y - goal.y;
  const double dist = std::hypot(dx, dy);
  return std::isfinite(dist) && dist <= traj_goal_tolerance_;
}

Eigen::Vector3d MincoPlanner::getCurrentSpeed() const
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (has_latest_odom_) {
    const auto & twist = latest_odom_.twist.twist;
    const double yaw = utils::quaternionToYaw(latest_odom_.pose.pose.orientation);

    double vx_global = 0.0;
    double vy_global = 0.0;
    double omega_global = 0.0;
    utils::compensateLeverArm(
      twist.linear.x,
      twist.linear.y,
      twist.angular.z,
      yaw,
      lidar_offset_x_,
      lidar_offset_y_,
      vx_global,
      vy_global,
      omega_global);

    // std::cout << "[MincoPlanner] Lever-arm compensation: raw_v=(" << twist.linear.x << ", "
    //           << twist.linear.y << ") wz=" << twist.angular.z << " yaw=" << yaw << " -> v=("
    //           << vx_global << ", " << vy_global << ")" << std::endl;

    return Eigen::Vector3d(vx_global, vy_global, twist.linear.z);
  }
  return Eigen::Vector3d::Zero();
}

double MincoPlanner::getCurrentYawFromOdom() const
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  if (!has_latest_odom_) {
    return 0.0;
  }
  return utils::quaternionToYaw(latest_odom_.pose.pose.orientation);
}

bool MincoPlanner::isTrajectoryTimeExpired(double now_s) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_last_traj_) {
    return true;
  }
  const double end_s = last_traj_.start_WT + last_traj_.getTotalDuration();
  return now_s > end_s;
}

double MincoPlanner::getEsdfDistance(const Eigen::Vector3d & pos) const
{
  return safety_checker_ ? safety_checker_->getDistance(pos) : 0.0;
}

void MincoPlanner::publishEscapeCommand(
  const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel)
{
  if (visualizer_) {
    visualizer_->publishRecoveryDebug(current_pose, escape_vel, 0.5);
  }

  std_msgs::msg::Header header_msg;
  header_msg.frame_id = output_frame_;
  header_msg.stamp = rclcpp::Clock().now();
  const double current_yaw = getCurrentYawFromOdom();
  utils::publishEscapeCommand(
    current_pose, escape_vel, current_yaw, opt_path_pub_, opt_trajectory_id_, header_msg);
}

void MincoPlanner::clearRecoveryDebugVisualization()
{
  if (visualizer_) {
    visualizer_->clearRecoveryDebug();
  }
}

}  // namespace minco_planner
