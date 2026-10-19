#include "mas2027_nav_executor/path_planner/path_planner.hpp"

#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "mas2027_nav_executor/common/environment/online_global_map.hpp"
#include "rog_map/rog_map_core/config.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace {

visualization_msgs::msg::MarkerArray terrainMarkers(
  const nav_msgs::msg::OccupancyGrid & grid)
{
  visualization_msgs::msg::MarkerArray result;
  result.markers.resize(3);
  for (size_t i = 0; i < result.markers.size(); ++i) {
    auto & marker = result.markers[i];
    marker.header = grid.header;
    marker.ns = "persistent_terrain";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = grid.info.resolution;
    marker.scale.y = grid.info.resolution;
    marker.scale.z = 0.03;
    marker.color.r = i == 0 ? 1.0F : i == 2 ? 0.95F : 0.0F;
    marker.color.g = i == 0 ? 0.55F : i == 2 ? 0.85F : 0.85F;
    marker.color.b = i == 1 ? 1.0F : 0.0F;
    marker.color.a = 0.8F;
  }
  for (size_t index = 0; index < grid.data.size(); ++index) {
    const int8_t label = grid.data[index];
    const size_t category = label == 5 ? 0U : label == 6 ? 1U : label == 7 ? 2U : 3U;
    if (category == 3U) continue;
    geometry_msgs::msg::Point point;
    point.x = grid.info.origin.position.x +
      (static_cast<unsigned int>(index % grid.info.width) + 0.5) * grid.info.resolution;
    point.y = grid.info.origin.position.y +
      (static_cast<unsigned int>(index / grid.info.width) + 0.5) * grid.info.resolution;
    point.z = 0.06;
    result.markers[category].points.push_back(point);
  }
  for (auto & marker : result.markers) {
    if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  return result;
}

}  // namespace

namespace mas2027_nav_executor {

PathPlanner::PathPlanner(std::shared_ptr<TerrainGrid> terrain, double odom_timeout_s,
  const std::string & odom_frame, double rog_map_timeout_s)
: terrain_(std::move(terrain)), odom_timeout_s_(odom_timeout_s),
  rog_map_timeout_s_(rog_map_timeout_s)
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("nav_executor_planner");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rog_map::Config config;
  config.loadFromRosNode(node_, "planner.rog_map");
  if (!terrain_ && (!config.unknown_as_occupied || config.prior_map_enable)) {
    throw std::invalid_argument(
      "online navigation requires projection.unknown_as_occupied=true and prior_map.enable=false");
  }
  rog_map_ = std::make_shared<rog_map::ROGMapROS>(node_, config, tf_buffer_);
  // ROGMap supplies live geometry; a non-null terrain adds prior global constraints.
  planner_ = std::make_shared<minco_planner::MincoPlanner>();
  planner_->setMap(rog_map_->queryInterface());
  // Declare startup-only map settings before MincoPlanner installs its runtime
  // parameter callback. ROS invokes that callback while declaring parameters.
  rcl_interfaces::msg::ParameterDescriptor global_map_descriptor;
  global_map_descriptor.read_only = true;
  global_map_descriptor.description = "Online global map setting selected at startup";
  const bool accumulate = node_->declare_parameter<bool>("planner.online_global_map.enabled", true, global_map_descriptor);
  OnlineGlobalMapConfig global_config;
  double rate = 0.0;
  if (!terrain_ && accumulate) {
    global_config.resolution = node_->declare_parameter<double>("planner.online_global_map.resolution", 0.1, global_map_descriptor);
    const auto max_cells = node_->declare_parameter<int64_t>("planner.online_global_map.max_cells", 2000000, global_map_descriptor);
    if (max_cells < 25) throw std::invalid_argument("online_global_map.max_cells must be >= 25");
    global_config.max_cells = static_cast<size_t>(max_cells);
    global_config.occupied_increment = node_->declare_parameter<int>("planner.online_global_map.occupied_increment", 3, global_map_descriptor);
    global_config.free_decrement = node_->declare_parameter<int>("planner.online_global_map.free_decrement", 1, global_map_descriptor);
    global_config.evidence_limit = node_->declare_parameter<int>("planner.online_global_map.evidence_limit", 5, global_map_descriptor);
    global_config.occupied_confirmations = node_->declare_parameter<int>("planner.online_global_map.occupied_confirmations", 2, global_map_descriptor);
    global_config.confirmation_max_gap_s = node_->declare_parameter<double>("planner.online_global_map.confirmation_max_gap_s", 1.5, global_map_descriptor);
    global_config.terrain_confirmations = node_->declare_parameter<int>("planner.online_global_map.terrain_confirmations", 3, global_map_descriptor);
    global_config.terrain_clear_confirmations = node_->declare_parameter<int>("planner.online_global_map.terrain_clear_confirmations", 10, global_map_descriptor);
    rate = node_->declare_parameter<double>("planner.online_global_map.update_rate_hz", 2.0, global_map_descriptor);
    if (!std::isfinite(rate) || rate <= 0 || rate > 20 ||
      global_config.resolution < config.resolution ||
      global_config.resolution / config.resolution > 32 ||
      std::abs(global_config.resolution / config.resolution -
        std::round(global_config.resolution / config.resolution)) > 1e-6) {
      throw std::invalid_argument("invalid online global map rate or resolution (must be an integer ROGMap multiple)");
    }
  }
  planner_->configure(node_, "planner", tf_buffer_);
  std::string planning_frame;
  node_->get_parameter("planner.frames.rog_frame", planning_frame);
  if (config.frame_id != odom_frame || planning_frame != odom_frame) {
    throw std::invalid_argument(
      "ROGMap frame, planner.frames.rog_frame and node.frames.odom must match");
  }
  bool unknown_blocked = false;
  node_->get_parameter("planner.exploration.unknown_as_occupied", unknown_blocked);
  if (!terrain_ && !unknown_blocked) {
    throw std::invalid_argument("online navigation requires exploration.unknown_as_occupied=true");
  }
  planner_->setTerrainGrid(terrain_);
  if (!terrain_ && accumulate) {
    online_global_map_ = std::make_shared<OnlineGlobalMap>(global_config, odom_frame);
    planner_->setOnlineTerrainMap(online_global_map_);
    global_map_pub_ = rclcpp::create_publisher<nav_msgs::msg::OccupancyGrid>(
      *node_,
      "/nav_executor/online_global_map", rclcpp::QoS(1).reliable().transient_local());
    global_terrain_pub_ = rclcpp::create_publisher<nav_msgs::msg::OccupancyGrid>(
      *node_, "/nav_executor/online_terrain_map",
      rclcpp::QoS(1).reliable().transient_local());
    global_uphill_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(
      *node_, "/nav_executor/online_terrain_uphill",
      rclcpp::QoS(1).reliable().transient_local());
    global_terrain_markers_pub_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      *node_, "/nav_executor/online_terrain_markers",
      rclcpp::QoS(1).reliable().transient_local());
    const auto adapter = std::dynamic_pointer_cast<rog_map::QueryAdapter>(rog_map_->queryInterface());
    if (!adapter) throw std::runtime_error("ROGMap snapshot adapter unavailable for global accumulation");
    global_map_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    global_map_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      [this, adapter, max_cells = global_config.max_cells]() {
        const auto source = adapter->snapshot();
        if (!source || !std::isfinite(source->stamp) ||
          std::abs(node_->now().seconds() - source->stamp) > rog_map_timeout_s_) return;
        const auto update_started = std::chrono::steady_clock::now();
        const auto result = online_global_map_->update(*source);
        const double update_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - update_started).count();
        if (result == OnlineGlobalMap::UpdateResult::UPDATED) {
          if (const auto snapshot = online_global_map_->snapshot()) {
            const auto cells = static_cast<size_t>(snapshot->sizeX()) * snapshot->sizeY();
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
              "Online global map: cells=%zu/%zu revision=%llu update=%.1f ms",
              cells, max_cells,
              static_cast<unsigned long long>(snapshot->revision()), update_ms);
            global_map_pub_->publish(snapshot->grid());
            const auto terrain_grid = snapshot->terrainGrid();
            global_terrain_pub_->publish(terrain_grid);
            global_uphill_pub_->publish(snapshot->uphillImage());
            global_terrain_markers_pub_->publish(terrainMarkers(terrain_grid));
          }
        } else if (result == OnlineGlobalMap::UpdateResult::CELL_LIMIT) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Online global map reached cell limit; preserving existing history without further growth");
        } else if (result == OnlineGlobalMap::UpdateResult::INVALID_INPUT) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Online global map rejected invalid or unaligned ROGMap snapshot");
        }
      }, global_map_callback_group_);
    RCLCPP_INFO(node_->get_logger(), "Online global map enabled: %.3f m/cell, %.1f Hz, limit=%zu cells",
      global_config.resolution, rate, global_config.max_cells);
  }
  task_manager_ = std::make_unique<TaskManager>(planner_, node_->get_logger(), odom_timeout_s_);
  task_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    task_manager_->tick();
  });
}

PathPlanner::~PathPlanner()
{
  global_map_timer_.reset();
  task_timer_.reset();
  task_manager_.reset();
  if (planner_) planner_->cleanup();
}

std::shared_ptr<rog_map::MapQueryInterface> PathPlanner::mapQuery() const
{
  return rog_map_->queryInterface();
}

std::shared_ptr<rog_map::MapQueryInterface> PathPlanner::globalMapQuery() const
{
  return online_global_map_ ? online_global_map_->snapshot() : nullptr;
}

bool PathPlanner::copyLatestGlobalPath(std::vector<geometry_msgs::msg::PoseStamped> & out) const
{
  if (!planner_) {
    out.clear();
    return false;
  }
  return planner_->copyLatestGlobalPath(out);
}

bool PathPlanner::acceptGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  const auto terrain = terrain_ ? terrain_->snapshot() : nullptr;
  if (terrain_ && !terrain) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal until static terrain cost and label maps are ready");
    return false;
  }
  const auto query = rog_map_->queryInterface();
  if (!query || query->sizeX() == 0U || query->sizeY() == 0U) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal until ROGMap has an online snapshot");
    return false;
  }
  const double rog_stamp_s = query->snapshotStampSeconds();
  const double rog_age_s = std::isfinite(rog_stamp_s) ?
    std::abs(node_->now().seconds() - rog_stamp_s) : std::numeric_limits<double>::infinity();
  if (!std::isfinite(rog_age_s) || rog_age_s > rog_map_timeout_s_) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal until ROGMap is fresh (age %.3f s, limit %.3f s)",
      rog_age_s, rog_map_timeout_s_);
    return false;
  }

  // Planning frame name kept for config compatibility; default odom.
  std::string planning_frame{"odom"};
  if (node_->has_parameter("planner.frames.rog_frame")) {
    node_->get_parameter("planner.frames.rog_frame", planning_frame);
  }
  if (planning_frame.empty()) {
    planning_frame = "odom";
  }

  // 远端（Foxglove 等）目标的时间戳来自客户端时钟，与本机有偏差时 tf2 会因"请求时间在未来"
  // 抛 extrapolation into the future 并静默丢弃目标；归零即可（tf2 把 0 视为"取最新 TF"）。
  geometry_msgs::msg::PoseStamped goal_query = goal;
  goal_query.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  geometry_msgs::msg::PoseStamped goal_in_planning = goal_query;
  const std::string source = goal.header.frame_id.empty() ? planning_frame : goal.header.frame_id;
  if (source != planning_frame) {
    try {
      goal_in_planning = tf_buffer_->transform(goal_query, planning_frame, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
        "Ignoring goal: cannot transform from %s to %s (%s)",
        source.c_str(), planning_frame.c_str(), ex.what());
      return false;
    }
  } else {
    goal_in_planning.header.frame_id = planning_frame;
  }

  // transform() 是否改写输出时间戳取决于实现，显式再归零，让第二次（转地形图坐标系）同样取最新 TF。
  goal_in_planning.header.stamp = goal_query.header.stamp;

  if (terrain) {
    geometry_msgs::msg::PoseStamped goal_in_map;
    try {
      goal_in_map = tf_buffer_->transform(goal_in_planning, terrain->cost.header.frame_id,
        tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Ignoring goal: terrain TF unavailable (%s)", ex.what());
      return false;
    }
    const Eigen::Vector2d target(goal_in_map.pose.position.x, goal_in_map.pose.position.y);
    if (!terrain->traversable(target)) {
      RCLCPP_WARN(node_->get_logger(), "Ignoring goal: occupied in static terrain");
      return false;
    }
  }
  geometry_msgs::msg::PoseStamped start;
  if (!planner_->getRobotPose(start)) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: no /Odometry received by planner yet");
    return false;
  }
  if (std::abs(node_->now().seconds() - rclcpp::Time(start.header.stamp).seconds()) >
    odom_timeout_s_) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: odometry is stale");
    return false;
  }
  unsigned int mx = 0, my = 0;
  if (!query->worldToMap(start.pose.position.x, start.pose.position.y, mx, my) ||
    !query->isFree(mx, my) ||
    !query->query({start.pose.position.x, start.pose.position.y, 0.0}).ok) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring goal: ROGMap is not clear at the robot pose");
    return false;
  }
  if (planner_->usesOnlineMapOnly() && planner_->goalKnownInvalid(start, goal_in_planning)) {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring goal (%.2f, %.2f): confirmed occupied in the live and accumulated "
      "maps, with no safe approach/ejection within the observed window",
      goal_in_planning.pose.position.x, goal_in_planning.pose.position.y);
    return false;
  }
  task_manager_->submitGoal(goal_in_planning);
  RCLCPP_INFO(node_->get_logger(), "Queued goal (%.2f, %.2f) in %s; waiting for global search and MINCO",
    goal_in_planning.pose.position.x, goal_in_planning.pose.position.y,
    planning_frame.c_str());
  return true;
}

}  // namespace mas2027_nav_executor
