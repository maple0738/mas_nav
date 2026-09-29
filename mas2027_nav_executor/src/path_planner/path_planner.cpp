#include "mas2027_nav_executor/path_planner/path_planner.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rog_map/rog_map_core/config.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  task_manager_ = std::make_unique<TaskManager>(planner_, node_->get_logger(), odom_timeout_s_);
  task_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this]() {
    task_manager_->tick();
  });
}

PathPlanner::~PathPlanner()
{
  task_timer_.reset();
  task_manager_.reset();
  if (planner_) planner_->cleanup();
}

std::shared_ptr<rog_map::MapQueryInterface> PathPlanner::mapQuery() const
{
  return rog_map_->queryInterface();
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
  task_manager_->submitGoal(goal_in_planning);
  RCLCPP_INFO(node_->get_logger(), "Queued goal (%.2f, %.2f) in %s; waiting for global search and MINCO",
    goal_in_planning.pose.position.x, goal_in_planning.pose.position.y,
    planning_frame.c_str());
  return true;
}

}  // namespace mas2027_nav_executor
