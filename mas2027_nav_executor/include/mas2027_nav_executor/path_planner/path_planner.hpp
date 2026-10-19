#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mas2027_nav_executor/path_planner/trajectory/minco_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mas2027_nav_executor/common/environment/terrain_grid.hpp"
#include "mas2027_nav_executor/task_manager/task_manager.hpp"
#include "rog_map/map_query_interface.hpp"

namespace rog_map { class ROGMapROS; }

namespace mas2027_nav_executor {

class OnlineGlobalMap;

class PathPlanner final
{
public:
  PathPlanner(std::shared_ptr<TerrainGrid> terrain, double odom_timeout_s,
    const std::string & odom_frame, double rog_map_timeout_s);
  ~PathPlanner();
  bool acceptGoal(const geometry_msgs::msg::PoseStamped & goal);
  bool acceptsTrajectory(const rclcpp::Time & stamp) const
  {
    return task_manager_ && task_manager_->acceptsTrajectory(stamp);
  }
  bool hasActiveGoal() const
  {
    return task_manager_ && task_manager_->hasActiveGoal();
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const { return node_; }
  std::shared_ptr<rog_map::MapQueryInterface> mapQuery() const;
  // Pin one immutable global map version for a complete future global search.
  std::shared_ptr<rog_map::MapQueryInterface> globalMapQuery() const;
  // 当前全局搜索输出的折线（odom 系），供 RViz 显示；无可用路径时返回 false。
  bool copyLatestGlobalPath(std::vector<geometry_msgs::msg::PoseStamped> & out) const;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_;
  minco_planner::MincoPlanner::Ptr planner_;
  std::unique_ptr<TaskManager> task_manager_;
  rclcpp::TimerBase::SharedPtr task_timer_;
  std::shared_ptr<OnlineGlobalMap> online_global_map_;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::CallbackGroup::SharedPtr global_map_callback_group_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_terrain_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr global_uphill_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_terrain_markers_pub_;
  std::shared_ptr<TerrainGrid> terrain_;
  double odom_timeout_s_;
  double rog_map_timeout_s_;
};

}  // namespace mas2027_nav_executor
