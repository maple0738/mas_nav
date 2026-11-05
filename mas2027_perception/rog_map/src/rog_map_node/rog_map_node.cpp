// ROG-Map 独立宿主节点。
//
// rog_map 上游是纯库包，ROGMapROS 通过 bindNode() 挂到外部节点上，本身不提供
// 可执行文件。本文件是本仓库新增的最小宿主：建节点、加载 `rog_map` 前缀下的参数、
// 构造 ROGMapROS，然后用多线程执行器 spin。
//
// 上游库源码由 CMakeLists 中 src/rog_map/*.cpp 与 src/rog_map_ros/*.cpp 的 glob
// 收集，本目录不在其中，因此这个文件不会混进上游代码，方便后续同步上游。
//
// 这个可执行文件不参与在线导航链路：ROGMap 由 nav_executor 进程内持有。
// 不要与导航同时启动，否则两份滑动地图会使用不同原点。
// 它只用于单独调试建图，参数需自备（节点名与参数前缀均为 rog_map）。
//
// 输入：
//   /Odometry         nav_msgs/Odometry     small_point_lio，父坐标系 odom
//   /cloud_registered_full sensor_msgs/PointCloud2  small_point_lio 稠密去畸变点云，odom 坐标系
//
// 输出：/rog_map/* 下的可视化点云、OccupancyGrid 与 MarkerArray，详见 README。

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <rog_map/rog_map_core/config.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("rog_map");

  rog_map::Config cfg;
  cfg.loadFromRosNode(node, "rog_map");

  auto rog_map_ros = std::make_shared<rog_map::ROGMapROS>(node, cfg);

  // ROGMapROS 为 odom、cloud、update 分别建了回调组，必须用多线程执行器，
  // 否则 update timer 会被点云回调阻塞。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
