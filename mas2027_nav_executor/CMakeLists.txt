cmake_minimum_required(VERSION 3.10)
project(mas2027_nav_executor)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  # PCLConfig resets its policy version to 3.10 before finding FLANN.
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(rog_map REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(mpc_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/mpc")
file(GLOB qpoases_sources "${mpc_source_dir}/qpOASES/src/*.cpp")
add_library(${PROJECT_NAME}_qpoases STATIC ${qpoases_sources})
set_target_properties(${PROJECT_NAME}_qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_qpoases PUBLIC
  "${mpc_source_dir}/qpOASES/include")
target_compile_definitions(${PROJECT_NAME}_qpoases PRIVATE __NO_COPYRIGHT__ LINUX)
target_compile_options(${PROJECT_NAME}_qpoases PRIVATE -Wno-format-truncation)

add_library(${PROJECT_NAME}_mpc STATIC
  src/path_executor/mpc/mpc_solver.cpp)
target_include_directories(${PROJECT_NAME}_mpc PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME}_mpc PUBLIC Eigen3::Eigen ${PROJECT_NAME}_qpoases)

set(minco_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/vendor/minco")
file(GLOB minco_utils_sources "${minco_source_dir}/src/utils/*.cpp")
file(GLOB minco_traj_sources "${minco_source_dir}/src/traj_opt/*.cpp")
add_library(${PROJECT_NAME}_planner STATIC
  ${minco_utils_sources}
  ${minco_traj_sources}
  src/path_planner/search/astar.cpp
  src/path_planner/search/global_path_searcher.cpp
  src/path_planner/search/smac/smac_planner_2d_simple.cpp
  src/path_planner/trajectory/corridor_generator.cpp
  src/path_planner/trajectory/arc_length_speed_profile.cpp
  src/path_planner/trajectory/local_path_processor.cpp
  src/path_planner/trajectory/minco_optimizer.cpp
  src/path_planner/trajectory/minco_planner.cpp
  src/path_planner/trajectory/trajectory_safety_checker.cpp
  src/common/environment/online_global_map.cpp
  src/common/environment/fused_map_query.cpp
  src/common/environment/terrain_grid.cpp
  src/common/environment/terrain_map_query.cpp
  "${minco_source_dir}/src/components/planner_mode_context.cpp"
  src/task_manager/recovery/recovery_behaivor.cpp
  "${minco_source_dir}/src/minco_utils.cpp"
  "${minco_source_dir}/src/visualizer.cpp"
  "${minco_source_dir}/src/performance/planner_performance_monitor.cpp")
target_include_directories(${PROJECT_NAME}_planner PUBLIC
  "${minco_source_dir}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
target_compile_definitions(${PROJECT_NAME}_planner PUBLIC
  ROOT_DIR="${minco_source_dir}/" ORIGIN_AT_CORNER USE_ROS2)
ament_target_dependencies(${PROJECT_NAME}_planner
  builtin_interfaces geometry_msgs interfaces nav_msgs rcl_interfaces rclcpp rclcpp_lifecycle
  rog_map sensor_msgs tf2_geometry_msgs tf2_ros visualization_msgs)
target_link_libraries(${PROJECT_NAME}_planner
  OpenMP::OpenMP_CXX yaml-cpp::yaml-cpp ${OpenCV_LIBS} dw)

add_executable(${PROJECT_NAME}_node src/nav_executor_node.cpp
  src/path_planner/path_planner.cpp src/path_executor/path_executor.cpp
  src/path_executor/monitoring/command_safety.cpp
  src/task_manager/task_manager.cpp
  src/common/environment/region_control.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  "${minco_source_dir}/include")
ament_target_dependencies(${PROJECT_NAME}_node
  example_interfaces
  geometry_msgs
  interfaces
  rog_map
  nav_msgs
  sensor_msgs
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_mpc ${PROJECT_NAME}_planner ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
# 2026-09-18：包内 launch/nav_executor.launch.py 已删除（只起本节点、不拉 map_server，
# 是个会静默跑在"没有地形图"状态的重复入口；生产唯一入口是
# mas2027_nav_bringup/launch/nav_executor_launch.py）。故这里不再安装 launch 目录。
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_online_global_map test/test_online_global_map.cpp)
  target_include_directories(test_online_global_map PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_online_global_map rclcpp rog_map nav_msgs)
  target_link_libraries(test_online_global_map ${PROJECT_NAME}_planner Eigen3::Eigen)
  ament_add_gtest(test_online_navigation test/test_online_navigation.cpp
    src/path_executor/monitoring/command_safety.cpp)
  target_include_directories(test_online_navigation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_online_navigation rclcpp rog_map tf2_ros tf2_geometry_msgs)
  target_link_libraries(test_online_navigation ${PROJECT_NAME}_planner Eigen3::Eigen)
  ament_add_gtest(test_smac_wall_clearance test/test_smac_wall_clearance.cpp)
  target_include_directories(test_smac_wall_clearance PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_smac_wall_clearance rclcpp rog_map)
  target_link_libraries(test_smac_wall_clearance ${PROJECT_NAME}_planner Eigen3::Eigen)
  ament_add_gtest(test_fused_map_query test/test_fused_map_query.cpp)
  target_include_directories(test_fused_map_query PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_fused_map_query rog_map)
  target_link_libraries(test_fused_map_query ${PROJECT_NAME}_planner Eigen3::Eigen)
  ament_add_gtest(test_region_online_priority
    test/test_region_online_priority.cpp
    src/common/environment/region_control.cpp
    src/common/environment/terrain_grid.cpp)
  target_include_directories(test_region_online_priority PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_region_online_priority
    geometry_msgs interfaces nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(test_region_online_priority Eigen3::Eigen)
  ament_add_gtest(test_replan_continuity test/test_replan_continuity.cpp)
  target_include_directories(test_replan_continuity PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/minco/include>)
  target_link_libraries(test_replan_continuity Eigen3::Eigen)
  ament_add_gtest(test_mpc_gimbal_follow
    test/test_mpc_gimbal_follow.cpp
    src/path_executor/path_executor.cpp
    src/path_executor/monitoring/command_safety.cpp
    src/common/environment/terrain_grid.cpp)
  target_include_directories(test_mpc_gimbal_follow PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_mpc_gimbal_follow
    geometry_msgs interfaces nav_msgs rclcpp rog_map sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(test_mpc_gimbal_follow ${PROJECT_NAME}_mpc Eigen3::Eigen)
  ament_add_gtest(test_region_yaw_ref
    test/test_region_yaw_ref.cpp
    src/common/environment/region_control.cpp
    src/common/environment/terrain_grid.cpp)
  target_include_directories(test_region_yaw_ref PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_region_yaw_ref
    geometry_msgs interfaces nav_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(test_region_yaw_ref Eigen3::Eigen)

  ament_add_gtest(test_global_goal_cell_clearance
    test/test_global_goal_cell_clearance.cpp)
  target_include_directories(test_global_goal_cell_clearance PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/minco/include>)
  ament_target_dependencies(test_global_goal_cell_clearance
    geometry_msgs nav_msgs rclcpp rog_map tf2_ros)
  target_link_libraries(test_global_goal_cell_clearance ${PROJECT_NAME}_planner Eigen3::Eigen)
  ament_add_gtest(test_minco_two_stage
    test/test_minco_two_stage.cpp)
  target_include_directories(test_minco_two_stage PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/minco/include>)
  ament_target_dependencies(test_minco_two_stage
    nav_msgs rclcpp rog_map)
  target_link_libraries(test_minco_two_stage ${PROJECT_NAME}_planner Eigen3::Eigen)
  # 目标接近减速带：共享速度包络 + 弧长速度剖面（加减速能力分离）。
  ament_add_gtest(test_goal_approach
    test/test_goal_approach.cpp
    src/path_planner/trajectory/arc_length_speed_profile.cpp)
  target_include_directories(test_goal_approach PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_goal_approach Eigen3::Eigen)
endif()

ament_package()
