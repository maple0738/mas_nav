cmake_minimum_required(VERSION 3.8)
project(map_server)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/map_server_node.cpp
  src/utils.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC include ${MSGPACK_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp rclcpp_components nav_msgs sensor_msgs)
rclcpp_components_register_nodes(${PROJECT_NAME}_core "map_server::MapServerNode")

add_executable(map_server_node src/main.cpp)
target_link_libraries(map_server_node ${PROJECT_NAME}_core)
ament_target_dependencies(map_server_node rclcpp)

install(TARGETS ${PROJECT_NAME}_core map_server_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_direction test/test_terrain_direction.cpp)
  target_link_libraries(test_terrain_direction ${PROJECT_NAME}_core)
  target_include_directories(test_terrain_direction PRIVATE include ${MSGPACK_INCLUDE_DIR})
endif()

ament_package()
