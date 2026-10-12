cmake_minimum_required(VERSION 3.8)
project(nav_serial_bridge)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

add_executable(nav_serial_bridge_node src/nav_serial_bridge.cpp)
target_include_directories(nav_serial_bridge_node PRIVATE include)
ament_target_dependencies(nav_serial_bridge_node rclcpp interfaces)

install(TARGETS nav_serial_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
  ament_add_gtest(test_interface_port test/test_interface_port.cpp)
  target_include_directories(test_interface_port PRIVATE include)
endif()

ament_package()
