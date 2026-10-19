cmake_minimum_required(VERSION 3.8)
project(rog_map)

# 上游下面那段的条件写的是 CMAKE_COMPILER_ID，而 CMake 里正确的变量名是
# CMAKE_CXX_COMPILER_ID，所以那个分支实际永不进入，C++17 只是碰巧命中了
# gcc-11 的默认标准。这里显式固定标准，避免换编译器后标准漂移；上游那段
# 保持原样不动，便于后续同步上游。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_ID MATCHES "Clang|GNU")
        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)
        add_compile_options(-Wall -Wextra -Wpedantic)
        add_compile_options(-Werror=unused-variable -Werror=return-type)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_definitions(
        QT_NO_VERSION_TAGGING
        ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
        ORIGIN_AT_CORNER
        USE_ROS2
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(library_name ${PROJECT_NAME})

file(GLOB_RECURSE ROG_MAP_SOURCES CONFIGURE_DEPENDS
        src/rog_map/*.cpp
        src/rog_map_ros/*.cpp
        include/rog_map/rog_map_core/raycaster.cpp
)

add_library(${library_name} STATIC
        ${ROG_MAP_SOURCES}
)

target_compile_definitions(${library_name}
        PUBLIC
                QT_NO_VERSION_TAGGING
                ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
                ORIGIN_AT_CORNER
                USE_ROS2
)

target_include_directories(${library_name}
        PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
        PRIVATE
                ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${library_name}
        rclcpp
        rclcpp_lifecycle
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_conversions
        tf2
        tf2_ros
)

target_link_libraries(${library_name}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        yaml-cpp
)

if(OpenMP_CXX_FOUND)
        target_link_libraries(${library_name} OpenMP::OpenMP_CXX)
endif()


# 本仓库新增：独立宿主节点。上游只提供静态库，没有可执行文件。
# 源码单独放在 src/rog_map_node/ 下，不会被上面收集库源码的 glob 命中。
add_executable(rog_map_node
        src/rog_map_node/rog_map_node.cpp
)

target_link_libraries(rog_map_node
        ${library_name}
)

ament_target_dependencies(rog_map_node
        rclcpp
        tf2_ros
)

install(TARGETS rog_map_node
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

# 2026-09-18：删除 install(DIRECTORY config/ ...)。config/ 下原本只有 ROS1 时代
# dynamic_reconfigure 的 visualization.cfg，全仓库无任何引用，已随之删除；
# 目录空了之后这条 install 会在 configure 阶段直接报错。ROGMap 的参数统一由
# nav_executor 的 planner.rog_map.* 提供（见 mas2027_nav_executor/config/planner_params.yaml）。

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        pcl_conversions
        tf2
        tf2_ros
        yaml_cpp_vendor
)

if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_add_gtest(test_projection_terrain test/test_projection_terrain.cpp)
        target_link_libraries(test_projection_terrain ${library_name})
        ament_add_gtest(test_query_adapter_cell_centers test/test_query_adapter_cell_centers.cpp)
        target_link_libraries(test_query_adapter_cell_centers ${library_name})
endif()

ament_package()
