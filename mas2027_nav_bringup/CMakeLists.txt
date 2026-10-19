cmake_minimum_required(VERSION 3.8)
project(mas2027_nav_bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

endif()

install(PROGRAMS
  scripts/measure_lidar_mount.py
  scripts/measure_lidar_mount.sh
  scripts/record_nav_diagnostics.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Keep the map manifest as the single selection point. Historical maps remain
# in the source tree, while an installed bringup contains only its active set.
function(_read_map_value file key output)
  file(STRINGS "${file}" lines REGEX "^[ \t]*${key}[ \t]*:")
  list(LENGTH lines count)
  if(NOT count EQUAL 1)
    message(FATAL_ERROR "${file}: expected exactly one ${key} entry")
  endif()
  list(GET lines 0 line)
  string(REGEX REPLACE "^[ \t]*${key}[ \t]*:[ \t]*" "" value "${line}")
  string(REGEX REPLACE "[ \t]+#.*$" "" value "${value}")
  string(STRIP "${value}" value)
  string(REGEX REPLACE "^\"(.*)\"$" "\\1" value "${value}")
  string(REGEX REPLACE "^'(.*)'$" "\\1" value "${value}")
  if(value STREQUAL "")
    message(FATAL_ERROR "${file}: ${key} must name a file")
  endif()
  set(${output} "${value}" PARENT_SCOPE)
endfunction()

function(_resolve_map_file base value output)
  if(IS_ABSOLUTE "${value}")
    set(path "${value}")
  else()
    get_filename_component(path "${base}/${value}" ABSOLUTE)
  endif()
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "Selected navigation map file does not exist: ${path}")
  endif()
  set(${output} "${path}" PARENT_SCOPE)
endfunction()

function(_install_selected_map_file path)
  file(RELATIVE_PATH relative "${CMAKE_CURRENT_SOURCE_DIR}" "${path}")
  if(relative MATCHES "^\\.\\." OR IS_ABSOLUTE "${relative}")
    message(STATUS "Navigation map is external and will not be copied: ${path}")
    return()
  endif()
  get_filename_component(directory "${relative}" DIRECTORY)
  install(FILES "${path}" DESTINATION "share/${PROJECT_NAME}/${directory}")
endfunction()

set(_map_manifest "${CMAKE_CURRENT_SOURCE_DIR}/config/navigation_map.yaml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_map_manifest}")
_read_map_value("${_map_manifest}" localization_pcd _selected_pcd)
_read_map_value("${_map_manifest}" occupancy_yaml _selected_yaml)
_read_map_value("${_map_manifest}" terrain_msgpack _selected_terrain)
_resolve_map_file("${CMAKE_CURRENT_SOURCE_DIR}" "${_selected_pcd}" _pcd_path)
_resolve_map_file("${CMAKE_CURRENT_SOURCE_DIR}" "${_selected_yaml}" _yaml_path)
_resolve_map_file("${CMAKE_CURRENT_SOURCE_DIR}" "${_selected_terrain}" _terrain_path)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_yaml_path}")
get_filename_component(_yaml_directory "${_yaml_path}" DIRECTORY)
_read_map_value("${_yaml_path}" image _selected_image)
_resolve_map_file("${_yaml_directory}" "${_selected_image}" _image_path)
_install_selected_map_file("${_pcd_path}")
_install_selected_map_file("${_yaml_path}")
_install_selected_map_file("${_terrain_path}")
_install_selected_map_file("${_image_path}")

ament_auto_package(
  INSTALL_TO_SHARE
  config
  rviz
)
