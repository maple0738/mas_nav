cmake_minimum_required(VERSION 3.8)
project(mas2027_nav_bringup)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(navigation_configuration test/test_configuration.py)
  ament_add_pytest_test(navigation_launch_profiles test/test_navigation_launch.py)
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
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${_map_manifest}" "${CMAKE_CURRENT_SOURCE_DIR}/mas2027_nav_bringup/configuration.py")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/mas2027_nav_bringup/configuration.py" "${_map_manifest}"
  RESULT_VARIABLE _map_result OUTPUT_VARIABLE _map_files ERROR_VARIABLE _map_error
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT _map_result EQUAL 0)
  message(FATAL_ERROR "Invalid navigation map configuration: ${_map_error}")
endif()
if(NOT _map_files STREQUAL "")
  string(REPLACE "\n" ";" _map_files "${_map_files}")
  foreach(_map_file IN LISTS _map_files)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_map_file}")
    _install_selected_map_file("${_map_file}")
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  rviz
)
