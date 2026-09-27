cmake_minimum_required(VERSION 3.8)
project(faucon_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(faucon_interfaces REQUIRED)

add_library(faucon_mission_navigator SHARED src/faucon_mission_navigator.cpp)

target_include_directories(faucon_mission_navigator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(faucon_mission_navigator
  rclcpp
  nav2_core
  nav2_behavior_tree
  nav2_util
  nav_msgs
  pluginlib
  faucon_interfaces)

pluginlib_export_plugin_description_file(nav2_core config/faucon_navigator_plugin.xml)

install(TARGETS faucon_mission_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

# Install config and launch directories
install(
  DIRECTORY  config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gps_waypoint_logger.py
  scripts/mission_manager.py
  scripts/follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# mission_core is a module (not an entrypoint), installed alongside mission_manager
install(FILES
  scripts/mission_core.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
