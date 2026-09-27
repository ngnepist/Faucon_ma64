cmake_minimum_required(VERSION 3.8)
project(faucon_bt_plugins)
# ============================================================================
# COMPILATION
# ============================================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Utilisation de C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ============================================================================
# DÉPENDANCES
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# ============================================================================
# PLUGIN : RobotNearMissionStart
# ============================================================================
add_library(robot_near_mission_start_bt_node SHARED
  src/robot_near_mission_start.cpp
)
target_compile_definitions(robot_near_mission_start_bt_node
  PRIVATE
    BT_PLUGIN_EXPORT
)
target_include_directories(robot_near_mission_start_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_near_mission_start_bt_node
  rclcpp
  std_srvs
  behaviortree_cpp
  nav2_behavior_tree
)
# ============================================================================
# PLUGIN : FindClosestMissionPose   
# ============================================================================
add_library(find_closest_mission_pose_bt_node SHARED
  src/find_closest_mission_pose.cpp)

ament_target_dependencies(find_closest_mission_pose_bt_node
  behaviortree_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs)

target_include_directories(find_closest_mission_pose_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(find_closest_mission_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)


# ============================================================================
# PLUGIN : RobotReadyToFollowMission
# ============================================================================
add_library(robot_ready_to_follow_mission_bt_node SHARED
  src/robot_ready_to_follow_mission.cpp
)

ament_target_dependencies(robot_ready_to_follow_mission_bt_node
  behaviortree_cpp
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(robot_ready_to_follow_mission_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(robot_ready_to_follow_mission_bt_node
  PRIVATE
    BT_PLUGIN_EXPORT
)

# ============================================================================
# PLUGIN : SelectMissionEntryPose
# ============================================================================
add_library(select_mission_entry_pose_bt_node SHARED
  src/select_mission_entry_pose.cpp
)

target_compile_definitions(select_mission_entry_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(select_mission_entry_pose_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(select_mission_entry_pose_bt_node
  behaviortree_cpp
  geometry_msgs
  nav_msgs
  rclcpp
)

# ============================================================================
# PLUGIN : ExtractRemainingPath
# ============================================================================
add_library(extract_remaining_path_bt_node SHARED
  src/extract_remaining_path.cpp
)

target_compile_definitions(extract_remaining_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(extract_remaining_path_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(extract_remaining_path_bt_node
  behaviortree_cpp
  nav_msgs
  rclcpp
)

# ============================================================================
# INSTALLATION
# ============================================================================
install(
  TARGETS 
  robot_near_mission_start_bt_node
  find_closest_mission_pose_bt_node
  robot_ready_to_follow_mission_bt_node
  select_mission_entry_pose_bt_node
  extract_remaining_path_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
# ============================================================================
# PACKAGE
# ============================================================================
ament_package()