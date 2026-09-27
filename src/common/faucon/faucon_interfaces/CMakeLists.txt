cmake_minimum_required(VERSION 3.8)
project(faucon_interfaces)

# ============================================================================
# DEPENDENCIES
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ============================================================================
# INTERFACES
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateFauconMission.action"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    action_msgs
)

# ============================================================================
# PACKAGE
# ============================================================================
ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()