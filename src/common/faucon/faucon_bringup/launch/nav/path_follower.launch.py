from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # À remplacer par le nom réel de ton package
    package_name = "faucon_config"
    package_share = get_package_share_directory(package_name)

    params_file = os.path.join(
        package_share,
        "config",
        "nav",
        "fauncon_nav2_params.yaml"
    )

    # ========================================================
    # bt_navigator
    # ========================================================
    bt_navigator = Node(
    package="nav2_bt_navigator",
    executable="bt_navigator",
    name="bt_navigator",
    output="screen",
    parameters=[params_file],
    )

    # ========================================================
    # Planner Server
    # ========================================================
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
    )

    # ========================================================
    # Controller Server
    # ========================================================
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("/cmd_vel", "/cmd_vel_nav"),
            ("/odom", "/odom"),
        ],
    )

    # ========================================================
    # velocity smoother node
    # ========================================================
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),
        ],
    )

    # ========================================================
    # Lifecycle Manager
    # ========================================================
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "bt_navigator",
                    "planner_server",
                    "controller_server",
                    "velocity_smoother"
                ]
            }
        ],
    )

    return LaunchDescription([
        bt_navigator,
        planner_server,
        controller_server,
        velocity_smoother,
        lifecycle_manager,
    ])