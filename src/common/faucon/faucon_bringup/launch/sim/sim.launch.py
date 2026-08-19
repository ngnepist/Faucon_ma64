import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription , DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # include_vehcile = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('srbc_bringup'),
    #                      'launch', 'vehicle', 'description.launch.py')
    #     ),
    #     launch_arguments={
    #     'use_sim_time': 'true'
    #     }.items()
    # )

    include_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('faucon_bringup'), "launch", "sim", "gazebo", "virtual_maize_field.launch.py")
        ),
        launch_arguments={
            "world_path": os.path.join(get_package_share_directory("virtual_maize_field"), "worlds", "virtual_maize_field"),
            "world_name": "generated.world",
        }.items(),
    )

    include_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('faucon_bringup'), "launch", "sim", "gazebo", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_mini": "false",
            "use_ros2_control": "true",
        }.items(),
    )

    include_vehicle_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('faucon_bringup'), "launch", "vehicle", "description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_mini": "false",
            "use_ros2_control": "true",
        }.items(),
    )

    # include_fix_gazebo_frame_id = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('srbc_bringup'),
    #                      'launch', 'sim', 'fix_gazebo_frame_id.launch.py')
    #     )
    # )
    # config_dir = FindPackageShare('srbc_config')

    # yaml_path = PathJoinSubstitution(
    #     [config_dir, 'config', 'vehicle', 'twist_mux.yaml'])

    # twist_mux_node = launch_ros.actions.Node(
    #     namespace='vehicle',
    #     package='twist_mux',
    #     executable='twist_mux',
    #     output='screen',
    #     remappings={('cmd_vel_out', 'cmd_vel')},
    #     parameters=[yaml_path]
    # )

    # vcu_driver_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('srbc_bringup'),
    #                      'launch', 'sim', 'vcu_driver_sim.launch.py')
    #     )
    # )

    return launch.LaunchDescription([
        include_vehicle_description,
        include_world,
        include_spawn_robot,
        # include_fix_gazebo_frame_id,
        # twist_mux_node,
        # vcu_driver_sim_launch
    ])