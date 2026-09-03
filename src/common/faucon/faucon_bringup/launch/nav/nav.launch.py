import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = "faucon_navigation"
    
    nav2_params_path = os.path.join(
        get_package_share_directory("faucon_config"), "config", "nav", "fauncon_nav2_params.yaml"
    )

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # mission manager
    mission_manager_cmd = Node(
        package="faucon_navigation",
        executable="mission_manager.py",
        name="mission_manager",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Launch them all!
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),            
            launch.actions.DeclareLaunchArgument(
                name="params_file",
                default_value=nav2_params_path,
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            launch.actions.DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            launch.actions.DeclareLaunchArgument(
                'use_intra_process_comms',
                default_value='True',
                description='Whether to use intra process communications',
            ),
            launch.actions.DeclareLaunchArgument(
                'use_localization',
                default_value='False',
                description='Whether to enable localization or not'
            ),
           
            # Launch the ROS 2 Navigation Stack
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                   os.path.join(nav2_launch_dir, "bringup_launch.py")  
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                    "use_intra_process_comms": LaunchConfiguration("use_intra_process_comms"),
                    "use_localization": LaunchConfiguration("use_localization"),

                }.items(),
            ),
            mission_manager_cmd,
        ]
    )
