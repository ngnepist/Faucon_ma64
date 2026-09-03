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
    
    bringup_dir = get_package_share_directory("faucon_bringup")
    loc_launch_dir = os.path.join(
        bringup_dir, "launch", "loc"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    #robot localization nodes 
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_launch_dir, 'dual_ekf_navsat.launch.py'))
    )


    # Launch them all!
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            robot_localization_cmd,
        ]
    )
