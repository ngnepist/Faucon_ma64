from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import SetEnvironmentVariable, AppendEnvironmentVariable
from os import path


def launch_setup(context, *args, **kwargs):
   
    pkg_path = get_package_share_directory("faucon_base_desc")
    
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    xacro_file_mini = os.path.join(pkg_path, "description_mini", "robot.urdf.xacro")
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mini_value = LaunchConfiguration("use_mini").perform(context)
    
    
    if use_mini_value.lower() == "true":
        robot_desc = xacro.process_file(xacro_file_mini).toxml()
        spawn_z = "0.4"
        spawn_y = "-2.94"
        use_ros2_control_value = "false"
    else:
        robot_desc = xacro.process_file(xacro_file).toxml()
        spawn_z = "1.0"
        spawn_y = "-2.83"
        use_ros2_control_value = LaunchConfiguration("use_ros2_control").perform(context)

    twist_mux_params = os.path.join(
        get_package_share_directory("faucon_config"), "config", "vehicle", "twist_mux.yaml"
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time
            }
        ],
    )

    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time
            }
        ],
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            twist_mux_params,
            {
                "use_sim_time": use_sim_time, 
                "use_stamped": PythonExpression(['"', use_ros2_control_value, '" == "true"'])
            },
        ],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

   
    return [
        robot_state_publisher,
        joint_state_publisher_node,
        # twist_mux,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            
            DeclareLaunchArgument(
                "use_sim_time", 
                default_value="true", 
                description="Use sim time if true"
            ),
            DeclareLaunchArgument(
                "use_mini", 
                default_value="false", 
                description="Use mini robot if true (forces use_ros2_control=false)"
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="ROS2 control enabled if true (ignored if use_mini=true)",
            ),
        
            OpaqueFunction(function=launch_setup),
        ]
    )