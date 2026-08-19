from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from os import path


def launch_setup(context, *args, **kwargs):
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_faucon_config = get_package_share_directory("faucon_config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mini_value = LaunchConfiguration("use_mini").perform(context)
    
    
    if use_mini_value.lower() == "true":
        spawn_z = "0.4"
        spawn_y = "-2.94"
        use_ros2_control_value = "false"
    else:
        spawn_z = "1.0"
        spawn_y = "-2.83"
        use_ros2_control_value = LaunchConfiguration("use_ros2_control").perform(context)
   
    control_4ws = Node(
        package="faucon_control",
        executable="control_4ws.py",
        name="control_4ws",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )
    
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "bot",
            "-allow_renaming",
            "true",
            "-x",
            "-2.28",
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            "-0.01",
            "-P",
            "-0.03",
            "-Y",
            "1.52",
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_faucon_config, "config", "sim", "bridge.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
            "/camera/depth_image",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "20"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    steer_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steer_controller"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        condition=IfCondition(PythonExpression(['"', use_ros2_control_value, '" == "true"'])),
    )

   
    return [
        spawn_robot,
        gz_bridge,
        control_4ws,
        ros_gz_image_bridge,
        steer_controller,
        velocity_controller,
        joint_broad_spawner,
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