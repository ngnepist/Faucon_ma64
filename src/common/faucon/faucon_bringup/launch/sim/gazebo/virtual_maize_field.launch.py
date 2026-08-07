from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from ros_gz_bridge.actions import RosGzBridge
from launch.substitutions import LaunchConfiguration, EnvironmentVariable



def generate_launch_description():
    # Path to launch the ros_gz_sim
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )
pkg_path = get_package_share_directory("faucon_base_desc")
    # World to launch
    world_path = os.path.join(get_package_share_directory('agri_worlds'), 
                                "worlds", 
                                "virtual_maize_field", 
                                "generated.world")

    agri_worlds_models_path = os.path.join(
        get_package_share_directory('agri_worlds'),
        'media',
        'models'
    )

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            os.pathsep,
            agri_worlds_models_path,
        ]
    )

    #parametre_du bridge
    bridge_config = os.path.join(
        get_package_share_directory('srbc_config'), 
        'config', 
        'sim',
        'ros_gz_bridges.yaml')

    # Lancer Gazebo (gz sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Arguments optionnels
    declare_bridge_name = DeclareLaunchArgument(
        'bridge_name',
        default_value='gazebo_bridge',
        description='Nom du nœud ros_gz_bridge'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=bridge_config,
        description='Fichier YAML de configuration pour le bridge'
    )

    # Utilisation de l’action RosGzBridge
    gz_bridge = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file'),
    )

    # Assemble la description du lancement
    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        declare_bridge_name,
        declare_config_file,
        gz_bridge
    ])