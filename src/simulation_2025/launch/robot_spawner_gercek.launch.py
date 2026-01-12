
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_share = FindPackageShare(package='simulation_2025').find('simulation_2025')

    model_path = os.path.join(pkg_share, 'urdf_gercek', 'otagg_car.urdf.xacro')
    urdf_dir = os.path.join(pkg_share, 'urdf_gercek')
    gz_sim_resource_path = os.path.join(pkg_share, 'models')
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_sim_resource_path

    # Xacro dosyasını işlemek için tam yol belirtelim ve include dizinini ekleyelim
    robot_description = ParameterValue(
        Command(['xacro ', model_path, ' ', 'use_sim_time:=false']), 
        value_type=str
    )


    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=model_path, 
        description='Absolute path to robot urdf file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 50.0,  # Increased from 30.0 to 50.0 for better sync
            'tf_prefix': '',
            'frame_prefix': '',
            'ignore_timestamp': False,  # Ensure proper timestamp handling
        }],
        output='screen',
        respawn=True,
        respawn_delay=1,  # Reduced from 2 to 1 second
        arguments=['--ros-args', '--log-level', 'warn'],  # Reduced log level
    )

    # Joint state publisher for manual joint control (if needed)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'rate': 50,  # Match robot_state_publisher frequency
            'source_list': ['/joint_states'],  # Explicit source list
        }],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )


    # Create the launch description and populate
    ld = LaunchDescription()
   
    # Add any actions
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld
