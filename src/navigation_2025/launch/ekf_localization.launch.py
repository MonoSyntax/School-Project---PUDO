from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_2025')
    parameters_file_dir = os.path.join(pkg_share, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "ekf.yaml"), {"use_sim_time": True}],
    )

    navsat_transform_node = Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[os.path.join(pkg_share, "config", "ekf.yaml"), {"use_sim_time": True}],
    )

    madgwick_filter=Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[os.path.join(pkg_share, "config", "ekf.yaml"), {"use_sim_time": True}],
    )    

    return LaunchDescription([
            navsat_transform_node,
            robot_localization_node,
            madgwick_filter,
        ]
    )