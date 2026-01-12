import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # RViz2 configuration file
    rviz_config_file = os.path.join(get_package_share_directory('navigation_2025'), 'config', 'rviz2.rviz')
    
    # Include another launch file
    sim_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('simulation_2025'), 'launch', 'teknofest_IGN.launch.py'))
    )
    
    ekf_localization = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation_2025'), 'launch', 'ekf_localization.launch.py'))
            )
        ]
    )

    localization = TimerAction(
        period=12.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation_2025'), 'launch', 'keepout_localization_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    navigation = TimerAction(
        period=20.0,  # Increased from 15.0 to 20.0
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation_2025'), 'launch', 'keepout_navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map_subscribe_transient_local': 'true'
                }.items()
            )
        ]
    )

    
    rviz2 = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen'
    )
   
    ld = LaunchDescription()

    ld.add_action(sim_launcher)
    ld.add_action(ekf_localization)
    ld.add_action(localization)
    ld.add_action(navigation)
    ld.add_action(rviz2)
    
    return ld
