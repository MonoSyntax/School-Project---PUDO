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
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation_2025'), 'launch', 'localization.launch.py')),
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
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('navigation_2025'), 'launch', 'navigation.launch.py')),
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
    
    # Traffic Sign Detection Node
    yolo_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='otagg_vision',
                executable='yolov12_node.py',
                name='yolo_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Traffic State Manager Node
    traffic_state_manager = TimerAction(
        period=16.0,
        actions=[
            Node(
                package='otagg_vision',
                executable='traffic_state_manager_node.py',
                name='traffic_state_manager',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Velocity Override Node - gates cmd_vel based on traffic state
    velocity_override = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='otagg_vision',
                executable='velocity_override_node.py',
                name='velocity_override',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'red_light_stop_distance': 12.0,
                    'red_light_approach_distance': 15.0,
                    'red_light_wait_duration': 10.0,
                    'stop_sign_stop_distance': 5.0,
                    'stop_sign_approach_distance': 10.0,
                    'stop_sign_wait_duration': 3.0,
                }]
            )
        ]
    )
    
    # Bus Stop Loop Node - automatic navigation between stops
    bus_stop_loop = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='otagg_vision',
                executable='bus_stop_loop_node.py',
                name='bus_stop_loop',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'loop_delay_sec': 2.0,
                }]
            )
        ]
    )
   
    ld = LaunchDescription()

    ld.add_action(sim_launcher)
    ld.add_action(ekf_localization)
    ld.add_action(localization)
    ld.add_action(navigation)
    ld.add_action(rviz2)
    ld.add_action(yolo_node)
    ld.add_action(traffic_state_manager)
    ld.add_action(velocity_override)
    ld.add_action(bus_stop_loop)
    
    return ld

