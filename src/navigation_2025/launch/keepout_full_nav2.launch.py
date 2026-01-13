import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Get package directories
    nav_pkg = get_package_share_directory('navigation_2025')
    sim_pkg = get_package_share_directory('simulation_2025')
    
    # RViz2 configuration file
    rviz_config_file = os.path.join(nav_pkg, 'config', 'rviz2.rviz')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    # 1. Simulation
    sim_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'teknofest_IGN.launch.py')
        )
    )
    
    # 2. EKF Localization - Start early
    ekf_localization = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'ekf_localization.launch.py')
                )
            )
        ]
    )

    # 3. AMCL Localization - Wait for EKF to stabilize
    localization = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'keepout_localization_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart')
                }.items()
            )
        ]
    )
    
    # 4. Lane Detection (YOLOPv2) - Start before navigation
    yolop_node = Node(
        package='otagg_vision',
        executable='yolopv2_node.py',
        name='yolop_lane_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'camera_topic': '/camera',
            'output_image_topic': '/lane_detection/image',
            'control_topic': '/lane_control',
            'show_visualization': False,  # Disable to reduce overhead
            'pid_kp': 0.003,
            'pid_ki': 0.0001,
            'pid_kd': 0.0015,
            'linear_speed': 1.5
        }]
    )

    yolop_launcher = TimerAction(
        period=15.0,
        actions=[yolop_node]
    )
    
    # 5. Lane to Costmap Bridge - Convert lane detection to obstacles
    lane_bridge_node = Node(
        package='navigation_2025',
        executable='lane_to_costmap_bridge.py',
        name='lane_to_costmap_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'lane_image_topic': '/lane_detection/image',
            'output_cloud_topic': '/lane_obstacles',
            'camera_frame': 'camera_link_optical',
            'lane_width': 3.5,  # Meters
            'detection_range': 15.0  # Meters
        }]
    )
    
    lane_bridge_launcher = TimerAction(
        period=16.0,
        actions=[lane_bridge_node]
    )
    
    # 6. Navigation (Nav2) - Start after everything is ready
    navigation = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'keepout_navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': LaunchConfiguration('autostart'),
                    'map_subscribe_transient_local': 'true'
                }.items()
            )
        ]
    )

    # 7. RViz2
    rviz2 = TimerAction(
        period=22.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
   
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    
    # Add nodes in sequence
    ld.add_action(sim_launcher)
    ld.add_action(ekf_localization)
    ld.add_action(localization)
    ld.add_action(yolop_launcher)
    ld.add_action(lane_bridge_launcher)
    ld.add_action(navigation)
    ld.add_action(rviz2)
    
    return ld