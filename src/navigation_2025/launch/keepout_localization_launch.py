# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('navigation_2025')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    keepout_yaml_file = LaunchConfiguration('keepout_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    keepout_params_file = LaunchConfiguration('keepout_params_file')
    
    # Separate lifecycle nodes for different managers
    localization_nodes = ['map_server', 'amcl']
    filter_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Map server parameters
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # Keepout filter parameters
    param_substitutions_keepout = {
        'use_sim_time': use_sim_time,
        'yaml_filename': keepout_yaml_file
    }

    configured_params_keepout = RewrittenYaml(
        source_file=keepout_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions_keepout,
        convert_types=True
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', 
            default_value='',
            description='Top-level namespace'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'teknofestObstacle.yaml'),
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            'keepout_map',
            default_value=os.path.join(bringup_dir, 'maps', 'keepout_teknofestObstacle.yaml'),
            description='Full path to keepout map yaml file'
        ),

        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'keepout_nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),

        DeclareLaunchArgument(
            'keepout_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'keepout_params.yaml'),
            description='Full path to the keepout parameters file'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),

        # Lifecycle Manager for Localization
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': localization_nodes},
                {'bond_timeout': 4.0},  # Increased timeout
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0}
            ]
        ),
        
        # Filter Mask Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            emulate_tty=True,
            parameters=[configured_params_keepout]
        ),

        # Costmap Filter Info Server
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            respawn=False,
            respawn_delay=2.0,
            emulate_tty=True,
            parameters=[configured_params_keepout]
        ),

        # Lifecycle Manager for Costmap Filters
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': filter_nodes},
                {'bond_timeout': 4.0},
                {'attempt_respawn_reconnection': True},
                {'bond_respawn_max_duration': 10.0}
            ]
        ),
    ])