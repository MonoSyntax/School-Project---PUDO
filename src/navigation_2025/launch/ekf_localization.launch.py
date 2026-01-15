#!/usr/bin/env python3
"""
EKF Localization Launch File
============================

Launches:
1. Madgwick IMU Filter - Fuses raw IMU + magnetometer
2. Robot Localization EKF - Fuses wheel odom + filtered IMU + GPS
3. Navsat Transform - Converts GPS to odometry frame
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for EKF localization stack"""
    
    # Get package directory
    pkg_share = get_package_share_directory('navigation_2025')
    
    # Configuration file path (unified config)
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # 1. Madgwick IMU Filter Node
    # Fuses raw IMU accelerometer + gyroscope + magnetometer
    # Outputs: filtered orientation to /imu/data
    madgwick_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[
            # Inputs from Gazebo simulation
            ('imu/data_raw', '/imu/data_raw'),  # Raw IMU (accel + gyro)
            ('imu/mag', '/imu/mag'),            # Magnetometer data
            # Output: filtered IMU with orientation
            ('imu/data', '/imu/data'),          # Filtered IMU → used by EKF
        ],
    )
    
    # 2. EKF Filter Node
    # Fuses: wheel odometry + filtered IMU + GPS odometry
    # Outputs: /odometry/filtered transform (odom → base_footprint)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        # Topics are specified directly in ekf.yaml (odom0, imu0, etc.)
    )
    
    # 3. Navsat Transform Node
    # Converts GPS (lat/lon) to odometry frame (x/y)
    # Outputs: /odometry/gps (used by EKF as odom1)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[
            ('imu', '/imu/data'),        # Use filtered IMU orientation
            ('gps/fix', '/gps/fix'),     # GPS input from Gazebo
            ('odometry/gps', 'odometry/gps'),  # GPS odometry output
            ('odometry/filtered', '/odometry/filtered'),  # EKF output
        ],
    )
    
    return LaunchDescription([
        madgwick_filter,
        robot_localization_node,
        navsat_transform_node,
    ])