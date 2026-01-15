#!/usr/bin/env python3
"""
Advanced Robot Diagnostics Tool
================================
Provides detailed analysis of robot system health with JSON output
Can be used by LLM agents for automated debugging
"""

import rclpy
from rclpy.node import Node
import json
import sys
import math
from datetime import datetime
from collections import defaultdict

# Message imports
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, JointState
from geometry_msgs.msg import Twist


class RobotDiagnostics(Node):
    """Comprehensive robot diagnostics node"""
    
    def __init__(self):
        super().__init__('robot_diagnostics')
        
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'tests': {},
            'warnings': [],
            'errors': [],
            'summary': {}
        }
        
        self.data_samples = defaultdict(list)
        self.subscription_timeout = 5.0  # seconds
        
        # Create subscriptions
        self.create_subscriptions()
        
    def create_subscriptions(self):
        """Create all diagnostic subscriptions"""
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)
        
        self.imu_raw_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_raw_callback, 10)
        
        self.imu_filtered_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_filtered_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
    def odom_callback(self, msg):
        """Process raw odometry data"""
        self.data_samples['odom'].append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'orientation_z': msg.pose.pose.orientation.z,
            'orientation_w': msg.pose.pose.orientation.w
        })
        
    def filtered_odom_callback(self, msg):
        """Process EKF filtered odometry"""
        self.data_samples['filtered_odom'].append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'orientation_z': msg.pose.pose.orientation.z,
            'orientation_w': msg.pose.pose.orientation.w
        })
        
    def imu_raw_callback(self, msg):
        """Process raw IMU data"""
        self.data_samples['imu_raw'].append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z
        })
        
    def imu_filtered_callback(self, msg):
        """Process filtered IMU data"""
        self.data_samples['imu_filtered'].append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'orientation_z': msg.orientation.z,
            'orientation_w': msg.orientation.w
        })
        
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid_ranges:
            self.data_samples['scan'].append({
                'min_range': min(valid_ranges),
                'max_range': max(valid_ranges),
                'valid_points': len(valid_ranges),
                'total_points': len(msg.ranges)
            })
        
    def gps_callback(self, msg):
        """Process GPS data"""
        self.data_samples['gps'].append({
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'status': msg.status.status
        })
        
    def joint_callback(self, msg):
        """Process joint state data"""
        if len(msg.name) > 0:
            joint_data = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    joint_data[name] = {
                        'position': msg.position[i],
                        'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0
                    }
            self.data_samples['joints'].append(joint_data)
    
    def analyze_odometry_drift(self):
        """Analyze odometry for sideways drift"""
        test_name = 'odometry_drift'
        
        if not self.data_samples['odom']:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'No odometry data received'
            }
            self.results['errors'].append('No odometry data')
            return
        
        # Get latest samples
        odom_samples = self.data_samples['odom'][-10:]
        
        # Calculate average sideways velocity
        avg_vy = sum(s['vy'] for s in odom_samples) / len(odom_samples)
        max_vy = max(abs(s['vy']) for s in odom_samples)
        
        # Check for drift
        if abs(avg_vy) < 0.05 and max_vy < 0.1:
            self.results['tests'][test_name] = {
                'status': 'PASS',
                'message': f'No sideways drift detected',
                'avg_vy': avg_vy,
                'max_vy': max_vy
            }
        elif abs(avg_vy) < 0.1:
            self.results['tests'][test_name] = {
                'status': 'WARN',
                'message': f'Minor sideways drift detected',
                'avg_vy': avg_vy,
                'max_vy': max_vy
            }
            self.results['warnings'].append(f'Minor drift: avg_vy={avg_vy:.4f}')
        else:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': f'SIGNIFICANT sideways drift detected',
                'avg_vy': avg_vy,
                'max_vy': max_vy
            }
            self.results['errors'].append(f'Sideways drift: avg_vy={avg_vy:.4f}')
    
    def analyze_ekf_fusion(self):
        """Analyze EKF fusion accuracy"""
        test_name = 'ekf_fusion'
        
        if not self.data_samples['odom'] or not self.data_samples['filtered_odom']:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'Missing odometry data'
            }
            return
        
        # Compare orientations
        odom = self.data_samples['odom'][-1]
        filtered = self.data_samples['filtered_odom'][-1]
        
        # Calculate orientation difference
        diff_z = abs(odom['orientation_z'] - filtered['orientation_z'])
        diff_w = abs(odom['orientation_w'] - filtered['orientation_w'])
        
        # Position difference
        pos_diff = math.sqrt(
            (odom['x'] - filtered['x'])**2 + 
            (odom['y'] - filtered['y'])**2
        )
        
        if diff_z < 0.1 and diff_w < 0.05:
            self.results['tests'][test_name] = {
                'status': 'PASS',
                'message': 'EKF fusion accurate',
                'orientation_diff_z': diff_z,
                'orientation_diff_w': diff_w,
                'position_diff': pos_diff
            }
        elif diff_z < 0.3:
            self.results['tests'][test_name] = {
                'status': 'WARN',
                'message': 'EKF fusion acceptable but not optimal',
                'orientation_diff_z': diff_z,
                'orientation_diff_w': diff_w,
                'position_diff': pos_diff
            }
            self.results['warnings'].append(f'EKF orientation diff: {diff_z:.3f}')
        else:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'EKF fusion inaccurate - orientation mismatch',
                'orientation_diff_z': diff_z,
                'orientation_diff_w': diff_w,
                'position_diff': pos_diff
            }
            self.results['errors'].append(f'EKF orientation mismatch: {diff_z:.3f}')
    
    def analyze_imu_data(self):
        """Analyze IMU data quality"""
        test_name = 'imu_quality'
        
        if not self.data_samples['imu_raw']:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'No IMU data received'
            }
            return
        
        # Check raw IMU
        raw_samples = self.data_samples['imu_raw'][-10:]
        
        # Check for NaN values
        has_nan = any(
            math.isnan(s['ax']) or math.isnan(s['ay']) or math.isnan(s['az'])
            for s in raw_samples
        )
        
        if has_nan:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'NaN values detected in IMU data'
            }
            self.results['errors'].append('IMU has NaN values')
            return
        
        # Check if gravity is present in raw data
        avg_az = sum(s['az'] for s in raw_samples) / len(raw_samples)
        
        # Check filtered IMU for gravity removal
        if self.data_samples['imu_filtered']:
            filtered_samples = self.data_samples['imu_filtered'][-10:]
            filtered_az = sum(s['az'] for s in filtered_samples) / len(filtered_samples)
            
            if abs(filtered_az) < 1.0:
                self.results['tests'][test_name] = {
                    'status': 'PASS',
                    'message': 'IMU data quality good, gravity removed',
                    'raw_az': avg_az,
                    'filtered_az': filtered_az
                }
            else:
                self.results['tests'][test_name] = {
                    'status': 'FAIL',
                    'message': 'Gravity NOT removed from IMU',
                    'raw_az': avg_az,
                    'filtered_az': filtered_az
                }
                self.results['errors'].append(f'Gravity not removed: az={filtered_az:.2f}')
        else:
            self.results['tests'][test_name] = {
                'status': 'WARN',
                'message': 'No filtered IMU data available',
                'raw_az': avg_az
            }
    
    def analyze_lidar_data(self):
        """Analyze LiDAR data quality"""
        test_name = 'lidar_quality'
        
        if not self.data_samples['scan']:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'No LiDAR data received'
            }
            return
        
        latest = self.data_samples['scan'][-1]
        
        valid_percent = (latest['valid_points'] / latest['total_points']) * 100
        
        if valid_percent > 80:
            self.results['tests'][test_name] = {
                'status': 'PASS',
                'message': 'LiDAR data quality good',
                'valid_points_percent': valid_percent,
                'min_range': latest['min_range'],
                'max_range': latest['max_range']
            }
        elif valid_percent > 50:
            self.results['tests'][test_name] = {
                'status': 'WARN',
                'message': 'LiDAR has some invalid points',
                'valid_points_percent': valid_percent
            }
            self.results['warnings'].append(f'LiDAR only {valid_percent:.1f}% valid')
        else:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'LiDAR data quality poor',
                'valid_points_percent': valid_percent
            }
            self.results['errors'].append(f'LiDAR only {valid_percent:.1f}% valid')
    
    def analyze_joint_states(self):
        """Analyze joint state data"""
        test_name = 'joint_states'
        
        if not self.data_samples['joints']:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'No joint state data received'
            }
            return
        
        latest = self.data_samples['joints'][-1]
        
        # Check for critical joints
        required_joints = [
            'front_left_steer_joint',
            'front_right_steer_joint',
            'rear_left_axle_joint',
            'rear_right_axle_joint'
        ]
        
        missing_joints = [j for j in required_joints if j not in latest]
        
        if not missing_joints:
            self.results['tests'][test_name] = {
                'status': 'PASS',
                'message': 'All required joints present',
                'joint_count': len(latest)
            }
        else:
            self.results['tests'][test_name] = {
                'status': 'FAIL',
                'message': 'Missing critical joints',
                'missing_joints': missing_joints
            }
            self.results['errors'].append(f'Missing joints: {missing_joints}')
    
    def run_diagnostics(self, duration=10.0):
        """Run all diagnostics for specified duration"""
        self.get_logger().info(f'Starting diagnostics for {duration} seconds...')
        
        import time
        start_time = time.time()
        
        # Collect data by spinning
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('Data collection complete. Analyzing...')
        
        # Run analyses
        self.analyze_odometry_drift()
        self.analyze_ekf_fusion()
        self.analyze_imu_data()
        self.analyze_lidar_data()
        self.analyze_joint_states()
        
        # Generate summary
        self.generate_summary()
        
        return self.results
    
    def generate_summary(self):
        """Generate test summary"""
        passed = sum(1 for t in self.results['tests'].values() if t['status'] == 'PASS')
        failed = sum(1 for t in self.results['tests'].values() if t['status'] == 'FAIL')
        warned = sum(1 for t in self.results['tests'].values() if t['status'] == 'WARN')
        total = len(self.results['tests'])
        
        self.results['summary'] = {
            'total_tests': total,
            'passed': passed,
            'failed': failed,
            'warnings': warned,
            'pass_rate': (passed / total * 100) if total > 0 else 0,
            'overall_status': 'PASS' if failed == 0 else 'FAIL'
        }


def main():
    """Main function"""
    rclpy.init()
    
    diagnostics = RobotDiagnostics()
    
    try:
        results = diagnostics.run_diagnostics(duration=10.0)
        
        # Print results
        print('\n' + '='*60)
        print('ROBOT DIAGNOSTICS RESULTS')
        print('='*60)
        print(json.dumps(results, indent=2))
        print('='*60)
        
        # Save to file
        output_file = f"diagnostics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f'\nResults saved to: {output_file}')
        
        # Exit code
        sys.exit(0 if results['summary']['overall_status'] == 'PASS' else 1)
        
    except KeyboardInterrupt:
        print('\nDiagnostics interrupted by user')
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()