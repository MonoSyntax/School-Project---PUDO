#!/usr/bin/env python3
"""
Configuration File Validator
============================
Validates nav2 and EKF configuration files for common issues
"""

import yaml
import sys
import os
from pathlib import Path


class ConfigValidator:
    """Validates robot configuration files"""
    
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.passes = []
        
    def validate_ekf_config(self, config_path):
        """Validate EKF configuration"""
        print(f"\n{'='*60}")
        print(f"Validating EKF Config: {config_path}")
        print(f"{'='*60}\n")
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.errors.append(f"Cannot read EKF config: {e}")
            return
        
        if 'ekf_filter_node' not in config:
            self.errors.append("Missing 'ekf_filter_node' section")
            return
        
        ekf = config['ekf_filter_node']['ros__parameters']
        
        # Check critical parameters
        self._check_param(ekf, 'frequency', 20.0, 50.0, "EKF frequency")
        self._check_param(ekf, 'two_d_mode', True, True, "2D mode")
        
        # Check frame IDs
        self._check_string(ekf, 'odom_frame', 'odom', "Odom frame")
        self._check_string(ekf, 'base_link_frame', 'base_footprint', "Base link frame")
        self._check_string(ekf, 'world_frame', 'odom', "World frame")
        
        # Check sensor configurations
        if 'odom0' in ekf:
            self._validate_odom_config(ekf, 'odom0')
        
        if 'imu0' in ekf:
            self._validate_imu_config(ekf, 'imu0')
            
        if 'odom1' in ekf:
            self._validate_gps_config(ekf, 'odom1')
    
    def validate_nav2_config(self, config_path):
        """Validate Nav2 configuration"""
        print(f"\n{'='*60}")
        print(f"Validating Nav2 Config: {config_path}")
        print(f"{'='*60}\n")
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.errors.append(f"Cannot read Nav2 config: {e}")
            return
        
        # Check controller settings
        if 'controller_server' in config:
            self._validate_controller(config['controller_server']['ros__parameters'])
        
        # Check costmap settings
        if 'local_costmap' in config:
            self._validate_local_costmap(config['local_costmap']['local_costmap']['ros__parameters'])
        
        if 'global_costmap' in config:
            self._validate_global_costmap(config['global_costmap']['global_costmap']['ros__parameters'])
        
        # Check planner settings
        if 'planner_server' in config:
            self._validate_planner(config['planner_server']['ros__parameters'])
    
    def _validate_odom_config(self, ekf, prefix):
        """Validate odometry configuration"""
        config_key = f'{prefix}_config'
        if config_key not in ekf:
            self.warnings.append(f"Missing {config_key}")
            return
        
        config = ekf[config_key]
        
        # Check that position is enabled
        if not config[0] or not config[1]:
            self.errors.append(f"{prefix}: X or Y position not enabled!")
        else:
            self.passes.append(f"{prefix}: Position enabled")
        
        # Check yaw
        if config[5]:
            self.passes.append(f"{prefix}: Yaw enabled")
        else:
            self.warnings.append(f"{prefix}: Yaw not enabled")
        
        # Check velocities
        if config[6] or config[7]:
            self.passes.append(f"{prefix}: Velocities enabled")
    
    def _validate_imu_config(self, ekf, prefix):
        """Validate IMU configuration"""
        config_key = f'{prefix}_config'
        if config_key not in ekf:
            self.warnings.append(f"Missing {config_key}")
            return
        
        config = ekf[config_key]
        
        # Check orientation (indices 3,4,5)
        if any(config[3:6]):
            self.warnings.append(f"{prefix}: IMU orientation enabled - may conflict with odom yaw!")
        else:
            self.passes.append(f"{prefix}: IMU orientation disabled (good)")
        
        # Check angular velocity (indices 9,10,11)
        if config[11]:  # yaw rate
            self.passes.append(f"{prefix}: Yaw rate enabled")
        
        # Check accelerations (indices 12,13,14)
        if any(config[12:15]):
            self.errors.append(f"{prefix}: Accelerations enabled - causes drift!")
        else:
            self.passes.append(f"{prefix}: Accelerations disabled (good)")
        
        # Check gravity removal
        if ekf.get(f'{prefix}_remove_gravitational_acceleration', False):
            self.passes.append(f"{prefix}: Gravity removal enabled")
        else:
            self.errors.append(f"{prefix}: Gravity removal NOT enabled!")
    
    def _validate_gps_config(self, ekf, prefix):
        """Validate GPS configuration"""
        config_key = f'{prefix}_config'
        if config_key not in ekf:
            return
        
        config = ekf[config_key]
        
        # Check if GPS is actually being used
        if any(config):
            self.warnings.append(f"{prefix}: GPS fusion enabled - may cause issues in simulation")
        else:
            self.passes.append(f"{prefix}: GPS disabled")
    
    def _validate_controller(self, controller):
        """Validate controller configuration"""
        # Check frequency
        freq = controller.get('controller_frequency', 0)
        if freq < 8:
            self.warnings.append(f"Controller frequency low: {freq} Hz")
        elif freq > 12:
            self.passes.append(f"Controller frequency good: {freq} Hz")
        
        # Check FollowPath settings
        if 'FollowPath' in controller:
            follow = controller['FollowPath']
            
            # Check cost scaling
            cost_gain = follow.get('cost_scaling_gain', 1.0)
            if cost_gain < 2.0:
                self.errors.append(f"cost_scaling_gain too low: {cost_gain} (should be >= 3.0)")
            else:
                self.passes.append(f"cost_scaling_gain good: {cost_gain}")
            
            cost_dist = follow.get('cost_scaling_dist', 1.0)
            if cost_dist < 1.0:
                self.warnings.append(f"cost_scaling_dist low: {cost_dist}")
            else:
                self.passes.append(f"cost_scaling_dist good: {cost_dist}")
    
    def _validate_local_costmap(self, costmap):
        """Validate local costmap configuration"""
        # Check size
        width = costmap.get('width', 0)
        height = costmap.get('height', 0)
        
        if width < 60 or height < 60:
            self.errors.append(f"Local costmap too small: {width}x{height} (should be >= 80x80)")
        else:
            self.passes.append(f"Local costmap size good: {width}x{height}")
        
        # Check footprint
        footprint = costmap.get('footprint', '')
        if '1.4' in footprint and '0.8' in footprint:
            self.passes.append("Footprint size correct")
        elif '1.1' in footprint and '0.6' in footprint:
            self.errors.append("Footprint TOO SMALL (2.2x1.2m instead of 2.8x1.6m)")
        else:
            self.warnings.append(f"Footprint unusual: {footprint}")
        
        # Check inflation
        if 'inflation_layer' in costmap:
            inflation = costmap['inflation_layer']
            cost_factor = inflation.get('cost_scaling_factor', 0)
            
            if cost_factor > 5.0:
                self.errors.append(f"cost_scaling_factor too high: {cost_factor} (should be <= 3.0)")
            elif cost_factor < 2.0:
                self.warnings.append(f"cost_scaling_factor very low: {cost_factor}")
            else:
                self.passes.append(f"cost_scaling_factor good: {cost_factor}")
    
    def _validate_global_costmap(self, costmap):
        """Validate global costmap configuration"""
        # Check footprint
        footprint = costmap.get('footprint', '')
        if '1.4' in footprint and '0.8' in footprint:
            self.passes.append("Global footprint size correct")
        elif '1.1' in footprint:
            self.errors.append("Global footprint TOO SMALL")
        
        # Check inflation
        if 'inflation_layer' in costmap:
            inflation = costmap['inflation_layer']
            cost_factor = inflation.get('cost_scaling_factor', 0)
            
            if cost_factor > 5.0:
                self.errors.append(f"Global cost_scaling_factor too high: {cost_factor}")
            else:
                self.passes.append(f"Global cost_scaling_factor OK: {cost_factor}")
    
    def _validate_planner(self, planner):
        """Validate planner configuration"""
        if 'GridBased' in planner:
            grid = planner['GridBased']
            
            # Check turning radius
            radius = grid.get('minimum_turning_radius', 0)
            if radius < 4.0:
                self.warnings.append(f"Turning radius small: {radius}m")
            else:
                self.passes.append(f"Turning radius good: {radius}m")
            
            # Check cost penalty
            penalty = grid.get('cost_penalty', 0)
            if penalty < 5.0:
                self.errors.append(f"cost_penalty too low: {penalty} (should be >= 8.0)")
            else:
                self.passes.append(f"cost_penalty good: {penalty}")
    
    def _check_param(self, config, key, min_val, max_val, description):
        """Check parameter value range"""
        if key not in config:
            self.warnings.append(f"Missing parameter: {key}")
            return
        
        val = config[key]
        if isinstance(min_val, bool):
            if val == min_val:
                self.passes.append(f"{description}: {val}")
            else:
                self.errors.append(f"{description}: {val} (expected {min_val})")
        else:
            if min_val <= val <= max_val:
                self.passes.append(f"{description}: {val}")
            else:
                self.warnings.append(f"{description}: {val} (expected {min_val}-{max_val})")
    
    def _check_string(self, config, key, expected, description):
        """Check string parameter"""
        if key not in config:
            self.warnings.append(f"Missing parameter: {key}")
            return
        
        val = config[key]
        if val == expected:
            self.passes.append(f"{description}: {val}")
        else:
            self.warnings.append(f"{description}: {val} (expected {expected})")
    
    def print_results(self):
        """Print validation results"""
        print(f"\n{'='*60}")
        print("VALIDATION RESULTS")
        print(f"{'='*60}\n")
        
        if self.passes:
            print(f"\033[92m✓ PASSED ({len(self.passes)})\033[0m")
            for p in self.passes:
                print(f"  ✓ {p}")
            print()
        
        if self.warnings:
            print(f"\033[93m⚠ WARNINGS ({len(self.warnings)})\033[0m")
            for w in self.warnings:
                print(f"  ⚠ {w}")
            print()
        
        if self.errors:
            print(f"\033[91m✗ ERRORS ({len(self.errors)})\033[0m")
            for e in self.errors:
                print(f"  ✗ {e}")
            print()
        
        # Summary
        total = len(self.passes) + len(self.warnings) + len(self.errors)
        print(f"{'='*60}")
        print(f"Total Checks: {total}")
        print(f"Passed: {len(self.passes)}")
        print(f"Warnings: {len(self.warnings)}")
        print(f"Errors: {len(self.errors)}")
        
        if self.errors:
            print(f"\n\033[91mSTATUS: FAILED\033[0m")
            return False
        elif self.warnings:
            print(f"\n\033[93mSTATUS: PASSED WITH WARNINGS\033[0m")
            return True
        else:
            print(f"\n\033[92mSTATUS: ALL CHECKS PASSED\033[0m")
            return True


def main():
    """Main function"""
    validator = ConfigValidator()
    
    # Try to auto-discover config paths if no arguments provided
    if len(sys.argv) < 2:
        try:
            from ament_index_python.packages import get_package_share_directory
            nav_pkg = get_package_share_directory('navigation_2025')
            ekf_path = os.path.join(nav_pkg, 'config', 'ekf.yaml')
            nav2_path = os.path.join(nav_pkg, 'config', 'keepout_nav2_params.yaml')
            
            print("Auto-discovered config paths:")
            print(f"  EKF: {ekf_path}")
            print(f"  Nav2: {nav2_path}")
        except Exception as e:
            print(f"Could not auto-discover config paths: {e}")
            print("Usage: ros2 run tests validate_config.py [ekf.yaml] [nav2_params.yaml]")
            sys.exit(1)
    else:
        ekf_path = sys.argv[1]
        nav2_path = sys.argv[2] if len(sys.argv) > 2 else None
    
    # Validate EKF config
    if os.path.exists(ekf_path):
        validator.validate_ekf_config(ekf_path)
    else:
        print(f"Error: {ekf_path} not found")
        sys.exit(1)
    
    # Validate Nav2 config if available
    if nav2_path and os.path.exists(nav2_path):
        validator.validate_nav2_config(nav2_path)
    elif nav2_path:
        print(f"Warning: {nav2_path} not found")
    
    # Print results
    success = validator.print_results()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()