#!/usr/bin/env python3
"""
Velocity Override Node - Traffic-Aware Velocity Control

This node intercepts velocity commands from Nav2 and applies traffic-aware
velocity control based on detected traffic signs and lights.

Subscribes to:
    /cmd_vel_nav (geometry_msgs/Twist) - Raw velocity from Nav2
    /traffic_state (otagg_vision_interfaces/TrafficState) - Aggregated traffic state
    /odom (nav_msgs/Odometry) - Current robot state

Publishes to:
    /cmd_vel (geometry_msgs/Twist) - Filtered velocity to robot
    /traffic_control_state (std_msgs/String) - Current control state for debugging

Author: OTAGG Team
"""

import math
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

from otagg_vision_interfaces.msg import TrafficState


class ControlState(IntEnum):
    """Traffic control state machine states."""
    NORMAL = 0
    
    # Traffic light states
    RED_LIGHT_DECELERATING = 10
    RED_LIGHT_WAITING = 11
    
    # Stop sign states
    STOP_SIGN_DECELERATING = 20
    STOP_SIGN_WAITING = 21
    STOP_SIGN_RESUMING = 22
    
    # PUDO (Pick-Up/Drop-Off) states
    PUDO_APPROACHING = 30
    PUDO_DOCKING = 31
    PUDO_WAITING = 32
    PUDO_MERGING = 33


@dataclass
class BusStop:
    """Bus stop waypoint data."""
    id: str
    dock_x: float
    dock_y: float
    dock_z: float
    dock_qx: float
    dock_qy: float
    dock_qz: float
    dock_qw: float


class VelocityOverrideNode(Node):
    """
    Traffic-aware velocity override controller.
    
    Implements a state machine that gates velocity commands based on
    detected traffic signs, lights, and PUDO requirements.
    """
    
    def __init__(self):
        super().__init__('velocity_override_node')
        self.get_logger().info("Velocity Override Node starting...")
        
        # === Parameters ===
        self.declare_parameter('red_light_stop_distance', 5.0)
        self.declare_parameter('red_light_approach_distance', 20.0)
        self.declare_parameter('stop_sign_stop_distance', 3.0)
        self.declare_parameter('stop_sign_approach_distance', 10.0)
        self.declare_parameter('stop_sign_wait_duration', 3.0)
        self.declare_parameter('pudo_approach_distance', 15.0)
        self.declare_parameter('pudo_wait_duration', 10.0)
        self.declare_parameter('decel_rate', 2.0)  # m/s^2
        self.declare_parameter('min_speed', 0.3)   # m/s minimum creep speed
        
        self.red_light_stop_dist = self.get_parameter('red_light_stop_distance').value
        self.red_light_approach_dist = self.get_parameter('red_light_approach_distance').value
        self.stop_sign_stop_dist = self.get_parameter('stop_sign_stop_distance').value
        self.stop_sign_approach_dist = self.get_parameter('stop_sign_approach_distance').value
        self.stop_sign_wait_dur = self.get_parameter('stop_sign_wait_duration').value
        self.pudo_approach_dist = self.get_parameter('pudo_approach_distance').value
        self.pudo_wait_dur = self.get_parameter('pudo_wait_duration').value
        self.decel_rate = self.get_parameter('decel_rate').value
        self.min_speed = self.get_parameter('min_speed').value
        
        # === State Machine ===
        self.state = ControlState.NORMAL
        self.state_entry_time: Optional[Time] = None
        self.last_nav_cmd = Twist()
        self.current_speed = 0.0
        
        # Handling flags to prevent re-triggering
        self.stop_sign_handled = False
        self.stop_sign_handled_position: Optional[tuple] = None
        self.bus_stop_handled = False
        self.bus_stop_handled_position: Optional[tuple] = None
        self.active_bus_stop: Optional[BusStop] = None
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Traffic state cache
        self.traffic_state: Optional[TrafficState] = None
        self.last_traffic_update: Optional[Time] = None
        
        # === Load Bus Stops ===
        self.bus_stops = self._load_bus_stops()
        
        # === Subscribers ===
        self.nav_cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10
        )
        self.traffic_sub = self.create_subscription(
            TrafficState, '/traffic_state', self.traffic_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/traffic_control_state', 10)
        self.pudo_trigger_pub = self.create_publisher(PoseStamped, '/pudo_goal', 10)
        
        # === Timer for state machine updates ===
        self.timer = self.create_timer(0.05, self.update_state_machine)  # 20 Hz
        
        self.get_logger().info("Velocity Override Node initialized.")
    
    def _load_bus_stops(self) -> list:
        """Load bus stop positions from parameters or hardcoded values."""
        # TODO: Load from yaml config. For now, hardcoded from user-provided data
        return [
            BusStop(
                id="stop_1",
                dock_x=48.365, dock_y=-55.317, dock_z=0.0,
                dock_qx=0.0, dock_qy=0.0, dock_qz=0.112, dock_qw=0.994
            ),
            BusStop(
                id="stop_2",
                dock_x=68.550, dock_y=-34.092, dock_z=0.0,
                dock_qx=0.0, dock_qy=0.0, dock_qz=0.772, dock_qw=0.635
            ),
            BusStop(
                id="stop_3",
                dock_x=31.629, dock_y=-18.040, dock_z=0.0,
                dock_qx=0.0, dock_qy=0.0, dock_qz=-0.995, dock_qw=0.096
            ),
        ]
    
    def _find_nearest_bus_stop(self) -> Optional[BusStop]:
        """Find the nearest bus stop to current position."""
        if not self.bus_stops:
            return None
        
        min_dist = float('inf')
        nearest = None
        for stop in self.bus_stops:
            dist = math.sqrt((stop.dock_x - self.robot_x)**2 + 
                           (stop.dock_y - self.robot_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = stop
        return nearest
    
    def _is_position_near(self, pos: tuple, threshold: float = 5.0) -> bool:
        """Check if robot is near a given position (for handled flag reset)."""
        if pos is None:
            return False
        dist = math.sqrt((pos[0] - self.robot_x)**2 + (pos[1] - self.robot_y)**2)
        return dist < threshold
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        
        # Reset handled flags if we've moved away from handled positions
        if self.stop_sign_handled and self.stop_sign_handled_position:
            if not self._is_position_near(self.stop_sign_handled_position, 15.0):
                self.stop_sign_handled = False
                self.stop_sign_handled_position = None
                self.get_logger().info("Stop sign handled flag reset (moved away)")
        
        if self.bus_stop_handled and self.bus_stop_handled_position:
            if not self._is_position_near(self.bus_stop_handled_position, 20.0):
                self.bus_stop_handled = False
                self.bus_stop_handled_position = None
                self.get_logger().info("Bus stop handled flag reset (moved away)")
    
    def traffic_callback(self, msg: TrafficState):
        """Update cached traffic state."""
        self.traffic_state = msg
        self.last_traffic_update = self.get_clock().now()
    
    def nav_cmd_callback(self, msg: Twist):
        """Cache the latest Nav2 velocity command."""
        self.last_nav_cmd = msg
    
    def update_state_machine(self):
        """Main state machine update loop."""
        current_time = self.get_clock().now()
        
        # Check traffic state staleness (timeout after 1 second)
        traffic_valid = (
            self.traffic_state is not None and
            self.last_traffic_update is not None and
            (current_time - self.last_traffic_update).nanoseconds / 1e9 < 1.0
        )
        
        # Calculate output velocity
        output_cmd = Twist()
        
        # State transition logic
        if self.state == ControlState.NORMAL:
            output_cmd = self._handle_normal_state(traffic_valid)
        
        elif self.state == ControlState.RED_LIGHT_DECELERATING:
            output_cmd = self._handle_red_light_decel()
        
        elif self.state == ControlState.RED_LIGHT_WAITING:
            output_cmd = self._handle_red_light_waiting(traffic_valid)
        
        elif self.state == ControlState.STOP_SIGN_DECELERATING:
            output_cmd = self._handle_stop_sign_decel()
        
        elif self.state == ControlState.STOP_SIGN_WAITING:
            output_cmd = self._handle_stop_sign_waiting()
        
        elif self.state == ControlState.STOP_SIGN_RESUMING:
            output_cmd = self._handle_stop_sign_resuming()
        
        elif self.state == ControlState.PUDO_APPROACHING:
            output_cmd = self._handle_pudo_approaching()
        
        elif self.state == ControlState.PUDO_DOCKING:
            output_cmd = self._handle_pudo_docking()
        
        elif self.state == ControlState.PUDO_WAITING:
            output_cmd = self._handle_pudo_waiting()
        
        elif self.state == ControlState.PUDO_MERGING:
            output_cmd = self._handle_pudo_merging()
        
        # Publish output
        self.cmd_pub.publish(output_cmd)
        
        # Publish state for debugging
        state_msg = String()
        state_msg.data = f"{self.state.name}"
        self.state_pub.publish(state_msg)
    
    def _transition_to(self, new_state: ControlState):
        """Transition to a new state and record entry time."""
        if self.state != new_state:
            self.get_logger().info(f"State transition: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_entry_time = self.get_clock().now()
    
    def _time_in_state(self) -> float:
        """Get time spent in current state in seconds."""
        if self.state_entry_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
    
    def _handle_normal_state(self, traffic_valid: bool) -> Twist:
        """Handle NORMAL state - check for triggers."""
        if not traffic_valid:
            return self.last_nav_cmd  # Pass through if no traffic data
        
        ts = self.traffic_state
        
        # Check RED LIGHT (highest priority)
        if ts.traffic_light_state == 3:  # RED
            if 0 < ts.traffic_light_distance < self.red_light_approach_dist:
                self._transition_to(ControlState.RED_LIGHT_DECELERATING)
                return self._apply_decel(ts.traffic_light_distance, self.red_light_stop_dist)
        
        # Check STOP SIGN
        if ts.stop_sign_detected and not self.stop_sign_handled:
            if 0 < ts.stop_sign_distance < self.stop_sign_approach_dist:
                self._transition_to(ControlState.STOP_SIGN_DECELERATING)
                return self._apply_decel(ts.stop_sign_distance, self.stop_sign_stop_dist)
        
        # Check BUS STOP (PUDO)
        if ts.bus_stop_detected and not self.bus_stop_handled:
            if 0 < ts.bus_stop_distance < self.pudo_approach_dist:
                self.active_bus_stop = self._find_nearest_bus_stop()
                self._transition_to(ControlState.PUDO_APPROACHING)
                return self._apply_decel(ts.bus_stop_distance, 3.0)  # Slow approach
        
        return self.last_nav_cmd  # Normal pass-through
    
    def _apply_decel(self, current_dist: float, target_stop_dist: float) -> Twist:
        """Apply smooth deceleration based on distance."""
        cmd = Twist()
        
        if current_dist <= target_stop_dist:
            # Full stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # Linear deceleration profile
            decel_range = current_dist - target_stop_dist
            max_decel_range = self.red_light_approach_dist - target_stop_dist
            speed_factor = min(1.0, decel_range / max(0.1, max_decel_range))
            
            cmd.linear.x = max(0.0, self.last_nav_cmd.linear.x * speed_factor)
            cmd.angular.z = self.last_nav_cmd.angular.z * speed_factor
        
        return cmd
    
    def _handle_red_light_decel(self) -> Twist:
        """Handle RED_LIGHT_DECELERATING state."""
        if self.traffic_state is None:
            return Twist()
        
        ts = self.traffic_state
        
        # Check if light turned green
        if ts.traffic_light_state == 1:  # GREEN
            self._transition_to(ControlState.NORMAL)
            return self.last_nav_cmd
        
        # Check if we should transition to waiting
        if ts.traffic_light_distance <= self.red_light_stop_dist:
            self._transition_to(ControlState.RED_LIGHT_WAITING)
            return Twist()  # Full stop
        
        return self._apply_decel(ts.traffic_light_distance, self.red_light_stop_dist)
    
    def _handle_red_light_waiting(self, traffic_valid: bool) -> Twist:
        """Handle RED_LIGHT_WAITING state."""
        if not traffic_valid:
            return Twist()  # Stay stopped if we lose traffic data
        
        ts = self.traffic_state
        
        # Wait for green light
        if ts.traffic_light_state == 1:  # GREEN
            self.get_logger().info("Green light detected! Resuming.")
            self._transition_to(ControlState.NORMAL)
            return self.last_nav_cmd
        
        return Twist()  # Stay stopped
    
    def _handle_stop_sign_decel(self) -> Twist:
        """Handle STOP_SIGN_DECELERATING state."""
        if self.traffic_state is None:
            return Twist()
        
        ts = self.traffic_state
        
        # Check if we should transition to waiting
        if ts.stop_sign_distance <= self.stop_sign_stop_dist or self.current_speed < 0.1:
            self._transition_to(ControlState.STOP_SIGN_WAITING)
            return Twist()
        
        return self._apply_decel(ts.stop_sign_distance, self.stop_sign_stop_dist)
    
    def _handle_stop_sign_waiting(self) -> Twist:
        """Handle STOP_SIGN_WAITING state - wait for required duration."""
        wait_time = self._time_in_state()
        
        if wait_time >= self.stop_sign_wait_dur:
            self.get_logger().info(f"Stop sign wait complete ({wait_time:.1f}s). Resuming.")
            self.stop_sign_handled = True
            self.stop_sign_handled_position = (self.robot_x, self.robot_y)
            self._transition_to(ControlState.STOP_SIGN_RESUMING)
            return self.last_nav_cmd
        
        # Log progress
        if int(wait_time) != int(wait_time - 0.05):  # Log once per second
            self.get_logger().info(f"Stop sign wait: {wait_time:.1f}/{self.stop_sign_wait_dur}s")
        
        return Twist()  # Stay stopped
    
    def _handle_stop_sign_resuming(self) -> Twist:
        """Handle STOP_SIGN_RESUMING state - brief transition back to normal."""
        if self._time_in_state() > 0.5:
            self._transition_to(ControlState.NORMAL)
        return self.last_nav_cmd
    
    def _handle_pudo_approaching(self) -> Twist:
        """Handle PUDO_APPROACHING state - slow down for bus stop."""
        if self.traffic_state is None or not self.traffic_state.bus_stop_detected:
            # Bus stop no longer detected, abort PUDO
            self._transition_to(ControlState.NORMAL)
            return self.last_nav_cmd
        
        ts = self.traffic_state
        
        # When close enough, trigger docking
        if ts.bus_stop_distance <= 5.0:
            self._transition_to(ControlState.PUDO_DOCKING)
            self._trigger_pudo_dock()
            return Twist()
        
        # Slow approach
        return self._apply_decel(ts.bus_stop_distance, 3.0)
    
    def _trigger_pudo_dock(self):
        """Trigger PUDO docking by publishing dock goal."""
        if self.active_bus_stop is None:
            self.get_logger().warn("No active bus stop for docking!")
            return
        
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = self.active_bus_stop.dock_x
        goal.pose.position.y = self.active_bus_stop.dock_y
        goal.pose.position.z = self.active_bus_stop.dock_z
        goal.pose.orientation.x = self.active_bus_stop.dock_qx
        goal.pose.orientation.y = self.active_bus_stop.dock_qy
        goal.pose.orientation.z = self.active_bus_stop.dock_qz
        goal.pose.orientation.w = self.active_bus_stop.dock_qw
        
        self.pudo_trigger_pub.publish(goal)
        self.get_logger().info(f"Published PUDO dock goal: {self.active_bus_stop.id}")
    
    def _handle_pudo_docking(self) -> Twist:
        """Handle PUDO_DOCKING state - let Nav2 navigate to dock."""
        # Check if we've reached the dock position
        if self.active_bus_stop is not None:
            dist_to_dock = math.sqrt(
                (self.active_bus_stop.dock_x - self.robot_x)**2 +
                (self.active_bus_stop.dock_y - self.robot_y)**2
            )
            if dist_to_dock < 1.0 and self.current_speed < 0.1:
                self.get_logger().info("Docked at bus stop. Starting passenger wait.")
                self._transition_to(ControlState.PUDO_WAITING)
                return Twist()
        
        # Allow Nav2 to control docking
        return self.last_nav_cmd
    
    def _handle_pudo_waiting(self) -> Twist:
        """Handle PUDO_WAITING state - passenger boarding time."""
        wait_time = self._time_in_state()
        
        if wait_time >= self.pudo_wait_dur:
            self.get_logger().info(f"PUDO wait complete ({wait_time:.1f}s). Merging to road.")
            self.bus_stop_handled = True
            self.bus_stop_handled_position = (self.robot_x, self.robot_y)
            self._transition_to(ControlState.PUDO_MERGING)
            return self.last_nav_cmd
        
        # Log progress
        if int(wait_time) != int(wait_time - 0.05):
            self.get_logger().info(f"PUDO wait: {wait_time:.1f}/{self.pudo_wait_dur}s")
        
        return Twist()  # Stay stopped for passengers
    
    def _handle_pudo_merging(self) -> Twist:
        """Handle PUDO_MERGING state - return to normal driving."""
        # Simple transition back after brief delay
        if self._time_in_state() > 1.0:
            self.active_bus_stop = None
            self._transition_to(ControlState.NORMAL)
        
        return self.last_nav_cmd


def main(args=None):
    rclpy.init(args=args)
    node = VelocityOverrideNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
