#!/usr/bin/env python3
"""
Velocity Override Node (Simplified)

Intercepts velocity commands and applies traffic-aware control.
Handles only: Traffic Lights and Stop Signs.

Subscribes to:
    /cmd_vel_nav (geometry_msgs/Twist) - Raw velocity from Nav2
    /traffic_state (otagg_vision_interfaces/TrafficState) - Traffic state
    /odom (nav_msgs/Odometry) - Robot state

Publishes to:
    /cmd_vel (geometry_msgs/Twist) - Filtered velocity
    /traffic_control_state (std_msgs/String) - Debug state
"""

import math
from enum import IntEnum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from otagg_vision_interfaces.msg import TrafficState


class ControlState(IntEnum):
    """Traffic control states."""
    NORMAL = 0
    RED_LIGHT_DECEL = 10
    RED_LIGHT_WAIT = 11
    STOP_SIGN_DECEL = 20
    STOP_SIGN_WAIT = 21
    STOP_SIGN_RESUME = 22


class VelocityOverrideNode(Node):
    """Traffic-aware velocity override controller."""
    
    def __init__(self):
        super().__init__('velocity_override_node')
        self.get_logger().info("Velocity Override started (lights + stop signs only)")
        
        # Parameters
        self.declare_parameter('red_light_stop_distance', 5.0)
        self.declare_parameter('red_light_approach_distance', 20.0)
        self.declare_parameter('stop_sign_stop_distance', 3.0)
        self.declare_parameter('stop_sign_approach_distance', 10.0)
        self.declare_parameter('stop_sign_wait_duration', 3.0)
        
        self.red_stop_dist = self.get_parameter('red_light_stop_distance').value
        self.red_approach_dist = self.get_parameter('red_light_approach_distance').value
        self.stop_stop_dist = self.get_parameter('stop_sign_stop_distance').value
        self.stop_approach_dist = self.get_parameter('stop_sign_approach_distance').value
        self.stop_wait_dur = self.get_parameter('stop_sign_wait_duration').value
        
        # State
        self.state = ControlState.NORMAL
        self.state_entry_time: Optional[Time] = None
        self.last_nav_cmd = Twist()
        self.current_speed = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Stop sign handling
        self.stop_sign_handled = False
        self.stop_sign_pos: Optional[tuple] = None
        
        # Traffic state
        self.traffic_state: Optional[TrafficState] = None
        self.last_traffic_time: Optional[Time] = None
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_cb, 10)
        self.create_subscription(TrafficState, '/traffic_state', self.traffic_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/traffic_control_state', 10)
        
        # Timer
        self.timer = self.create_timer(0.05, self.update)
    
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        
        # Reset stop sign handled if moved away
        if self.stop_sign_handled and self.stop_sign_pos:
            dist = math.sqrt(
                (self.stop_sign_pos[0] - self.robot_x)**2 +
                (self.stop_sign_pos[1] - self.robot_y)**2
            )
            if dist > 15.0:
                self.stop_sign_handled = False
                self.stop_sign_pos = None
    
    def traffic_cb(self, msg: TrafficState):
        self.traffic_state = msg
        self.last_traffic_time = self.get_clock().now()
    
    def nav_cmd_cb(self, msg: Twist):
        self.last_nav_cmd = msg
    
    def update(self):
        now = self.get_clock().now()
        
        # Check traffic state validity (1s timeout)
        valid = (
            self.traffic_state is not None and
            self.last_traffic_time is not None and
            (now - self.last_traffic_time).nanoseconds / 1e9 < 1.0
        )
        
        # State machine
        output = Twist()
        
        if self.state == ControlState.NORMAL:
            output = self._normal(valid)
        elif self.state == ControlState.RED_LIGHT_DECEL:
            output = self._red_decel()
        elif self.state == ControlState.RED_LIGHT_WAIT:
            output = self._red_wait(valid)
        elif self.state == ControlState.STOP_SIGN_DECEL:
            output = self._stop_decel()
        elif self.state == ControlState.STOP_SIGN_WAIT:
            output = self._stop_wait()
        elif self.state == ControlState.STOP_SIGN_RESUME:
            output = self._stop_resume()
        
        self.cmd_pub.publish(output)
        
        # Debug state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
    
    def _transition(self, new_state: ControlState):
        if self.state != new_state:
            self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_entry_time = self.get_clock().now()
    
    def _time_in_state(self) -> float:
        if self.state_entry_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
    
    def _decel(self, current_dist: float, stop_dist: float) -> Twist:
        cmd = Twist()
        if current_dist <= stop_dist:
            return cmd  # Full stop
        
        range_ = current_dist - stop_dist
        max_range = self.red_approach_dist - stop_dist
        factor = min(1.0, range_ / max(0.1, max_range))
        
        cmd.linear.x = max(0.0, self.last_nav_cmd.linear.x * factor)
        cmd.angular.z = self.last_nav_cmd.angular.z * factor
        return cmd
    
    def _normal(self, valid: bool) -> Twist:
        if not valid:
            return self.last_nav_cmd
        
        ts = self.traffic_state
        
        # Check RED light
        if ts.traffic_light_state == 3:  # RED
            if 0 < ts.traffic_light_distance < self.red_approach_dist:
                self._transition(ControlState.RED_LIGHT_DECEL)
                return self._decel(ts.traffic_light_distance, self.red_stop_dist)
        
        # Check stop sign
        if ts.stop_sign_detected and not self.stop_sign_handled:
            if 0 < ts.stop_sign_distance < self.stop_approach_dist:
                self._transition(ControlState.STOP_SIGN_DECEL)
                return self._decel(ts.stop_sign_distance, self.stop_stop_dist)
        
        return self.last_nav_cmd
    
    def _red_decel(self) -> Twist:
        if self.traffic_state is None:
            return Twist()
        
        ts = self.traffic_state
        
        # Green = go
        if ts.traffic_light_state == 1:
            self._transition(ControlState.NORMAL)
            return self.last_nav_cmd
        
        # Reached stop distance
        if ts.traffic_light_distance <= self.red_stop_dist:
            self._transition(ControlState.RED_LIGHT_WAIT)
            return Twist()
        
        return self._decel(ts.traffic_light_distance, self.red_stop_dist)
    
    def _red_wait(self, valid: bool) -> Twist:
        if not valid:
            return Twist()
        
        if self.traffic_state.traffic_light_state == 1:  # GREEN
            self.get_logger().info("Green light! Resuming.")
            self._transition(ControlState.NORMAL)
            return self.last_nav_cmd
        
        return Twist()
    
    def _stop_decel(self) -> Twist:
        if self.traffic_state is None:
            return Twist()
        
        ts = self.traffic_state
        
        if ts.stop_sign_distance <= self.stop_stop_dist or self.current_speed < 0.1:
            self._transition(ControlState.STOP_SIGN_WAIT)
            return Twist()
        
        return self._decel(ts.stop_sign_distance, self.stop_stop_dist)
    
    def _stop_wait(self) -> Twist:
        wait = self._time_in_state()
        
        if wait >= self.stop_wait_dur:
            self.get_logger().info(f"Stop complete ({wait:.1f}s). Resuming.")
            self.stop_sign_handled = True
            self.stop_sign_pos = (self.robot_x, self.robot_y)
            self._transition(ControlState.STOP_SIGN_RESUME)
            return self.last_nav_cmd
        
        return Twist()
    
    def _stop_resume(self) -> Twist:
        if self._time_in_state() > 0.5:
            self._transition(ControlState.NORMAL)
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
