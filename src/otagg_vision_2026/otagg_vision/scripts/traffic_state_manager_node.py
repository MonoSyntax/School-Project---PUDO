#!/usr/bin/env python3
"""
Traffic State Manager Node (Simplified)

Aggregates raw traffic sign detections into reliable navigation state.
Handles only: Traffic Lights and Stop Signs.

Subscribes to: /traffic_signs (otagg_vision_interfaces/TrafficSignArray)
Publishes to: /traffic_state (otagg_vision_interfaces/TrafficState)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from collections import defaultdict
from enum import IntEnum

from otagg_vision_interfaces.msg import TrafficSignArray, TrafficState


class LightState(IntEnum):
    """Traffic light state enum."""
    UNKNOWN = 0
    GREEN = 1
    YELLOW = 2
    RED = 3


class SignTracker:
    """Tracks detections with temporal filtering."""
    
    def __init__(self, min_detections: int = 3, time_window_sec: float = 2.0):
        self.min_detections = min_detections
        self.time_window_sec = time_window_sec
        self.detections = []
    
    def add_detection(self, timestamp: Time, distance: float):
        self.detections.append((timestamp, distance))
    
    def prune_old(self, current_time: Time):
        cutoff = current_time.nanoseconds - int(self.time_window_sec * 1e9)
        self.detections = [(t, d) for t, d in self.detections if t.nanoseconds > cutoff]
    
    def is_confirmed(self) -> bool:
        return len(self.detections) >= self.min_detections
    
    def get_average_distance(self) -> float:
        if not self.detections:
            return -1.0
        distances = [d for _, d in self.detections if d > 0]
        return sum(distances) / len(distances) if distances else -1.0


class TrafficStateManagerNode(Node):
    """Aggregates detections into navigation-ready traffic state."""
    
    def __init__(self):
        super().__init__('traffic_state_manager_node')
        self.get_logger().info("Traffic State Manager started (lights + stop signs only)")
        
        # Parameters
        self.declare_parameter('min_detections', 3)
        self.declare_parameter('time_window_sec', 2.0)
        self.declare_parameter('timeout_sec', 3.0)
        
        min_det = self.get_parameter('min_detections').value
        time_window = self.get_parameter('time_window_sec').value
        self.timeout = self.get_parameter('timeout_sec').value
        
        # Trackers
        self.trackers = defaultdict(lambda: SignTracker(min_det, time_window))
        
        # Traffic light state
        self.light_state = LightState.UNKNOWN
        self.light_distance = -1.0
        self.last_light_time = None
        
        # Subscription
        self.sub = self.create_subscription(
            TrafficSignArray, '/traffic_signs', self.detection_callback, 10
        )
        
        # Publisher
        self.pub = self.create_publisher(TrafficState, '/traffic_state', 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.publish_state)
    
    def detection_callback(self, msg: TrafficSignArray):
        current_time = Time.from_msg(msg.header.stamp)
        
        for det in msg.detections:
            self.trackers[det.sign_class].add_detection(current_time, det.estimated_distance)
            if det.sign_class in ['red', 'yellow', 'green', 'stop_sign']:
                 self.get_logger().debug(f"Added detection: {det.sign_class} at {det.estimated_distance:.2f}m")

        for tracker in self.trackers.values():
            tracker.prune_old(current_time)
        
        self.update_light_state(current_time)
    
    def update_light_state(self, current_time: Time):
        # English class names from new model
        red = self.trackers.get('red')
        yellow = self.trackers.get('yellow')
        green = self.trackers.get('green')
        
        red_ok = red and red.is_confirmed()
        yellow_ok = yellow and yellow.is_confirmed()
        green_ok = green and green.is_confirmed()
        
        prev = self.light_state
        
        if red_ok:
            self.light_state = LightState.RED
            self.light_distance = red.get_average_distance()
            self.last_light_time = current_time
        elif yellow_ok:
            self.light_state = LightState.YELLOW
            self.light_distance = yellow.get_average_distance()
            self.last_light_time = current_time
        elif green_ok:
            self.light_state = LightState.GREEN
            self.light_distance = green.get_average_distance()
            self.last_light_time = current_time
        elif self.last_light_time:
            elapsed = (current_time.nanoseconds - self.last_light_time.nanoseconds) / 1e9
            if elapsed > self.timeout:
                self.light_state = LightState.UNKNOWN
                self.light_distance = -1.0
        
        if self.light_state != prev:
            self.get_logger().info(f"Light State Changed: {prev.name} -> {self.light_state.name} (Dist: {self.light_distance:.2f}m)")
    
    def publish_state(self):
        msg = TrafficState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        
        # Traffic light
        msg.traffic_light_state = int(self.light_state)
        msg.traffic_light_distance = self.light_distance
        
        # Stop sign (no stop sign in current model, keeping for future)
        stop = self.trackers.get('stop_sign')
        msg.stop_sign_detected = stop.is_confirmed() if stop else False
        msg.stop_sign_distance = stop.get_average_distance() if stop else -1.0
        
        # Nav override
        if self.light_state == LightState.RED and 0 < msg.traffic_light_distance < 20.0:
            msg.nav_override_state = 1  # STOP
        elif msg.stop_sign_detected and 0 < msg.stop_sign_distance < 10.0:
            msg.nav_override_state = 1  # STOP
        else:
            msg.nav_override_state = 0  # NONE
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficStateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
