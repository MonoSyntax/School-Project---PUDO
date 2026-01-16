#!/usr/bin/env python3
"""
Traffic State Manager Node

Aggregates raw traffic sign detections into reliable navigation state.
Implements temporal filtering and state machine logic for traffic lights.

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
    """Traffic light state enum matching message constants."""
    UNKNOWN = 0
    GREEN = 1
    YELLOW = 2
    RED = 3


class SignTracker:
    """Tracks detections of a specific sign class with temporal filtering."""
    
    def __init__(self, min_detections: int = 3, time_window_sec: float = 2.0):
        self.min_detections = min_detections
        self.time_window_sec = time_window_sec
        self.detections = []  # List of (timestamp, distance) tuples
    
    def add_detection(self, timestamp: Time, distance: float):
        """Add a detection with timestamp."""
        self.detections.append((timestamp, distance))
    
    def prune_old(self, current_time: Time):
        """Remove detections older than time window."""
        cutoff = current_time.nanoseconds - int(self.time_window_sec * 1e9)
        self.detections = [(t, d) for t, d in self.detections if t.nanoseconds > cutoff]
    
    def is_confirmed(self) -> bool:
        """Check if enough recent detections to confirm presence."""
        return len(self.detections) >= self.min_detections
    
    def get_average_distance(self) -> float:
        """Get average distance from recent detections."""
        if not self.detections:
            return -1.0
        distances = [d for _, d in self.detections if d > 0]
        return sum(distances) / len(distances) if distances else -1.0


class TrafficStateManagerNode(Node):
    """Aggregates detections into navigation-ready traffic state."""
    
    def __init__(self):
        super().__init__('traffic_state_manager_node')
        self.get_logger().info("Traffic State Manager Node started.")
        
        # Parameters
        self.declare_parameter('light_min_detections', 3)
        self.declare_parameter('sign_min_detections', 2)
        self.declare_parameter('time_window_sec', 2.0)
        self.declare_parameter('no_detection_timeout_sec', 3.0)
        
        self.light_min_det = self.get_parameter('light_min_detections').value
        self.sign_min_det = self.get_parameter('sign_min_detections').value
        self.time_window = self.get_parameter('time_window_sec').value
        self.timeout = self.get_parameter('no_detection_timeout_sec').value
        
        # Sign trackers - keyed by sign_class
        self.trackers = defaultdict(lambda: SignTracker(
            min_detections=self.sign_min_det,
            time_window_sec=self.time_window
        ))
        
        # Traffic light state machine
        self.light_state = LightState.UNKNOWN
        self.light_distance = -1.0
        self.last_light_detection_time = None
        
        # Subscription
        self.detection_sub = self.create_subscription(
            TrafficSignArray,
            '/traffic_signs',
            self.detection_callback,
            10
        )
        
        # Publisher
        self.state_pub = self.create_publisher(
            TrafficState,
            '/traffic_state',
            10
        )
        
        # Timer for periodic state publishing
        self.timer = self.create_timer(0.1, self.publish_state)
    
    def detection_callback(self, msg: TrafficSignArray):
        """Process incoming detections."""
        current_time = Time.from_msg(msg.header.stamp)
        
        for det in msg.detections:
            sign_class = det.sign_class
            distance = det.estimated_distance
            
            # Update tracker
            self.trackers[sign_class].add_detection(current_time, distance)
        
        # Prune old detections from all trackers
        for tracker in self.trackers.values():
            tracker.prune_old(current_time)
        
        # Update traffic light state machine
        self.update_light_state(current_time)
    
    def update_light_state(self, current_time: Time):
        """Update traffic light state machine based on detections."""
        # Turkish class names from Turkey Road Sign dataset
        red_tracker = self.trackers.get('kirmizi')    # red
        yellow_tracker = self.trackers.get('sari')    # yellow
        green_tracker = self.trackers.get('yesil')    # green
        
        # Check for confirmed light detections (prioritize by safety)
        red_confirmed = red_tracker and red_tracker.is_confirmed()
        yellow_confirmed = yellow_tracker and yellow_tracker.is_confirmed()
        green_confirmed = green_tracker and green_tracker.is_confirmed()
        
        prev_state = self.light_state
        
        # State transitions (prioritize RED for safety)
        if red_confirmed:
            self.light_state = LightState.RED
            self.light_distance = red_tracker.get_average_distance()
            self.last_light_detection_time = current_time
        elif yellow_confirmed:
            self.light_state = LightState.YELLOW
            self.light_distance = yellow_tracker.get_average_distance()
            self.last_light_detection_time = current_time
        elif green_confirmed:
            self.light_state = LightState.GREEN
            self.light_distance = green_tracker.get_average_distance()
            self.last_light_detection_time = current_time
        else:
            # Check timeout
            if self.last_light_detection_time:
                elapsed = (current_time.nanoseconds - self.last_light_detection_time.nanoseconds) / 1e9
                if elapsed > self.timeout:
                    self.light_state = LightState.UNKNOWN
                    self.light_distance = -1.0
        
        # Log state changes
        if self.light_state != prev_state:
            self.get_logger().info(f"Traffic light state: {prev_state.name} -> {self.light_state.name}")
    
    def publish_state(self):
        """Publish aggregated traffic state."""
        msg = TrafficState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        
        # Traffic light
        msg.traffic_light_state = int(self.light_state)
        msg.traffic_light_distance = self.light_distance
        
        # Stop sign (Turkish: dur)
        stop_tracker = self.trackers.get('dur')
        msg.stop_sign_detected = stop_tracker.is_confirmed() if stop_tracker else False
        msg.stop_sign_distance = stop_tracker.get_average_distance() if stop_tracker else -1.0
        
        # Turn restrictions (Turkish names)
        no_left = self.trackers.get('soladonulmez')    # no left turn
        no_right = self.trackers.get('sagadonulmez')   # no right turn
        no_entry = self.trackers.get('girisyok')       # no entry
        
        msg.no_left_turn = no_left.is_confirmed() if no_left else False
        msg.no_right_turn = no_right.is_confirmed() if no_right else False
        msg.no_entry = no_entry.is_confirmed() if no_entry else False
        msg.road_closed = False  # Not available in Turkey Road Sign dataset
        
        # Bus stop (Turkish: durak)
        bus_tracker = self.trackers.get('durak')
        msg.bus_stop_detected = bus_tracker.is_confirmed() if bus_tracker else False
        msg.bus_stop_distance = bus_tracker.get_average_distance() if bus_tracker else -1.0
        
        # Speed limit (Turkish: '20' and '30')
        speed_20 = self.trackers.get('20')
        speed_30 = self.trackers.get('30')
        
        if speed_20 and speed_20.is_confirmed():
            msg.speed_limit_kmh = 20
            self.get_logger().warn("Speed limit detected: 20 km/h", throttle_duration_sec=5.0)
        elif speed_30 and speed_30.is_confirmed():
            msg.speed_limit_kmh = 30
            self.get_logger().warn("Speed limit detected: 30 km/h", throttle_duration_sec=5.0)
        else:
            msg.speed_limit_kmh = 0
        
        # Compute nav_override_state based on priority
        # Priority: RED_LIGHT > STOP_SIGN > BUS_STOP
        # Constants: NONE=0, STOP=1, SLOW=2, PUDO=3
        NAV_NONE = 0
        NAV_STOP = 1
        NAV_SLOW = 2
        NAV_PUDO = 3
        
        override_state = NAV_NONE
        
        # Check red/yellow light (highest priority)
        if self.light_state == LightState.RED:
            if msg.traffic_light_distance > 0 and msg.traffic_light_distance < 5.0:
                override_state = NAV_STOP
            elif msg.traffic_light_distance > 0 and msg.traffic_light_distance < 20.0:
                override_state = NAV_SLOW
        elif self.light_state == LightState.YELLOW:
            if msg.traffic_light_distance > 0 and msg.traffic_light_distance < 15.0:
                override_state = NAV_SLOW
        
        # Check stop sign (if no light override)
        if override_state == NAV_NONE and msg.stop_sign_detected:
            if msg.stop_sign_distance > 0 and msg.stop_sign_distance < 3.0:
                override_state = NAV_STOP
            elif msg.stop_sign_distance > 0 and msg.stop_sign_distance < 10.0:
                override_state = NAV_SLOW
        
        # Check bus stop for PUDO (lowest priority)
        if override_state == NAV_NONE and msg.bus_stop_detected:
            if msg.bus_stop_distance > 0 and msg.bus_stop_distance < 15.0:
                override_state = NAV_PUDO
        
        msg.nav_override_state = override_state
        
        # Handled flags - these are managed by velocity_override_node
        # We set them to False here, velocity override will track separately
        msg.stop_sign_handled = False
        msg.bus_stop_handled = False
        
        self.state_pub.publish(msg)


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
