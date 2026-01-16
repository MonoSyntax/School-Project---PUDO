#!/usr/bin/env python3
"""
Bus Stop Loop Node - Continuously navigates through bus stops

Sends navigation goals to loop through predefined bus stops.
The velocity_override_node handles PUDO wait times at each stop.

Publishes to: /goal_pose (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String


class BusStop:
    """Bus stop data."""
    def __init__(self, id: str, x: float, y: float, z: float, 
                 qx: float, qy: float, qz: float, qw: float):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw


# Bus stops from user-provided coordinates
BUS_STOPS = [
    BusStop("stop_1", 48.365, -55.317, 0.0, 0.0, 0.0, 0.112, 0.994),
    BusStop("stop_2", 68.550, -34.092, 0.0, 0.0, 0.0, 0.772, 0.635),
    BusStop("stop_3", 31.629, -18.040, 0.0, 0.0, 0.0, -0.995, 0.096),
]


class BusStopLoopNode(Node):
    """Loops through bus stops continuously."""
    
    def __init__(self):
        super().__init__('bus_stop_loop_node')
        self.get_logger().info("Bus Stop Loop Node started.")
        
        # Parameters
        self.declare_parameter('loop_delay_sec', 2.0)
        self.loop_delay = self.get_parameter('loop_delay_sec').value
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Current stop index
        self.current_stop_idx = 0
        self.goal_in_progress = False
        
        # Subscribe to traffic control state to know when PUDO is done
        self.control_state_sub = self.create_subscription(
            String, '/traffic_control_state', self.control_state_callback, 10
        )
        self.current_control_state = "NORMAL"
        
        # Wait for action server
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Action server connected!")
        
        # Start the loop after a delay
        self.timer = self.create_timer(3.0, self.send_next_goal)
    
    def control_state_callback(self, msg: String):
        """Track traffic control state."""
        self.current_control_state = msg.data
    
    def send_next_goal(self):
        """Send navigation goal to next bus stop."""
        if self.goal_in_progress:
            return
        
        stop = BUS_STOPS[self.current_stop_idx]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = stop.x
        goal_msg.pose.pose.position.y = stop.y
        goal_msg.pose.pose.position.z = stop.z
        goal_msg.pose.pose.orientation.x = stop.qx
        goal_msg.pose.pose.orientation.y = stop.qy
        goal_msg.pose.pose.orientation.z = stop.qz
        goal_msg.pose.pose.orientation.w = stop.qw
        
        self.get_logger().info(f"Navigating to {stop.id} ({stop.x:.1f}, {stop.y:.1f})")
        
        self.goal_in_progress = True
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            self.goal_in_progress = False
            return
        
        self.get_logger().info("Goal accepted. Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            stop = BUS_STOPS[self.current_stop_idx]
            self.get_logger().info(f"Reached {stop.id}! PUDO will handle waiting.")
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")
        
        # Move to next stop (loop)
        self.current_stop_idx = (self.current_stop_idx + 1) % len(BUS_STOPS)
        self.goal_in_progress = False
        
        # Small delay before next goal
        self.get_logger().info(f"Next stop: {BUS_STOPS[self.current_stop_idx].id} (in {self.loop_delay}s)")


def main(args=None):
    rclpy.init(args=args)
    node = BusStopLoopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
