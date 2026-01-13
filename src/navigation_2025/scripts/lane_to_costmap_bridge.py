#!/usr/bin/env python3
"""
Lane to Costmap Bridge Node
============================

Converts YOLOPv2 lane detection to PointCloud2 obstacles for Nav2 costmaps.
This allows the vehicle to follow lanes while still being able to change lanes
or cut through dashed lines when needed.

Subscribes:
    - /lane_detection/image (sensor_msgs/Image): Lane segmentation image

Publishes:
    - /lane_obstacles (sensor_msgs/PointCloud2): Lane boundaries as obstacles
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class LaneToCostmapBridge(Node):
    """Converts lane detection to costmap obstacles"""

    def __init__(self):
        super().__init__('lane_to_costmap_bridge')

        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('lane_image_topic', '/lane_detection/image')
        self.declare_parameter('output_cloud_topic', '/lane_obstacles')
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('lane_width', 3.5)
        self.declare_parameter('detection_range', 15.0)
        self.declare_parameter('obstacle_density', 0.2)  # Points per meter
        self.declare_parameter('lane_cost_weight', 50)  # 0-100, lower = prefer staying in lane

        lane_topic = self.get_parameter('lane_image_topic').value
        output_topic = self.get_parameter('output_cloud_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.lane_width = self.get_parameter('lane_width').value
        self.detection_range = self.get_parameter('detection_range').value
        self.density = self.get_parameter('obstacle_density').value
        self.lane_cost = self.get_parameter('lane_cost_weight').value

        self.bridge = CvBridge()

        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image,
            lane_topic,
            self.image_callback,
            10
        )
        
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )

        self.get_logger().info('Lane to Costmap Bridge initialized')
        self.get_logger().info(f'Lane cost weight: {self.lane_cost}/100 (0=always follow, 100=ignore lanes)')

    def image_callback(self, msg: Image):
        """Process lane detection image and publish obstacles"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Extract red channel (lane markings from YOLOPv2)
            red_channel = cv_image[:, :, 2]
            
            # Threshold to get lane pixels
            _, lane_mask = cv2.threshold(red_channel, 200, 255, cv2.THRESH_BINARY)
            
            # Find lane boundaries
            points = self.extract_lane_boundaries(lane_mask, cv_image.shape)
            
            if len(points) > 0:
                # Convert to PointCloud2 and publish
                cloud_msg = self.create_pointcloud(points, msg.header.stamp)
                self.cloud_pub.publish(cloud_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Lane processing error: {e}')

    def extract_lane_boundaries(self, lane_mask, image_shape):
        """Extract lane boundary points from segmentation mask"""
        height, width = image_shape[:2]
        points = []
        
        # Process image from bottom (near) to top (far)
        # Skip bottom 20% (vehicle hood) and top 30% (horizon)
        start_row = int(height * 0.2)
        end_row = int(height * 0.7)
        
        # Process every few rows based on density
        row_step = max(1, int(1.0 / self.density))
        
        for y in range(start_row, end_row, row_step):
            row = lane_mask[y, :]
            
            # Find transitions (lane edges)
            diff = np.diff(row.astype(int))
            edges = np.where(np.abs(diff) > 100)[0]
            
            if len(edges) >= 2:
                # Convert pixel coordinates to world coordinates
                # Assuming camera at ~1m height, 30° down angle
                distance = self.pixel_to_distance(y, height)
                
                if distance > self.detection_range:
                    continue
                
                for edge_x in edges:
                    # Convert to lateral offset
                    lateral = self.pixel_to_lateral(edge_x, width, distance)
                    
                    # Only add points based on lane cost setting
                    # Lower cost = more points (stronger preference to stay in lane)
                    if self.should_add_point(distance):
                        points.append([distance, lateral, 0.0])
        
        return np.array(points, dtype=np.float32)

    def pixel_to_distance(self, pixel_y, image_height):
        """Convert pixel Y coordinate to forward distance"""
        # Simple inverse perspective mapping
        # Assumes camera 1m high, 30° down, 110° FOV
        normalized_y = (pixel_y - image_height * 0.2) / (image_height * 0.5)
        distance = 2.0 + normalized_y * self.detection_range
        return max(1.0, min(distance, self.detection_range))

    def pixel_to_lateral(self, pixel_x, image_width, distance):
        """Convert pixel X coordinate to lateral offset"""
        # Center of image is vehicle centerline
        center_x = image_width / 2
        normalized_x = (pixel_x - center_x) / (image_width / 2)
        
        # Lateral offset scales with distance
        max_lateral = self.lane_width * 2
        lateral = normalized_x * max_lateral * (distance / 10.0)
        return lateral

    def should_add_point(self, distance):
        """Decide whether to add point based on lane cost and distance"""
        # Near points (0-5m): Always add if cost < 80
        # Far points (5-15m): Add based on cost threshold
        
        if distance < 5.0:
            return self.lane_cost < 80
        elif distance < 10.0:
            return self.lane_cost < 60
        else:
            return self.lane_cost < 40

    def create_pointcloud(self, points, timestamp):
        """Create PointCloud2 message from numpy array"""
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.camera_frame
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(cloud_data)
        
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = LaneToCostmapBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down lane bridge node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()