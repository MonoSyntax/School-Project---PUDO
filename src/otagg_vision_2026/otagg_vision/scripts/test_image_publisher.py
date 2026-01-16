#!/usr/bin/env python3
"""
Test Image Publisher for Traffic Sign Detection

This script publishes images from the validation dataset to /camera_compressed
to test the detection node without setting up the full simulation.
"""

import os
import sys
import glob
import random
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        
        self.publisher_ = self.create_publisher(CompressedImage, '/camera_compressed', 10)
        self.bridge = CvBridge()
        
        # Dataset path
        dataset_path = "/home/berke/ros2_ws/Sign dataset/Turkey Road Sign.v1i.yolov12/valid/images"
        self.image_files = glob.glob(os.path.join(dataset_path, "*.jpg"))
        
        if not self.image_files:
            self.get_logger().error(f"No images found in {dataset_path}")
            sys.exit(1)
            
        self.get_logger().info(f"Found {len(self.image_files)} test images")
        self.get_logger().info("Publishing random images to /camera_compressed every 2 seconds...")
        
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        # Pick random image
        img_path = random.choice(self.image_files)
        cv_img = cv2.imread(img_path)
        
        if cv_img is None:
            self.get_logger().warn(f"Failed to load {img_path}")
            return
            
        # Create message
        msg = self.bridge.cv2_to_compressed_imgmsg(cv_img)
        
        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {os.path.basename(img_path)}")

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
