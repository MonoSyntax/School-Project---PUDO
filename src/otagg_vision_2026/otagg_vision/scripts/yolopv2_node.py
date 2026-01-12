#!/usr/bin/env python3
"""
YOLOPv2 Lane Detection to Costmap Node
Projects detected lane markings onto the 3D ground plane as obstacles for Nav2.
"""

from ament_index_python.packages import get_package_share_directory

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

# Force X11 backend for OpenCV on Wayland
os.environ['QT_QPA_PLATFORM'] = 'xcb'

class YOLOPv2LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('yolopv2_lane_detection')

        # Camera Parameters (Derived from camera.xacro)
        # Resolution: 672x376, FOV: 1.91986 rad
        self.img_w = 672
        self.img_h = 376
        self.fx = 235.3
        self.fy = 235.3
        self.cx = 336.0
        self.cy = 188.0
        
        # Extrinsics (Base to Camera)
        self.cam_height = 0.8  # Meters
        
        # ROS Interfaces
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        self.cloud_pub = self.create_publisher(PointCloud2, '/lane_obstacles', 10)
        self.image_pub = self.create_publisher(Image, '/lane_detection/image', 10)
        
        try:
            pkg_share = get_package_share_directory('otagg_vision')
            model_path = os.path.join(pkg_share, 'models', 'yolopv2.pt')
        except Exception as e:
            self.get_logger().error(f'Failed to get model path: {e}')
            raise e
        
        self.bridge = CvBridge()
        
        # Initialize Inference Model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Loading YOLOPv2 on {self.device}...')
        try:
            self.model = torch.jit.load(model_path)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info(f'YOLOPv2 loaded successfully on {self.device}')
        except Exception as e:
            self.get_logger().error(f'Model load failed: {e}')
            raise e

    def image_callback(self, msg):
        """
        Callback for camera images. Runs inference and publishes 3D obstacles.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Preprocessing: Resize to model input size (640x640)
            input_img = cv2.resize(cv_image, (640, 640))
            img_tensor = input_img.astype(np.float32) / 255.0
            img_tensor = np.transpose(img_tensor, (2, 0, 1))
            img_tensor = np.expand_dims(img_tensor, axis=0)
            img_tensor = torch.from_numpy(img_tensor).to(self.device)

            # Inference
            with torch.no_grad():
                _, _, ll_seg_out = self.model(img_tensor)

            # Post-processing: Extract lane mask
            ll_seg_mask = ll_seg_out[:, 0, :, :].cpu().numpy()[0]
            ll_seg_mask = (ll_seg_mask > 0.5).astype(np.uint8) * 255
            
            # Resize mask to original resolution for correct projection
            ll_seg_mask = cv2.resize(ll_seg_mask, (self.img_w, self.img_h))

            # Project to 3D and publish
            points = self.project_to_3d(ll_seg_mask)
            if points:
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = "camera_link_optical"
                
                pc2 = point_cloud2.create_cloud_xyz32(header, points)
                self.cloud_pub.publish(pc2)

            # Publish debug visualization
            color_mask = np.zeros_like(cv_image)
            color_mask[ll_seg_mask > 0] = [0, 0, 255]
            debug_img = cv2.addWeighted(cv_image, 1, color_mask, 0.5, 0)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

    def project_to_3d(self, mask):
        """
        Projects 2D mask pixels to 3D points on the ground plane.
        """
        points = []
        stride = 10  # Downsample for performance
        
        y_idxs, x_idxs = np.where(mask > 0)
        if len(y_idxs) == 0:
            return []

        for i in range(0, len(y_idxs), stride):
            u, v = x_idxs[i], y_idxs[i]
            
            # Pinhole projection
            norm_x = (u - self.cx) / self.fx
            norm_y = (v - self.cy) / self.fy
            
            # Filter horizon
            if norm_y <= 0.1:
                continue

            # Project to ground (Y_optical = cam_height)
            Z_depth = self.cam_height / norm_y
            X_lat = norm_x * Z_depth
            
            # Valid depth range check
            if 0.1 < Z_depth < 15.0:
                # Optical Frame: X=Right, Y=Down, Z=Forward
                points.append([X_lat, self.cam_height, Z_depth])

        return points

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()