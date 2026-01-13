#!/usr/bin/env python3
"""
YOLOPv2 Lane Detection ROS2 Node (Optimized)
=============================================

Optimized for lane following with better performance and accuracy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

os.environ['QT_QPA_PLATFORM'] = 'xcb'

def select_device(device=''):
    device = str(device).strip().lower().replace('cuda:', '').replace('none', '')
    cpu = device == 'cpu'
    if cpu:
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
    elif device:
        os.environ['CUDA_VISIBLE_DEVICES'] = device
        assert torch.cuda.is_available(), f'CUDA unavailable, invalid device {device} requested'

    cuda = not cpu and torch.cuda.is_available()
    return torch.device('cuda:0' if cuda else 'cpu')

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

def lane_line_mask(ll):
    if ll.ndim == 3:
        ll = ll.unsqueeze(0)
    
    ll_seg_mask = torch.argmax(ll, dim=1)
    ll_seg_mask = ll_seg_mask.squeeze().detach().cpu().numpy()
    
    return ll_seg_mask

class YOLOPv2LaneDetectionNode(Node):
    """Optimized YOLOPv2 lane detection node"""

    def __init__(self):
        super().__init__('yolopv2_lane_detection_node')

        # Parameters
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('output_image_topic', '/lane_detection/image')
        self.declare_parameter('control_topic', '/lane_control')
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('show_visualization', False)
        self.declare_parameter('output_width', 1280)
        self.declare_parameter('output_height', 720)
        self.declare_parameter('process_every_n_frames', 2)  # Skip frames for performance

        camera_topic = self.get_parameter('camera_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        control_topic = self.get_parameter('control_topic').value
        model_path = self.get_parameter('model_path').value
        self.use_half = self.get_parameter('use_half_precision').value
        self.show_viz = self.get_parameter('show_visualization').value
        self.out_width = self.get_parameter('output_width').value
        self.out_height = self.get_parameter('output_height').value
        self.frame_skip = self.get_parameter('process_every_n_frames').value

        if not model_path:
            try:
                pkg_share = get_package_share_directory('otagg_vision')
                model_path = os.path.join(pkg_share, 'models', 'yolopv2.pt')
            except Exception:
                model_path = 'yolopv2.pt'

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_lane_mask = None

        self.device = select_device('0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Loading model: {model_path}')

        try:
            self.lane_model = torch.jit.load(model_path).to(self.device)
            self.lane_model.eval()
            if self.use_half:
                self.lane_model.half()
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Model loading error: {e}')
            raise

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.control_pub = self.create_publisher(Twist, control_topic, 10)

        if self.show_viz:
            cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Lane Detection", self.out_width, self.out_height)

        self.get_logger().info('YOLOPv2 Lane Detection Node started!')

    def image_callback(self, msg: Image):
        """Process incoming camera frame"""
        self.frame_count += 1
        
        # Frame skipping for performance
        if self.frame_count % self.frame_skip != 0:
            # Still publish last result if available
            if self.last_lane_mask is not None:
                try:
                    output_msg = self.bridge.cv2_to_imgmsg(self.last_lane_mask, encoding='bgr8')
                    output_msg.header = msg.header
                    self.image_pub.publish(output_msg)
                except Exception:
                    pass
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        frame = cv2.resize(frame, (self.out_width, self.out_height))
        
        try:
            # Prepare image for model
            img = letterbox(frame, 640, stride=32)[0]

            if img.shape[2] == 4:
                img = img[:, :, :3]

            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.use_half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Run inference
            with torch.no_grad():
                _, _, ll = self.lane_model(img)
            
            ll_seg_mask = lane_line_mask(ll)
            ll = ll.squeeze().detach().cpu().numpy()

            if ll.ndim == 3:
                ll = np.argmax(ll, axis=0)
            
            ll = ll.astype(np.uint8)

            # Resize to output dimensions
            if ll.shape != frame.shape[:2]:
                ll = cv2.resize(ll, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            
            # Overlay lane detection on frame
            foreground = ll == 1
            output_frame = frame.copy()
            
            if frame.shape[2] == 4:
                output_frame = output_frame[:, :, :3]
            
            # Red overlay for lanes with transparency
            overlay = output_frame.copy()
            overlay[foreground] = [0, 0, 255]
            output_frame = cv2.addWeighted(output_frame, 0.7, overlay, 0.3, 0)
            
            # Draw lane center line
            try:
                contours, _ = cv2.findContours(
                    (ll_seg_mask > 0.5).astype(np.uint8), 
                    cv2.RETR_EXTERNAL, 
                    cv2.CHAIN_APPROX_SIMPLE
                )
                
                if len(contours) >= 2:
                    # Find two largest contours (left and right lanes)
                    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
                    
                    # Get bottom center points of each lane
                    lane_points = []
                    for contour in contours:
                        if cv2.contourArea(contour) < 200:
                            continue
                        points = contour[:, 0, :]
                        bottom_points = points[points[:, 1] > self.out_height * 0.6]
                        if len(bottom_points) > 0:
                            center_x = int(np.mean(bottom_points[:, 0]))
                            center_y = int(np.mean(bottom_points[:, 1]))
                            lane_points.append([center_x, center_y])
                    
                    # Draw lane center
                    if len(lane_points) >= 2:
                        center_x = int((lane_points[0][0] + lane_points[1][0]) / 2)
                        center_y = int((lane_points[0][1] + lane_points[1][1]) / 2)
                        cv2.circle(output_frame, (center_x, center_y), 10, (0, 255, 0), -1)
                        
                        # Draw vehicle center reference
                        vehicle_center = self.out_width // 2
                        cv2.line(output_frame, (vehicle_center, self.out_height), 
                                (vehicle_center, int(self.out_height * 0.6)), (255, 255, 0), 3)
                        
                        # Draw offset text
                        offset = center_x - vehicle_center
                        cv2.putText(output_frame, f'Offset: {offset}px', (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            except Exception as e:
                self.get_logger().debug(f'Contour processing: {e}')

            self.last_lane_mask = output_frame

        except Exception as e:
            self.get_logger().warn(f'Lane detection error: {e}')
            output_frame = frame

        # Publish processed image
        try:
            output_msg = self.bridge.cv2_to_imgmsg(output_frame, encoding='bgr8')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'Image publishing error: {e}')

        # Visualization
        if self.show_viz:
            cv2.imshow("Lane Detection", output_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('User requested shutdown.')
                rclpy.shutdown()

    def destroy_node(self):
        if self.show_viz:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2LaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()