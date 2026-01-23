#!/usr/bin/env python3
"""
YOLOPv2 Lane Detection ROS2 Node (Expanded Limits)
==================================================
- Slider limits expanded (-2.0 to 2.0) for aggressive tuning.
- Precision increased (0.001 steps).
- No auto-zoom logic; purely manual geometric control.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import torch
import numpy as np
import sys
import os

# Force X11 backend for OpenCV on Wayland
os.environ['QT_QPA_PLATFORM'] = 'xcb'

# --- HELPER FUNCTIONS ---
def select_device(device='', batch_size=None):
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
# ------------------------

class YOLOPv2LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('yolopv2_lane_detection_node')

        # --- Descriptors ---
        # EXPANDED LIMITS: -2.0 to 2.0 allows points outside the image frame
        # FINER STEP: 0.001 allows precise pixel-perfect tuning
        scale_desc = ParameterDescriptor(floating_point_range=[
            FloatingPointRange(from_value=-2.0, to_value=2.0, step=0.001)
        ])

        # --- Parameters ---
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('output_image_topic', '/lane_detection/image')
        self.declare_parameter('output_bev_topic', '/lane_detection/bev')
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('output_width', 672)
        self.declare_parameter('output_height', 376)

        # Dynamic Parameters (Manual BEV Control)
        # Defaults set to your known "good" starting point
        self.declare_parameter('bev_top_width_scale', 0.8, scale_desc)
        self.declare_parameter('bev_bottom_width_scale', 1.0, scale_desc)
        self.declare_parameter('bev_top_height_scale', 0.75, scale_desc)
        self.declare_parameter('bev_bottom_height_scale', 0.8, scale_desc)

        # Read Initial Values
        self.bev_top_w = self.get_parameter('bev_top_width_scale').value
        self.bev_bot_w = self.get_parameter('bev_bottom_width_scale').value
        self.bev_top_h = self.get_parameter('bev_top_height_scale').value
        self.bev_bot_h = self.get_parameter('bev_bottom_height_scale').value

        self.out_width = self.get_parameter('output_width').value
        self.out_height = self.get_parameter('output_height').value
        self.show_viz = self.get_parameter('show_visualization').value
        
        # ROS Interfaces
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.get_parameter('camera_topic').value, self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, self.get_parameter('output_image_topic').value, 10)
        self.bev_pub = self.create_publisher(Image, self.get_parameter('output_bev_topic').value, 10)

        # Initialize Matrix
        self.init_bev_matrix()

        # Add Callback for Dynamic Updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Model Loading
        model_path = self.get_parameter('model_path').value
        if not model_path:
            try:
                pkg_share = get_package_share_directory('otagg_vision')
                model_path = os.path.join(pkg_share, 'models', 'yolopv2.pt')
            except Exception:
                model_path = 'yolopv2.pt'

        self.device = select_device('0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Device: {self.device} | Model: {model_path}')
        
        try:
            self.lane_model = torch.jit.load(model_path).to(self.device)
            self.lane_model.eval()
            if self.get_parameter('use_half_precision').value:
                self.lane_model.half()
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Model load error: {e}')
            raise

        if self.show_viz:
            cv2.namedWindow("Lane Detection (Front | BEV)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Lane Detection (Front | BEV)", self.out_width * 2, self.out_height)

    def parameter_callback(self, params):
        """Callback to handle dynamic parameter updates."""
        success = True
        bev_changed = False

        for param in params:
            if param.name == 'bev_top_width_scale':
                self.bev_top_w = param.value
                bev_changed = True
            elif param.name == 'bev_bottom_width_scale':
                self.bev_bot_w = param.value
                bev_changed = True
            elif param.name == 'bev_top_height_scale':
                self.bev_top_h = param.value
                bev_changed = True
            elif param.name == 'bev_bottom_height_scale':
                self.bev_bot_h = param.value
                bev_changed = True
                
        if bev_changed:
            self.init_bev_matrix()
            
        return SetParametersResult(successful=success)

    def init_bev_matrix(self):
        w, h = self.out_width, self.out_height
        
        # 1. Source Points (Trapezoid on Road)
        src = np.float32([
            [w * (0.5 - self.bev_top_w / 2), h * self.bev_top_h],     # TL
            [w * (0.5 + self.bev_top_w / 2), h * self.bev_top_h],     # TR
            [w * (0.5 - self.bev_bot_w / 2), h * self.bev_bot_h],     # BL
            [w * (0.5 + self.bev_bot_w / 2), h * self.bev_bot_h]      # BR
        ])
        
        # 2. Destination Points (Full Screen)
        dst = np.float32([
            [0, 0], [w, 0], [0, h], [w, h]
        ])
        
        self.bev_matrix = cv2.getPerspectiveTransform(src, dst)
        self.get_logger().info(f"BEV Updated. Top Width: {self.bev_top_w:.3f} | Top Height: {self.bev_top_h:.3f}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        if frame.shape[1] != self.out_width or frame.shape[0] != self.out_height:
             frame = cv2.resize(frame, (self.out_width, self.out_height))

        # --- Detection Logic ---
        try:
            img = letterbox(frame, 640, stride=32)[0]
            if img.shape[2] == 4: img = img[:, :, :3]
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.get_parameter('use_half_precision').value else img.float()
            img /= 255.0
            if img.ndimension() == 3: img = img.unsqueeze(0)

            with torch.no_grad():
                _, _, ll = self.lane_model(img)
            
            ll_seg_mask = lane_line_mask(ll)
            ll = ll.squeeze().detach().cpu().numpy()
            if ll.ndim == 3: ll = np.argmax(ll, axis=0)
            ll = ll.astype(np.uint8)

            if ll.shape != frame.shape[:2]:
                ll = cv2.resize(ll, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            
            height, width = img.shape[2:]
            foreground = ll == 1

        except Exception as e:
            return

        try:
            contours, _ = cv2.findContours(
                (ll_seg_mask > 0.5).astype(np.uint8), 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            lanes, close_lanes = [[0, 0], [0, 0]], [999, 999]
            mask_width = ll_seg_mask.shape[1]
            mean = mask_width / 2

            for contour in contours:
                if cv2.contourArea(contour) < 200: continue
                points = contour[:, 0, :]
                if len(contour) < 20: continue
                
                min_x = np.mean(points[:, 0])
                min_y = np.mean(points[:, 1])
                max_y = np.max(points[:, 1])
                
                distance = abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0])
                
                if close_lanes[0] > distance:
                    close_lanes[1], lanes[1] = close_lanes[0], lanes[0]
                    close_lanes[0], lanes[0] = distance, [min_x, min_y]
                elif close_lanes[1] > distance:
                    close_lanes[1], lanes[1] = distance, [min_x, min_y]

            midpoint_x = int((lanes[0][0] + lanes[1][0]) / 2 * (width / mask_width))
            midpoint_y = int((lanes[0][1] + lanes[1][1]) / 2 * (height / ll_seg_mask.shape[0]))
            
            x_new = int(midpoint_x * self.out_width / 640)
            y_new = int((midpoint_y * self.out_height / 640) * 2)
            
            cv2.circle(frame, (x_new, y_new), 7, (255, 255, 255), -1)
            if frame.shape[2] == 4: frame = frame[:, :, :3]
            frame[foreground] = [0, 0, 255]

        except Exception:
            pass

        # --- BEV GENERATION ---
        bev_frame = cv2.warpPerspective(frame, self.bev_matrix, (self.out_width, self.out_height))
        
        try:
            output_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)

            bev_msg = self.bridge.cv2_to_imgmsg(bev_frame, encoding='bgr8')
            bev_msg.header = msg.header
            self.bev_pub.publish(bev_msg)
        except Exception:
            pass

        if self.show_viz:
            combined_view = np.hstack((frame, bev_frame))
            cv2.imshow("Lane Detection (Front | BEV)", combined_view)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                rclpy.shutdown()

    def destroy_node(self):
        if self.show_viz: cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOPv2LaneDetectionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()