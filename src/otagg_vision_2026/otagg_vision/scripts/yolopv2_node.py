#!/usr/bin/env python3
"""
YOLOPv2 Lane Detection ROS2 Node
=================================

Bu node, Gazebo simülasyonundan gelen kamera görüntülerini alır ve
YOLOPv2 modeli ile şerit tespiti yapar.

Subscribes:
    - /camera/image_raw (sensor_msgs/Image): Gazebo kamera görüntüsü

Publishes:
    - /lane_detection/image (sensor_msgs/Image): İşlenmiş görüntü
    - /lane_detection/control (std_msgs/Float32): Direksiyon kontrol değeri

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

import cv2
import torch
import numpy as np
import sys
from ament_index_python.packages import get_package_share_directory
from simple_pid import PID

import os

# Force X11 backend for OpenCV on Wayland
os.environ['QT_QPA_PLATFORM'] = 'xcb'

def select_device(device='', batch_size=None):
    # Cihaz seçimi (CPU veya CUDA)
    device = str(device).strip().lower().replace('cuda:', '').replace('none', '')  # to string, 'cuda:0' to '0'
    cpu = device == 'cpu'
    if cpu:
        os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # force torch.cuda.is_available() = False
    elif device:  # non-cpu device requested
        os.environ['CUDA_VISIBLE_DEVICES'] = device  # set environment variable
        assert torch.cuda.is_available(), f'CUDA unavailable, invalid device {device} requested'  # check availability

    cuda = not cpu and torch.cuda.is_available()
    return torch.device('cuda:0' if cuda else 'cpu')

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Görüntüyü modele uygun boyuta (kare veya dikdörtgen) getirmek için kenarlara gri dolgu ekler (resize + pad)
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)

def lane_line_mask(ll):
    # Modelden çıkan ham veriyi (logits) ikili maskeye (0 veya 1) çevirir.
    # ll: (Batch, 2, Height, Width) -> Genellikle şerit ve arka plan
    if ll.ndim == 3: # Eger batch boyutu yoksa ekle
        ll = ll.unsqueeze(0)
    
    # Argmax ile en yüksek olasılıklı sınıfı seç (Şerit mi değil mi?)
    ll_seg_mask = torch.argmax(ll, dim=1)
    
    # Tensor'dan numpy'a çevir
    ll_seg_mask = ll_seg_mask.squeeze().detach().cpu().numpy()
    
    return ll_seg_mask

class YOLOPv2LaneDetectionNode(Node):
    """YOLOPv2 tabanlı şerit tespit ROS2 node'u"""

    def __init__(self):
        super().__init__('yolopv2_lane_detection_node')

        # Parametreleri tanımla
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('output_image_topic', '/lane_detection/image')
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('model_path', '')
        self.declare_parameter('use_half_precision', True)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('output_width', 1280)
        self.declare_parameter('output_height', 720)
        
        # PID parametreleri
        self.declare_parameter('pid_kp', 0.005)
        self.declare_parameter('pid_ki', 0.0001)
        self.declare_parameter('pid_kd', 0.002)
        self.declare_parameter('linear_speed', 1.0)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.use_half = self.get_parameter('use_half_precision').get_parameter_value().bool_value
        self.show_viz = self.get_parameter('show_visualization').get_parameter_value().bool_value
        self.out_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.out_height = self.get_parameter('output_height').get_parameter_value().integer_value
        
        kp = self.get_parameter('pid_kp').get_parameter_value().double_value
        ki = self.get_parameter('pid_ki').get_parameter_value().double_value
        kd = self.get_parameter('pid_kd').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        
        self.pid = PID(kp, ki, kd, setpoint=0.0)
        self.pid.output_limits = (-1.0, 1.0)  # Açısal hız limitleri

        if not model_path:
            try:
                pkg_share = get_package_share_directory('otagg_vision')
                model_path = os.path.join(pkg_share, 'models', 'yolopv2.pt')
            except Exception:
                model_path = 'yolopv2.pt'

        self.bridge = CvBridge()

        self.device = select_device('0' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Cihaz: {self.device}')
        self.get_logger().info(f'Model yükleniyor: {model_path}')

        try:
            self.lane_model = torch.jit.load(model_path).to(self.device)
            self.lane_model.eval()
            if self.use_half:
                self.lane_model.half()
            self.get_logger().info('Model başarıyla yüklendi!')
        except Exception as e:
            self.get_logger().error(f'Model yüklenirken hata: {e}')
            raise

        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Kamera topic: {camera_topic}')

        ### TEK SEFER ÇALIŞACAK KODLAR BAŞI ###
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)
        self.control_pub = self.create_publisher(Twist, control_topic, 10)

        # Visualization penceresi
        if self.show_viz:
            cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Lane Detection", self.out_width, self.out_height)

        self.get_logger().info('YOLOPv2 Lane Detection Node başlatıldı!')
        ### TEK SEFER ÇALIŞACAK KODLAR SONU ###


    def image_callback(self, msg: Image):
        ### FRAME LOOP BAŞI ###

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Görüntü dönüştürme hatası: {e}')
            return

        frame = cv2.resize(frame, (self.out_width, self.out_height))
        
        control = 0.0
        try:
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

            with torch.no_grad():
                _, _, ll = self.lane_model(img)
            
            ll_seg_mask = lane_line_mask(ll)
            ll = ll.squeeze().detach().cpu().numpy()

            if ll.ndim == 3:
                ll = np.argmax(ll, axis=0)
            
            ll = ll.astype(np.uint8)

            if ll.shape != frame.shape[:2]:
                ll = cv2.resize(ll, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            
            height, width = img.shape[2:]
            foreground = ll == 1

        except Exception as e:
            self.get_logger().warn(f'Şerit takibi hatası: {e}')
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
                if cv2.contourArea(contour) < 200:
                    continue
                points = contour[:, 0, :]
                if len(contour) < 20:
                    continue
                
                min_x = np.mean(points[:, 0])
                min_y = np.mean(points[:, 1])
                max_y = np.max(points[:, 1])
                
                distance = abs(min_x - mean) + abs(max_y - ll_seg_mask.shape[0])
                
                if close_lanes[0] > distance:
                    close_lanes[1] = close_lanes[0]
                    lanes[1] = lanes[0]
                    close_lanes[0] = distance
                    lanes[0] = [min_x, min_y]
                elif close_lanes[1] > distance:
                    close_lanes[1] = distance
                    lanes[1] = [min_x, min_y]

            midpoint_x = int((lanes[0][0] + lanes[1][0]) / 2 * (width / mask_width))
            midpoint_y = int((lanes[0][1] + lanes[1][1]) / 2 * (height / ll_seg_mask.shape[0]))
            x_new = int(midpoint_x * self.out_width / 640)
            y_new = int((midpoint_y * self.out_height / 640) * 2)
            
            cv2.circle(frame, (x_new, y_new), 7, (255, 255, 255), -1)
            
            control = float(-1 * ((width / 2) - midpoint_x))
            self.get_logger().debug(f'Kontrol değeri: {control}')

            if frame.shape[2] == 4:
                frame = frame[:, :, :3]
            
            frame[foreground] = [0, 0, 255]

        except Exception as e:
            self.get_logger().warn(f'Kontur işleme hatası: {e}')
            control = 0.0

        pid_output = self.pid(control)

        control_msg = Twist()
        control_msg.linear.x = self.linear_speed
        control_msg.angular.z = pid_output
        # PID ayarlanacak çok sola çekiyor
        # self.control_pub.publish(control_msg)


        try:
            output_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            output_msg.header = msg.header
            self.image_pub.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f'Görüntü yayınlama hatası: {e}')

        if self.show_viz:
            cv2.imshow("Lane Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Kullanıcı tarafından kapatıldı.')
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
        node.get_logger().info('Node kapatılıyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()