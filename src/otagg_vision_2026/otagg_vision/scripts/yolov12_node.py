#!/usr/bin/env python3
"""
Modern Turkish Traffic Detection Node - YOLOv12
Simplified, robust, and production-ready with latest YOLOv12 model
"""

import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict
import time


class TrafficDetectionNode(Node):
    """Modern single-model traffic detection"""
    
    # Detection categories for color coding and alerts
    CATEGORIES = {
        'traffic_light': ['kirmizi','sari','yesil'],
        'critical_sign': ['dur', 'yol_ver', 'girisyok', 'parkyasak', 'soladonulmez'],
        'speed_limit': ['hiz_siniri_20', 'hiz_siniri_30', 'hiz_siniri_50', 
                       'hiz_siniri_60', 'hiz_siniri_70', 'hiz_siniri_80',
                       'hiz_siniri_100', 'hiz_siniri_120'],
        'pedestrian': ['insan'],
        'vehicle': ['araba', 'kamyon', 'otobus', 'motor', 'bisiklet'],
        'information_sign': ['park'],
    }
    
    # Colors (BGR format)
    COLORS = {
        'traffic_light': (0, 0, 255),      # Red
        'critical_sign': (0, 165, 255),    # Orange
        'speed_limit': (0, 255, 0),        # Green
        'pedestrian': (0, 255, 255),       # Yellow
        'vehicle': (255, 0, 0),            # Blue
        'default': (255, 255, 255),        # White
    }
    
    def __init__(self):
        super().__init__('traffic_detection_node')
        
        # Parameters
        self.declare_parameter('model_name', 'best.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('show_display', True)
        self.declare_parameter('fps_limit', 30)
        
        self.model_name = self.get_parameter('model_name').value
        self.conf_threshold = self.get_parameter('confidence').value
        self.iou_threshold = self.get_parameter('iou').value
        self.show_display = self.get_parameter('show_display').value
        self.fps_limit = self.get_parameter('fps_limit').value
        
        # Initialize
        self.bridge = CvBridge()
        self.last_process_time = 0
        self.frame_interval = 1.0 / self.fps_limit
        self.fps = 0
        self.detection_history = defaultdict(int)
        
        # Alert throttling
        self.last_alert_time = defaultdict(float)
        self.alert_throttle_duration = 2.0  # seconds
        
        # Load model
        self._load_model()
        
        # Setup ROS2
        self._setup_ros2()
        
        self.get_logger().info("✅ Traffic Detection Node (YOLOv12) ready!")
        self.get_logger().info(f"   Confidence: {self.conf_threshold}")
        self.get_logger().info(f"   Device: {self.device}")
    
    def _load_model(self):
        """Load YOLOv12 model"""
        try:
            # Get model path
            pkg_dir = get_package_share_directory('otagg_vision')
            model_path = Path(pkg_dir) / 'models' / self.model_name
            
            if not model_path.exists():
                raise FileNotFoundError(f"Model not found: {model_path}")
            
            # Setup device
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            
            # Load YOLOv12 model
            self.get_logger().info(f"Loading YOLOv12 model: {model_path}")
            self.model = YOLO(str(model_path))
            self.model.to(self.device)
            
            # Warmup
            self.get_logger().info("Warming up YOLOv12 model...")
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model(dummy, verbose=False)
            
            self.get_logger().info("✅ YOLOv12 model loaded and ready!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise
    
    def _setup_ros2(self):
        """Setup ROS2 publishers and subscribers"""
        
        # QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/camera_compressed',
            self.camera_callback,
            qos
        )
        
        # Publishers
        self.alert_pub = self.create_publisher(String, '/traffic/alerts', 10)
        self.image_pub = self.create_publisher(
            CompressedImage, 
            '/traffic/annotated/compressed', 
            10
        )
    
    def camera_callback(self, msg: CompressedImage):
        """Process camera frames"""
        
        # FPS limiting
        current_time = time.time()
        if current_time - self.last_process_time < self.frame_interval:
            return
        
        try:
            # Decode image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            
            # Run detection
            start = time.time()
            detections = self._detect(frame)
            inference_time = time.time() - start
            
            # Calculate FPS
            self.fps = 1.0 / (time.time() - self.last_process_time) if self.last_process_time > 0 else 0
            self.last_process_time = current_time
            
            # Process detections
            self._handle_detections(detections)
            
            # Visualize
            annotated = self._draw_detections(frame, detections, inference_time)
            
            # Display
            if self.show_display:
                cv2.imshow("Traffic Detection", annotated)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rclpy.shutdown()
            
            # Publish annotated image
            self._publish_image(annotated)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
    
    def _detect(self, frame: np.ndarray) -> list:
        """Run YOLO detection"""
        
        results = self.model(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False
        )
        
        detections = []
        for r in results:
            for box in r.boxes:
                class_id = int(box.cls.cpu())
                class_name = r.names[class_id]
                confidence = float(box.conf.cpu())
                category = self._get_category(class_name)
                
                det = {
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': box.xyxy[0].cpu().numpy().astype(int),
                    'category': category
                }
                detections.append(det)        
        return detections
    
    def _get_category(self, class_name: str) -> str:
        """Get detection category"""
        for category, classes in self.CATEGORIES.items():
            if class_name in classes:
                return category
        return 'default'
    
    def _handle_detections(self, detections: list):
        """Handle critical detections and alerts"""
        
        for det in detections:
            name = det['class_name']
            category = det['category']
            
            # Track detection history
            self.detection_history[name] += 1
            
            # Critical alerts
            alert = None
            if name == 'kirmizi_isik':
                alert = "🔴 RED LIGHT - STOP!"
            elif name == 'dur':
                alert = "🛑 STOP SIGN"
            elif name == 'yol_ver':
                alert = "⚠️  YIELD"
            elif category == 'pedestrian':
                alert = "👤 PEDESTRIAN DETECTED"
            
            # Publish alert
            if alert:
                current_time = time.time()
                if current_time - self.last_alert_time[alert] > self.alert_throttle_duration:
                    self.get_logger().warn(alert)
                    self.last_alert_time[alert] = current_time
                    
                msg = String()
                msg.data = alert
                self.alert_pub.publish(msg)
    
    def _draw_detections(
        self, 
        frame: np.ndarray, 
        detections: list, 
        inference_time: float
    ) -> np.ndarray:
        """Draw detections on frame"""
        
        annotated = frame.copy()
        h, w = frame.shape[:2]
        
        # Draw each detection
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            name = det['class_name']
            conf = det['confidence']
            category = det['category']
            
            # Get color
            color = self.COLORS.get(category, self.COLORS['default'])
            
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw label background
            label = f"{name} {conf:.2f}"
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            cv2.rectangle(
                annotated,
                (x1, y1 - label_h - 10),
                (x1 + label_w, y1),
                color,
                -1
            )
            
            # Draw label text
            cv2.putText(
                annotated, label, (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2  # Changed text color to black
            )
        
        # Draw info panel
        self._draw_info_panel(annotated, detections, inference_time)
        
        return annotated
    
    def _draw_info_panel(
        self, 
        frame: np.ndarray, 
        detections: list,
        inference_time: float
    ):
        """Draw info panel on frame"""
        
        # Background
        cv2.rectangle(frame, (10, 10), (300, 120), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (300, 120), (255, 255, 255), 2)
        
        # Info text
        info = [
            f"FPS: {self.fps:.1f}",
            f"Inference: {inference_time*1000:.1f}ms",
            f"Detections: {len(detections)}",
            f"Device: {self.device.upper()}"
        ]
        
        y = 35
        for text in info:
            cv2.putText(
                frame, text, (20, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )
            y += 25
        
        # Detection summary
        if detections:
            categories = defaultdict(int)
            for det in detections:
                categories[det['category']] += 1
            
            summary = ", ".join([f"{cat}: {count}" for cat, count in categories.items()])
            cv2.putText(
                frame, summary, (10, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )
    
    def _publish_image(self, frame: np.ndarray):
        """Publish annotated image"""
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")


def main(args=None):
    """Main entry point"""
    
    print("\n" + "="*70)
    print("🚦 TURKISH TRAFFIC DETECTION NODE - YOLOv12")
    print("="*70)
    print("Starting...\n")
    
    rclpy.init(args=args)
    
    try:
        node = TrafficDetectionNode()
        
        print("="*70)
        print("✅ Node running!")
        print("="*70)
        print("Press Ctrl+C to stop\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("Shutting down...")
        print("="*70)
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()