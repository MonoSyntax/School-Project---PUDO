#!/usr/bin/env python3
"""
YOLOv11 Traffic Sign Detection Node with Scientific Stabilization

Features:
- Matches training parameters (imgsz=640, iou=0.7)
- Exponential Moving Average (EMA) smoothing for bbox stability
- Detection persistence tracking to reduce flicker
- Configurable confidence thresholds

Subscribes to: /camera_compressed (sensor_msgs/CompressedImage)
Publishes to: /traffic_signs (otagg_vision_interfaces/TrafficSignArray)
"""

import os
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from otagg_vision_interfaces.msg import TrafficSignArray, TrafficSignDetection


# Known real-world sign sizes (width in meters) for distance estimation
# Turkey Road Sign dataset class names (Turkish)
SIGN_SIZES_METERS = {
    # Traffic lights (30cm diameter)
    'kirmizi': 0.30,  # red light
    'sari': 0.30,     # yellow light
    'yesil': 0.30,    # green light
    # Regulatory signs (60cm)
    'dur': 0.60,           # stop
    'girisyok': 0.60,      # no entry
    'sagadonulmez': 0.60,  # no right turn
    'soladonulmez': 0.60,  # no left turn
    'sag': 0.60,           # right only
    'sol': 0.60,           # left only
    'ilerisag': 0.60,      # straight or right
    'ilerisol': 0.60,      # straight or left
    # Speed limit signs (60cm)
    '20': 0.60,  # 20 km/h speed limit
    '30': 0.60,  # 30 km/h speed limit
    # Informational signs (50cm)
    'durak': 0.50,     # bus stop
    'park': 0.45,      # parking
    'parkyasak': 0.45, # no parking
}


@dataclass
class TrackedDetection:
    """Tracked detection with EMA smoothing."""
    class_name: str
    bbox: np.ndarray  # [x1, y1, x2, y2]
    confidence: float
    frames_seen: int = 1
    frames_missing: int = 0
    last_update_time: float = 0.0


class DetectionTracker:
    """
    Tracks detections across frames with EMA smoothing.
    
    Scientific approach:
    - Uses IoU-based matching to associate detections across frames
    - Applies Exponential Moving Average to smooth bbox coordinates
    - Requires minimum frames before confirming detection (reduces false positives)
    - Allows persistence for brief occlusions
    """
    
    def __init__(self, ema_alpha: float = 0.4, min_frames: int = 2, 
                 max_missing: int = 3, iou_threshold: float = 0.3):
        self.ema_alpha = ema_alpha  # Higher = more responsive, lower = smoother
        self.min_frames = min_frames  # Frames required to confirm detection
        self.max_missing = max_missing  # Frames before dropping track
        self.iou_threshold = iou_threshold  # IoU for matching tracks
        self.tracks: Dict[int, TrackedDetection] = {}
        self.next_track_id = 0
    
    def compute_iou(self, box1: np.ndarray, box2: np.ndarray) -> float:
        """Compute IoU between two bounding boxes [x1,y1,x2,y2]."""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def update(self, detections: List[Tuple[str, np.ndarray, float]], 
               current_time: float) -> List[TrackedDetection]:
        """
        Update tracks with new detections.
        
        Args:
            detections: List of (class_name, bbox, confidence)
            current_time: Current timestamp
            
        Returns:
            List of confirmed tracked detections
        """
        # Mark all tracks as unmatched
        matched_tracks = set()
        matched_detections = set()
        
        # Match detections to existing tracks using IoU
        for det_idx, (class_name, bbox, conf) in enumerate(detections):
            best_iou = 0.0
            best_track_id = None
            
            for track_id, track in self.tracks.items():
                if track_id in matched_tracks:
                    continue
                if track.class_name != class_name:
                    continue
                    
                iou = self.compute_iou(bbox, track.bbox)
                if iou > best_iou and iou >= self.iou_threshold:
                    best_iou = iou
                    best_track_id = track_id
            
            if best_track_id is not None:
                # Update existing track with EMA smoothing
                track = self.tracks[best_track_id]
                track.bbox = (self.ema_alpha * bbox + 
                             (1 - self.ema_alpha) * track.bbox)
                track.confidence = (self.ema_alpha * conf + 
                                   (1 - self.ema_alpha) * track.confidence)
                track.frames_seen += 1
                track.frames_missing = 0
                track.last_update_time = current_time
                matched_tracks.add(best_track_id)
                matched_detections.add(det_idx)
        
        # Create new tracks for unmatched detections
        for det_idx, (class_name, bbox, conf) in enumerate(detections):
            if det_idx in matched_detections:
                continue
            
            self.tracks[self.next_track_id] = TrackedDetection(
                class_name=class_name,
                bbox=bbox.copy(),
                confidence=conf,
                frames_seen=1,
                frames_missing=0,
                last_update_time=current_time
            )
            self.next_track_id += 1
        
        # Update missing counts for unmatched tracks
        for track_id in list(self.tracks.keys()):
            if track_id not in matched_tracks:
                self.tracks[track_id].frames_missing += 1
                
                # Remove stale tracks
                if self.tracks[track_id].frames_missing > self.max_missing:
                    del self.tracks[track_id]
        
        # Return confirmed detections (seen enough frames)
        confirmed = [track for track in self.tracks.values() 
                     if track.frames_seen >= self.min_frames]
        return confirmed


class TrafficSignDetectorNode(Node):
    """ROS 2 node for stabilized traffic sign detection."""
    
    def __init__(self):
        super().__init__('traffic_sign_detector_node')
        self.get_logger().info("Traffic Sign Detector Node (Stabilized) started.")
        
        # === Parameters matching training config ===
        # From training: imgsz=640, iou=0.7, max_det=300
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('iou_threshold', 0.7)  # Match training!
        self.declare_parameter('image_size', 640)  # Match training!
        self.declare_parameter('max_detections', 300)  # Match training!
        
        # Stabilization parameters
        self.declare_parameter('ema_alpha', 0.4)  # EMA smoothing factor
        self.declare_parameter('min_frames_confirm', 1)  # Frames to confirm
        self.declare_parameter('max_frames_missing', 1)  # Persistence frames
        self.declare_parameter('track_iou_threshold', 0.3)  # Track matching
        
        # Visualization
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('focal_length_px', 500.0)
        
        # Get parameter values
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.imgsz = self.get_parameter('image_size').value
        self.max_det = self.get_parameter('max_detections').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.min_frames = self.get_parameter('min_frames_confirm').value
        self.max_missing = self.get_parameter('max_frames_missing').value
        self.track_iou = self.get_parameter('track_iou_threshold').value
        self.publish_viz = self.get_parameter('publish_visualization').value
        self.focal_length = self.get_parameter('focal_length_px').value
        
        # Log configuration
        self.get_logger().info(f"Config: conf={self.conf_threshold}, iou={self.iou_threshold}, "
                               f"imgsz={self.imgsz}, ema={self.ema_alpha}")
        
        # Initialize detection tracker
        self.tracker = DetectionTracker(
            ema_alpha=self.ema_alpha,
            min_frames=self.min_frames,
            max_missing=self.max_missing,
            iou_threshold=self.track_iou
        )
        
        # QoS profile
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,  # Reduced for lower latency
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Camera subscription
        self.camera_sub = self.create_subscription(
            CompressedImage, '/camera_compressed',
            self.camera_callback, qos_profile=self.qos_profile
        )
        self.bridge = CvBridge()
        
        # Publisher
        self.detection_pub = self.create_publisher(TrafficSignArray, '/traffic_signs', 10)
        
        # Device setup
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        
        # Load model
        models_dir = os.path.join(get_package_share_directory('otagg_vision'), 'models')
        sign_model_path = os.path.join(models_dir, 'best.pt')
        self.get_logger().info(f'Loading model: {sign_model_path}')
        self.sign_model = YOLO(sign_model_path)
        
        self.get_logger().info("Model loaded. Ready for detection.")
    
    def estimate_distance(self, bbox_width: int, sign_class: str) -> float:
        """Estimate distance using pinhole camera model."""
        real_width = SIGN_SIZES_METERS.get(sign_class, 0.5)
        if bbox_width <= 0:
            return -1.0
        return (real_width * self.focal_length) / bbox_width
    
    def camera_callback(self, msg: CompressedImage):
        """Process camera frame with stabilized detection."""
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        if frame is None:
            return
        
        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Raw detections from YOLO (using training parameters)
        raw_detections = []
        try:
            results = self.sign_model(
                frame_rgb,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                max_det=self.max_det,
                verbose=False
            )
            
            for r in results:
                for box in r.boxes:
                    bbox = box.xyxy[0].cpu().numpy()
                    cls_id = int(box.cls.item())
                    conf = float(box.conf.item())
                    class_name = self.sign_model.names[cls_id]
                    raw_detections.append((class_name, bbox, conf))
                    
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
        
        # Update tracker with raw detections -> get stabilized output
        confirmed_detections = self.tracker.update(raw_detections, current_time)
        
        # Create detection array message
        detection_array = TrafficSignArray()
        detection_array.header = msg.header
        
        for track in confirmed_detections:
            bbox = track.bbox.astype(int)
            x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            bbox_cx = (x1 + x2) // 2
            bbox_cy = (y1 + y2) // 2
            
            distance = self.estimate_distance(bbox_width, track.class_name)
            
            detection = TrafficSignDetection()
            detection.header = msg.header
            detection.sign_class = track.class_name
            detection.confidence = float(track.confidence)
            detection.bbox_x = int(bbox_cx)
            detection.bbox_y = int(bbox_cy)
            detection.bbox_width = int(bbox_width)
            detection.bbox_height = int(bbox_height)
            detection.estimated_distance = float(distance)
            
            detection_array.detections.append(detection)
            
            # Visualization
            if self.publish_viz:
                # Color based on frames seen (more stable = greener)
                stability = min(track.frames_seen / 5.0, 1.0)
                color = (0, int(255 * stability), int(255 * (1 - stability)))
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{track.class_name} {track.confidence:.2f} [{track.frames_seen}f]"
                
                (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (x1, y1 - lh - 10), (x1 + lw, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Publish stabilized detections
        self.detection_pub.publish(detection_array)
        
        # Show visualization
        if self.publish_viz:
            # Add stats overlay
            stats = f"Tracks: {len(self.tracker.tracks)} | Confirmed: {len(confirmed_detections)}"
            cv2.putText(frame, stats, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("Traffic Sign Detection (Stabilized)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
