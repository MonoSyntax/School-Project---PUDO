#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge, CvBridgeError

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("Camera node has been started.")

        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.camera_subscriber = self.create_subscription(CompressedImage,
                                            "/camera_compressed",
                                            self.camera_callback,
                                            qos_profile=self.qos_profile) 
        self.bridge= CvBridge()

        ### TEK SEFER ÇALIŞACAK KODLARI BURAYA YAZILABİLİR BAŞ ###


        
        ### TEK SEFER ÇALIŞACAK KODLARI BURAYA YAZILABİLİR SON ###
    def camera_callback(self, msg):
        ### KOD YAZILABİLECEK ARALIK BAŞI ###

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))

        cv2.imshow("Edge Detected Image", cv_image)
        cv2.waitKey(1)
        
        ### KOD YAZILABİLECEK ARALIK SONU ###
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
