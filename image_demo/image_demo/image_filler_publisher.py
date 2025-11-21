#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageFillerPublisher(Node):
    def __init__(self):
        super().__init__('image_filler_publisher')
        self.pub = self.create_publisher(Image, 'image_in', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2 Hz
        self.get_logger().info('image_filler_publisher started -> publishing on "image_in"')

        # Optional: if you drop a file at examples/data/filler.jpg it will publish that
        self.fallback_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            'examples', 'data', 'filler.jpg'
        )

    def make_placeholder(self, w=640, h=480):
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (32, 32, 32)
        cv2.rectangle(img, (40, 40), (w-40, h-40), (200, 200, 200), 2)
        cv2.putText(img, 'ROS 2 Image Demo', (60, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, 'Topic: image_in', (60, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (180, 255, 180), 2, cv2.LINE_AA)
        return img

    def timer_cb(self):
        # Try to read filler.jpg if present, else generate placeholder
        if os.path.exists(self.fallback_path):
            img = cv2.imread(self.fallback_path, cv2.IMREAD_COLOR)
            if img is None:
                img = self.make_placeholder()
        else:
            img = self.make_placeholder()

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)
        self.get_logger().debug('Published filler frame')

def main(args=None):
    rclpy.init(args=args)
    node = ImageFillerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
