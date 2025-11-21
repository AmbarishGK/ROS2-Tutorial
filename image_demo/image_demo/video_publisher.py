#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter('video_path', '')
        video_path = self.get_parameter('video_path').get_parameter_value().string_value

        # If not provided, try a sensible default in your repo
        if not video_path:
            repo_guess = os.path.join(
                os.path.expanduser('~'),
                'Desktop','thesi','ROS2-Tutorial','examples','data','sample.mp4'
            )
            video_path = repo_guess

        if not os.path.exists(video_path):
            raise FileNotFoundError(f'Video not found: {video_path}')

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Failed to open video: {video_path}')

        self.pub = self.create_publisher(Image, 'image_in', 10)
        self.bridge = CvBridge()

        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        if fps <= 1.0:  # guard weird metadata
            fps = 30.0
        self.timer = self.create_timer(1.0 / fps, self.timer_cb)

        self.get_logger().info(f'Publishing frames from {video_path} at ~{fps:.1f} FPS on "image_in"')

    def timer_cb(self):
        ok, frame = self.cap.read()
        if not ok:
            # loop the video
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Could not read frame after loop reset.')
                return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
