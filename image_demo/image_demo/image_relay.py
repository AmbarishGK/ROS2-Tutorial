#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageRelay(Node):
    def __init__(self):
        super().__init__('image_relay')
        self.sub = self.create_subscription(Image, 'image_in', self.cb, 10)
        self.pub = self.create_publisher(Image, 'image_out', 10)
        self.get_logger().info('image_relay started: image_in -> image_out')

    def cb(self, msg: Image):
        # Straight pass-through; RViz will subscribe to "image_out"
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
