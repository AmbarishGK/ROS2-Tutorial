#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RDuration

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion

import torch
from mmdet3d.apis import init_model, inference_detector


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) around Z into a geometry_msgs/Quaternion."""
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class LiDARDetectionNode(Node):
    def __init__(self):
        # Node name = "mmdetect_lidar_demo"
        super().__init__("mmdetect_lidar_demo")

        # Parameters for config & checkpoint so you can override via ROS params if needed
        self.declare_parameter(
        "config_file",
        "/workspace/mmdetection3d/checkpoints/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py",
        )
        self.declare_parameter(
            "checkpoint_file",
            "/workspace/mmdetection3d/checkpoints/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth",
        )

        self.declare_parameter("lidar_topic", "/velodyne_points")

        self.config_file = self.get_parameter("config_file").get_parameter_value().string_value
        self.checkpoint_file = self.get_parameter("checkpoint_file").get_parameter_value().string_value
        self.lidar_topic = self.get_parameter("lidar_topic").get_parameter_value().string_value

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"Config: {self.config_file}")
        self.get_logger().info(f"Checkpoint: {self.checkpoint_file}")

        # Publishers
        self.edge_marker_pub = self.create_publisher(Marker, "bounding_box_edges", 10)
        self.cube_marker_pub = self.create_publisher(MarkerArray, "lidar_bounding_boxes", 10)

        # Load model once
        self.get_logger().info("Loading MMDetection3D model...")
        self.model = init_model(self.config_file, self.checkpoint_file, device=self.device)
        self.get_logger().info("Model ready.")

        # Subscriber to LiDAR point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            10,
        )

    def lidar_callback(self, msg: PointCloud2):
        try:
            pts = self.point_cloud2_to_array(msg)  # (N,4): x y z intensity
            if pts.size == 0:
                self.get_logger().warn("Received empty point cloud.")
                return

            self.get_logger().debug(f"LiDAR cloud with {pts.shape[0]} points. Running inference...")
            result = inference_detector(self.model, pts)
            self.get_logger().debug("Inference done.")

            det = getattr(result[0], "pred_instances_3d", None)
            if det is None:
                self.get_logger().info("No pred_instances_3d in result.")
                return

            boxes = getattr(det, "bboxes_3d", None)
            scores = getattr(det, "scores_3d", None)
            labels = getattr(det, "labels_3d", None)

            if boxes is None or not hasattr(boxes, "tensor") or boxes.tensor.numel() == 0:
                self.get_logger().info("No boxes detected.")
                return

            # Optional: filter by score threshold
            if scores is not None:
                keep = scores > 0.4
                if keep.sum() == 0:
                    self.get_logger().info("All boxes below score threshold.")
                    return
                boxes.tensor = boxes.tensor[keep]
                scores = scores[keep]
                labels = labels[keep] if labels is not None else None

            frame = msg.header.frame_id or "base_link"
            self.publish_bounding_box_edges(frame, boxes)
            self.publish_bounding_boxes(frame, boxes, scores, labels)

        except Exception as e:
            self.get_logger().error(f"Error during inference: {e}")

    @staticmethod
    def point_cloud2_to_array(msg: PointCloud2) -> np.ndarray:
        """Convert sensor_msgs/PointCloud2 -> (N,4) numpy array (x,y,z,intensity)."""
        pts = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2], 0.0])  # zero intensity is fine
        return np.asarray(pts, dtype=np.float32)

    @staticmethod
    def compute_rotated_corners(x, y, z, l, w, h, yaw):
        """Compute 8 corners of 3D bbox rotated by yaw around z."""
        dx, dy = l * 0.5, w * 0.5
        corners = [[dx, dy], [-dx, dy], [-dx, -dy], [dx, -dy]]
        rot = [
            [
                x + c[0] * math.cos(yaw) - c[1] * math.sin(yaw),
                y + c[0] * math.sin(yaw) + c[1] * math.cos(yaw),
                z,
            ]
            for c in corners
        ]
        bottom = [Point(x=c[0], y=c[1], z=z) for c in rot]
        top = [Point(x=c[0], y=c[1], z=z + h) for c in rot]
        return bottom, top

    def publish_bounding_box_edges(self, frame_id: str, boxes):
        """Publish a single LINE_LIST marker with all bbox edges."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "bbox_edges"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line thickness
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = RDuration(seconds=0.5).to_msg()

        T = boxes.tensor  # (M, 7): x y z l w h yaw
        for i in range(T.shape[0]):
            x, y, z, l, w, h, yaw = [t.item() for t in T[i]]
            bottom, top = self.compute_rotated_corners(x, y, z, l, w, h, yaw)
            edges = [
                (bottom[0], bottom[1]),
                (bottom[1], bottom[2]),
                (bottom[2], bottom[3]),
                (bottom[3], bottom[0]),
                (top[0], top[1]),
                (top[1], top[2]),
                (top[2], top[3]),
                (top[3], top[0]),
                (bottom[0], top[0]),
                (bottom[1], top[1]),
                (bottom[2], top[2]),
                (bottom[3], top[3]),
            ]
            for a, b in edges:
                marker.points.append(a)
                marker.points.append(b)

        self.edge_marker_pub.publish(marker)

    def publish_bounding_boxes(self, frame_id: str, boxes, scores, labels):
        """Publish CUBE markers for each 3D bbox."""
        arr = MarkerArray()
        T = boxes.tensor
        for i in range(T.shape[0]):
            x, y, z, l, w, h, yaw = [t.item() for t in T[i]]

            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "bbox_cubes"
            m.id = int(i)
            m.type = Marker.CUBE
            m.action = Marker.ADD

            # Pose
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation = yaw_to_quat(yaw)

            # Size
            m.scale.x = float(l)
            m.scale.y = float(w)
            m.scale.z = float(h)

            # Color (green, semi-transparent)
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.4

            m.lifetime = RDuration(seconds=0.5).to_msg()
            arr.markers.append(m)

        self.cube_marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LiDARDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
