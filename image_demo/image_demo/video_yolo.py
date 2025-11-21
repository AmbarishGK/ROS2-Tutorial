#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Try structured detection messages if installed
USE_VISION_MSGS = True
try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
except Exception:
    USE_VISION_MSGS = False

# Ultralytics YOLO
from ultralytics import YOLO

class VideoYOLO(Node):
    def __init__(self):
        super().__init__('video_yolo')

        # Parameters
        self.declare_parameter('video_path', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_thres', 0.25)

        # Resolve paths
        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf = float(self.get_parameter('conf_thres').value)

        if not video_path:
            # default to repo video
            video_path = os.path.join(os.path.expanduser('~'),
                                      'Desktop','thesi','ROS2-Tutorial','examples','data','sample.mp4')
        if not os.path.exists(video_path):
            raise FileNotFoundError(f'Video not found: {video_path}')

        if not model_path:
            # default to repo model (yolov8n.pt you already have)
            model_path = os.path.join(os.path.expanduser('~'),
                                      'Desktop','thesi','ROS2-Tutorial','examples','data','yolov8n.pt')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'Model not found: {model_path}')

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.conf_thres = conf

        # Video capture
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f'Failed to open video: {video_path}')

        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        if fps <= 1.0:  # guard odd metadata
            fps = 30.0

        # Publishers
        self.pub_img = self.create_publisher(Image, 'image_yolo', 10)
        if USE_VISION_MSGS:
            self.pub_det = self.create_publisher(Detection2DArray, 'detections_yolo', 10)
        else:
            self.pub_det_json = self.create_publisher(String, 'detections_yolo_json', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f'Streaming "{video_path}" at ~{fps:.1f} FPS | conf>={self.conf_thres} '
            f'-> topics: /image_yolo and /detections_yolo{"_json" if not USE_VISION_MSGS else ""}'
        )

        # Class names from model (Ultralytics COCO by default)
        self.names = self.model.names if hasattr(self.model, 'names') else {}

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            # loop video
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Could not read frame after loop reset.')
                return

        # Run YOLO (returns results list)
        results = self.model.predict(source=frame, conf=self.conf_thres, verbose=False)

        # Draw and build messages
        det_array = None
        if USE_VISION_MSGS:
            det_array = Detection2DArray()

        # Ultralytics results: boxes.xyxy, boxes.conf, boxes.cls
        annotated = frame.copy()
        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for i in range(len(r.boxes)):
                xyxy = r.boxes.xyxy[i].tolist()
                conf = float(r.boxes.conf[i].item())
                cls_id = int(r.boxes.cls[i].item()) if r.boxes.cls is not None else -1
                label = self.names.get(cls_id, str(cls_id))

                x1, y1, x2, y2 = [int(v) for v in xyxy]
                # Draw rectangle and label on annotated frame
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f'{label} {conf:.2f}', (x1, max(0, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                # Build detections
                if USE_VISION_MSGS:
                    det = Detection2D()
                    det.bbox = BoundingBox2D()
                    det.bbox.center.position.x = (x1 + x2) / 2.0
                    det.bbox.center.position.y = (y1 + y2) / 2.0
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label
                    hyp.hypothesis.score = conf  # confidence/accuracy
                    det.results.append(hyp)
                    det_array.detections.append(det)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.pub_img.publish(img_msg)

        # Publish detections
        if USE_VISION_MSGS:
            self.pub_det.publish(det_array)
        else:
            # compact JSON fallback
            dets = []
            for r in results:
                if r.boxes is None or len(r.boxes) == 0:
                    continue
                for i in range(len(r.boxes)):
                    x1,y1,x2,y2 = [float(v) for v in r.boxes.xyxy[i].tolist()]
                    conf = float(r.boxes.conf[i].item())
                    cls_id = int(r.boxes.cls[i].item()) if r.boxes.cls is not None else -1
                    label = self.names.get(cls_id, str(cls_id))
                    dets.append({"label": label, "conf": conf, "bbox_xyxy": [x1,y1,x2,y2]})
            s = String()
            s.data = __import__('json').dumps(dets)
            self.pub_det_json.publish(s)

def main(args=None):
    rclpy.init(args=args)
    node = VideoYOLO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
