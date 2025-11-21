import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
cv2 = __import__('cv2')

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('conf', 0.25)

        self.model_path = self.get_parameter('model').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.conf = float(self.get_parameter('conf').get_parameter_value().double_value)

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.pub_img = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.pub_det = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        self.get_logger().info(f'Subscribed image: {self.image_topic}')
        self.get_logger().info('Publishes annotated image: /yolo/image_annotated')
        self.get_logger().info('Publishes detections: /yolo/detections')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        t0 = time.time()
        results = self.model(frame, conf=self.conf, verbose=False)
        inf_ms = (time.time() - t0) * 1000.0

        det_array = Detection2DArray()
        det_array.header = msg.header
        annotated = frame.copy()

        for r in results:
            if getattr(r, 'boxes', None) is None:
                continue
            for b in r.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                conf = float(b.conf[0].item()) if b.conf is not None else 0.0
                cls_id = int(b.cls[0].item()) if b.cls is not None else -1
                cls_name = r.names.get(cls_id, str(cls_id)) if hasattr(r, 'names') else str(cls_id)

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'{cls_name} {conf:.2f}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 4), (x1 + tw, y1), (0, 255, 0), -1)
                cv2.putText(annotated, label, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.x = (x1 + x2) / 2.0
                det.bbox.center.y = (y1 + y2) / 2.0
                det.bbox.size_x = (x2 - x1)
                det.bbox.size_y = (y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.id = str(cls_id)
                hyp.score = conf
                det.results.append(hyp)
                det_array.detections.append(det)

        self.pub_det.publish(det_array)
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_img.publish(out_msg)
        self.get_logger().debug(f'YOLO: {len(det_array.detections)} dets @ {inf_ms:.1f} ms')

def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
