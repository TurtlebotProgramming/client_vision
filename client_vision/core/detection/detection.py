import os
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from client_vision_interfaces.msg import TurtlebotDetection
import numpy as np

class ClientVision(Node):
    def __init__(self):
        super().__init__('client_vision_subscriber')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        model_path = os.path.join(
            get_package_share_directory('client_vision'), 'model', 'best.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Model: {model_path}')

  
        self.model.predict(source=np.zeros((480, 640, 3), dtype=np.uint8),
                           imgsz=640, half=True, device='cuda', verbose=False)
        self.get_logger().info('Model warmup done')

        self.br = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self._event = threading.Event()

        self.yolo_pub_ = self.create_publisher(TurtlebotDetection, '/detection', 1)

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, sensor_qos)

        threading.Thread(target=self._infer_loop, daemon=True).start()

    def _image_cb(self, msg: Image):
        try:
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return
        with self._lock:
            self._frame = frame
        self._event.set()

    def _infer_loop(self):
        while rclpy.ok():
            self._event.wait()
            self._event.clear()

            with self._lock:
                frame = self._frame
                self._frame = None

            if frame is None:
                continue

            results = self.model.predict(
                source=frame,
                imgsz=640,
                half=True,
                device='cuda',
                verbose=False,
                save=False,
            )

            msg = TurtlebotDetection()
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
                    cls   = int(box.cls[0])
                    score = float(box.conf[0])
                    msg.class_ids.append(cls)
                    msg.score.append(score)
                    msg.x1.append(x1)
                    msg.y1.append(y1)
                    msg.x2.append(x2)
                    msg.y2.append(y2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'{cls} {score:.2f}', (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            self.yolo_pub_.publish(msg)

            cv2.imshow('YOLO Detection', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ClientVision())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
