import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO

class ConjunctionController(Node):
    def __init__(self):
        super().__init__('conjunction_controller')

        # Параметры
        self.declare_parameter('image_topic', '/color/image')
        image_topic = self.get_parameter('image_topic').value

        self.sign_names = [["i10"], ["i5"]] # l r
        self.sign_thresholds = [0.25, 0.25]

        # YOLO
        self.model = YOLO('best.pt')
        # self.model.set_classes(self.sign_names, self.model.get_text_pe(self.sign_names))

        self.bridge = CvBridge()

        # ROS entities
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST) 

        self.debug_pub = self.create_publisher(Image, '/yolo/debug_image', qos) 

        self.sign_pub = self.create_publisher(String, '/yolo/sign', 10)

        self.get_logger().info('YOLO cmd_vel node started')

        self.finished = False

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = self.model(frame, verbose=False, device=0)[0]

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            cls_name = self.model.names[cls_id]

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            label = f"{cls_name} {conf:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 5, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2
            )

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)
        # self.get_logger().info(results.boxes)

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            cls_name = self.model.names[cls_id]

            for sign_name, conf_thr in zip(self.sign_names, self.sign_thresholds):
                x1, y1, x2, y2 = box.xyxy[0]

                w = abs(x2 - x1)
                h = abs(y2 - y1)

                if (
                    cls_name in sign_name
                    and conf >= conf_thr
                    and h > 0
                    and w / h > 0.4
                ):
                    sign_msg = String()
                    sign_msg.data = f"{cls_name} {conf}"
                    self.sign_pub.publish(sign_msg)
                    self.finished = True
                    return

def main():
    rclpy.init()
    node = ConjunctionController()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
