import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

from ultralytics import YOLOE

class StartController(Node):
    def __init__(self):
        super().__init__('start_controller')

        # Параметры
        self.declare_parameter('image_topic', '/color/image')
        self.declare_parameter('target_class', 'green circle')
        self.declare_parameter('conf_threshold', 0.96)

        image_topic = self.get_parameter('image_topic').value

        # YOLO
        self.model = YOLOE('yoloe-11s-seg.pt')
        names = ["green circle"]
        self.model.set_classes(names, self.model.get_text_pe(names))

        self.bridge = CvBridge()

        # ROS entities
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST) 

        self.debug_pub = self.create_publisher(Image, '/yolo/debug_image', qos) 

        self.state_pub = self.create_publisher(String, '/allow_move', 10)

        self.get_logger().info('YOLO cmd_vel node started')

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = self.model(frame, verbose=False, device='cpu')[0]

        target_found = False
        target_class = self.get_parameter('target_class').value
        conf_thr = self.get_parameter('conf_threshold').value

        self.get_logger().info(
            f"Boxes count: {len(results.boxes)}",
            throttle_duration_sec=1.0
        )

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

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            cls_name = self.model.names[cls_id]

            if cls_name == target_class and conf >= conf_thr:
                target_found = True
                break

        if target_found:
            state_msg = String()
            state_msg.data = "yes"
            self.state_pub.publish(state_msg)
            self.destroy_node()
            return


def main():
    rclpy.init()
    node = StartController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
