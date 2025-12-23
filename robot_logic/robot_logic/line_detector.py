import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64, String
from cv_bridge import CvBridge
import cv2
import numpy as np

class BirdViewLaneDetector(Node):
    def __init__(self):
        super().__init__('birdview_lane_detector')
        # размер итогового изображения для обработки
        self.warp_w = 640
        self.warp_h = 480

        self.src_points = np.float32([
            # сверху слева
            [200, 280],
            # сверху справа
            [640, 280],
            # снизу справа
            [800, 450],
            # снизу слева
            [40, 450]
        ])

        self.dst_points = np.float32([
            [0, 0],
            [self.warp_w, 0],
            [self.warp_w, self.warp_h],
            [0, self.warp_h]
        ])

        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        # ширина полосы в пикселях... я ток после третьего заезда поняла что походу полоски походу??? разные по ширине
        self.LANE_WIDTH_PX = 517

        # пороги HSV
        # для желтого
        self.lower_yellow = np.array([15, 80, 80])
        self.upper_yellow = np.array([40, 255, 255])

        # для белого
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 30, 255])

        self.yellow_bias = 0.4

        # если будут белые пятна то они будут больше 200 чтобы не считать всякие блики и полоски конусов
        self.min_area = 200

        self.subscription = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        self.publisher_error = self.create_publisher(Float64, '/line_error', 10)
        self.bridge = CvBridge()

        self.prev_error = 0.0
        self.screen_center_x = self.warp_w / 2

        # оформим подписку на лидар и камеру глубины
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/depth/image', self.depth_callback, 10)

        self.STATE_LANE = "LANE"
        self.STATE_TUNNEL = "TUNNEL"
        self.current_state = self.STATE_LANE

        # дистанции до стен по лидару
        self.left_dist = 0.0
        self.right_dist = 0.0

        # нашла ли камера глубины потолок туннеля
        self.ceiling_detected = False

        # заглушки для выхода из туннеля
        self.tunnel_exit_confirmation = 0
        self.ceiling_lost_timer = 0

        # Пороги для оранжевых конусов
        self.lower_orange = np.array([0, 100, 100])
        self.upper_orange = np.array([25, 255, 255])

        self.min_cone_area = 300

        self.cone_memory = 0
        self.cone_memory_timer = 0

        self.finish_pub = self.create_publisher(String, "/robot_finish", 10)

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            h, w = depth_image.shape
            roi = depth_image[30:130, (w//2 - 60):(w//2 + 60)]

            valid_mask = np.isfinite(roi) & (roi > 0.1) & (roi < 10.0)
            valid_depths = roi[valid_mask]

            if len(valid_depths) > 30:
                avg_depth = np.mean(valid_depths)
                if avg_depth < 3.0:
                    self.ceiling_detected = True
                    self.ceiling_lost_timer = 0
                else:
                    self.ceiling_lost_timer += 1
            else:
                self.ceiling_lost_timer += 1

            # если мы не видим туннель (потолок) 15 кадров туннель закончился
            if self.ceiling_lost_timer > 15:
                self.ceiling_detected = False
        except:
            pass

        # self.get_logger().info(f"ПОТОЛОК: {self.ceiling_detected}")

    def scan_callback(self, msg):
        def get_stable_dist(angle_deg):
            start_idx = angle_deg - 15
            end_idx = angle_deg + 15
            
            sector = []

            for i in range(start_idx, end_idx):
                idx = i % 360
                r = msg.ranges[idx]

                if 0.1 < r < 3.5:
                    sector.append(r)

            return np.median(sector) if len(sector) > 0 else 3.5

        self.left_dist = get_stable_dist(90)
        self.right_dist = get_stable_dist(270)

    def image_callback(self, msg):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # применяем Bird's Eye View - спасибо авторейсу корейцев 2017 года
        warped_image = cv2.warpPerspective(raw_image, self.M, (self.warp_w, self.warp_h))

        hsv = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)

        # Создаем маску для конусов
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_white = cv2.subtract(mask_white, mask_orange)

        # Ищем контуры конусов
        contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        closest_cone_x = None
        closest_cone_y = -1

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_cone_area:
                finish_msg = String()
                finish_msg.data = "vpered"
                self.finish_pub.publish(finish_msg)
                return
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Нам нужен самый нижний конус на картинке (ближайший к роботу)
                    if cy > closest_cone_y:
                        closest_cone_y = cy
                        closest_cone_x = cx

        m_y = cv2.moments(mask_yellow)
        m_w = cv2.moments(mask_white)

        found_y = m_y['m00'] > self.min_area
        found_w = m_w['m00'] > self.min_area

        cx_y = int(m_y['m10'] / m_y['m00']) if found_y else 0
        cx_w = int(m_w['m10'] / m_w['m00']) if found_w else 0

        if self.current_state == self.STATE_LANE:
            if self.ceiling_detected and not found_y and not found_w:
                current_state = self.STATE_TUNNEL
                # self.get_logger().info("ВХОД В ТУННЕЛЬ")

        elif self.current_state == self.STATE_TUNNEL:
            # если потолок исчез и мы хоть одну линию видим то туннель закончился
            if not self.ceiling_detected and (found_y or found_w):
                self.tunnel_exit_confirmation += 1
            else:
                self.tunnel_exit_confirmation = 0
            if self.tunnel_exit_confirmation >= 3:
                self.current_state = self.STATE_LANE
                # self.get_logger().info("ВЫХОД ИЗ ТУННЕЛЯ")

        # по умолчанию берем за центр центр экрана
        target_x = self.screen_center_x
        half_dist = self.LANE_WIDTH_PX / 2

        if self.current_state == self.STATE_TUNNEL:
            lidar_diff = self.left_dist - self.right_dist
            tunnel_multiplier = 100
            max_tunnel_offset = 90
            offset = np.clip(lidar_diff * tunnel_multiplier, -max_tunnel_offset, max_tunnel_offset)

            # если прижались к стене
            # рулим вправо
            if self.left_dist < 0.30: offset = 100
            # рулим влево
            if self.right_dist < 0.30: offset = -100
            target_x = self.screen_center_x + offsets
        else:
            dist_from_yellow = self.LANE_WIDTH_PX * self.yellow_bias
            dist_from_white = self.LANE_WIDTH_PX * (1.0 - self.yellow_bias)

            if found_y and found_w:
                # видим обе линии
                # белая должна быть справа!!!
                if cx_w > cx_y + (self.LANE_WIDTH_PX / 3):
                    target_x = cx_y + dist_from_yellow
                    self.last_direction = 0
                else:
                    # если линии перепутались то берем желтую за цель
                    target_x = cx_y + (self.LANE_WIDTH_PX / 2)
            elif found_y:
                # видим только желтую линию - даем вправо
                target_x = cx_y + dist_from_yellow
                self.last_direction = -1
            elif found_w:
                # видим только белую линию - даем влево
                target_x = cx_w - dist_from_white
                self.last_direction = 1
            else:
                # если линий нет то используем старую ошибку
                # последняя линия была желтая
                if self.last_direction == -1:
                    # даем вправо
                    target_x = self.screen_center_x + (self.screen_center_x * 0.5)
                # последнюю видели белую
                elif self.last_direction == 1:
                    # даем влево
                    target_x = self.screen_center_x - (self.screen_center_x * 0.5)
                else:
                    target_x = self.screen_center_x + (self.prev_error * self.screen_center_x)

        current_avoidance_dir = 0
        avoidance_offset = 140

        if closest_cone_x is not None:
            if closest_cone_x < target_x:
                current_avoidance_dir = 1
            else:
                current_avoidance_dir = -1
            self.cone_memory = current_avoidance_dir
            self.cone_memory_timer = 15
        else:
            if self.cone_memory_timer > 0:
                self.cone_memory_timer -= 1
                current_avoidance_dir = self.cone_memory
            else:
                self.cone_memory = 0

        if current_avoidance_dir == 1:
            target_x += avoidance_offset
        elif current_avoidance_dir == -1:
            target_x -= avoidance_offset

        error = (target_x - self.screen_center_x) / self.screen_center_x
        error = np.clip(error, -1.0, 1.0)
        self.prev_error = error

        msg = Float64()
        msg.data = float(error)
        self.publisher_error.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BirdViewLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()