import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

class LineFollowerPID(Node):
    def __init__(self):
        super().__init__('line_follower_pid')
        # параметры ПИД
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        # линейная скорость
        self.declare_parameter('speed', 1.0) 
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        # в топик /line_error идут - (0.0 - в центре), (>0 - левее), (<0 - правее)
        self.sub_error = self.create_subscription(Float64, '/line_error', self.error_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # разрешаем ли движение для светофора
        self.move_allowed = False
        self.sub_allow = self.create_subscription(String, '/allow_move', self.allow_move_callback, 10)

        # подписка на знак
        self.sub_side = self.create_subscription(String, '/yolo/sign', self.side_callback, 10)
        self.sign = 'Left'
        self.is_maneuvering = False
        self.maneuver_start_time = None
        self.maneuver_duration = 20.0

    def side_callback(self, msg):
        if self.is_maneuvering:
            return

        elif "i5" in msg.data:
            self.sign = 'Right'
            self.is_maneuvering = True
            self.maneuver_start_time = self.get_clock().now()
            self.get_logger().info("ВИЖУ ЗНАК: ПОВОРОТ ВПРАВО")

    def allow_move_callback(self, msg):
        if msg.data == "yes":
            if not self.move_allowed:
                self.get_logger().info("ПОЛУЧЕНО 'YES'. Начинаю движение!")
            self.move_allowed = True
        
    def error_callback(self, msg):
        if not self.move_allowed:
            stop_twist = Twist()
            self.pub_cmd.publish(stop_twist)
            return

        max_angular_vel = 1.5
        base_speed = self.get_parameter('speed').value

        error = msg.data

        if self.is_maneuvering:
            elapsed = (self.get_clock().now() - self.maneuver_start_time).nanoseconds / 1e9
            if elapsed > self.maneuver_duration:
                max_angular_vel = 2.5
                base_speed = 0
                self.is_maneuvering = False
                self.sign = 'Left'
                self.get_logger().info("МАНЕВР ЗАВЕРШЕН: ВОЗВРАТ К ОБЫЧНОЙ ЛИНИИ")

        if self.sign == 'Right':
            offset = 0.8
            error = error - offset

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        max_angular_vel = 1.5
        min_speed = 0.05
        
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        base_speed = 0.2

        # ПИД ЛОГИКА
        # пропорциональная часть
        p_term = kp * error
        # интегральная часть
        self.integral += error * dt
        self.integral = max(min(self.integral, 0.5), -0.5) 
        i_term = ki * self.integral
        # дифференциальная часть
        d_term = kd * (error - self.prev_error) / dt
        self.prev_error = error
        # итоговое управление
        steering = p_term + i_term + d_term

        # чтоб руль не дергало
        if steering > max_angular_vel: steering = max_angular_vel
        if steering < -max_angular_vel: steering = -max_angular_vel

        # если ошибка большая (ну те поворот) сбрасываем скорость
        speed_multiplier = 1.0 - abs(error) * 0.95
        current_speed = max(base_speed * speed_multiplier, min_speed)

        twist = Twist()
        twist.linear.x = float(current_speed)
        # можно убирать/ставить минус если он не туда поворачивает
        twist.angular.z = float(-steering)

        self.pub_cmd.publish(twist)

def main():
    rclpy.init()
    node = LineFollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()