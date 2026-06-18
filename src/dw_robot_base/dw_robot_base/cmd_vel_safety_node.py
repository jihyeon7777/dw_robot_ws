#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    value를 min_value ~ max_value 범위로 제한한다.
    """
    return max(min_value, min(value, max_value))


def apply_acceleration_limit(
    current_value: float,
    target_value: float,
    max_acceleration: float,
    dt: float
) -> float:
    """
    현재 속도에서 목표 속도로 바로 바꾸지 않고,
    최대 가속도 제한 안에서 천천히 변화시킨다.
    """
    if dt <= 0.0:
        return current_value

    max_delta = max_acceleration * dt
    delta = target_value - current_value

    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta

    return current_value + delta


class CmdVelSafetyNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel_safe')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_lateral_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.8)

        self.declare_parameter('max_linear_acceleration', 0.3)
        self.declare_parameter('max_angular_acceleration', 1.0)

        self.declare_parameter('enable_acceleration_limit', True)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        self.max_linear_velocity = float(
            self.get_parameter('max_linear_velocity').value
        )
        self.max_lateral_velocity = float(
            self.get_parameter('max_lateral_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )

        self.max_linear_acceleration = float(
            self.get_parameter('max_linear_acceleration').value
        )
        self.max_angular_acceleration = float(
            self.get_parameter('max_angular_acceleration').value
        )

        self.enable_acceleration_limit = bool(
            self.get_parameter('enable_acceleration_limit').value
        )

        # -----------------------------
        # State variables
        # -----------------------------
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_vel_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_topic,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info('cmd_vel_safety_node started')
        self.get_logger().info(f'input_topic: {self.input_topic}')
        self.get_logger().info(f'output_topic: {self.output_topic}')
        self.get_logger().info(f'max_linear_velocity: {self.max_linear_velocity:.3f} m/s')
        self.get_logger().info(f'max_lateral_velocity: {self.max_lateral_velocity:.3f} m/s')
        self.get_logger().info(f'max_angular_velocity: {self.max_angular_velocity:.3f} rad/s')
        self.get_logger().info(f'cmd_vel_timeout: {self.cmd_vel_timeout:.3f} s')

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel_raw를 받아서 목표 속도로 저장한다.
        linear.x(전후), linear.y(좌우 평행이동, 메카넘), angular.z(yaw)를 사용한다.
        """
        raw_linear_x = msg.linear.x
        raw_linear_y = msg.linear.y
        raw_angular_z = msg.angular.z

        # 속도 제한
        self.target_linear_x = clamp(
            raw_linear_x,
            -self.max_linear_velocity,
            self.max_linear_velocity
        )

        self.target_linear_y = clamp(
            raw_linear_y,
            -self.max_lateral_velocity,
            self.max_lateral_velocity
        )

        self.target_angular_z = clamp(
            raw_angular_z,
            -self.max_angular_velocity,
            self.max_angular_velocity
        )

        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """
        주기적으로 안전한 /cmd_vel_safe를 출력한다.
        """
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            return

        time_since_last_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9

        # timeout 발생 시 목표 속도를 0으로 만든다.
        if time_since_last_cmd > self.cmd_vel_timeout:
            safe_target_linear_x = 0.0
            safe_target_linear_y = 0.0
            safe_target_angular_z = 0.0
        else:
            safe_target_linear_x = self.target_linear_x
            safe_target_linear_y = self.target_linear_y
            safe_target_angular_z = self.target_angular_z

        # 가속도 제한 적용
        if self.enable_acceleration_limit:
            self.current_linear_x = apply_acceleration_limit(
                self.current_linear_x,
                safe_target_linear_x,
                self.max_linear_acceleration,
                dt
            )

            self.current_linear_y = apply_acceleration_limit(
                self.current_linear_y,
                safe_target_linear_y,
                self.max_linear_acceleration,
                dt
            )

            self.current_angular_z = apply_acceleration_limit(
                self.current_angular_z,
                safe_target_angular_z,
                self.max_angular_acceleration,
                dt
            )
        else:
            self.current_linear_x = safe_target_linear_x
            self.current_linear_y = safe_target_linear_y
            self.current_angular_z = safe_target_angular_z

        # 아주 작은 값은 0으로 처리
        if abs(self.current_linear_x) < 1e-4:
            self.current_linear_x = 0.0

        if abs(self.current_linear_y) < 1e-4:
            self.current_linear_y = 0.0

        if abs(self.current_angular_z) < 1e-4:
            self.current_angular_z = 0.0

        # 안전 속도 명령 publish
        cmd_msg = Twist()
        cmd_msg.linear.x = self.current_linear_x
        cmd_msg.linear.y = self.current_linear_y
        cmd_msg.linear.z = 0.0

        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = self.current_angular_z

        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelSafetyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 종료 전에 정지 명령 한 번 publish
    stop_msg = Twist()
    node.cmd_pub.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()