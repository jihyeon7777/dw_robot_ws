#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    """
    2D 로봇에서는 roll = 0, pitch = 0, yaw만 사용한다.
    yaw 각도를 quaternion으로 변환한다.
    """
    half_yaw = yaw * 0.5

    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    return qx, qy, qz, qw


def normalize_angle(angle: float) -> float:
    """
    각도를 -pi ~ pi 범위로 정규화한다.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FakeDiffDriveNode(Node):
    def __init__(self):
        super().__init__('fake_diff_drive_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint')

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.left_wheel_joint_name = str(self.get_parameter('left_wheel_joint_name').value)
        self.right_wheel_joint_name = str(self.get_parameter('right_wheel_joint_name').value)

        # -----------------------------
        # Robot state
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_safe',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update)

        self.get_logger().info('fake_diff_drive_node started')
        self.get_logger().info(f'wheel_radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'wheel_separation: {self.wheel_separation:.3f} m')
        self.get_logger().info(f'publish_rate: {self.publish_rate:.1f} Hz')

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel 명령을 저장한다.
        Differential Drive 로봇에서는 일반적으로
        linear.x와 angular.z만 사용한다.
        """
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """
        주기적으로 호출되는 함수.
        1. cmd_vel timeout 확인
        2. differential drive 기구학 계산
        3. 가상 로봇 위치 적분
        4. /odom publish
        5. /tf publish
        6. /joint_states publish
        """
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0:
            return

        # -----------------------------
        # Timeout safety
        # -----------------------------
        time_since_last_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9

        if time_since_last_cmd > self.cmd_vel_timeout:
            v = 0.0
            w = 0.0
        else:
            v = self.cmd_linear_x
            w = self.cmd_angular_z

        # -----------------------------
        # Differential drive kinematics
        # -----------------------------
        # 왼쪽/오른쪽 바퀴 선속도 [m/s]
        left_wheel_linear_velocity = v - (w * self.wheel_separation / 2.0)
        right_wheel_linear_velocity = v + (w * self.wheel_separation / 2.0)

        # 바퀴 각속도 [rad/s]
        left_wheel_angular_velocity = left_wheel_linear_velocity / self.wheel_radius
        right_wheel_angular_velocity = right_wheel_linear_velocity / self.wheel_radius

        # 바퀴 회전 위치 적분 [rad]
        self.left_wheel_position += left_wheel_angular_velocity * dt
        self.right_wheel_position += right_wheel_angular_velocity * dt

        # -----------------------------
        # Robot pose integration
        # -----------------------------
        # 단순 Euler 적분 방식
        delta_x = v * math.cos(self.yaw) * dt
        delta_y = v * math.sin(self.yaw) * dt
        delta_yaw = w * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw = normalize_angle(self.yaw + delta_yaw)

        # -----------------------------
        # Publish outputs
        # -----------------------------
        self.publish_odom(now, v, w)
        self.publish_tf(now)
        self.publish_joint_states(now, left_wheel_angular_velocity, right_wheel_angular_velocity)

    def publish_odom(self, now, linear_velocity: float, angular_velocity: float):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # 간단한 covariance 예시.
        # 실제 로봇에서는 센서 성능에 맞춰 조정해야 한다.
        odom_msg.pose.covariance[0] = 0.01      # x
        odom_msg.pose.covariance[7] = 0.01      # y
        odom_msg.pose.covariance[35] = 0.05     # yaw

        odom_msg.twist.covariance[0] = 0.01     # vx
        odom_msg.twist.covariance[35] = 0.05    # wz

        self.odom_pub.publish(odom_msg)

    def publish_tf(self, now):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_joint_states(
        self,
        now,
        left_wheel_angular_velocity: float,
        right_wheel_angular_velocity: float
    ):
        joint_msg = JointState()
        joint_msg.header.stamp = now.to_msg()

        joint_msg.name = [
            self.left_wheel_joint_name,
            self.right_wheel_joint_name
        ]

        joint_msg.position = [
            self.left_wheel_position,
            self.right_wheel_position
        ]

        joint_msg.velocity = [
            left_wheel_angular_velocity,
            right_wheel_angular_velocity
        ]

        joint_msg.effort = []

        self.joint_state_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeDiffDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()