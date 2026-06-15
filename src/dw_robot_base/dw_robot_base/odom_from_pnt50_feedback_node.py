import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle: float):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OdomFromPnt50FeedbackNode(Node):
    def __init__(self):
        super().__init__('odom_from_pnt50_feedback_node')

        # 실제 로봇 값에 맞게 나중에 조정
        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('ticks_per_wheel_rev', 1000.0)

        # 방향이 반대로 나오면 -1.0으로 변경
        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)

        self.declare_parameter('feedback_topic', '/pnt50_feedback')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_wheel_rev = float(self.get_parameter('ticks_per_wheel_rev').value)

        self.left_sign = float(self.get_parameter('left_sign').value)
        self.right_sign = float(self.get_parameter('right_sign').value)

        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Float32MultiArray,
            self.feedback_topic,
            self.feedback_callback,
            10
        )

        self.get_logger().info('odom_from_pnt50_feedback_node started')
        self.get_logger().info(f'feedback_topic: {self.feedback_topic}')
        self.get_logger().info(f'wheel_radius: {self.wheel_radius}')
        self.get_logger().info(f'wheel_separation: {self.wheel_separation}')
        self.get_logger().info(f'ticks_per_wheel_rev: {self.ticks_per_wheel_rev}')

    def feedback_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('/pnt50_feedback must contain [left_ticks, right_ticks]')
            return

        left_ticks = int(msg.data[0])
        right_ticks = int(msg.data[1])

        now = self.get_clock().now()

        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        delta_left_ticks = (left_ticks - self.last_left_ticks) * self.left_sign
        delta_right_ticks = (right_ticks - self.last_right_ticks) * self.right_sign

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = now

        left_distance = self.ticks_to_distance(delta_left_ticks)
        right_distance = self.ticks_to_distance(delta_right_ticks)

        delta_s = (left_distance + right_distance) * 0.5
        delta_theta = (right_distance - left_distance) / self.wheel_separation

        theta_mid = self.theta + delta_theta * 0.5

        self.x += delta_s * math.cos(theta_mid)
        self.y += delta_s * math.sin(theta_mid)
        self.theta = normalize_angle(self.theta + delta_theta)

        linear_x = delta_s / dt
        angular_z = delta_theta / dt

        self.publish_odom(now, linear_x, angular_z)

    def ticks_to_distance(self, ticks: float):
        wheel_revolutions = ticks / self.ticks_per_wheel_rev
        return wheel_revolutions * 2.0 * math.pi * self.wheel_radius

    def publish_odom(self, stamp, linear_x: float, angular_z: float):
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = linear_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_z

        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10

        odom_msg.twist.covariance[0] = 0.05
        odom_msg.twist.covariance[7] = 0.05
        odom_msg.twist.covariance[35] = 0.10

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp.to_msg()
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


def main(args=None):
    rclpy.init(args=args)
    node = OdomFromPnt50FeedbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()