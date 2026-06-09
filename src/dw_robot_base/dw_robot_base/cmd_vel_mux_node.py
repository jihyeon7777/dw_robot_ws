#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMuxNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        # Parameters
        self.declare_parameter('input_source', 'keyboard')

        self.declare_parameter('keyboard_topic', '/cmd_vel_keyboard')
        self.declare_parameter('rf_topic', '/cmd_vel_rf')
        self.declare_parameter('output_topic', '/cmd_vel_raw')

        self.declare_parameter('keyboard_timeout', 0.5)
        self.declare_parameter('rf_timeout', 0.5)
        self.declare_parameter('publish_rate', 30.0)

        self.keyboard_topic = self.get_parameter('keyboard_topic').value
        self.rf_topic = self.get_parameter('rf_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.keyboard_timeout = float(self.get_parameter('keyboard_timeout').value)
        self.rf_timeout = float(self.get_parameter('rf_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.last_keyboard_msg = None
        self.last_rf_msg = None

        self.last_keyboard_time = None
        self.last_rf_time = None

        self.last_selected_source = None
        self.last_invalid_source = None

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_topic,
            10
        )

        self.keyboard_sub = self.create_subscription(
            Twist,
            self.keyboard_topic,
            self.keyboard_callback,
            10
        )

        self.rf_sub = self.create_subscription(
            Twist,
            self.rf_topic,
            self.rf_callback,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('cmd_vel_mux_node started')
        self.get_logger().info(f'keyboard_topic: {self.keyboard_topic}')
        self.get_logger().info(f'rf_topic: {self.rf_topic}')
        self.get_logger().info(f'output_topic: {self.output_topic}')
        self.get_logger().info('input_source can be changed with:')
        self.get_logger().info('ros2 param set /cmd_vel_mux_node input_source keyboard')
        self.get_logger().info('ros2 param set /cmd_vel_mux_node input_source rf')

    def keyboard_callback(self, msg: Twist):
        if not self.is_valid_twist(msg):
            self.get_logger().warn('Invalid keyboard Twist message ignored')
            return

        self.last_keyboard_msg = self.copy_twist(msg)
        self.last_keyboard_time = self.get_clock().now()

    def rf_callback(self, msg: Twist):
        if not self.is_valid_twist(msg):
            self.get_logger().warn('Invalid RF Twist message ignored')
            return

        self.last_rf_msg = self.copy_twist(msg)
        self.last_rf_time = self.get_clock().now()

    def timer_callback(self):
        input_source = self.get_parameter('input_source').value
        input_source = str(input_source).lower()

        now = self.get_clock().now()

        if input_source == 'keyboard':
            selected_msg = self.get_valid_msg_or_zero(
                msg=self.last_keyboard_msg,
                stamp=self.last_keyboard_time,
                timeout=self.keyboard_timeout,
                now=now
            )

        elif input_source == 'rf':
            selected_msg = self.get_valid_msg_or_zero(
                msg=self.last_rf_msg,
                stamp=self.last_rf_time,
                timeout=self.rf_timeout,
                now=now
            )

        else:
            selected_msg = self.zero_twist()

            if self.last_invalid_source != input_source:
                self.get_logger().warn(
                    f'Invalid input_source "{input_source}". '
                    'Use "keyboard" or "rf". Publishing zero cmd_vel.'
                )
                self.last_invalid_source = input_source

        if self.last_selected_source != input_source:
            self.get_logger().info(f'cmd_vel_mux selected input_source: {input_source}')
            self.last_selected_source = input_source

        self.cmd_pub.publish(selected_msg)

    def get_valid_msg_or_zero(self, msg, stamp, timeout, now):
        if msg is None or stamp is None:
            return self.zero_twist()

        age = (now - stamp).nanoseconds * 1e-9

        if age > timeout:
            return self.zero_twist()

        return self.copy_twist(msg)

    def is_valid_twist(self, msg: Twist):
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]

        return all(math.isfinite(v) for v in values)

    def zero_twist(self):
        return Twist()

    def copy_twist(self, msg: Twist):
        copied = Twist()

        copied.linear.x = msg.linear.x
        copied.linear.y = msg.linear.y
        copied.linear.z = msg.linear.z

        copied.angular.x = msg.angular.x
        copied.angular.y = msg.angular.y
        copied.angular.z = msg.angular.z

        return copied


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMuxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()