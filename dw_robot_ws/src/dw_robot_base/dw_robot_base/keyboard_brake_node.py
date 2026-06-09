#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


HELP_TEXT = """
Keyboard control with brake key

Moving:
   u    i    o
   j    k    l
   m    ,    .

i : forward
, : backward
j : turn left
l : turn right
u/o/m/. : curved motion

k : BRAKE ON + publish zero velocity
space : stop without brake
q/z : increase/decrease linear speed
w/x : increase/decrease angular speed

CTRL-C to quit
"""


class KeyboardBrakeNode(Node):
    def __init__(self):
        super().__init__('keyboard_brake_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_keyboard')
        self.declare_parameter('brake_topic', '/brake_cmd')

        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('angular_speed', 0.50)

        self.declare_parameter('linear_step', 0.02)
        self.declare_parameter('angular_step', 0.10)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.brake_pub = self.create_publisher(Bool, self.brake_topic, 10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('keyboard_brake_node started')
        self.get_logger().info(f'cmd_vel_topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'brake_topic: {self.brake_topic}')
        print(HELP_TEXT)
        self.print_speed()

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                if key == '\x03':
                    break

                self.handle_key(key)
                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
            self.publish_zero()
            self.publish_brake(True)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        readable, _, _ = select.select([sys.stdin], [], [], 0.1)

        if readable:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def handle_key(self, key):
        twist = Twist()

        if key == 'i':
            self.publish_brake(False)
            twist.linear.x = self.linear_speed

        elif key == ',':
            self.publish_brake(False)
            twist.linear.x = -self.linear_speed

        elif key == 'j':
            self.publish_brake(False)
            twist.angular.z = self.angular_speed

        elif key == 'l':
            self.publish_brake(False)
            twist.angular.z = -self.angular_speed

        elif key == 'u':
            self.publish_brake(False)
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

        elif key == 'o':
            self.publish_brake(False)
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed

        elif key == 'm':
            self.publish_brake(False)
            twist.linear.x = -self.linear_speed
            twist.angular.z = -self.angular_speed

        elif key == '.':
            self.publish_brake(False)
            twist.linear.x = -self.linear_speed
            twist.angular.z = self.angular_speed

        elif key == 'k':
            self.publish_zero()
            self.publish_brake(True)
            self.get_logger().warn('BRAKE ON by key k')
            return

        elif key == ' ':
            self.publish_zero()
            self.get_logger().info('Stop without brake')
            return

        elif key == 'q':
            self.linear_speed += self.linear_step
            self.print_speed()
            return

        elif key == 'z':
            self.linear_speed = max(0.0, self.linear_speed - self.linear_step)
            self.print_speed()
            return

        elif key == 'w':
            self.angular_speed += self.angular_step
            self.print_speed()
            return

        elif key == 'x':
            self.angular_speed = max(0.0, self.angular_speed - self.angular_step)
            self.print_speed()
            return

        else:
            return

        self.cmd_pub.publish(twist)

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def publish_brake(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.brake_pub.publish(msg)

    def print_speed(self):
        self.get_logger().info(
            f'linear_speed: {self.linear_speed:.3f} m/s, '
            f'angular_speed: {self.angular_speed:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBrakeNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()