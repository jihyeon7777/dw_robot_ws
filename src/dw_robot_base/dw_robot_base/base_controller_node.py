#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


class BaseControllerNode(Node):
    def __init__(self):
        super().__init__('base_controller_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('wheel_cmd_topic', '/wheel_cmd')
        self.declare_parameter('brake_topic', '/brake_cmd')

        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('max_wheel_velocity', 0.30)

        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 30.0)

        self.declare_parameter('output_mode', 'topic')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('use_brake_protocol', True)
        self.declare_parameter('brake_on_timeout', True)
        self.declare_parameter('brake_on_zero_cmd', False)
        self.declare_parameter('brake_deadband', 0.02)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheel_cmd_topic = self.get_parameter('wheel_cmd_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value

        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.max_wheel_velocity = float(self.get_parameter('max_wheel_velocity').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.output_mode = self.get_parameter('output_mode').value

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)

        self.use_brake_protocol = bool(self.get_parameter('use_brake_protocol').value)
        self.brake_on_timeout = bool(self.get_parameter('brake_on_timeout').value)
        self.brake_on_zero_cmd = bool(self.get_parameter('brake_on_zero_cmd').value)
        self.brake_deadband = float(self.get_parameter('brake_deadband').value)

        self.last_cmd = Twist()
        self.last_cmd_time = None

        self.external_brake_requested = False
        self.serial_port = None
        self.brake_engaged = None

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10
        )

        self.brake_sub = self.create_subscription(
            Bool,
            self.brake_topic,
            self.brake_callback,
            10
        )

        self.wheel_cmd_pub = self.create_publisher(
            Float32MultiArray,
            self.wheel_cmd_topic,
            10
        )

        if self.output_mode == 'serial':
            self.open_serial()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('base_controller_node started')
        self.get_logger().info(f'cmd_vel_topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'wheel_cmd_topic: {self.wheel_cmd_topic}')
        self.get_logger().info(f'brake_topic: {self.brake_topic}')
        self.get_logger().info(f'output_mode: {self.output_mode}')
        self.get_logger().info(f'use_brake_protocol: {self.use_brake_protocol}')

    def open_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baudrate,
                timeout=0.05
            )
            self.get_logger().info(f'Serial opened: {self.serial_port_name}')

        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.serial_port_name}: {e}')

    def cmd_callback(self, msg):
        if not self.is_valid_twist(msg):
            self.get_logger().warn('Invalid Twist ignored')
            return

        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

        # Topic mode: this node is a pure Twist -> /wheel_cmd converter.
        # Timeout/stop is owned upstream by cmd_vel_safety_node (smooth
        # decel to zero) and downstream by the pnt50_driver_node hardware
        # failsafe, so we just forward each fresh command immediately.
        if self.output_mode != 'serial':
            self.compute_and_publish(self.last_cmd)

    def brake_callback(self, msg):
        self.external_brake_requested = bool(msg.data)

        # Topic mode: the electric brake is owned by pnt50_driver_node, so we
        # do not act on /brake_cmd here (avoids duplicated brake/stop logic).
        if self.output_mode != 'serial':
            return

        if self.external_brake_requested:
            self.get_logger().warn('External brake requested')
        else:
            self.get_logger().info('External brake released')

    def compute_and_publish(self, cmd):
        linear_x = cmd.linear.x
        angular_z = cmd.angular.z

        left_velocity = linear_x - angular_z * self.wheel_separation / 2.0
        right_velocity = linear_x + angular_z * self.wheel_separation / 2.0

        left_norm = self.normalize_velocity(left_velocity)
        right_norm = self.normalize_velocity(right_velocity)

        self.publish_wheel_cmd(
            left_velocity,
            right_velocity,
            left_norm,
            right_norm
        )

    def timer_callback(self):
        # Topic mode is event-driven (see cmd_callback); the timer only drives
        # the serial hardware path, which is itself a standalone motor driver.
        if self.output_mode != 'serial':
            return

        cmd, timed_out = self.get_safe_cmd()

        linear_x = cmd.linear.x
        angular_z = cmd.angular.z

        left_velocity = linear_x - angular_z * self.wheel_separation / 2.0
        right_velocity = linear_x + angular_z * self.wheel_separation / 2.0

        left_norm = self.normalize_velocity(left_velocity)
        right_norm = self.normalize_velocity(right_velocity)

        if self.external_brake_requested:
            self.publish_wheel_cmd(0.0, 0.0, 0.0, 0.0)
            self.send_stop_command()

            if self.use_brake_protocol:
                self.send_brake_command(True)

            return

        self.publish_wheel_cmd(
            left_velocity,
            right_velocity,
            left_norm,
            right_norm
        )

        if timed_out:
            self.send_stop_command()

            if self.use_brake_protocol and self.brake_on_timeout:
                self.send_brake_command(True)

            return

        is_zero_cmd = (
            abs(left_norm) < self.brake_deadband and
            abs(right_norm) < self.brake_deadband
        )

        if is_zero_cmd:
            self.send_stop_command()

            if self.use_brake_protocol and self.brake_on_zero_cmd:
                self.send_brake_command(True)

            return

        if self.use_brake_protocol:
            self.send_brake_command(False)

        self.send_motor_command(left_norm, right_norm)

    def get_safe_cmd(self):
        if self.last_cmd_time is None:
            return Twist(), True

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9

        if age > self.cmd_timeout:
            return Twist(), True

        return self.last_cmd, False

    def normalize_velocity(self, velocity):
        if self.max_wheel_velocity <= 0.0:
            return 0.0

        value = velocity / self.max_wheel_velocity
        return self.clamp(value, -1.0, 1.0)

    def publish_wheel_cmd(self, left_velocity, right_velocity, left_norm, right_norm):
        msg = Float32MultiArray()
        msg.data = [
            float(left_velocity),
            float(right_velocity),
            float(left_norm),
            float(right_norm),
        ]
        self.wheel_cmd_pub.publish(msg)

    def send_motor_command(self, left_norm, right_norm):
        if self.serial_port is None:
            return

        try:
            command = f'M,{left_norm:.3f},{right_norm:.3f}\n'
            self.serial_port.write(command.encode('utf-8'))

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_port = None

    def send_stop_command(self):
        if self.serial_port is None:
            return

        try:
            self.serial_port.write(b'S\n')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial stop error: {e}')
            self.serial_port = None

    def send_brake_command(self, enable):
        if self.serial_port is None:
            return

        if self.brake_engaged == enable:
            return

        try:
            if enable:
                self.serial_port.write(b'B,1\n')
                self.get_logger().warn('Brake ON')
            else:
                self.serial_port.write(b'B,0\n')
                self.get_logger().info('Brake OFF')

            self.brake_engaged = enable

        except serial.SerialException as e:
            self.get_logger().error(f'Serial brake error: {e}')
            self.serial_port = None

    def is_valid_twist(self, msg):
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]
        return all(math.isfinite(v) for v in values)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def stop_motors(self):
        self.publish_wheel_cmd(0.0, 0.0, 0.0, 0.0)

        if self.output_mode == 'serial' and self.serial_port is not None:
            try:
                self.serial_port.write(b'S\n')

                if self.use_brake_protocol:
                    self.serial_port.write(b'B,1\n')

            except serial.SerialException:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()

        if node.serial_port is not None:
            node.serial_port.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()