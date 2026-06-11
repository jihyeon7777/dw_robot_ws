#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray


class RfArduinoNode(Node):
    def __init__(self):
        super().__init__('rf_arduino_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_timeout', 0.05)

        self.declare_parameter('output_topic', '/cmd_vel_rf')
        self.declare_parameter('brake_topic', '/brake_cmd')
        self.declare_parameter('pnt50_feedback_topic', '/pnt50_feedback')
        self.declare_parameter('publish_pnt50_feedback', True)

        self.declare_parameter('publish_rate', 50.0)

        self.declare_parameter('max_linear_velocity', 0.10)
        self.declare_parameter('max_angular_velocity', 0.5)

        self.declare_parameter('throttle_center', 1505)
        self.declare_parameter('throttle_min', 1000)
        self.declare_parameter('throttle_max', 2000)

        self.declare_parameter('steering_center', 1495)
        self.declare_parameter('steering_min', 1000)
        self.declare_parameter('steering_max', 2000)

        self.declare_parameter('deadzone_us', 40)

        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('invert_steering', False)

        self.declare_parameter('enable_required', True)
        self.declare_parameter('enable_threshold', 1500)
        self.declare_parameter('enable_active_high', True)

        self.declare_parameter('brake_threshold', 1500)
        self.declare_parameter('brake_active_high', True)
        self.declare_parameter('brake_on_signal_loss', True)

        self.declare_parameter('serial_timeout_sec', 0.3)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.read_timeout = float(self.get_parameter('read_timeout').value)

        self.output_topic = self.get_parameter('output_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value
        self.pnt50_feedback_topic = self.get_parameter('pnt50_feedback_topic').value
        self.publish_pnt50_feedback = bool(self.get_parameter('publish_pnt50_feedback').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)

        self.throttle_center = int(self.get_parameter('throttle_center').value)
        self.throttle_min = int(self.get_parameter('throttle_min').value)
        self.throttle_max = int(self.get_parameter('throttle_max').value)

        self.steering_center = int(self.get_parameter('steering_center').value)
        self.steering_min = int(self.get_parameter('steering_min').value)
        self.steering_max = int(self.get_parameter('steering_max').value)

        self.deadzone_us = int(self.get_parameter('deadzone_us').value)

        self.invert_throttle = bool(self.get_parameter('invert_throttle').value)
        self.invert_steering = bool(self.get_parameter('invert_steering').value)

        self.enable_required = bool(self.get_parameter('enable_required').value)
        self.enable_threshold = int(self.get_parameter('enable_threshold').value)
        self.enable_active_high = bool(self.get_parameter('enable_active_high').value)

        self.brake_threshold = int(self.get_parameter('brake_threshold').value)
        self.brake_active_high = bool(self.get_parameter('brake_active_high').value)
        self.brake_on_signal_loss = bool(self.get_parameter('brake_on_signal_loss').value)

        self.serial_timeout_sec = float(self.get_parameter('serial_timeout_sec').value)

        self.serial_port = None
        self.last_serial_time = None
        self.last_brake_state = None

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_topic,
            10
        )

        self.brake_pub = self.create_publisher(
            Bool,
            self.brake_topic,
            10
        )

        self.pnt50_feedback_pub = self.create_publisher(
            Float32MultiArray,
            self.pnt50_feedback_topic,
            10
        )

        self.open_serial()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('rf_arduino_node started')
        self.get_logger().info(f'port: {self.port}')
        self.get_logger().info(f'baudrate: {self.baudrate}')
        self.get_logger().info(f'output_topic: {self.output_topic}')
        self.get_logger().info(f'brake_topic: {self.brake_topic}')
        self.get_logger().info(f'pnt50_feedback_topic: {self.pnt50_feedback_topic}')
        self.get_logger().info(
            'Expected serial format: '
            'throttle,steering,enable,brake,signal_ok,pulse1,pulse2,rpm1,rpm2'
        )

    def open_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.read_timeout
            )
            self.get_logger().info(f'Serial opened: {self.port}')

        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')

    def timer_callback(self):
        if self.serial_port is None:
            self.publish_zero()
            if self.brake_on_signal_loss:
                self.publish_brake(True)
            return

        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

            if line == '':
                self.check_serial_timeout()
                return

            parsed = self.parse_line(line)

            if parsed is None:
                self.publish_zero()
                return

            (
                throttle_us,
                steering_us,
                enable_us,
                brake_us,
                signal_ok,
                pulse1,
                pulse2,
                rpm1,
                rpm2,
            ) = parsed

            self.last_serial_time = self.get_clock().now()

            self.publish_pnt50_feedback_msg(
                pulse1=pulse1,
                pulse2=pulse2,
                rpm1=rpm1,
                rpm2=rpm2
            )

            if signal_ok == 0:
                self.publish_zero()
                if self.brake_on_signal_loss:
                    self.publish_brake(True)
                return

            brake_requested = self.is_brake_requested(brake_us)

            if brake_requested:
                self.publish_zero()
                self.publish_brake(True)
                return

            self.publish_brake(False)

            if self.enable_required:
                if not self.is_enabled(enable_us):
                    self.publish_zero()
                    return

            throttle_norm = self.normalize_pwm(
                value=throttle_us,
                center=self.throttle_center,
                min_value=self.throttle_min,
                max_value=self.throttle_max,
                deadzone=self.deadzone_us
            )

            steering_norm = self.normalize_pwm(
                value=steering_us,
                center=self.steering_center,
                min_value=self.steering_min,
                max_value=self.steering_max,
                deadzone=self.deadzone_us
            )

            if self.invert_throttle:
                throttle_norm *= -1.0

            if self.invert_steering:
                steering_norm *= -1.0

            cmd = Twist()
            cmd.linear.x = throttle_norm * self.max_linear_velocity
            cmd.angular.z = steering_norm * self.max_angular_velocity

            if not self.is_valid_twist(cmd):
                self.publish_zero()
                return

            self.cmd_pub.publish(cmd)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.serial_port = None
            self.publish_zero()

            if self.brake_on_signal_loss:
                self.publish_brake(True)

        except Exception as e:
            self.get_logger().warn(f'RF parse error: {e}')
            self.publish_zero()

    def parse_line(self, line):
        parts = line.split(',')

        if len(parts) == 9:
            try:
                throttle_us = int(float(parts[0]))
                steering_us = int(float(parts[1]))
                enable_us = int(float(parts[2]))
                brake_us = int(float(parts[3]))
                signal_ok = int(float(parts[4]))

                pulse1 = float(parts[5])
                pulse2 = float(parts[6])
                rpm1 = float(parts[7])
                rpm2 = float(parts[8])

                return (
                    throttle_us,
                    steering_us,
                    enable_us,
                    brake_us,
                    signal_ok,
                    pulse1,
                    pulse2,
                    rpm1,
                    rpm2,
                )

            except ValueError:
                self.get_logger().warn(f'Invalid value in serial line: {line}')
                return None

        if len(parts) == 5:
            # Backward compatibility:
            # throttle,steering,enable,brake,signal_ok
            try:
                throttle_us = int(float(parts[0]))
                steering_us = int(float(parts[1]))
                enable_us = int(float(parts[2]))
                brake_us = int(float(parts[3]))
                signal_ok = int(float(parts[4]))

                pulse1 = 0.0
                pulse2 = 0.0
                rpm1 = 0.0
                rpm2 = 0.0

                return (
                    throttle_us,
                    steering_us,
                    enable_us,
                    brake_us,
                    signal_ok,
                    pulse1,
                    pulse2,
                    rpm1,
                    rpm2,
                )

            except ValueError:
                self.get_logger().warn(f'Invalid integer in serial line: {line}')
                return None

        if len(parts) == 4:
            # Backward compatibility:
            # throttle,steering,enable,signal_ok
            try:
                throttle_us = int(float(parts[0]))
                steering_us = int(float(parts[1]))
                enable_us = int(float(parts[2]))
                brake_us = 1000
                signal_ok = int(float(parts[3]))

                pulse1 = 0.0
                pulse2 = 0.0
                rpm1 = 0.0
                rpm2 = 0.0

                return (
                    throttle_us,
                    steering_us,
                    enable_us,
                    brake_us,
                    signal_ok,
                    pulse1,
                    pulse2,
                    rpm1,
                    rpm2,
                )

            except ValueError:
                self.get_logger().warn(f'Invalid integer in serial line: {line}')
                return None

        self.get_logger().warn(f'Invalid serial line: {line}')
        return None

    def publish_pnt50_feedback_msg(self, pulse1, pulse2, rpm1, rpm2):
        if not self.publish_pnt50_feedback:
            return

        msg = Float32MultiArray()
        msg.data = [
            float(pulse1),
            float(pulse2),
            float(rpm1),
            float(rpm2),
        ]

        self.pnt50_feedback_pub.publish(msg)

    def normalize_pwm(self, value, center, min_value, max_value, deadzone):
        if value <= 0:
            return 0.0

        error = value - center

        if abs(error) <= deadzone:
            return 0.0

        if error > 0:
            denominator = max_value - center - deadzone
            if denominator <= 0:
                return 0.0
            normalized = (error - deadzone) / denominator
        else:
            denominator = center - min_value - deadzone
            if denominator <= 0:
                return 0.0
            normalized = (error + deadzone) / denominator

        return self.clamp(normalized, -1.0, 1.0)

    def is_enabled(self, enable_us):
        if enable_us <= 0:
            return False

        if self.enable_active_high:
            return enable_us >= self.enable_threshold

        return enable_us <= self.enable_threshold

    def is_brake_requested(self, brake_us):
        if brake_us <= 0:
            return False

        if self.brake_active_high:
            return brake_us >= self.brake_threshold

        return brake_us <= self.brake_threshold

    def check_serial_timeout(self):
        if self.last_serial_time is None:
            self.publish_zero()
            if self.brake_on_signal_loss:
                self.publish_brake(True)
            return

        now = self.get_clock().now()
        age = (now - self.last_serial_time).nanoseconds * 1e-9

        if age > self.serial_timeout_sec:
            self.publish_zero()
            if self.brake_on_signal_loss:
                self.publish_brake(True)

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

    def publish_zero(self):
        self.cmd_pub.publish(Twist())

    def publish_brake(self, enabled):
        if self.last_brake_state == enabled:
            return

        msg = Bool()
        msg.data = bool(enabled)
        self.brake_pub.publish(msg)

        if enabled:
            self.get_logger().warn('RF brake ON')
        else:
            self.get_logger().info('RF brake OFF')

        self.last_brake_state = enabled

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)
    node = RfArduinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.publish_brake(True)

        if node.serial_port is not None:
            node.serial_port.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()