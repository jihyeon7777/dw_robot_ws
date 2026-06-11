#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import serial


class RcPnt50SerialNode(Node):
    def __init__(self):
        super().__init__('rc_pnt50_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('topic_name', '/rc_pnt50_state')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.topic_name = self.get_parameter('topic_name').value

        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.topic_name,
            10
        )

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.02
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(f'Opened serial port: {self.port}')
        self.get_logger().info(f'Baudrate: {self.baudrate}')
        self.get_logger().info(f'Publishing topic: {self.topic_name}')
        self.get_logger().info(
            'Expected CSV: throttle,steering,enable,brake,signal_ok,pulse1,pulse2,rpm1,rpm2'
        )

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not line:
            return

        parts = line.split(',')

        if len(parts) != 9:
            # Arduino가 처음 출력하는 헤더 줄은 여기서 자동으로 무시됨
            return

        try:
            throttle_us = float(parts[0])
            steering_us = float(parts[1])
            enable_us = float(parts[2])
            brake_us = float(parts[3])
            signal_ok = float(parts[4])
            pulse1 = float(parts[5])
            pulse2 = float(parts[6])
            rpm1 = float(parts[7])
            rpm2 = float(parts[8])
        except ValueError:
            return

        msg = Float32MultiArray()
        msg.data = [
            throttle_us,
            steering_us,
            enable_us,
            brake_us,
            signal_ok,
            pulse1,
            pulse2,
            rpm1,
            rpm2,
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RcPnt50SerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()