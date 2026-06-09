#!/usr/bin/env python3
import math
import threading
import time

import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped


G_TO_MPS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0


def i16(lo, hi):
    value = (hi << 8) | lo
    if value >= 32768:
        value -= 65536
    return value


def checksum_ok(frame):
    return (sum(frame[:10]) & 0xFF) == frame[10]


def euler_to_quaternion(roll, pitch, yaw):
    """
    roll, pitch, yaw: radians
    return: x, y, z, w
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


class HWT901BNode(Node):
    def __init__(self):
        super().__init__("hwt901b_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 50.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 10)
        self.rpy_pub = self.create_publisher(Vector3Stamped, "/imu/rpy", 10)

        self.lock = threading.Lock()
        self.running = True
        self.buf = bytearray()

        self.latest_acc_g = [0.0, 0.0, 0.0]
        self.latest_gyro_dps = [0.0, 0.0, 0.0]
        self.latest_rpy_deg = [0.0, 0.0, 0.0]
        self.latest_mag_raw = [0, 0, 0]
        self.have_acc = False
        self.have_gyro = False
        self.have_angle = False
        self.have_mag = False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        self.reader_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.reader_thread.start()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_data)

        self.get_logger().info(
            f"HWT901B node started. port={self.port}, baudrate={self.baudrate}, frame_id={self.frame_id}"
        )

    def serial_read_loop(self):
        while self.running and rclpy.ok():
            try:
                data = self.ser.read(256)
                if data:
                    self.buf.extend(data)
                    self.extract_frames()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    def extract_frames(self):
        while len(self.buf) >= 11:
            if self.buf[0] != 0x55:
                del self.buf[0]
                continue

            packet_type = self.buf[1]

            if packet_type not in (0x51, 0x52, 0x53, 0x54, 0x59):
                del self.buf[0]
                continue

            frame = bytes(self.buf[:11])

            if checksum_ok(frame):
                self.parse_frame(frame)
                del self.buf[:11]
            else:
                del self.buf[0]

    def parse_frame(self, frame):
        packet_type = frame[1]

        with self.lock:
            if packet_type == 0x51:
                ax = i16(frame[2], frame[3]) / 32768.0 * 16.0
                ay = i16(frame[4], frame[5]) / 32768.0 * 16.0
                az = i16(frame[6], frame[7]) / 32768.0 * 16.0

                self.latest_acc_g = [ax, ay, az]
                self.have_acc = True

            elif packet_type == 0x52:
                wx = i16(frame[2], frame[3]) / 32768.0 * 2000.0
                wy = i16(frame[4], frame[5]) / 32768.0 * 2000.0
                wz = i16(frame[6], frame[7]) / 32768.0 * 2000.0

                self.latest_gyro_dps = [wx, wy, wz]
                self.have_gyro = True

            elif packet_type == 0x53:
                roll = i16(frame[2], frame[3]) / 32768.0 * 180.0
                pitch = i16(frame[4], frame[5]) / 32768.0 * 180.0
                yaw = i16(frame[6], frame[7]) / 32768.0 * 180.0

                self.latest_rpy_deg = [roll, pitch, yaw]
                self.have_angle = True

            elif packet_type == 0x54:
                mx = i16(frame[2], frame[3])
                my = i16(frame[4], frame[5])
                mz = i16(frame[6], frame[7])

                self.latest_mag_raw = [mx, my, mz]
                self.have_mag = True

    def publish_data(self):
        with self.lock:
            acc_g = self.latest_acc_g[:]
            gyro_dps = self.latest_gyro_dps[:]
            rpy_deg = self.latest_rpy_deg[:]
            mag_raw = self.latest_mag_raw[:]
            have_angle = self.have_angle
            have_acc = self.have_acc
            have_gyro = self.have_gyro
            have_mag = self.have_mag

        now = self.get_clock().now().to_msg()

        if have_acc or have_gyro or have_angle:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            roll_rad = rpy_deg[0] * DEG_TO_RAD
            pitch_rad = rpy_deg[1] * DEG_TO_RAD
            yaw_rad = rpy_deg[2] * DEG_TO_RAD

            if have_angle:
                qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
            else:
                imu_msg.orientation_covariance[0] = -1.0

            if have_gyro:
                imu_msg.angular_velocity.x = gyro_dps[0] * DEG_TO_RAD
                imu_msg.angular_velocity.y = gyro_dps[1] * DEG_TO_RAD
                imu_msg.angular_velocity.z = gyro_dps[2] * DEG_TO_RAD
            else:
                imu_msg.angular_velocity_covariance[0] = -1.0

            if have_acc:
                imu_msg.linear_acceleration.x = acc_g[0] * G_TO_MPS2
                imu_msg.linear_acceleration.y = acc_g[1] * G_TO_MPS2
                imu_msg.linear_acceleration.z = acc_g[2] * G_TO_MPS2
            else:
                imu_msg.linear_acceleration_covariance[0] = -1.0

            # 임시 covariance. 나중에 EKF 튜닝할 때 수정.
            imu_msg.orientation_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.10,
            ]
            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            imu_msg.linear_acceleration_covariance = [
                0.10, 0.0, 0.0,
                0.0, 0.10, 0.0,
                0.0, 0.0, 0.10,
            ]

            self.imu_pub.publish(imu_msg)

            rpy_msg = Vector3Stamped()
            rpy_msg.header.stamp = now
            rpy_msg.header.frame_id = self.frame_id
            rpy_msg.vector.x = rpy_deg[0]
            rpy_msg.vector.y = rpy_deg[1]
            rpy_msg.vector.z = rpy_deg[2]
            self.rpy_pub.publish(rpy_msg)

        if have_mag:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id

            # 현재는 WIT raw 자기장 값이다.
            # 정확한 Tesla 단위 변환은 센서별 스케일 확인 후 수정한다.
            mag_msg.magnetic_field.x = float(mag_raw[0])
            mag_msg.magnetic_field.y = float(mag_raw[1])
            mag_msg.magnetic_field.z = float(mag_raw[2])

            mag_msg.magnetic_field_covariance = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ]

            self.mag_pub.publish(mag_msg)

    def destroy_node(self):
        self.running = False
        try:
            if hasattr(self, "ser") and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HWT901BNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()