#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool


class Pnt50DriverNode(Node):
    def __init__(self):
        super().__init__('pnt50_driver_node')

        self.declare_parameter('wheel_cmd_topic', '/wheel_cmd')
        self.declare_parameter('brake_topic', '/brake_cmd')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 19200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('rear_slave_id', 2)

        self.declare_parameter('max_rpm', 500)

        self.declare_parameter('left_is_motor1', True)
        self.declare_parameter('left_sign', 1)
        self.declare_parameter('right_sign', 1)
        self.declare_parameter('rear_sign', -1)

        # 'differential' (현재 기본) 또는 'mecanum'.
        self.declare_parameter('drive_mode', 'differential')

        # 메카넘 바퀴별 방향 부호. 순서 [FL, FR, RL, RR].
        # Step 2 하드웨어 테스트(2026-06-18)에서 전진 기준으로 확정한 값.
        self.declare_parameter('front_left_sign', -1)
        self.declare_parameter('front_right_sign', 1)
        self.declare_parameter('rear_left_sign', -1)
        self.declare_parameter('rear_right_sign', 1)

        # 드라이버 채널(motor1/motor2)이 좌/우 바퀴에 어떻게 붙었는지.
        # 하드웨어 테스트 결과: 앞 드라이버는 motor1=FL, motor2=FR (정상),
        # 뒤 드라이버는 motor1=RR, motor2=RL 로 좌우가 바뀌어 있음.
        self.declare_parameter('swap_front_channels', False)
        self.declare_parameter('swap_rear_channels', True)

        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('send_rate', 20.0)

        self.declare_parameter('use_dual_pid207', True)

        self.wheel_cmd_topic = self.get_parameter('wheel_cmd_topic').value
        self.brake_topic = self.get_parameter('brake_topic').value

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.slave_id = int(self.get_parameter('slave_id').value)
        self.rear_slave_id = int(self.get_parameter('rear_slave_id').value)

        self.max_rpm = int(self.get_parameter('max_rpm').value)

        self.left_is_motor1 = bool(self.get_parameter('left_is_motor1').value)
        self.left_sign = int(self.get_parameter('left_sign').value)
        self.right_sign = int(self.get_parameter('right_sign').value)
        self.rear_sign = int(self.get_parameter('rear_sign').value)

        self.drive_mode = str(self.get_parameter('drive_mode').value)
        self.front_left_sign = int(self.get_parameter('front_left_sign').value)
        self.front_right_sign = int(self.get_parameter('front_right_sign').value)
        self.rear_left_sign = int(self.get_parameter('rear_left_sign').value)
        self.rear_right_sign = int(self.get_parameter('rear_right_sign').value)

        self.swap_front_channels = bool(self.get_parameter('swap_front_channels').value)
        self.swap_rear_channels = bool(self.get_parameter('swap_rear_channels').value)

        # Front driver (2 motors) + rear driver (2 motors) on the same RS485 bus.
        # Each PNT50 controls a pair of motors, so 2 drivers = 4 wheels.
        self.slave_ids = [self.slave_id, self.rear_slave_id]

        # Per-driver direction. Rear driver is wired opposite the front one.
        self.slave_sign = {
            self.slave_id: 1,
            self.rear_slave_id: self.rear_sign,
        }

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.send_rate = float(self.get_parameter('send_rate').value)

        self.use_dual_pid207 = bool(self.get_parameter('use_dual_pid207').value)

        self.serial_port = None

        self.last_left_norm = 0.0
        self.last_right_norm = 0.0

        # 메카넘 4바퀴 정규화 명령. 순서 [FL, FR, RL, RR].
        self.last_fl_norm = 0.0
        self.last_fr_norm = 0.0
        self.last_rl_norm = 0.0
        self.last_rr_norm = 0.0

        self.last_cmd_time = None

        self.brake_requested = False
        self.brake_command_sent = False
        # 드라이버는 전원 투입 시 전기브레이크가 걸린 채 시작될 수 있다.
        # 브레이크가 아닐 때 한 번 명시적으로 해제(PID175=0)를 보낸다.
        self.brake_released = False

        self.target_rpm_pub = self.create_publisher(
            Int16MultiArray,
            '/pnt50/target_rpm',
            10
        )

        self.comm_ok_pub = self.create_publisher(
            Bool,
            '/pnt50/comm_ok',
            10
        )

        self.wheel_cmd_sub = self.create_subscription(
            Float32MultiArray,
            self.wheel_cmd_topic,
            self.wheel_cmd_callback,
            10
        )

        self.brake_sub = self.create_subscription(
            Bool,
            self.brake_topic,
            self.brake_callback,
            10
        )

        self.open_serial()

        self.timer = self.create_timer(
            1.0 / self.send_rate,
            self.timer_callback
        )

        self.get_logger().info('pnt50_driver_node started')
        self.get_logger().info(f'wheel_cmd_topic: {self.wheel_cmd_topic}')
        self.get_logger().info(f'brake_topic: {self.brake_topic}')
        self.get_logger().info(f'port: {self.port}')
        self.get_logger().info(f'baudrate: {self.baudrate}')
        self.get_logger().info(f'slave_ids: {self.slave_ids}')
        self.get_logger().info(f'drive_mode: {self.drive_mode}')
        self.get_logger().info(f'max_rpm: {self.max_rpm}')
        self.get_logger().info(f'use_dual_pid207: {self.use_dual_pid207}')

    def open_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=0.05,
                write_timeout=0.05
            )
            self.get_logger().info(f'Serial opened: {self.port}')

        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')

    def wheel_cmd_callback(self, msg):
        if self.drive_mode == 'mecanum':
            self.handle_mecanum_wheel_cmd(msg)
            return

        # differential: /wheel_cmd = [left_vel, right_vel, left_norm, right_norm]
        if len(msg.data) < 4:
            self.get_logger().warn('Invalid /wheel_cmd length')
            return

        left_norm = float(msg.data[2])
        right_norm = float(msg.data[3])

        if not math.isfinite(left_norm) or not math.isfinite(right_norm):
            self.get_logger().warn('Invalid wheel command ignored')
            return

        self.last_left_norm = self.clamp(left_norm, -1.0, 1.0)
        self.last_right_norm = self.clamp(right_norm, -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    def handle_mecanum_wheel_cmd(self, msg):
        # mecanum: /wheel_cmd = [FL_norm, FR_norm, RL_norm, RR_norm]
        if len(msg.data) < 4:
            self.get_logger().warn('Invalid mecanum /wheel_cmd length')
            return

        fl = float(msg.data[0])
        fr = float(msg.data[1])
        rl = float(msg.data[2])
        rr = float(msg.data[3])

        if not all(math.isfinite(v) for v in (fl, fr, rl, rr)):
            self.get_logger().warn('Invalid wheel command ignored')
            return

        self.last_fl_norm = self.clamp(fl, -1.0, 1.0)
        self.last_fr_norm = self.clamp(fr, -1.0, 1.0)
        self.last_rl_norm = self.clamp(rl, -1.0, 1.0)
        self.last_rr_norm = self.clamp(rr, -1.0, 1.0)
        self.last_cmd_time = self.get_clock().now()

    def brake_callback(self, msg):
        previous_state = self.brake_requested
        self.brake_requested = bool(msg.data)

        if self.brake_requested and not previous_state:
            self.brake_command_sent = False
            self.brake_released = False
            self.get_logger().warn('Brake requested')

        elif not self.brake_requested and previous_state:
            self.brake_command_sent = False
            self.get_logger().info('Brake released')

    def timer_callback(self):
        driver_cmds = self.get_driver_commands()

        self.publish_target_rpm(driver_cmds)

        if self.serial_port is None:
            self.publish_comm_ok(False)
            return

        if self.brake_requested:
            zero_ok = self.send_zero_rpm_command()

            if not self.brake_command_sent:
                brake_ok = True
                for sid in self.slave_ids:
                    brake_ok = self.write_pnt_brake_pid175(
                        sid,
                        motor1_brake=True,
                        motor2_brake=True
                    ) and brake_ok

                if brake_ok:
                    self.brake_command_sent = True
                    self.get_logger().warn('PID 175 PNT electric brake sent')

                self.publish_comm_ok(zero_ok and brake_ok)

            else:
                self.publish_comm_ok(zero_ok)

            return

        # 브레이크 상태가 아니면 시작 시(또는 해제 직후) 한 번 전기브레이크를 푼다.
        if not self.brake_released:
            release_ok = True
            for sid in self.slave_ids:
                release_ok = self.write_pnt_brake_pid175(
                    sid,
                    motor1_brake=False,
                    motor2_brake=False
                ) and release_ok

            if release_ok:
                self.brake_released = True
                self.get_logger().info('PNT electric brake released')

        ok = True
        for sid, m1, m2 in driver_cmds:
            if self.use_dual_pid207:
                ok = self.write_dual_rpm_pid207(sid, m1, m2) and ok
            else:
                ok1 = self.write_single_word(sid, 130, m1)
                ok2 = self.write_single_word(sid, 131, m2)
                ok = ok1 and ok2 and ok

        self.publish_comm_ok(ok)

    def get_driver_commands(self):
        """
        드라이버별 (slave_id, motor1_rpm, motor2_rpm) 목록을 반환한다.

        mecanum: front driver(ID1) = (FL, FR), rear driver(ID2) = (RL, RR).
        differential: 기존 좌/우 명령에 드라이버별 slave_sign 적용.
        """
        if self.drive_mode == 'mecanum':
            if self.is_timed_out() or self.brake_requested:
                fl = fr = rl = rr = 0
            else:
                fl = int(self.last_fl_norm * self.max_rpm * self.front_left_sign)
                fr = int(self.last_fr_norm * self.max_rpm * self.front_right_sign)
                rl = int(self.last_rl_norm * self.max_rpm * self.rear_left_sign)
                rr = int(self.last_rr_norm * self.max_rpm * self.rear_right_sign)

            # 드라이버 채널 순서: 기본 (왼쪽, 오른쪽) = (motor1, motor2).
            # 뒤 드라이버는 좌우가 반대로 결선돼 있어 swap_rear_channels=True.
            front = (fr, fl) if self.swap_front_channels else (fl, fr)
            rear = (rr, rl) if self.swap_rear_channels else (rl, rr)

            return [
                (self.slave_id,
                 self.clamp_int16(front[0]), self.clamp_int16(front[1])),
                (self.rear_slave_id,
                 self.clamp_int16(rear[0]), self.clamp_int16(rear[1])),
            ]

        motor1_rpm, motor2_rpm = self.get_target_rpms()

        cmds = []
        for sid in self.slave_ids:
            sign = self.slave_sign[sid]
            cmds.append((
                sid,
                self.clamp_int16(motor1_rpm * sign),
                self.clamp_int16(motor2_rpm * sign),
            ))
        return cmds

    def get_target_rpms(self):
        if self.is_timed_out() or self.brake_requested:
            left_rpm = 0
            right_rpm = 0
        else:
            left_rpm = int(self.last_left_norm * self.max_rpm * self.left_sign)
            right_rpm = int(self.last_right_norm * self.max_rpm * self.right_sign)

        if self.left_is_motor1:
            motor1_rpm = left_rpm
            motor2_rpm = right_rpm
        else:
            motor1_rpm = right_rpm
            motor2_rpm = left_rpm

        motor1_rpm = self.clamp_int16(motor1_rpm)
        motor2_rpm = self.clamp_int16(motor2_rpm)

        return motor1_rpm, motor2_rpm

    def is_timed_out(self):
        if self.last_cmd_time is None:
            return True

        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9

        return age > self.cmd_timeout

    def send_zero_rpm_command(self):
        ok = True
        for sid in self.slave_ids:
            if self.use_dual_pid207:
                ok = self.write_dual_rpm_pid207(sid, 0, 0) and ok
            else:
                ok1 = self.write_single_word(sid, 130, 0)
                ok2 = self.write_single_word(sid, 131, 0)
                ok = ok1 and ok2 and ok
        return ok

    def write_pnt_brake_pid175(self, slave_id, motor1_brake=True, motor2_brake=True):
        """
        PID 175: PID_PNT_BRAKE

        DL = 1: brake motor1
        DH = 1: brake motor2

        Both motors brake:
        DATA = 0x0101
        """

        pid = 175

        dl = 1 if motor1_brake else 0
        dh = 1 if motor2_brake else 0

        data = (dh << 8) | dl

        return self.write_single_word(slave_id, pid, data)

    def write_dual_rpm_pid207(self, slave_id, motor1_rpm, motor2_rpm):
        pid = 207
        quantity = 2
        byte_count = 4

        payload = bytearray()
        payload.append(slave_id)
        payload.append(0x10)
        payload += pid.to_bytes(2, byteorder='big', signed=False)
        payload += quantity.to_bytes(2, byteorder='big', signed=False)
        payload.append(byte_count)
        payload += int(motor1_rpm).to_bytes(2, byteorder='big', signed=True)
        payload += int(motor2_rpm).to_bytes(2, byteorder='big', signed=True)

        frame = self.add_crc(payload)

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(frame)
            response = self.serial_port.read(8)

            if len(response) < 8:
                return False

            expected_prefix = bytes([
                slave_id,
                0x10,
                0x00,
                0xCF,
                0x00,
                0x02
            ])

            if response[:6] != expected_prefix:
                return False

            return self.check_crc(response)

        except serial.SerialException as e:
            self.get_logger().error(f'Modbus serial error: {e}')
            self.serial_port = None
            return False

    def write_single_word(self, slave_id, pid, value):
        payload = bytearray()
        payload.append(slave_id)
        payload.append(0x06)
        payload += int(pid).to_bytes(2, byteorder='big', signed=False)
        payload += int(value).to_bytes(2, byteorder='big', signed=True)

        frame = self.add_crc(payload)

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(frame)
            response = self.serial_port.read(8)

            if len(response) < 8:
                return False

            if response[:6] != frame[:6]:
                return False

            return self.check_crc(response)

        except serial.SerialException as e:
            self.get_logger().error(f'Modbus serial error: {e}')
            self.serial_port = None
            return False

    def add_crc(self, payload):
        crc = self.modbus_crc16(payload)
        frame = bytearray(payload)
        frame += crc.to_bytes(2, byteorder='little')
        return bytes(frame)

    def check_crc(self, frame):
        if len(frame) < 3:
            return False

        data = frame[:-2]
        received_crc = int.from_bytes(frame[-2:], byteorder='little')
        calculated_crc = self.modbus_crc16(data)

        return received_crc == calculated_crc

    def modbus_crc16(self, data):
        crc = 0xFFFF

        for byte in data:
            crc ^= byte

            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1

        return crc & 0xFFFF

    def publish_target_rpm(self, driver_cmds):
        # 드라이버 순서대로 평탄화: [id1_m1, id1_m2, id2_m1, id2_m2]
        msg = Int16MultiArray()
        data = []
        for _, m1, m2 in driver_cmds:
            data.append(int(m1))
            data.append(int(m2))
        msg.data = data
        self.target_rpm_pub.publish(msg)

    def publish_comm_ok(self, ok):
        msg = Bool()
        msg.data = bool(ok)
        self.comm_ok_pub.publish(msg)

    def send_zero_rpm(self):
        if self.serial_port is None:
            return

        self.send_zero_rpm_command()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def clamp_int16(self, value):
        return int(max(-32768, min(32767, value)))


def main(args=None):
    rclpy.init(args=args)
    node = Pnt50DriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_rpm()

        if node.serial_port is not None:
            node.serial_port.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()