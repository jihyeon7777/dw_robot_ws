#!/usr/bin/env python3
"""Minimal Modbus RTU helper for MDROBOT/PNT50 motor controllers.

This module intentionally has no ROS dependencies so it can be reused in
other projects or tested from a normal Python script.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

import serial


class ModbusError(RuntimeError):
    pass


class ModbusCRCError(ModbusError):
    pass


class ModbusTimeoutError(ModbusError):
    pass


class ModbusResponseError(ModbusError):
    pass


# Frequently used PID values from the MDROBOT Modbus document.
PID_VERSION = 1
PID_COMMAND = 10
PID_ALARM_RESET = 12
PID_INV_SIGN_CMD = 16
PID_INV_SIGN_CMD2 = 18
PID_USE_LIMIT_SW = 17
PID_USE_LIMIT_SW2 = 29
PID_CTRL_STATUS = 34
PID_CTRL_STATUS2 = 39
PID_VEL_CMD = 130
PID_VEL_CMD2 = 131
PID_INT_RPM_DATA = 138
PID_TQ_DATA = 139
PID_VOLT_IN = 143
PID_RETURN_TYPE = 149
PID_PNT_VEL_CMD = 207
PID_PNT_OPEN_VEL_CMD = 208
PID_PNT_TQ_CMD = 209
PID_PNT_MAIN_DATA = 210
PID_PNT_MONITOR = 216


@dataclass
class Pnt50Status:
    motor1_rpm: int = 0
    motor2_rpm: int = 0
    status1: int = 0
    status2: int = 0


def crc16_modbus(data: bytes) -> int:
    """Return Modbus RTU CRC16 as an integer.

    The low CRC byte is transmitted first in Modbus RTU.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame_without_crc: bytes) -> bytes:
    crc = crc16_modbus(frame_without_crc)
    return frame_without_crc + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def check_crc(frame: bytes) -> None:
    if len(frame) < 3:
        raise ModbusResponseError(f"Frame too short: {frame.hex(' ')}")
    received = frame[-2] | (frame[-1] << 8)
    calculated = crc16_modbus(frame[:-2])
    if received != calculated:
        raise ModbusCRCError(
            f"CRC mismatch: received=0x{received:04X}, calculated=0x{calculated:04X}, frame={frame.hex(' ')}"
        )


def int16_to_u16(value: int) -> int:
    value = int(value)
    if value < -32768 or value > 32767:
        raise ValueError(f"int16 out of range: {value}")
    return value & 0xFFFF


def u16_to_int16(value: int) -> int:
    value &= 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def word_to_hi_lo(value: int) -> tuple[int, int]:
    value &= 0xFFFF
    return (value >> 8) & 0xFF, value & 0xFF


class Pnt50ModbusClient:
    """Small blocking Modbus RTU client for PNT50/MDROBOT controllers."""

    def __init__(
        self,
        port: str,
        baudrate: int = 19200,
        slave_id: int = 1,
        timeout: float = 0.08,
        inter_frame_delay: float = 0.02,
    ) -> None:
        self.port_name = port
        self.baudrate = int(baudrate)
        self.slave_id = int(slave_id)
        self.timeout = float(timeout)
        self.inter_frame_delay = float(inter_frame_delay)
        self.serial: Optional[serial.Serial] = None

    def open(self) -> None:
        if self.serial is not None and self.serial.is_open:
            return
        self.serial = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        time.sleep(0.05)

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _ensure_open(self) -> serial.Serial:
        if self.serial is None or not self.serial.is_open:
            self.open()
        assert self.serial is not None
        return self.serial

    def _transaction(self, request: bytes, expected_len: int) -> bytes:
        ser = self._ensure_open()
        ser.reset_input_buffer()
        ser.write(request)
        ser.flush()
        response = ser.read(expected_len)
        time.sleep(self.inter_frame_delay)

        if len(response) != expected_len:
            raise ModbusTimeoutError(
                f"Expected {expected_len} bytes, got {len(response)} bytes. request={request.hex(' ')}, response={response.hex(' ')}"
            )
        check_crc(response)
        if response[0] != self.slave_id:
            raise ModbusResponseError(f"Unexpected slave id: {response[0]} != {self.slave_id}")
        if response[1] & 0x80:
            raise ModbusResponseError(f"Modbus exception response: {response.hex(' ')}")
        return response

    def read_words(self, pid: int, count: int = 1) -> List[int]:
        """Read holding registers using function 3."""
        if count <= 0 or count > 125:
            raise ValueError("count must be 1..125")
        request_wo_crc = bytes([
            self.slave_id,
            0x03,
            (pid >> 8) & 0xFF,
            pid & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ])
        request = append_crc(request_wo_crc)
        expected_len = 5 + 2 * count
        response = self._transaction(request, expected_len)
        if response[1] != 0x03 or response[2] != 2 * count:
            raise ModbusResponseError(f"Invalid read response: {response.hex(' ')}")
        words: List[int] = []
        for i in range(count):
            hi = response[3 + 2 * i]
            lo = response[4 + 2 * i]
            words.append((hi << 8) | lo)
        return words

    def write_word(self, pid: int, value: int, signed: bool = True) -> None:
        """Write one 16-bit register using function 6."""
        raw = int16_to_u16(value) if signed else int(value) & 0xFFFF
        hi, lo = word_to_hi_lo(raw)
        request_wo_crc = bytes([
            self.slave_id,
            0x06,
            (pid >> 8) & 0xFF,
            pid & 0xFF,
            hi,
            lo,
        ])
        request = append_crc(request_wo_crc)
        response = self._transaction(request, 8)
        if response[:6] != request_wo_crc:
            raise ModbusResponseError(f"Write echo mismatch: req={request.hex(' ')}, res={response.hex(' ')}")

    def write_words(self, pid: int, values: Iterable[int], signed: bool = True) -> None:
        """Write multiple 16-bit registers using function 16."""
        raw_words: List[int] = []
        for value in values:
            raw_words.append(int16_to_u16(value) if signed else int(value) & 0xFFFF)
        count = len(raw_words)
        if count <= 0 or count > 123:
            raise ValueError("values length must be 1..123")

        payload = bytearray()
        for word in raw_words:
            hi, lo = word_to_hi_lo(word)
            payload.extend([hi, lo])

        request_wo_crc = bytes([
            self.slave_id,
            0x10,
            (pid >> 8) & 0xFF,
            pid & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
            len(payload),
        ]) + bytes(payload)
        request = append_crc(request_wo_crc)
        response = self._transaction(request, 8)
        expected_header = bytes([
            self.slave_id,
            0x10,
            (pid >> 8) & 0xFF,
            pid & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ])
        if response[:6] != expected_header:
            raise ModbusResponseError(f"Write multiple response mismatch: req={request.hex(' ')}, res={response.hex(' ')}")

    def set_dual_rpm(self, motor1_rpm: int, motor2_rpm: int, mode: str = "pnt_vel") -> None:
        """Set target speeds for both motors.

        mode='pnt_vel' writes PID_PNT_VEL_CMD(207) with function 16.
        mode='separate' writes PID_VEL_CMD(130) and PID_VEL_CMD2(131) with function 6.
        """
        motor1_rpm = int(motor1_rpm)
        motor2_rpm = int(motor2_rpm)
        if mode == "pnt_vel":
            self.write_words(PID_PNT_VEL_CMD, [motor1_rpm, motor2_rpm], signed=True)
        elif mode == "separate":
            self.write_word(PID_VEL_CMD, motor1_rpm, signed=True)
            self.write_word(PID_VEL_CMD2, motor2_rpm, signed=True)
        else:
            raise ValueError(f"Unknown command mode: {mode}")

    def stop(self, mode: str = "pnt_vel") -> None:
        self.set_dual_rpm(0, 0, mode=mode)
