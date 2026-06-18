# pip install pyserial
import serial
import time

PORT = "/dev/ttyUSB0" # Windows 예: COM3 / Linux 예: /dev/ttyUSB0
BAUDRATE = 19200

# MDROBOT ID 1 -> 2 변경 패킷
packet = bytes([183, 184, 254, 133, 2, 170, 2, 96])

with serial.Serial(PORT, BAUDRATE, bytesize=8, parity="N", stopbits=1, timeout=0.5) as ser:
    time.sleep(0.2)
    ser.write(packet)
    ser.flush()

    # 응답이 있을 수도/없을 수도 있으니 일단 읽어봄
    rx = ser.read(64)
    print("TX:", list(packet))
    print("RX:", list(rx))