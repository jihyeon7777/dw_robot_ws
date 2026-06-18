# MDROBOT PNT50 Motor Driver

## 드라이버 구성

로봇은 MDROBOT 계열 PNT50 드라이버를 사용한다.

| 항목 | 값 |
|---|---:|
| 드라이버 모델명 | PNT50 |
| 드라이버 개수 | 2 |
| 전체 채널 수 | 2채널 x 2대 |
| RS485 연결 | USB-RS485 1개 공유 bus |
| serial port | `/dev/ttyUSB0` |
| baudrate | 19200 |
| front driver slave ID | 1 |
| rear driver slave ID | 2 |

## 드라이버 / 바퀴 매핑

기본 해석:

```text
front driver slave ID 1 → front_left, front_right
rear driver slave ID 2  → rear_left, rear_right
```

권장 배열 순서:

```text
[front_left, front_right, rear_left, rear_right]
```

실제 채널 매핑은 반드시 하드웨어 테스트로 확인한다.

## Modbus / RS485 주의사항

- USB-RS485 하나의 bus에 PNT50 드라이버 2대가 연결된다.
- 각 드라이버는 slave ID로 구분한다.
- front driver와 rear driver의 slave ID를 혼동하면 안 된다.
- serial port와 baudrate는 코드에 하드코딩하지 말고 YAML 파라미터로 관리한다.
- 실제 모터 방향은 장착 방향에 따라 달라질 수 있으므로 각 바퀴별 direction multiplier를 파라미터로 둔다.

## 권장 파라미터 예시

```yaml
motor:
  port: /dev/ttyUSB0
  baudrate: 19200

  slave_id: 1          # front driver
  rear_slave_id: 2     # rear driver

  wheel_order:
    - front_left
    - front_right
    - rear_left
    - rear_right

  front_left_direction: 1.0
  front_right_direction: 1.0
  rear_left_direction: 1.0
  rear_right_direction: 1.0
```

## 메카넘 속도 변환

입력:

```text
vx: 전후 속도
vy: 좌우 속도
wz: yaw 회전 속도
```

기본 식:

```text
front_left  = (vx - vy - (lx + ly) * wz) / r
front_right = (vx + vy + (lx + ly) * wz) / r
rear_left   = (vx + vy - (lx + ly) * wz) / r
rear_right  = (vx - vy + (lx + ly) * wz) / r
```

단, 실제 모터 방향, 바퀴 장착 방향, 드라이버 채널 방향에 따라 부호가 달라질 수 있다.

## 출력 단위

구현 시 출력 단위를 명확히 한다.

가능한 단위:

```text
rad/s
RPM
normalized [-1.0, 1.0]
```

권장 구조:

```text
Twist
  ↓
mecanum kinematics
  ↓
wheel angular velocity
  ↓
RPM conversion
  ↓
motor driver command
```

정규화는 안전 제한이 아니다. 안전 제한은 별도의 safety node 또는 `/cmd_vel_safe` 단계에서 처리한다.

## 확인할 기존 파일

```text
src/dw_robot_base/dw_robot_base/base_controller_node.py
src/dw_robot_base/dw_robot_base/pnt50_driver_node.py
src/dw_robot_base/dw_robot_base/pnt50_modbus.py
src/dw_robot_base/config/*.yaml
src/dw_robot_base/launch/*.launch.py
```

## TODO

```text
TODO: PNT50 register map 확인
TODO: 속도 명령 register 확인
TODO: brake 관련 PID/register 확인
TODO: front/rear driver 각각의 channel mapping 확인
TODO: CRC16 계산부 확인
TODO: pnt50_modbus.py와 pnt50_driver_node.py 중복 구현 여부 확인
TODO: 각 바퀴 direction multiplier 실주행 테스트로 확정
```