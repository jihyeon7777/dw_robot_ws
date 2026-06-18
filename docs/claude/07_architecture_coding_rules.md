# Architecture and Coding Rules

## 권장 제어 아키텍처

앞으로는 메카넘 4륜 로봇 기준으로 다음 구조를 권장한다.

```text
RF / Keyboard / Auto command
        ↓
input mode manager
        ↓
cmd_vel mux
        ↓
cmd_vel safety limiter
        ↓
mecanum controller
        ↓
4-wheel motor command
        ↓
MDROBOT PNT50 motor driver x 2
```

권장 토픽 흐름:

```text
/cmd_vel_rf
/cmd_vel_keyboard
/cmd_vel_auto
        ↓
/cmd_vel_raw
        ↓
/cmd_vel_safe
        ↓
/wheel_cmd
        ↓
motor driver nodes
```

`/cmd_vel_safe`까지는 일반적인 `geometry_msgs/Twist`를 사용할 수 있다.

## 안전 처리 원칙

속도 제한, 가속도 제한, timeout 처리는 한 곳에서 명확히 관리하는 것이 좋다.

기존 코드에는 여러 노드가 각각 timeout을 가지고 있을 수 있다. 이 경우 로봇이 안 움직이거나 갑자기 멈출 때 원인 파악이 어려워진다.

현재 구조에서는 timeout / 정지 책임을 다음 두 계층으로만 분리한다.

```text
cmd_vel_safety_node   애플리케이션 안전의 단일 주인
                      - 입력(/cmd_vel_raw) timeout 시 0으로 부드럽게 감속
                      - 속도 / 가속도 제한
pnt50_driver_node     하드웨어 failsafe의 단일 주인
                      - /wheel_cmd 스트림이 끊기면 모터 정지
                      - 전기 브레이크(PID 175) 실행
base_controller_node  topic 모드에서는 순수 Twist -> /wheel_cmd 변환기
                      - 자체 timeout / brake 판단을 하지 않는다
                      - serial 모드(아두이노 직접 구동)일 때만 자체 정지/브레이크 로직 사용
```

즉 "입력 끊김"은 `cmd_vel_safety_node`가, "다운스트림 노드 사망"은 `pnt50_driver_node` failsafe가 책임진다. 같은 timeout 로직을 여러 노드에 중복으로 두지 않는다.

권장 의미 구분:

```text
zero cmd      속도 명령 0, 일반 정지
brake cmd     전기 브레이크 또는 강한 정지 요청
estop         긴급 정지, 수동 해제 전까지 재출발 금지
timeout stop  입력 신호 끊김으로 인한 자동 정지
```

권장 안전 우선순위:

```text
1. estop
2. RF signal lost / invalid input
3. cmd timeout
4. speed / acceleration limit
5. mode-specific command
```

## Launch 파일 작성 원칙

launch 파일은 실제 하드웨어 조합별로 명확히 나눈다.

권장 launch 구성:

```text
rf_manual.launch.py
  RF 조종기 기반 수동 조종

keyboard_manual.launch.py
  키보드 기반 테스트 조종

mecanum_driver.launch.py
  메카넘 컨트롤러 + 모터 드라이버

camera_plate.launch.py
  카메라 + 번호판 인식

robot_bringup.launch.py
  실제 로봇 전체 bringup

rviz_odom.launch.py
  RViz + robot_state_publisher + odometry 확인
```

launch 파일에 모든 파라미터를 직접 적지 말고, 가능한 한 `config/*.yaml`에 둔다.

## 설정 파일 원칙

로봇 치수, 속도 제한, 포트, frame_id, 모터 방향, 드라이버 slave ID 등은 코드에 하드코딩하지 않는다.

권장 설정 항목:

```yaml
robot:
  wheel_radius: 0.0625
  lx: 0.2225
  ly: 0.2625
  lx_plus_ly: 0.485

motor:
  port: /dev/ttyUSB0
  baudrate: 19200
  slave_id: 1          # front driver
  rear_slave_id: 2     # rear driver

rf_controller:
  arduino_board: Arduino Uno R4 WiFi
  input_type: PWM
  serial_baudrate: 115200
  ch1: throttle
  ch2: yaw
  ch4: estop
  ch7: mode_select

mode_select:
  source_channel: CH7
  pwm_direction: high_to_low
  step_1: MANUAL
  step_2: MECANUM
  step_3: AUTO
  thresholds: TODO

camera:
  resolution_width: 320   # 현재. 인식 안정화 후 상향 예정 (목표 1920x1080)
  resolution_height: 240
  fps: 5
  xyz: [0.2625, 0.0, 0.1]
  rpy: [0.0, 0.0, 0.0]

frames:
  base_frame: base_link
  odom_frame: odom
  imu_frame: imu_link
  camera_frame: camera_link
```

## 코딩 원칙

1. 기존 구조를 먼저 확인한다.
2. 현재 코드가 차동 구동 기준인지, 메카넘 4륜 기준인지 구분한다.
3. 2바퀴 기준 변수명 `left/right`를 4바퀴 제어에 그대로 재사용하지 않는다.
4. 4바퀴 배열 순서는 반드시 문서와 코드 주석에 명시한다.
5. 하드웨어 포트, slave ID, 바퀴 방향, 치수는 파라미터화한다.
6. ROS 노드 안에 복잡한 수식을 직접 묶어두지 말고, 가능한 한 순수 함수로 분리한다.
7. 실제 하드웨어를 움직이는 코드는 dry-run 또는 fake mode를 둘 수 있으면 둔다.
8. 안전 제한 없이 모터 명령을 바로 내보내는 코드를 만들지 않는다.
9. 기존 launch 파일을 무작정 덮어쓰지 말고, 새 목적에 맞는 launch 파일을 분리한다.
10. 수정 후에는 최소한 빌드와 린트 테스트를 실행한다.
11. `CH4 estop`은 모든 모드와 모든 명령보다 우선하도록 작성한다.
12. `CH7` 모드 변경 시 이전 모드 명령이 남지 않도록 속도 명령을 초기화한다.
13. `AUTO` 모드는 실패 조건과 timeout 조건 없이 모터를 계속 움직이게 만들지 않는다.

## 금지할 것

- 실제 하드웨어 포트나 slave ID를 근거 없이 추정하지 말 것
- 차동 구동 공식을 메카넘 로봇에 그대로 적용하지 말 것
- 2바퀴 기준 `/wheel_cmd` 구조를 4바퀴 제어에 조용히 재사용하지 말 것
- 안전 제한을 제거하지 말 것
- `CH4 estop`보다 모드 명령을 우선하지 말 것
- `AUTO` 모드를 timeout 또는 실패 처리 없이 계속 진행시키지 말 것
- TODO 값을 임의로 확정값처럼 작성하지 말 것