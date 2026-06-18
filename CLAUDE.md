# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 개요

이 저장소는 차동 구동(differential-drive) 로봇인 "DW robot"을 위한 ROS 2 (Jazzy) 워크스페이스다.
모든 패키지는 `ament_python` 빌드 타입을 사용한다. `src/` 아래에 4개의 패키지가 있다.

- `dw_robot_base` — 핵심 패키지. cmd_vel 중재/안전 처리, 모터 드라이버(시리얼 + Modbus RTU),
  오도메트리, 키보드/RF 텔레옵, USB 카메라 노드를 포함한다. 실제 작업은 거의 다 이 패키지에서 일어난다.
- `dw_robot_bringup` — 최상위 launch 편의 패키지(현재는 `sim_bringup.launch.py` 하나뿐).
- `dw_robot_description` — URDF/xacro 로봇 모델과 RViz 디스플레이 launch.
- `hwt901b_driver` — WitMotion HWT901B IMU용 독립 드라이버 노드(시리얼 프로토콜 파서).

최상위에 `motordriver.py`라는 별도 파일이 있는데, 어떤 ROS 패키지에도 속하지 않는다. MDROBOT 모터
컨트롤러의 slave ID를 변경하기 위해 raw Modbus 패킷을 한 번 보내는 일회용 스크립트이며, 다른 곳에서
import되지 않는다.

## 빌드 / 린트 / 테스트

워크스페이스 루트(`/home/jh/dw_robot_ws`)에서 표준 `colcon` 워크플로우를 사용한다.

```bash
colcon build --symlink-install      # 전체 패키지 빌드
colcon build --symlink-install --packages-select dw_robot_base   # 단일 패키지만 빌드
source install/setup.bash           # 노드/launch 파일 실행 전에 반드시 필요
```

각 패키지의 테스트는 ament 표준 보일러플레이트(`test_copyright.py`, `test_flake8.py`, `test_pep257.py`)이며
pytest/colcon으로 실행한다.

```bash
colcon test --packages-select dw_robot_base
colcon test-result --verbose
```

colcon 없이 테스트 파일 하나만 빠르게 실행하려면:

```bash
python3 -m pytest src/dw_robot_base/test/test_flake8.py
```

노드 로직 자체에 대한 단위 테스트는 없고, ament 린터(flake8, pep257, copyright 헤더 검사)만 존재한다.
`pnt50_modbus.py`는 단독으로 테스트하거나 다른 프로젝트에서 재사용할 수 있도록 의도적으로 ROS 의존성
없이 작성되어 있다.

## 아키텍처

### cmd_vel 파이프라인 (토픽 체인)

로봇의 이동 명령은 토픽으로만 연결된(직접 함수 호출이 아닌) 작고 단일 책임을 가진 노드들의 고정된
체인을 통해 흐른다. 각 노드가 상류/하류 노드의 특정 동작을 전제로 하고 있기 때문에, 노드 하나를
건드리기 전에 이 체인 전체를 이해해야 한다.

```
keyboard_brake_node  ─┐
                       ├─→ cmd_vel_mux_node ─→ cmd_vel_safety_node ─→ {fake_diff_drive_node | base_controller_node}
rf_arduino_node       ─┘   (/cmd_vel_raw)      (/cmd_vel_safe)
(/cmd_vel_keyboard,
 /cmd_vel_rf)
```

- **`cmd_vel_mux_node`**: `input_source` 파라미터(런타임에 `ros2 param set`으로 변경 가능)를 기준으로
  키보드/RF 입력 소스를 선택한다. 선택된 소스가 타임아웃되었거나 유효하지 않으면 0을 publish한다.
- **`cmd_vel_safety_node`**: 설정된 속도/가속도 한계로 클램핑하고, 타임아웃 시 명령을 0으로 만든다.
  속도/가속도 제한이 실제로 강제되는 곳은 여기 한 군데뿐이며, 하류의 바퀴별 클램핑은 안전 제한이
  아니라 단순 정규화에 불과하다.
- **`base_controller_node`**: `Twist`를 차동 구동 바퀴 속도로 변환하고, `[-1, 1]`로 정규화하여
  `/wheel_cmd`(`[left_vel, right_vel, left_norm, right_norm]`)를 publish한다. `output_mode: serial`일
  때는 보드로 단순 ASCII 시리얼 명령(`M,<l>,<r>`, `S`, `B,1`/`B,0`)도 직접 전송한다. 자체 브레이크
  상태 머신(`use_brake_protocol`, `brake_on_timeout`, `brake_on_zero_cmd`)을 가지고 있으며 이는
  `pnt50_driver_node`의 브레이크 처리와는 독립적이다.
- **`fake_diff_drive_node`**: 실제 베이스를 대신하는 시뮬레이션용 노드. `/cmd_vel_safe`를 직접
  적분하여 `/odom`, `/tf`, `/joint_states`를 만든다(`/wheel_cmd`는 거치지 않음). `*_fake*` launch
  파일에서 실제 모터 스택 대신 사용된다.
- **`pnt50_driver_node`**: `/wheel_cmd`를 받아 정규화된 바퀴 속도를 RPM으로 변환하고, 시리얼을 통해
  듀얼 MDROBOT/PNT50 모터 컨트롤러에 Modbus RTU(함수 코드 0x06/0x10, CRC16)로 통신한다. 모니터링용으로
  `/pnt50/target_rpm`, `/pnt50/comm_ok`를 publish한다. PID 175(전기 브레이크)를 이용한 자체 브레이크
  처리가 있으며 `base_controller_node`의 브레이크 프로토콜과는 별개다 — 이 둘은 같은 launch 파일
  조합에서 함께 쓰이도록 설계되어 있지 않다(아래 launch 파일 항목 참고).
- **`odom_from_pnt50_feedback_node`**: cmd_vel 체인과는 별도로 동작한다. 바퀴 틱 피드백
  (`/pnt50_feedback`, `[left_ticks, right_ticks]`)을 받아 기본적인 차동 구동 dead reckoning으로
  `/odom` + `/tf`를 만든다.

브레이크는 병렬적인 사이드 채널(`/brake_cmd`, `std_msgs/Bool`)로, `keyboard_brake_node`나
`rf_arduino_node`가 publish하고 `base_controller_node`와 `pnt50_driver_node`가 각각 독립적으로
구독해서 처리한다.

### Modbus / 모터 제어

`pnt50_modbus.py`는 MDROBOT/PNT50 컨트롤러용으로 독립적인(ROS에 의존하지 않는) Modbus RTU 클라이언트
(`Pnt50ModbusClient`)로, PID 상수, CRC16, 레지스터 읽기/쓰기 헬퍼를 제공한다. `pnt50_driver_node.py`는
이 클라이언트를 쓰지 않고 동일한 저수준 프레이밍/CRC 로직을 자체적으로 다시 구현하고 있다 — 두 곳이
통합되어 있지 않으므로, 프로토콜 관련 수정이 필요하면 양쪽 모두에 적용해야 할 수 있다.

### 입력 소스

- `rf_arduino_node`는 아두이노로부터 시리얼로 9개 필드 CSV 한 줄을 읽는다
  (`throttle,steering,enable,brake,signal_ok,pulse1,pulse2,rpm1,rpm2`). RC PWM 펄스를 `Twist`와
  브레이크 상태, 모터 피드백으로 변환한다. 이전 아두이노 펌웨어와의 하위 호환을 위해 4개/5개 필드
  형식도 받아들인다.
- `keyboard_brake_node`는 터미널 raw 모드 텔레옵이다(`i,j,k,l,u,o,m,.`로 이동, `k`=브레이크,
  `space`=브레이크 없이 정지, `q/z/w/x`로 속도 스텝 조정).
- `rc_pnt50_serial_node`는 더 단순한/오래된 버전으로, 아두이노의 raw CSV를 그냥 `Float32MultiArray`로
  재publish만 한다(cmd_vel 체인에는 연결되어 있지 않으며, `rf_arduino_node`보다 먼저 만들어진 것으로
  보인다).

### Launch 파일 (`dw_robot_base/launch/`)

실제 사용 중인 하드웨어 조합에 맞는 launch 파일을 골라야 한다. 각 파일은 특정 노드 조합을 연결하며,
서로 단순한 상위/하위 집합 관계가 아니다.

- `keyboard_fake.launch.py` / `rf_fake.launch.py`: 텔레옵 → mux → safety → **fake** diff drive
  (실제 모터 없는 시뮬레이션) + RViz.
- `keyboard_real.launch.py` / `keyboard_pnt50.launch.py` / `rf_pnt50.launch.py`: 텔레옵 → mux →
  safety → `base_controller_node`(`_pnt50` 변형에서는 `pnt50_driver_node`도 포함) — **실제** 하드웨어
  대상.
- `rf_pnt50_rviz.launch.py`: 별도의 더 좁은 경로 — robot_state_publisher + `odom_from_pnt50_feedback_node`
  + RViz만 실행하며, 전체 텔레옵/safety/제어 체인 없이 실제 바퀴 피드백 기반 오도메트리만 시각화한다.
- `pnt50_driver.launch.py`: PNT50 모터 드라이버만 단독 실행.

모든 launch 파일은 `dw_robot_base/config/*.yaml`의 파라미터를 `FindPackageShare`/
`get_package_share_directory`를 통해 로드한다. 따라서 파라미터를 바꿀 때는 launch 파일에 직접
적기보다 YAML 파일을 수정해야 한다(단, `rf_pnt50.launch.py`의 `input_source`나 카메라 파라미터처럼
일부 인라인 오버라이드는 예외).

### 곳곳에 있는 토픽 타임아웃

cmd_vel/텔레옵 체인의 거의 모든 노드가 "마지막 메시지 수신 후 경과 시간"을 각자 독립적으로 추적하고,
타임아웃 시 출력을 0으로 만들거나 브레이크를 요청한다 — `cmd_vel_mux_node`, `cmd_vel_safety_node`,
`base_controller_node`, `pnt50_driver_node`, `rf_arduino_node` 모두 각자의 `*_timeout`/`cmd_timeout`
파라미터를 가지고 있다. "로봇이 안 움직인다" 또는 "로봇이 멈추지 않는다" 같은 문제를 디버깅할 때는
이 독립적인 타임아웃들 중 어느 것이 발동했는지 확인해야 한다 — 이들은 중앙에서 관리되지 않는다.

### HWT901B IMU 드라이버

`hwt901b_driver/hwt901b_node.py`는 ROS publish 타이머(`publish_data`)와 분리된 자체 시리얼 읽기
스레드(`serial_read_loop`)를 돌리며, WitMotion 바이너리 프레임 프로토콜(0x55 헤더, 패킷 타입
0x51/0x52/0x53/0x54는 각각 가속도/자이로/각도/지자기, 체크섬은 앞 10바이트의 합)을 파싱한다. 현재
이 워크스페이스의 어떤 launch 파일에서도 참조되지 않는 독립 노드다.
