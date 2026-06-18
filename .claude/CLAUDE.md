# CLAUDE.md

이 파일은 Claude Code가 이 ROS 2 워크스페이스에서 작업할 때 먼저 읽는 최상위 가이드다.

이 파일에는 전체 내용을 길게 쓰지 않는다. 작업 종류에 따라 `docs/claude/` 아래의 관련 문서를 추가로 읽고 작업한다.

## 프로젝트 핵심 요약

이 워크스페이스는 전기자동차 화재 대응용 소방 모바일 로봇을 개발하기 위한 ROS 2 Jazzy 워크스페이스다.

로봇의 목표는 차량 앞으로 접근한 뒤, 카메라로 번호판을 인식하고, 차량 하부로 진입하여 하부에 드릴로 구멍을 뚫고 물을 분사하는 것이다.

현재 로봇은 다음 구조를 목표로 한다.

- 4륜 메카넘 휠 모바일 로봇
- MDROBOT PNT50 2채널 드라이버 2대
- 총 4개 모터 제어
- RF 조종기 + Arduino 입력
- USB 카메라 기반 번호판 인식
- 홀센서 기반 펄스 카운트
- HWT901B 계열 IMU 사용 예정

기존 코드에는 차동 구동 또는 2바퀴 기준 구조가 남아 있을 수 있다. 하지만 앞으로의 기준은 4륜 메카넘 로봇이다.

## 작업 전 반드시 지킬 것

1. 실제 모터가 움직일 수 있는 코드는 항상 안전 조건을 먼저 확인한다.
2. `CH4 estop`은 모든 모드와 모든 명령보다 우선한다.
3. 기존 2바퀴 `left/right` 구조를 4바퀴 메카넘 제어에 그대로 재사용하지 않는다.
4. 4바퀴 순서는 항상 `[front_left, front_right, rear_left, rear_right]`를 기준으로 한다.
5. 하드웨어 포트, slave ID, 바퀴 방향, 로봇 치수는 코드에 하드코딩하지 말고 YAML 파라미터로 관리한다.
6. TODO 값은 임의로 확정하지 않는다.
7. 작업 후에는 빌드를 직접하면서 확인한다.

## 작업 종류별로 읽을 문서

| 작업 종류 | 먼저 읽을 문서 |
|---|---|
| 프로젝트 목적 / 전체 구조 확인 | `docs/claude/00_project_overview.md` |
| 로봇 치수 / 하드웨어 스펙 확인 | `docs/claude/01_hardware.md` |
| 조종기 모드 / CH7 / estop 작업 | `docs/claude/02_control_modes.md` |
| PNT50 / Modbus / 모터 드라이버 작업 | `docs/claude/03_motor_pnt50.md` |
| 카메라 / 번호판 인식 / LED 작업 | `docs/claude/04_camera_perception.md` |
| IMU / 홀센서 / 오도메트리 작업 | `docs/claude/05_sensors.md` |
| Raspberry Pi / 노트북 / ROS_DOMAIN_ID 작업 | `docs/claude/06_network_development.md` |
| 코드 구조 / launch / config / 코딩 규칙 | `docs/claude/07_architecture_coding_rules.md` |
| 아직 미정인 항목 확인 | `docs/claude/99_todo.md` |

## 기본 명령

워크스페이스 루트 (Pi / 노트북 공통, `~`는 각 기기의 홈으로 확장됨):

```bash
~/dw_robot_ws
```

빌드:

```bash
colcon build --symlink-install
source install/setup.bash
```

특정 패키지 빌드:

```bash
colcon build --symlink-install --packages-select dw_robot_base
source install/setup.bash
```

테스트:

```bash
colcon test --packages-select dw_robot_base
colcon test-result --verbose
```

## 주요 패키지

```text
src/
  dw_robot_base
  dw_robot_bringup
  dw_robot_description
  hwt901b_driver
```

대부분의 실제 로봇 제어 작업은 `dw_robot_base`에서 이루어진다.