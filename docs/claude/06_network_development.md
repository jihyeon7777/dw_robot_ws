# Network and Development Environment

## 개발 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 메인 컴퓨터 | Raspberry Pi |
| 보조 개발 장비 | 노트북 |
| 주요 개발 도구 | Claude Code |
| 보조 개발 도구 | Antigravity |

## 컴퓨터 / 네트워크 설정

| 항목 | 값 |
|---|---|
| Raspberry Pi hostname | `pi` |
| 노트북 hostname | `jh` |
| ROS_DOMAIN_ID | `30` |
| 네트워크 방식 | 무선 |
| camera/perception 실행 위치 | Raspberry Pi |

공통 환경 설정:

```bash
export ROS_DOMAIN_ID=30
```

## 권장 역할 분리

```text
Raspberry Pi:
  RF Arduino 입력
  motor driver
  safety node
  mecanum controller
  camera
  plate perception
  core bringup

Laptop:
  SSH 개발
  RViz
  ros2 topic echo
  ros2 bag record
  debugging
```

## 주의사항

- Pi와 노트북의 `ROS_DOMAIN_ID`가 다르면 ROS 2 토픽이 서로 보이지 않는다.
- 무선 네트워크 상태가 불안정하면 카메라 영상 또는 토픽 통신이 끊길 수 있다.
- 실제 로봇 제어에 필요한 핵심 노드는 가능한 한 Raspberry Pi에서 실행한다.
- 노트북은 개발, 모니터링, RViz, 디버깅 용도로 사용한다.

## 기본 빌드 명령

워크스페이스 루트 (Pi / 노트북 공통, `~`는 각 기기의 홈으로 확장됨):

```bash
~/dw_robot_ws
```

전체 빌드:

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