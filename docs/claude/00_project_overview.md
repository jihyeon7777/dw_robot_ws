# Project Overview

## 목적

이 프로젝트는 전기자동차 화재 대응용 소방 모바일 로봇을 개발하기 위한 ROS 2 Jazzy 워크스페이스다.

로봇의 최종 임무는 다음과 같다.

1. 전기자동차 화재 현장에서 차량 앞으로 접근한다.
2. 카메라로 차량 번호판을 인식한다.
3. 번호판 중심좌표를 기준으로 차량 중앙에 정렬한다.
4. 차량 하부로 진입한다.
5. 차량 하부 중심 부근에서 드릴로 구멍을 뚫는다.
6. 구멍을 통해 물을 분사하여 배터리 화재를 진압한다.

## 현재 개발 방향

현재는 다음 기능들을 단계적으로 구현하는 중이다.

- RF 조종기 기반 수동 조종
- CH7 기반 3단계 모드 전환
- CH4 기반 estop
- 4륜 메카넘 구동
- PNT50 2대 기반 4모터 제어
- USB 카메라 기반 번호판 인식
- 번호판 중심 기준 정렬
- 홀센서 기반 이동 거리 추정
- IMU 기반 yaw 보정
- 차량 하부 진입 로직
- 드릴 / 물 분사 장치 제어

## 중요한 전제

기존 코드에는 차동 구동 또는 2바퀴 기준 코드가 남아 있을 수 있다.

하지만 실제 로봇의 최종 구조는 다음과 같다.

```text
4-wheel mecanum mobile robot
PNT50 2-channel motor driver x 2
4 motors total
```

따라서 새 코드를 작성하거나 기존 코드를 수정할 때는 4륜 메카넘 기준으로 판단한다.

## 패키지 개요

```text
src/
  dw_robot_base
  dw_robot_bringup
  dw_robot_description
  hwt901b_driver
```

### dw_robot_base

핵심 패키지다.

주요 역할:

- 조종기 / 키보드 입력 처리
- cmd_vel 처리
- 안전 제한
- 메카넘 제어
- 모터 드라이버 통신
- 카메라 노드
- RF / Arduino 입력 처리
- 홀센서 피드백 처리
- 오도메트리 관련 노드

### dw_robot_bringup

상위 launch 편의 패키지다.

실제 로봇 전체 bringup launch 또는 통합 실행용 launch를 둘 수 있다.

### dw_robot_description

URDF / xacro / RViz / robot_state_publisher 관련 패키지다.

로봇 치수, 카메라 위치, IMU 위치, 드릴/분사 장치 위치가 정해지면 이 패키지를 수정한다.

### hwt901b_driver

WitMotion HWT901B IMU용 독립 드라이버 패키지다.

IMU 관련 작업 전에는 반드시 `imu_spec/` 폴더를 먼저 확인한다.

## 최상위 motordriver.py

워크스페이스 최상위의 `motordriver.py`는 일반 ROS 패키지 코드로 보지 않는다.

MDROBOT 모터 컨트롤러의 slave ID 변경 등을 위해 raw Modbus 패킷을 보내는 일회용 스크립트일 수 있다.