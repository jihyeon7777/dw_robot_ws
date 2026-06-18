# Sensors

## 홀센서

Arduino가 홀센서 펄스를 카운트한다.

홀센서 데이터는 다음 용도로 사용할 수 있다.

- 바퀴 회전량 추정
- 모터 피드백
- 오도메트리 계산
- 주행 거리 추정
- 차량 하부 진입 거리 추정

홀센서 펄스를 오도메트리에 사용할 경우 다음 값을 명확히 해야 한다.

```text
TODO: 바퀴 1회전당 펄스 수
TODO: 감속비
TODO: 펄스 카운트 방향
TODO: 각 모터별 펄스 매핑
TODO: front_left / front_right / rear_left / rear_right 순서
```

## IMU

IMU는 WitMotion HWT901B 계열 센서를 사용할 예정이다.

IMU 센서 관련 스펙과 매뉴얼은 `imu_spec` 폴더에 정리되어 있다.

IMU 관련 작업을 할 때는 코드 수정 전에 반드시 다음 폴더를 먼저 확인한다.

```text
imu_spec/
```

## IMU 사용 목적

- yaw 추정
- 자세 유지
- 메카넘 이동 중 heading 유지
- 오도메트리 보정
- 자율주행 중 방향 안정화

## IMU 작업 시 확인할 내용

```text
센서 모델명
시리얼 통신 설정
baudrate
출력 데이터 포맷
좌표계
frame_id
publish rate
orientation 단위
angular velocity 단위
linear acceleration 단위
checksum 방식
ROS 메시지 매핑 방식
```

기존 `hwt901b_driver` 패키지와 `imu_spec` 폴더의 내용이 다를 경우, `imu_spec`의 실제 매뉴얼 내용을 우선 확인하고 차이를 문서화한다.

## 오도메트리 방향

메카넘 로봇의 오도메트리는 차동 구동과 다르다.

기존 차동 구동 오도메트리 공식을 그대로 사용하면 안 된다.

메카넘 오도메트리 구현 시 필요한 정보:

```text
4개 바퀴의 회전량
바퀴 반지름
lx + ly
각 바퀴 방향 부호
encoder 또는 hall sensor pulse per revolution
IMU yaw 사용 여부
```

권장 방향:

1. 초기에는 wheel odometry만 구현한다.
2. 이후 IMU yaw를 이용해 heading 안정화 또는 보정을 추가한다.
3. 자율주행에서 차량 하부로 들어가는 거리 추정에는 홀센서 기반 이동 거리와 IMU yaw를 함께 사용하는 것을 검토한다.

## 확인할 기존 파일

```text
imu_spec/
src/hwt901b_driver/
src/dw_robot_base/dw_robot_base/odom_from_pnt50_feedback_node.py
```