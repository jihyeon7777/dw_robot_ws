# Control Modes

## 모드 개요

로봇에는 3가지 주행 모드가 있다.

```text
MANUAL
MECANUM
AUTO
```

모드는 RF 조종기의 `CH7` 3단계 스위치로 변경한다.

`CH7` PWM 값은 큰 값에서 작은 값 방향으로 변한다.

| CH7 단계 | PWM 방향 | 모드 | 설명 |
|---:|---|---|---|
| 1단계 | 큰 값 | `MANUAL` | 일반 수동 조종 |
| 2단계 | 중간 값 | `MECANUM` | 메카넘 수동 이동 |
| 3단계 | 작은 값 | `AUTO` | 자율주행 |

## 조종기 채널

현재 기준 채널 구성:

```text
CH1: throttle
CH2: yaw
CH4: estop
CH7: mode select
```

| 항목 | 값 |
|---|---|
| Arduino 보드 | Arduino Uno R4 WiFi |
| RF receiver 모델 | `Receiver_Specifications.md` 참고 |
| 수신 방식 | PWM |
| Arduino serial baudrate | 115200 |
| throttle | CH1 |
| yaw | CH2 |
| estop | CH4 |
| mode 변경 | CH7 |

RF receiver 세부 스펙이 필요하면 `Receiver_Specifications.md`를 먼저 읽는다.

## 안전 우선순위

항상 다음 순서를 지킨다.

```text
1. estop
2. RF signal valid 확인
3. CH7 mode 판별
4. 선택된 mode에 맞는 cmd_vel 생성 또는 선택
5. safety limiter 통과
6. motor command 출력
```

`CH4 estop`은 모든 모드보다 우선한다.

AUTO 모드에서도 estop이 들어오면 즉시 모터 명령을 차단해야 한다.

## MANUAL Mode

기본 수동 조종 모드다.

스키드 스티어링 또는 차동 구동 로봇처럼 움직인다.

```text
CH1 throttle → 전진 / 후진
CH2 yaw      → 좌회전 / 우회전
```

목적:

- 운전자가 로봇을 차량 근처까지 이동

## MECANUM Mode

메카넘 휠의 평행 이동 기능을 사용하는 수동 조종 모드다.

목표 동작:

```text
전진
후진
좌측 평행 이동
우측 평행 이동
필요 시 제한적인 yaw 회전
```

ROS Twist 기준:

```text
linear.x   전후 이동
linear.y   좌우 평행 이동
angular.z  yaw 회전
```

조종기 채널 수가 부족하면 yaw 회전보다 좌우 평행 이동을 우선할 수 있다.

## AUTO Mode

카메라 기반 번호판 인식과 자율 진입 로직을 사용하는 모드다.

기본 시나리오:

1. 운전자가 MANUAL 모드로 차량 앞으로 접근한다.
2. 카메라가 번호판을 인식한다.
3. 번호판이 인식되면 LED를 켠다.
4. 운전자가 CH7로 AUTO 모드로 전환한다.
5. 번호판 bbox 중심좌표를 기준으로 차량 중앙에 정렬한다.
6. 목표 정지 거리까지 접근한다.
7. 번호판이 보이지 않게 된 이후에는 별도 로직으로 차량 하부 중심까지 진입한다.
8. 하부 중심 도달 후 드릴 작업과 물 분사를 수행한다.

## TODO

```text
TODO: CH7 각 단계 실제 PWM 값 측정
TODO: CH7 mode threshold 결정
TODO: CH4 estop PWM 동작 범위 확인
TODO: AUTO mode 진입 조건 확정
TODO: AUTO mode 해제 조건 확정
TODO: AUTO mode 실패 시 정지/복귀 조건 확정
```