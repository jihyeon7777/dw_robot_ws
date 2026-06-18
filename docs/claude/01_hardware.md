# Hardware Specification

## 로봇 기본 구조

이 로봇은 4개의 메카넘 휠을 사용하는 모바일 로봇이다.

구동계는 다음과 같다.

- MDROBOT PNT50 드라이버 사용
- 2채널 드라이버 2대
- 총 4개 모터 제어
- USB-RS485 1개 공유 bus 사용

## 로봇 치수

| 항목 | 값 | 설명 |
|---|---:|---|
| wheel diameter | 0.125 m | 바퀴 지름 |
| wheel radius | 0.0625 m | 바퀴 반지름 |
| front-rear wheel center distance | 445 mm | 앞뒤 바퀴 중심거리 |
| left-right wheel center distance | 525 mm | 좌우 바퀴 중심거리 |
| `lx` | 222.5 mm | 로봇 중심에서 바퀴까지 x 방향 거리 |
| `ly` | 262.5 mm | 로봇 중심에서 바퀴까지 y 방향 거리 |
| `lx_plus_ly` | 485 mm | 메카넘 제어 파라미터 |
| robot width | 570 mm | 전체 외형 폭 |
| robot length | 570 mm | 전체 외형 길이 |
| robot height | 130 mm | 전체 높이 |
| ground clearance | 8 mm | 지상고 |

meter 단위 기준:

```text
wheel_radius = 0.0625
lx = 0.2225
ly = 0.2625
lx_plus_ly = 0.485
width = 0.570
length = 0.570
height = 0.130
ground_clearance = 0.008
```

## 바퀴 이름 기준

항상 다음 이름을 사용한다.

```text
front_left
front_right
rear_left
rear_right
```

배열 순서는 항상 다음을 기준으로 한다.

```text
[front_left, front_right, rear_left, rear_right]
```

이 순서는 모터 명령, 홀센서 피드백, 오도메트리, 디버그 토픽에서 모두 통일한다.

## 주의사항

기존 코드가 2바퀴 차동 구동 기준일 수 있다.

4륜 메카넘 제어 작업에서는 `left/right`만 있는 구조를 그대로 사용하지 말고, 반드시 4개 바퀴를 명확히 분리한다.