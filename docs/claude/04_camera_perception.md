# Camera and Plate Perception

## 카메라 스펙

| 항목 | 값 |
|---|---|
| 카메라 모델 | USB 카메라 |
| 연결 방식 | USB |
| 해상도 | 320 x 240 (현재) |
| FPS | 5 |
| 장착 위치 x | 262.5 mm |
| 장착 위치 y | 0 mm |
| 장착 위치 z | 100 mm |
| 장착 각도 rpy | 0 0 0 |
| 번호판 인식 방식 | OpenCV |
| 1단계 target stop distance | 2 m |
| bbox 기준 정지 threshold | 0.5 m |

카메라 자체는 2M(1920 x 1080)까지 지원하지만, 해상도가 너무 높으면 번호판 인식이 잘 안 되는 문제가 있어 현재는 320 x 240으로 낮춰서 사용한다. 인식 알고리즘이 안정화되면 해상도를 다시 올릴 수 있다 (TODO 참고).

ROS / URDF / TF에서는 meter 단위로 사용한다.

```text
camera xyz = 0.2625 0.0 0.1
camera rpy = 0.0 0.0 0.0
```

## 번호판 인식 목표

기본 목표:

```text
1. USB camera image 수신
2. OpenCV 기반 번호판 검출
3. 번호판 bbox 계산
4. bbox 중심좌표 계산
5. 번호판 인식 성공 여부 publish
6. 인식 성공 시 LED ON 신호 publish
7. AUTO mode에서 bbox 중심 기준 정렬
8. target stop distance 기준 접근 제어
```

## 권장 토픽 예시

```text
/camera/image_raw
/plate_detected
/plate_bbox
/plate_center
/plate_align_error
/led_cmd
```

정확한 메시지 타입은 구현 단계에서 결정한다.

## AUTO mode와의 관계

AUTO mode에서는 번호판 bbox 중심좌표를 기준으로 차량 중앙에 정렬한다.

번호판이 보이지 않게 된 이후 차량 하부 중심까지 진입하는 방식은 아직 미정이다.

## 주의사항

- perception 노드와 주행 제어 노드는 가능하면 분리한다.
- 카메라 FPS가 5이므로 perception timeout을 반드시 고려한다.
- 번호판 인식 실패 시 AUTO mode가 계속 진행될지, 정지할지, dead reckoning으로 이어갈지는 아직 확정하지 않는다.
- 확정되지 않은 동작은 TODO로 남긴다.

## TODO

```text
TODO: OpenCV 번호판 인식 알고리즘 구체화
TODO: 인식 안정화 후 카메라 해상도를 320x240 -> 더 높은 해상도(목표 1920x1080)로 상향
TODO: plate bbox 기반 거리 추정 방식 확정
TODO: 번호판이 사라진 뒤 하부 중심까지 진입하는 방식 결정
TODO: 차량 하부 중심 도달 판단 방식
TODO: LED 제어 방식 확정
```