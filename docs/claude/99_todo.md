# TODO and Unknowns

이 문서는 아직 확정되지 않은 항목을 모아두는 문서다.

Claude Code는 아래 값을 임의로 확정하지 않는다. 실제 측정, 매뉴얼 확인, 사용자 확인 후 수정한다.

## RF 조종기 / Arduino

```text
DONE: CH7 각 단계의 실제 PWM 값 측정 (MANUAL 1991 / MECANUM 1493 / AUTO 996)
DONE: CH7 mode threshold 결정 (manual>=1700, auto<=1250; rf_arduino.yaml)
DONE: CH1/CH2 throttle/steering PWM 보정 (rf_arduino.yaml 반영)
TODO: 아두이노 펌웨어가 CSV 10번째 필드로 CH7 PWM을 전송하도록 추가 (현재 9필드)
TODO: CH4 estop PWM 동작 범위 확인
TODO: RF receiver 세부 스펙은 Receiver_Specifications.md에서 확인
TODO: Arduino firmware 파일 위치 확인
```

## 모터 / PNT50

```text
TODO: PNT50 각 채널과 실제 바퀴 매핑 확인
TODO: 각 바퀴 direction multiplier 실주행 테스트로 확정
TODO: PNT50 register map 확인
TODO: PNT50 brake / speed command register 확인
TODO: 홀센서 pulse per revolution 확인
TODO: 모터 감속비 확인
TODO: 최대 주행 속도 확정
TODO: 최대 회전 속도 확정
TODO: 최대 가속도 제한 확정
```

## 카메라 / 번호판

```text
TODO: OpenCV 번호판 인식 알고리즘 구체화
TODO: 인식 안정화 후 카메라 해상도를 320x240 -> 목표 1920x1080으로 상향 (현재는 인식률 때문에 낮춤)
TODO: plate bbox 기반 거리 추정 방식 확정
TODO: 번호판이 사라진 뒤 하부 중심까지 진입하는 방식 결정
TODO: 차량 하부 중심 도달 판단 방식
TODO: LED 제어 방식 확정
```

## 드릴 / 물 분사

```text
TODO: 드릴 제어 방식
TODO: 드릴 구동 모터/액추에이터 스펙
TODO: 드릴 위치 제어 방식
TODO: 물 분사 제어 방식
TODO: 물 분사 밸브/펌프 제어 방식
```

## IMU / 오도메트리

```text
TODO: IMU 설정값은 imu_spec 폴더 확인 후 확정
TODO: IMU frame_id 확정
TODO: IMU yaw를 오도메트리에 어떻게 사용할지 결정
TODO: 메카넘 wheel odometry 공식 구현
TODO: 홀센서 기반 거리 추정 정확도 확인
```