# sim — 시뮬레이션

실기 없이 주행 · 인식 로직을 시험하기 위한 월드와 시나리오를 둡니다. 시뮬레이터(Gazebo, Isaac Sim
등)는 아직 정하지 않았고, 정하면 아래처럼 나눌 계획입니다.

```
sim/
├── worlds/       맵 · 환경 파일
├── models/       시뮬레이션 전용 모델 (로봇 본체 URDF는 software/ros2_ws/src/nubi_description)
└── scenarios/    시험 시나리오 (출발점, 장애물 배치, 성공 조건)
```

시뮬레이터를 정하면 버전과 실행 방법을 여기에 적습니다.
