# Isaac Sim - ROS2 좌표 변환 및 경로 계획 시스템

Isaac Sim과 ROS2 좌표계 간의 변환 및 최적 경로 계획을 위한 통합 시스템입니다.

## 주요 기능

### 1. 좌표 변환 시스템
- Isaac Sim 좌표계에서 ROS2 좌표계로 정확한 변환
- 선형 변환 모델을 통한 정밀한 좌표 매핑
- 역변환(ROS2→Isaac Sim) 지원

### 2. 물체 위치 추적
- Isaac Sim에서 물체 위치 발행 (목업/실제 시뮬레이션)
- 좌표 변환을 통한 ROS2 기반 위치 추적
- JSON 형식의 메시지로 간편한 통합

### 3. 경로 계획 및 최적화
- 외판원 문제(TSP) 해결을 통한 최적 경로 계산
- 이동 시간 및 거리 추정
- 최적 경로 정보를 ROS2 토픽으로 발행

## 시스템 구성

시스템은 세 개의 독립적인 노드로 구성되어 있습니다:

1. **isaac_sim_publisher.py**: Isaac Sim 물체 위치를 발행하는 노드
2. **coordinate_transformer.py**: 좌표계 변환을 수행하는 노드
3. **path_time_calculator.py**: 경로 계획 및 시간 계산을 수행하는 노드

## 통신 토픽

- `/isaac_sim/object_positions`: Isaac Sim 좌표계 물체 위치
- `/ros2/object_positions`: 변환된 ROS2 좌표계 물체 위치
- `/robot/optimal_route`: 계산된 최적 경로 정보
- `/robot/optimal_path`: 경로 좌표 정보

## 사용 방법

### 환경 설정 및 실행

ROS2 환경을 설정하고 각 스크립트를 직접 실행합니다:

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 각 노드 실행 (별도의 터미널에서)
python3 isaac_sim_publisher.py
python3 coordinate_transformer.py
python3 path_time_calculator.py
```

### 데이터 확인

토픽 메시지 확인:
```bash
# Isaac Sim 물체 위치
ros2 topic echo /isaac_sim/object_positions

# 변환된 ROS2 좌표
ros2 topic echo /ros2/object_positions

# 최적 경로
ros2 topic echo /robot/optimal_route
```

## 직접 개발한 좌표 변환 파라미터

Isaac Sim과 ROS2 좌표계 간의 변환식:
```
ROS2_x = 1.050398 * ISAAC_x + 0.052001 * ISAAC_y + 80.194741
ROS2_y = -0.021784 * ISAAC_x + 1.011847 * ISAAC_y + -62.842775
```

## 기존 TSP Solver와의 통합

이 시스템은 기존의 [ROS2 TSP Solver](https://github.com/assistonia/ros2_tsp) 기능을 확장하여 Isaac Sim 좌표를 통합한 버전입니다.

