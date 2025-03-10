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

## 시스템 구성 및 토픽 흐름

시스템은 세 개의 독립적인 노드로 구성되어 있으며, 다음과 같은 토픽 흐름을 갖습니다:

1. **isaac_sim_publisher.py**
   - 발행 토픽: `/isaac_sim/object_positions`
   - 기능: Isaac Sim 좌표계의 물체 위치 데이터 발행
   - 데이터 소스: `publish_issacsim.txt` 파일에서 물체 좌표 로드 가능

2. **coordinate_transformer.py**
   - 구독 토픽: `/isaac_sim/object_positions`
   - 발행 토픽: `/object_positions`
   - 기능: Isaac Sim 좌표계를 ROS2 좌표계로 변환

3. **path_time_calculator.py**
   - 구독 토픽: `/object_positions`
   - 발행 토픽: 
     - `/robot/optimal_route` (경로 정보)
     - `/robot/optimal_path` (경로 좌표)
   - 기능: 변환된 좌표를 기반으로 최적 이동 경로 계산

## 사용 방법

### 기본 설정 및 실행

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 개별 노드 실행 (각각 별도의 터미널에서)
python3 isaac_sim_publisher.py
python3 coordinate_transformer.py
python3 path_time_calculator.py
```

### 기존 TSP 시스템 실행

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 기존 TSP 시간 계산 시스템 실행
python3 time_estimate.py
```

### 물체 좌표 수정 방법

물체 좌표는 `publish_issacsim.txt` 파일에서 수정할 수 있습니다:
