# ROS2 TSP Solver

외판원 문제(TSP)를 해결하여 ROS2 Navigation2 시스템에서 최적 경로를 찾는 프로그램입니다.

## 주요 기능

- waypoint.txt 파일에서 웨이포인트 로드
- 외판원 문제(TSP) 해결을 통한 최적 경로 계산
- 최적 경로 정보를 ROS2 토픽으로 발행
- 경로 계산 후 로봇을 원점으로 이동

## 사용 방법

```bash
source /opt/ros/humble/setup.bash
python3 time_estimate.py
```

