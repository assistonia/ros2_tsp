실행방법

source /opt/ros/humble/setup.bash
python3 time_estimate.py

waypoint.txt 에 웨이포인트 저장하면 각 순서대로 체크합니다.
좌표의 경우 ros2 좌표입니다.
10개 이하까지는 문제없이 가능

그리고 계산이 완료된후 계산 내용은 nav2_path_times.json에 저장됩니다.