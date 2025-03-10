#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
import json
import numpy as np
import math
import time
from itertools import permutations

class PathTimeCalculator(Node):
    """
    ROS2 좌표로 변환된 위치를 받아 경로 계획 및 시간을 계산하는 노드
    """
    
    def __init__(self):
        super().__init__('path_time_calculator')
        
        # 로봇 속도 설정 (m/s)
        self.robot_speed = 0.5
        
        # 현재 로봇 위치 (초기값)
        self.current_robot_pose = [0.0, 0.0, 0.0]  # x, y, z
        
        # 물체 위치 저장 변수
        self.object_positions = {}
        
        # 구독자 설정
        self.ros2_position_subscription = self.create_subscription(
            String,
            '/object_positions',  # 변환된 ROS2 물체 위치 토픽
            self.position_callback,
            10
        )
        
        # 발행자 설정
        self.optimal_route_publisher = self.create_publisher(
            String,
            '/robot/optimal_route',  # 최적 경로 발행 토픽
            10
        )
        
        self.optimal_path_publisher = self.create_publisher(
            Path,
            '/robot/optimal_path',  # 최적 경로 좌표 발행 토픽
            10
        )
        
        # 주기적으로 최적 경로 계산 및 발행 (2초마다)
        self.timer = self.create_timer(2.0, self.calculate_optimal_path)
        
        self.get_logger().info("PathTimeCalculator 초기화 완료")
    
    def position_callback(self, msg):
        """
        ROS2 좌표계의 물체 위치를 받는 콜백 함수
        """
        try:
            # JSON 문자열을 파싱
            self.object_positions = json.loads(msg.data)
            self.get_logger().info(f"물체 위치 수신: {len(self.object_positions)} 물체")
        except Exception as e:
            self.get_logger().error(f"물체 위치 처리 중 오류 발생: {e}")
    
    def calculate_distance(self, pos1, pos2):
        """
        두 위치 간의 2D 거리를 계산
        """
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def calculate_path_time(self, path):
        """
        경로의 이동 시간을 계산
        """
        total_distance = 0.0
        
        for i in range(1, len(path)):
            pos1 = self.object_positions[path[i-1]]
            pos2 = self.object_positions[path[i]]
            total_distance += self.calculate_distance(pos1, pos2)
        
        # 시간 = 거리 / 속도
        travel_time = total_distance / self.robot_speed
        
        return travel_time, total_distance
    
    def calculate_optimal_path(self):
        """
        최적 경로를 계산하고 발행
        """
        if not self.object_positions:
            self.get_logger().warning("물체 위치 정보가 없습니다. 최적 경로 계산을 건너뜁니다.")
            return
        
        # 물체 이름 목록
        object_names = list(self.object_positions.keys())
        
        if len(object_names) <= 1:
            self.get_logger().warning("최소 2개 이상의 물체가 필요합니다.")
            return
        
        # 시작점 설정 (첫 번째 물체)
        start_object = object_names[0]
        
        # 나머지 물체들에 대해 모든 가능한 경로 계산
        remaining_objects = object_names[1:]
        possible_routes = list(permutations(remaining_objects))
        
        best_route = None
        best_time = float('inf')
        best_distance = float('inf')
        
        for route in possible_routes:
            full_route = [start_object] + list(route)
            travel_time, travel_distance = self.calculate_path_time(full_route)
            
            if travel_time < best_time:
                best_time = travel_time
                best_distance = travel_distance
                best_route = full_route
        
        # 최적 경로 발행
        self.publish_optimal_route(best_route, best_time, best_distance)
    
    def publish_optimal_route(self, route, time, distance):
        """
        계산된 최적 경로를 발행
        """
        if not route:
            return
        
        # 경로 정보 준비
        route_info = {
            'route': route,
            'time': time,
            'distance': distance
        }
        
        # 경로 정보 문자열 발행
        route_msg = String()
        route_msg.data = json.dumps(route_info)
        self.optimal_route_publisher.publish(route_msg)
        
        # 경로 좌표 발행
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for obj_name in route:
            if obj_name in self.object_positions:
                pos = self.object_positions[obj_name]
                
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(pos[0])
                pose.pose.position.y = float(pos[1])
                pose.pose.position.z = float(pos[2])
                pose.pose.orientation.w = 1.0  # 기본 방향 설정
                
                path_msg.poses.append(pose)
        
        self.optimal_path_publisher.publish(path_msg)
        
        # 결과 로그 출력
        route_str = " -> ".join(route)
        self.get_logger().info(f"최적 경로: {route_str}")
        self.get_logger().info(f"예상 이동 시간: {time:.2f}초, 이동 거리: {distance:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    path_time_calculator = PathTimeCalculator()
    
    try:
        rclpy.spin(path_time_calculator)
    except KeyboardInterrupt:
        pass
    finally:
        path_time_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 