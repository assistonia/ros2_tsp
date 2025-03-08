#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import math
import itertools
import json
import time
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class Nav2OptimalPathFinder(Node):
    def __init__(self):
        super().__init__('nav2_optimal_path_finder')
        self.action_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.avg_speed = 0.5  # 로봇 평균 속도 (m/s)
        
        # 웨이포인트 파일에서 좌표 불러오기
        self.points = self.load_waypoints('waypoint.txt')
        self.get_logger().info(f"웨이포인트 {len(self.points)}개 로드 완료: {', '.join(self.points.keys())}")
        
        self.simulation_mode = False  # 실제 ROS2 시스템 사용
        
        # 경로 구독자 설정
        self.path = None
        self.path_received = False
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.path_subscription = self.create_subscription(
            Path,
            '/plan',  # ROS2 네비게이션 시스템에서 경로 토픽
            self.path_callback,
            qos_profile
        )
        
        # 목적지 발행자 설정
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',  # ROS2 네비게이션 목표 위치 토픽
            10
        )
        
        # 최적 경로 발행자 설정
        self.optimal_route_publisher = self.create_publisher(
            String,
            '/optimal_route',  # 최적 경로 발행 토픽
            10
        )
        
        # 경로 좌표 발행자 설정
        self.route_path_publisher = self.create_publisher(
            Path,
            '/optimal_route_path',  # 최적 경로 좌표 발행 토픽
            10
        )
        
        self.get_logger().info("Nav2OptimalPathFinder 초기화 완료")
        
    def path_callback(self, msg):
        self.path = msg
        self.path_received = True
        self.get_logger().info(f"경로 수신: {len(msg.poses)}개 포인트")

    def request_path(self, start_pose, goal_pose):
        if self.simulation_mode:
            # 시뮬레이션 모드 코드는 유지
            self.get_logger().info("시뮬레이션 모드: 가상 경로 계산 중")
            path = self.create_simulated_path(start_pose, goal_pose)
            return path
        
        # 실제 ROS2 네비게이션 시스템 사용 시
        self.get_logger().info(f"경로 계산 요청: {start_pose.pose.position.x:.2f},{start_pose.pose.position.y:.2f} -> {goal_pose.pose.position.x:.2f},{goal_pose.pose.position.y:.2f}")
        
        # 경로 수신 플래그 초기화
        self.path_received = False
        
        # 목표 위치 발행 (실제 이동 없이 경로만 계산)
        self.goal_publisher.publish(goal_pose)
        
        # 경로 수신 대기
        timeout = 5.0  # 5초 타임아웃
        start_time = time.time()
        
        while not self.path_received and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.path_received:
            self.get_logger().error("경로 계산 타임아웃")
            return None
            
        return self.path

    def create_simulated_path(self, start_pose, goal_pose):
        # 시뮬레이션 모드에서 사용할 가상 경로 생성
        from nav_msgs.msg import Path
        
        path = Path()
        path.header.frame_id = "map"
        
        # 시작점과 끝점 사이에 가상의 포인트 10개 생성
        num_points = 10
        sx, sy = start_pose.pose.position.x, start_pose.pose.position.y
        gx, gy = goal_pose.pose.position.x, goal_pose.pose.position.y
        
        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = sx + t * (gx - sx)
            pose.pose.position.y = sy + t * (gy - sy)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
            
        return path

    def calculate_path_length(self, path):
        dist = 0.0
        poses = path.poses
        for i in range(len(poses)-1):
            p1, p2 = poses[i].pose.position, poses[i+1].pose.position
            dist += ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
        return dist

    def get_current_pose(self):
        # 현재 로봇 위치 반환
        # 실제 환경에서는 로봇의 현재 위치를 가져오는 로직 구현
        # 예제에서는 임의의 위치 사용
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.w = 1.0
        self.get_logger().info(f"현재 위치: x={pose.pose.position.x}, y={pose.pose.position.y}")
        return pose
        
    def get_pose_from_stamped(self, pose_stamped):
        """PoseStamped 메시지에서 pose 정보 추출"""
        return pose_stamped
        
    def get_pose_coords(self, name, coords):
        """좌표 정보로부터 PoseStamped 메시지 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = coords[0]
        pose.pose.position.y = coords[1]
        pose.pose.orientation.w = 1.0
        return pose
        
    def get_pose_xy(self, pose):
        """PoseStamped 메시지에서 x, y 좌표 추출"""
        return [pose.pose.position.x, pose.pose.position.y]

    def calculate_segment_time(self, start_name, end_name):
        """두 지점 사이의 경로 시간 계산"""
        start_pose = self.get_pose(start_name) if start_name != 'current' else self.get_current_pose()
        end_pose = self.get_pose(end_name)
        
        path = self.request_path(start_pose, end_pose)
        if path is None:
            self.get_logger().error(f"{start_name}에서 {end_name}까지 경로를 찾을 수 없습니다.")
            return None
            
        dist = self.calculate_path_length(path)
        time_to_destination = dist / self.avg_speed
        
        self.get_logger().info(f"경로 [{start_name} → {end_name}] 거리: {dist:.2f}m, 예상 이동시간: {time_to_destination:.2f}초")
        
        return {
            'start': start_name,
            'end': end_name,
            'distance': dist,
            'time': time_to_destination
        }

    def find_best_path(self):
        self.get_logger().info("외판원 문제 최적 경로 찾기 시작...")
        waypoint_names = list(self.points.keys())
        
        # 모든 경로 조합 계산 (외판원 문제)
        permutations = list(itertools.permutations(waypoint_names))
        
        # 각 경로 조합별 총 시간 계산
        route_times = {}
        segment_data = {}
        
        # 먼저 모든 지점 간 이동 시간 계산
        self.get_logger().info("모든 지점 간 이동 시간 계산 중...")
        for start_name in ['current'] + waypoint_names:
            for end_name in waypoint_names:
                if start_name != end_name:
                    segment_key = f"{start_name}_{end_name}"
                    segment_info = self.calculate_segment_time(start_name, end_name)
                    if segment_info:
                        segment_data[segment_key] = segment_info
        
        # 모든 경로 조합의 총 시간 계산
        for perm in permutations:
            total_time = 0.0
            total_dist = 0.0
            valid_route = True
            
            # 현재 위치에서 첫 번째 지점까지
            first_segment_key = f"current_{perm[0]}"
            if first_segment_key not in segment_data:
                valid_route = False
                continue
                
            total_time += segment_data[first_segment_key]['time']
            total_dist += segment_data[first_segment_key]['distance']
            
            # 나머지 지점들 사이의 이동
            for i in range(len(perm) - 1):
                segment_key = f"{perm[i]}_{perm[i+1]}"
                if segment_key not in segment_data:
                    valid_route = False
                    break
                    
                total_time += segment_data[segment_key]['time']
                total_dist += segment_data[segment_key]['distance']
            
            if valid_route:
                route_times[perm] = {
                    'total_time': total_time,
                    'total_distance': total_dist
                }
        
        if not route_times:
            self.get_logger().error("유효한 경로를 찾을 수 없습니다.")
            return
            
        # 최적 경로 찾기
        best_route = min(route_times, key=lambda x: route_times[x]['total_time'])
        best_time = route_times[best_route]['total_time']
        best_distance = route_times[best_route]['total_distance']
        
        # 결과 저장
        output_data = {
            'current_position': {'x': 0.0, 'y': 0.0},
            'all_segments': [segment_data[key] for key in segment_data],
            'routes': [{
                'route': list(route),
                'total_time': info['total_time'],
                'total_distance': info['total_distance']
            } for route, info in route_times.items()],
            'best_route': list(best_route),
            'best_time': best_time,
            'best_distance': best_distance
        }

        with open('nav2_path_times.json', 'w') as f:
            json.dump(output_data, f, indent=4)
        
        # 결과 출력
        route_str = ' → '.join(['current'] + list(best_route))
        self.get_logger().info(f'최적 경로: {route_str}')
        self.get_logger().info(f'총 거리: {best_distance:.2f}m, 총 소요 시간: {best_time:.2f}초')
        
        # 최적 경로를 ROS 토픽으로 발행
        self.publish_optimal_route(best_route, best_time, best_distance)
        
        # 계산 완료 후 제자리로 이동
        self.return_to_home()
    
    def publish_optimal_route(self, best_route, best_time, best_distance):
        """최적 경로를 ROS 토픽으로 발행"""
        self.get_logger().info("최적 경로 토픽 발행 시작 (60초간 발행)")
        
        # 경로 정보 저장
        self.best_route = best_route
        self.best_time = best_time
        self.best_distance = best_distance
        
        # 발행 시작 시간 기록
        self.publish_start_time = time.time()
        
        # 타이머 생성 (1초마다 발행)
        self.publish_timer = self.create_timer(1.0, self.publish_route_callback)
        
        # 초기 발행 실행
        self.publish_route_callback()
    
    def publish_route_callback(self):
        """타이머에 의해 호출되는 경로 발행 콜백"""
        # 60초 경과 확인
        elapsed_time = time.time() - self.publish_start_time
        if elapsed_time > 60.0:
            self.get_logger().info("60초 경과, 경로 발행 종료")
            self.publish_timer.cancel()
            return
            
        # 남은 시간 계산
        remaining_time = 60.0 - elapsed_time
        
        # 문자열 메시지로 발행
        route_msg = String()
        route_str = ','.join(list(self.best_route))
        route_msg.data = f"optimal_route:{route_str}|time:{self.best_time:.2f}|distance:{self.best_distance:.2f}|remaining:{remaining_time:.1f}"
        self.optimal_route_publisher.publish(route_msg)
        
        # 경로 좌표를 Path 메시지로 발행
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 현재 위치 추가
        current_pose = self.get_current_pose()
        path_msg.poses.append(current_pose)
        
        # 경로 상의 각 지점 추가
        for point_name in self.best_route:
            point_pose = self.get_pose(point_name)
            path_msg.poses.append(point_pose)
            
        self.route_path_publisher.publish(path_msg)
        self.get_logger().info(f"최적 경로 발행 중... (남은 시간: {remaining_time:.1f}초)")
    
    def return_to_home(self):
        """계산 완료 후 로봇을 제자리(0,0)로 이동"""
        self.get_logger().info("계산 완료, 로봇을 제자리로 이동 중...")
        
        # 제자리 위치 설정
        home_pose = PoseStamped()
        home_pose.header.frame_id = "map"
        home_pose.header.stamp = self.get_clock().now().to_msg()
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.orientation.w = 1.0
        
        # 제자리로 이동 명령 발행
        self.goal_publisher.publish(home_pose)
        self.get_logger().info("제자리 이동 명령 발행 완료")

    def get_pose(self, name):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.points[name][0]
        pose.pose.position.y = self.points[name][1]
        pose.pose.orientation.w = 1.0
        return pose

    def load_waypoints(self, filename):
        """waypoint.txt 파일에서 웨이포인트 좌표 로드"""
        waypoints = {}
        try:
            with open(filename, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue  # 주석이나 빈 줄 무시
                        
                    parts = line.split(',')
                    if len(parts) >= 3:
                        name = parts[0].strip()
                        try:
                            x = float(parts[1].strip())
                            y = float(parts[2].strip())
                            waypoints[name] = [x, y]
                            self.get_logger().info(f"웨이포인트 로드: {name} ({x}, {y})")
                        except ValueError:
                            self.get_logger().error(f"웨이포인트 형식 오류: {line}")
            
            if not waypoints:
                self.get_logger().error(f"웨이포인트 파일에서 유효한 좌표를 찾을 수 없습니다: {filename}")
                # 기본 웨이포인트 설정
                waypoints = {
                    'a': [-1.0, -8.0],
                    'b': [-1.0, 3.0],
                    'c': [-16.4, -8.0],
                    'd': [16.4, 3.0]
                }
                self.get_logger().info("기본 웨이포인트를 사용합니다.")
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 로드 중 오류 발생: {str(e)}")
            # 기본 웨이포인트 설정
            waypoints = {
                'a': [-1.0, -8.0],
                'b': [-1.0, 3.0],
                'c': [-16.4, -8.0],
                'd': [16.4, 3.0]
            }
            self.get_logger().info("기본 웨이포인트를 사용합니다.")
            
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = Nav2OptimalPathFinder()
        
        # 노드 초기화 대기
        time.sleep(2.0)  # ROS2 시스템이 노드를 인식할 시간 부여
        
        # 경로 계산 시작
        node.get_logger().info("외판원 문제 해결 시작...")
        node.find_best_path()
        node.get_logger().info("경로 계산 완료, 60초간 발행 중...")
        
        # 토픽 발행이 완료될 때까지 대기
        publish_end_time = time.time() + 65.0  # 발행 시간 + 여유시간
        
        while rclpy.ok() and time.time() < publish_end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        node.get_logger().info("프로그램 종료")
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("사용자에 의해 프로그램 종료됨")
    except Exception as e:
        print(f"오류 발생: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
