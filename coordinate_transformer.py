#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

class CoordinateTransformer(Node):
    """
    Isaac Sim 좌표계에서 ROS2 좌표계로 변환하는 노드
    """
    
    def __init__(self):
        super().__init__('coordinate_transformer')
        
        # 변환 파라미터 설정 (coordinate_transform_final.py에서 가져옴)
        self.params = [1.050398, 0.052001, 80.194741, -0.021784, 1.011847, -62.842775]
        
        # 구독자 설정
        self.position_subscription = self.create_subscription(
            String,
            '/isaac_sim/object_positions',  # Isaac Sim 물체 위치 토픽
            self.position_callback,
            10
        )
        
        # 발행자 설정
        self.ros2_position_publisher = self.create_publisher(
            String,
            '/object_positions',  # 변환된 ROS2 물체 위치 토픽
            10
        )
        
        self.get_logger().info("CoordinateTransformer 초기화 완료")
    
    def isaac_to_ros2(self, isaac_x, isaac_y):
        """
        Isaac Sim 좌표를 ROS2 좌표로 변환
        
        Parameters:
            isaac_x: Isaac Sim의 X 좌표
            isaac_y: Isaac Sim의 Y 좌표
            
        Returns:
            (ros2_x, ros2_y): ROS2 좌표
        """
        a, b, c, d, e, f = self.params
        ros2_x = a * isaac_x + b * isaac_y + c
        ros2_y = d * isaac_x + e * isaac_y + f
        return ros2_x, ros2_y
    
    def position_callback(self, msg):
        """
        위치 정보를 받아 좌표 변환하고 발행하는 콜백 함수
        """
        try:
            # JSON 문자열을 파싱
            isaac_positions = json.loads(msg.data)
            
            # ROS2 좌표계로 변환
            ros2_positions = {}
            for obj_name, pos in isaac_positions.items():
                isaac_x, isaac_y, isaac_z = pos
                ros2_x, ros2_y = self.isaac_to_ros2(isaac_x, isaac_y)
                ros2_positions[obj_name] = [ros2_x, ros2_y, isaac_z]  # Z 좌표는 그대로 유지
            
            # 변환된 좌표를 JSON 문자열로 변환
            ros2_position_msg = String()
            ros2_position_msg.data = json.dumps(ros2_positions)
            
            # 발행
            self.ros2_position_publisher.publish(ros2_position_msg)
            # 너무 많은 로그가 출력되므로 주석 처리
            # self.get_logger().info(f"ROS2 좌표로 변환 완료: {len(ros2_positions)} 물체")
            
            # 로그로 변환 결과 예시 출력 (첫 번째 물체만)
            if ros2_positions and False:  # 로그 비활성화
                obj_name = list(ros2_positions.keys())[0]
                isaac_pos = isaac_positions[obj_name]
                ros2_pos = ros2_positions[obj_name]
                self.get_logger().debug(
                    f"변환 예시 - {obj_name}: "
                    f"Isaac ({isaac_pos[0]:.2f}, {isaac_pos[1]:.2f}) -> "
                    f"ROS2 ({ros2_pos[0]:.2f}, {ros2_pos[1]:.2f})"
                )
        
        except Exception as e:
            self.get_logger().error(f"좌표 변환 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    coordinate_transformer = CoordinateTransformer()
    
    try:
        rclpy.spin(coordinate_transformer)
    except KeyboardInterrupt:
        pass
    finally:
        coordinate_transformer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 