#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random

class IsaacSimPublisher(Node):
    """
    Isaac Sim에서 물체 위치를 발행하는 목업 노드
    실제로는 Isaac Sim과 연동되지만, 여기서는 목업 데이터를 발행
    """
    
    def __init__(self):
        super().__init__('isaac_sim_publisher')
        
        # 표준 위치 데이터 정의 (Isaac Sim 좌표계)
        self.standard_pos = {
            'Chungmu1': [-25.54326, 119.27288, 1.0502],
            'Chungmu2': [-52.4361, 134.70523, 1.03068],
            'Object1': [-82.71, 86.87, 1.0],
            'Object2': [-91.37, 96.45, 1.0],
            'Object3': [-83.19, 86.85, 1.0],
            'Object4': [-75.75, 79.82, 1.0],
            'Object5': [-84.44, 93.18, 1.0]
        }
        
        # 발행자 설정
        self.position_publisher = self.create_publisher(
            String,
            '/isaac_sim/object_positions',  # 물체 위치 발행 토픽
            10
        )
        
        # 위치 발행 타이머 (1초마다)
        self.timer = self.create_timer(1.0, self.publish_positions)
        
        # 랜덤 변화량 추가를 위한 파라미터
        self.add_noise = False  # 위치에 랜덤 변화를 추가할지 여부
        self.noise_level = 0.1  # 변화량 수준 (미터)
        
        self.get_logger().info("IsaacSimPublisher 초기화 완료")
    
    def publish_positions(self):
        """물체 위치를 발행하는 콜백 함수"""
        # 원본 데이터 복사
        current_pos = self.standard_pos.copy()
        
        # 랜덤 변화량 추가 (시뮬레이션에서 약간의 변화 재현)
        if self.add_noise:
            for obj_name, pos in current_pos.items():
                noise_x = random.uniform(-self.noise_level, self.noise_level)
                noise_y = random.uniform(-self.noise_level, self.noise_level)
                current_pos[obj_name][0] += noise_x
                current_pos[obj_name][1] += noise_y
        
        # 위치 정보를 JSON 문자열로 변환
        position_msg = String()
        position_msg.data = json.dumps(current_pos)
        
        # 발행
        self.position_publisher.publish(position_msg)
        self.get_logger().info(f"Isaac Sim 물체 위치 발행: {len(current_pos)} 물체")

def main(args=None):
    rclpy.init(args=args)
    isaac_sim_publisher = IsaacSimPublisher()
    
    try:
        rclpy.spin(isaac_sim_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_sim_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 