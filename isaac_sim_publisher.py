#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import random
import os

class IsaacSimPublisher(Node):
    """
    Isaac Sim에서 물체 위치를 발행하는 목업 노드
    실제로는 Isaac Sim과 연동되지만, 여기서는 목업 데이터를 발행
    """
    
    def __init__(self):
        super().__init__('isaac_sim_publisher')
        
        # 좌표 파일 경로
        self.coord_file = 'publish_issacsim.txt'
        
        # 표준 위치 데이터 정의 (Isaac Sim 좌표계)
        self.standard_pos = self.load_positions_from_file()
        
        if not self.standard_pos:
            # 파일이 없는 경우 기본 데이터 사용
            self.standard_pos = {
                'Chungmu1': [-25.54326, 119.27288, 1.0502],
                'Chungmu2': [-52.4361, 134.70523, 1.03068],
                'Object1': [-82.71, 86.87, 1.0],
                'Object2': [-91.37, 96.45, 1.0],
                'Object3': [-83.19, 86.85, 1.0],
                'Object4': [-75.75, 79.82, 1.0],
                'Object5': [-84.44, 93.18, 1.0]
            }
            
            # 기본 데이터를 파일로 저장
            self.save_positions_to_file()
        
        # 발행자 설정
        self.position_publisher = self.create_publisher(
            String,
            '/isaac_sim/object_positions',  # 물체 위치 발행 토픽
            10
        )
        
        # 위치 발행 타이머 (1초마다)
        self.timer = self.create_timer(1.0, self.publish_positions)
        
        # 파일 변경 감지 타이머 (5초마다)
        self.file_check_timer = self.create_timer(5.0, self.check_file_change)
        self.last_modified_time = self.get_file_modified_time()
        
        # 랜덤 변화량 추가를 위한 파라미터
        self.add_noise = False  # 위치에 랜덤 변화를 추가할지 여부
        self.noise_level = 0.1  # 변화량 수준 (미터)
        
        self.get_logger().info("IsaacSimPublisher 초기화 완료")
        self.get_logger().info(f"물체 {len(self.standard_pos)}개 로드 완료")
    
    def load_positions_from_file(self):
        """텍스트 파일에서 물체 위치 정보를 로드"""
        positions = {}
        
        try:
            if not os.path.exists(self.coord_file):
                return {}
                
            with open(self.coord_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    # 주석이나 빈 줄 건너뛰기
                    if not line or line.startswith('#'):
                        continue
                    
                    parts = line.split(',')
                    if len(parts) >= 4:
                        name = parts[0].strip()
                        try:
                            x = float(parts[1])
                            y = float(parts[2])
                            z = float(parts[3])
                            positions[name] = [x, y, z]
                        except ValueError:
                            self.get_logger().warning(f"잘못된 좌표 형식: {line}")
            
            return positions
        except Exception as e:
            self.get_logger().error(f"파일 로드 오류: {e}")
            return {}
    
    def save_positions_to_file(self):
        """물체 위치 정보를 텍스트 파일로 저장"""
        try:
            with open(self.coord_file, 'w') as f:
                f.write("# 물체 좌표 데이터 (Isaac Sim 좌표계)\n")
                f.write("# 형식: 이름,x좌표,y좌표,z좌표\n\n")
                
                f.write("# 참조 물체\n")
                for name in ['Chungmu1', 'Chungmu2']:
                    if name in self.standard_pos:
                        pos = self.standard_pos[name]
                        f.write(f"{name},{pos[0]},{pos[1]},{pos[2]}\n")
                
                f.write("\n# 이동 대상 물체\n")
                for name in self.standard_pos:
                    if name not in ['Chungmu1', 'Chungmu2']:
                        pos = self.standard_pos[name]
                        f.write(f"{name},{pos[0]},{pos[1]},{pos[2]}\n")
                
                f.write("\n# 추가 물체를 여기에 작성할 수 있습니다\n")
                f.write("# 예: NewObject,-50.0,100.0,1.0\n")
                
            self.get_logger().info(f"좌표 파일 저장 완료: {self.coord_file}")
        except Exception as e:
            self.get_logger().error(f"파일 저장 오류: {e}")
    
    def get_file_modified_time(self):
        """파일의 마지막 수정 시간 반환"""
        try:
            if os.path.exists(self.coord_file):
                return os.path.getmtime(self.coord_file)
            return 0
        except:
            return 0
    
    def check_file_change(self):
        """파일 변경 감지 및 다시 로드"""
        current_time = self.get_file_modified_time()
        if current_time > self.last_modified_time:
            self.get_logger().info(f"좌표 파일 변경 감지. 다시 로드합니다.")
            self.standard_pos = self.load_positions_from_file()
            self.last_modified_time = current_time
            self.get_logger().info(f"물체 {len(self.standard_pos)}개 로드 완료")
    
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
        # 너무 많은 로그가 출력되므로 주석 처리
        # self.get_logger().info(f"Isaac Sim 물체 위치 발행: {len(current_pos)} 물체")

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