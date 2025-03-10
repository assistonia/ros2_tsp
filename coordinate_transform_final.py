import numpy as np

# 최적화된 변환 파라미터
params = [1.050398, 0.052001, 80.194741, -0.021784, 1.011847, -62.842775]

def isaac_to_ros2(isaac_x, isaac_y):
    '''
    Isaac Sim 좌표를 ROS2 좌표로 변환
    
    Parameters:
        isaac_x: Isaac Sim의 X 좌표
        isaac_y: Isaac Sim의 Y 좌표
        
    Returns:
        (ros2_x, ros2_y): ROS2 좌표
    '''
    a, b, c, d, e, f = params
    ros2_x = a * isaac_x + b * isaac_y + c
    ros2_y = d * isaac_x + e * isaac_y + f
    return ros2_x, ros2_y

def ros2_to_isaac(ros2_x, ros2_y):
    '''
    ROS2 좌표를 Isaac Sim 좌표로 변환
    
    Parameters:
        ros2_x: ROS2의 X 좌표
        ros2_y: ROS2의 Y 좌표
        
    Returns:
        (isaac_x, isaac_y): Isaac Sim 좌표
    '''
    a, b, c, d, e, f = params
    
    # 역변환 행렬 계산
    det = a*e - b*d
    if abs(det) < 1e-10:  # 행렬식이 0에 가까우면 역행렬이 존재하지 않음
        print("경고: 변환 행렬의 행렬식이 0에 가까워 정확한 역변환이 불가능합니다.")
        return None
    
    # 역변환 계수 계산
    inv_a = e / det
    inv_b = -b / det
    inv_c = (b*f - c*e) / det
    inv_d = -d / det
    inv_e = a / det
    inv_f = (c*d - a*f) / det
    
    # 역변환 적용
    isaac_x = inv_a * ros2_x + inv_b * ros2_y + inv_c
    isaac_y = inv_d * ros2_x + inv_e * ros2_y + inv_f
    
    return isaac_x, isaac_y

# 사용 예시
if __name__ == "__main__":
    # Isaac Sim 좌표를 ROS2 좌표로 변환
    isaac_coord = (-52.43, 134.70)  # obj2의 Isaac Sim 좌표
    ros2_coord = isaac_to_ros2(*isaac_coord)
    print(f"Isaac Sim 좌표 {isaac_coord} -> ROS2 좌표 ({ros2_coord[0]:.2f}, {ros2_coord[1]:.2f})")
    
    # ROS2 좌표를 Isaac Sim 좌표로 변환
    isaac_reverse = ros2_to_isaac(*ros2_coord)
    print(f"ROS2 좌표 ({ros2_coord[0]:.2f}, {ros2_coord[1]:.2f}) -> Isaac Sim 좌표 ({isaac_reverse[0]:.2f}, {isaac_reverse[1]:.2f})")
    
    # Obj 데이터 검증
    obj1_isaac = (-25.54, 119.27)
    obj2_isaac = (-52.43, 134.70)
    
    print("\n각 Object의 Isaac Sim 좌표에서 예상되는 ROS2 좌표:")
    print(f"obj1: Isaac Sim {obj1_isaac} -> ROS2 좌표 ({isaac_to_ros2(*obj1_isaac)[0]:.2f}, {isaac_to_ros2(*obj1_isaac)[1]:.2f})")
    print(f"obj2: Isaac Sim {obj2_isaac} -> ROS2 좌표 ({isaac_to_ros2(*obj2_isaac)[0]:.2f}, {isaac_to_ros2(*obj2_isaac)[1]:.2f})")
