from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """세 개의 노드를 함께 실행하는 launch 파일"""
    
    # Isaac Sim 물체 위치 발행 노드 (목업)
    isaac_sim_publisher_node = Node(
        package='ros2_coordinate_system',
        executable='isaac_sim_publisher.py',
        name='isaac_sim_publisher',
        output='screen'
    )
    
    # 좌표 변환 노드
    coordinate_transformer_node = Node(
        package='ros2_coordinate_system',
        executable='coordinate_transformer.py',
        name='coordinate_transformer',
        output='screen'
    )
    
    # 경로 계획 및 시간 계산 노드
    path_time_calculator_node = Node(
        package='ros2_coordinate_system',
        executable='path_time_calculator.py',
        name='path_time_calculator',
        output='screen'
    )
    
    return LaunchDescription([
        isaac_sim_publisher_node,
        coordinate_transformer_node,
        path_time_calculator_node
    ]) 