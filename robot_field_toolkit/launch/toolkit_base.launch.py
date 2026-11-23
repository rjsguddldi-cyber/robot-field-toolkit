"""
툴킷 전체 실행 (시뮬레이션 시간). Gazebo 다음, SLAM/Nav2 전에 켠다.

명령:  명령원 -> /cmd_vel -> safety_interlock -> /cmd_vel_checked -> fault_injector -> /wheel_cmd
팔:    /arm/pose_request -> safety_interlock -> /arm/pose_command -> arm_controller
LiDAR: /scan_raw -> scan_fault_injector -> /scan   (SLAM·Nav2도 이 /scan을 쓰므로 꼭 켜야 함)
감시:  motion_monitor(E1xx), sensor_monitor(E2xx), safety_interlock(E3xx), nav_monitor(E4xx)
풋프린트: arm_footprint (팔이 펼쳐지거나 짐을 들면 Nav2 costmap 풋프린트 확대)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_field_toolkit'), 'config')
    sim = {'use_sim_time': True}

    def node(executable, params=None, extra=None):
        parameters = [os.path.join(cfg, params)] if params else []
        parameters.append({**sim, **(extra or {})})
        return Node(package='robot_field_toolkit', executable=executable, name=executable,
                    output='screen', parameters=parameters)

    return LaunchDescription([
        node('safety_interlock', 'safety_interlock.yaml'),
        node('fault_injector', extra={'input_topic': 'cmd_vel_checked'}),
        node('arm_controller'),
        node('scan_fault_injector', 'sensor_monitor.yaml'),
        node('motion_monitor', 'motion_monitor.yaml'),
        node('sensor_monitor', 'sensor_monitor.yaml'),
        node('nav_monitor', 'nav_monitor.yaml'),
        node('arm_footprint'),   # 팔 상태·하중별 Nav2 풋프린트 (PHM 추가 4)
        Node(package='diagnostic_aggregator', executable='aggregator_node',
             name='diagnostic_aggregator', output='screen',
             parameters=[os.path.join(cfg, 'diagnostic_aggregator.yaml'), sim]),
    ])
