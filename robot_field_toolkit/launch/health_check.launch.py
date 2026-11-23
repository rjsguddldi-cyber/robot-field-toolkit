"""PHM 정기 건강 점검 러너 실행. toolkit_base, Gazebo, Nav2가 켜진 뒤에 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_field_toolkit'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='20', description='사이클 수'),
        DeclareLaunchArgument('load_state', default_value='empty', description='하중 상태'),
        DeclareLaunchArgument('csv_name', default_value='',
                              description='이어 쓸 CSV 이름 (비우면 새 파일)'),
        DeclareLaunchArgument('condition', default_value='',
                              description='시험 조건 표시 (예: wheel_wear=0.03)'),
        Node(package='robot_field_toolkit', executable='health_runner',
             name='health_runner', output='screen',
             parameters=[os.path.join(cfg, 'health_runner.yaml'), {
                 'use_sim_time': True,
                 'cycles': ParameterValue(LaunchConfiguration('cycles'), value_type=int),
                 'load_state': LaunchConfiguration('load_state'),
                 'csv_name': LaunchConfiguration('csv_name'),
                 'condition': LaunchConfiguration('condition')}]),
    ])
