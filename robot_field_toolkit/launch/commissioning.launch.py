"""MoMa 사이클 시운전 러너 실행. toolkit_base, Gazebo, Nav2가 켜진 뒤에 실행한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('robot_field_toolkit'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('note', default_value='', description='리포트 메모'),
        Node(package='robot_field_toolkit', executable='commissioning_runner',
             name='commissioning_runner', output='screen',
             parameters=[os.path.join(cfg, 'commissioning.yaml'),
                         {'use_sim_time': True, 'note': LaunchConfiguration('note')}]),
    ])
