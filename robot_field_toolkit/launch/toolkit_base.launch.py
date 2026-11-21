"""1층 툴킷 실행: fault_injector + motion_monitor + diagnostic_aggregator (시뮬레이션 시간)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_field_toolkit')
    monitor_params = os.path.join(pkg_dir, 'config', 'motion_monitor.yaml')
    aggregator_params = os.path.join(pkg_dir, 'config', 'diagnostic_aggregator.yaml')

    return LaunchDescription([
        Node(
            package='robot_field_toolkit',
            executable='fault_injector',
            name='fault_injector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='robot_field_toolkit',
            executable='motion_monitor',
            name='motion_monitor',
            output='screen',
            parameters=[monitor_params, {'use_sim_time': True}],
        ),
        # /diagnostics -> /diagnostics_agg (rqt_robot_monitor가 보는 토픽)
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[aggregator_params, {'use_sim_time': True}],
        ),
    ])
