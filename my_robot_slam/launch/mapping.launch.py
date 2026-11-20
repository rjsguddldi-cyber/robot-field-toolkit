import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    slam_dir = get_package_share_directory('my_robot_slam')

    slam_params = os.path.join(slam_dir, 'params', 'slam_toolbox.yaml')

    return LaunchDescription([

        # slam_toolbox 비동기 맵핑 노드 (lifecycle 노드)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True}],
        ),

        # lifecycle manager: slam_toolbox 자동 configure → activate
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 0.0,
            }],
        ),
    ])
