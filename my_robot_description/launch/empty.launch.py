import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    description_dir = get_package_share_directory('my_robot_description')
    ros_gz_sim_dir  = get_package_share_directory('ros_gz_sim')

    urdf_file         = os.path.join(description_dir, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([

        # Gazebo 실행 (빈 월드)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': '-r empty.sdf',
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # URDF → /robot_description 토픽 + TF 발행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
            ],
        ),

        # /robot_description을 읽어 Gazebo에 로봇 스폰
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1',
            ],
            output='screen',
        ),
    ])
