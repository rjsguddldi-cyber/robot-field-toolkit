import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('my_robot_description')
    gazebo_dir = get_package_share_directory('my_robot_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(description_dir, 'urdf', 'my_robot.urdf.xacro')
    # wheel_wear: PHM 4단계 바퀴 마모율 (기본 0 = 마모 없음)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' wheel_wear:=', LaunchConfiguration('wheel_wear')]),
        value_type=str)

    world_file = os.path.join(gazebo_dir, 'worlds', 'simple_room.world')
    bridge_params = os.path.join(gazebo_dir, 'params', 'my_robot_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('wheel_wear', default_value='0.0',
                              description='바퀴 마모율 (0.03 = 반지름 3% 감소)'),
        # Gazebo 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r {world_file}',
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

        # 로봇 스폰
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

        # ROS ↔ Gazebo 토픽 브릿지
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
            output='screen',
        ),

        # joint_states 브릿지 (world 이름이 simple_room 으로 스코핑됨)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/simple_room/model/my_robot/joint_state'
                '@sensor_msgs/msg/JointState'
                '[gz.msgs.Model',
            ],
            remappings=[
                ('/world/simple_room/model/my_robot/joint_state', '/joint_states'),
            ],
            output='screen',
        ),
    ])