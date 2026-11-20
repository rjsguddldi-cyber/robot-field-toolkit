import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav_dir = get_package_share_directory('my_robot_navigation')
    params  = os.path.join(nav_dir, 'params', 'nav2_params.yaml')

    # 지도는 저장소 안(my_robot_navigation/maps)에 두고 패키지 경로로 찾는다.
    default_map = os.path.join(nav_dir, 'maps', 'simple_room.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file',
    )

    sim = {'use_sim_time': True}

    # ── Localization ─────────────────────────────────────────────────────────

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params, sim, {'yaml_filename': LaunchConfiguration('map')}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params, sim],
    )

    lifecycle_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{**sim,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 0.0,
        }],
    )

    # ── Navigation ────────────────────────────────────────────────────────────

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params, sim],
        # 스무더 입력으로 보냄 (공식 nav2_bringup 방식, 되먹임 고리 방지)
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params, sim],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params, sim],
    )

    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params, sim],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params, sim],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params, sim],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params, sim],
        # 입력 /cmd_vel_nav -> 출력 /cmd_vel (Nav2가 /cmd_vel에 내는 발행자는 이것 하나)
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{**sim,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        map_arg,
        map_server,
        amcl,
        lifecycle_loc,
        controller,
        smoother,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        # 주행 노드 켜기를 10초 늦춘다. 바로 켜면 AMCL이 map TF를 내기 전에
        # global_costmap 활성화가 실패해 Nav2 전체가 멈춘다(DEVLOG 11/20).
        TimerAction(period=10.0, actions=[lifecycle_nav]),
    ])
