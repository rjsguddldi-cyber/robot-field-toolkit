"""
2축 팔 자세 제어 노드. 자세 판단은 core/arm_pose.py.

구독: /arm/pose_command (String, 자세 이름. 3층부터는 safety_interlock이 보냄)
      /joint_states (sensor_msgs/JointState)
발행: /arm/shoulder_cmd, /arm/elbow_cmd (Float64 [rad], 브리지 -> Gazebo 관절 위치 제어)
      /arm/state (String, JSON: pose, status(moving/reached), elapsed, error)
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.arm_pose import ArmPoseTracker
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
import yaml

# 관절 이름 -> 명령 토픽
CMD_TOPICS = {'shoulder_joint': 'arm/shoulder_cmd', 'elbow_joint': 'arm/elbow_cmd'}


class ArmController(Node):
    """이름 붙은 자세 요청을 관절 명령으로 바꾸고 상태를 알린다."""

    def __init__(self):
        super().__init__('arm_controller')
        self.declare_parameter('poses_file', '')
        self.declare_parameter('state_rate', 5.0)

        poses_file = self.get_parameter('poses_file').value or os.path.join(
            get_package_share_directory('robot_field_toolkit'), 'config', 'arm_poses.yaml')
        with open(poses_file, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.tracker = ArmPoseTracker(cfg['poses'], cfg['joint_names'], cfg['tolerance'])

        self.cmd_pubs = {j: self.create_publisher(Float64, CMD_TOPICS[j], 10)
                         for j in cfg['joint_names']}
        self.state_pub = self.create_publisher(String, 'arm/state', 10)
        self.create_subscription(String, 'arm/pose_command', self.on_pose_command, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)

        self.create_timer(1.0 / self.get_parameter('state_rate').value, self.publish_state)
        self.create_timer(1.0, self.send_target)  # 목표를 1초마다 다시 보냄 (늦게 켜진 브리지 대비)
        self.get_logger().info(f'arm_controller 시작, 자세: {list(cfg["poses"])}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose_command(self, msg):
        name = msg.data.strip()
        try:
            self.tracker.request(name, self.now_sec())
        except KeyError:
            self.get_logger().warn(f"모르는 자세 '{name}'. 가능: {list(self.tracker.poses)}")
            return
        self.get_logger().info(f'자세 요청: {name}')
        self.send_target()

    def send_target(self):
        for joint, angle in self.tracker.target().items():
            self.cmd_pubs[joint].publish(Float64(data=float(angle)))

    def on_joint_states(self, msg):
        self.tracker.update_joints(dict(zip(msg.name, msg.position)))

    def publish_state(self):
        err = self.tracker.max_error()
        state = {
            'pose': self.tracker.target_name,
            'status': self.tracker.status(),
            'elapsed': round(self.tracker.elapsed(self.now_sec()), 2),
            'error': None if err is None else round(err, 3),
        }
        self.state_pub.publish(String(data=json.dumps(state)))


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
