"""
팔 상태·하중별 Nav2 풋프린트 발행 (PHM 추가 4). 판정은 core/footprint.py.

입력: /arm/state (JSON pose, status), /field_toolkit/load_state (String, 마지막 값 유지)
출력: /local_costmap/footprint, /global_costmap/footprint (geometry_msgs/Polygon)
      바뀔 때 바로, 그리고 2초마다 다시 (costmap이 늦게 켜져도 받게)
rviz2에서는 costmap의 published_footprint(로봇 외곽선)로 확인한다.
"""

import json

from geometry_msgs.msg import Point32, Polygon
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_field_toolkit.core.footprint import footprint_for
from std_msgs.msg import String

TOPICS = ('local_costmap/footprint', 'global_costmap/footprint')


class ArmFootprint(Node):
    """팔 상태가 바뀌면 costmap 풋프린트를 바꾼다."""

    def __init__(self):
        super().__init__('arm_footprint')
        self.pubs = [self.create_publisher(Polygon, t, 10) for t in TOPICS]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'arm/state', self.on_arm, 10)
        self.create_subscription(String, 'field_toolkit/load_state', self.on_load, latched)
        self.pose, self.status, self.load = 'stowed', 'reached', 'empty'
        self.current = None
        self.create_timer(2.0, self.publish)

    def on_arm(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.pose, self.status = s.get('pose'), s.get('status')
        self.update()

    def on_load(self, msg):
        self.load = msg.data or 'empty'
        self.update()

    def update(self):
        name, _ = footprint_for(self.pose, self.status, self.load)
        if name != self.current:
            self.get_logger().info(
                f'풋프린트 {self.current} -> {name} (팔 {self.pose}/{self.status}, 하중 {self.load})')
            self.current = name
            self.publish()

    def publish(self):
        _, pts = footprint_for(self.pose, self.status, self.load)
        poly = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in pts])
        for p in self.pubs:
            p.publish(poly)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFootprint()
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
