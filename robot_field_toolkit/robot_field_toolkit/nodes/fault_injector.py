"""
구동 고장 주입 노드 (E1xx 시험용).

/cmd_vel(상위 명령)을 받아서, mode 파라미터에 따라 일부러 망가뜨린 뒤
/wheel_cmd(구동기 입력)로 내보낸다. 브리지가 /wheel_cmd를 Gazebo cmd_vel로 넘긴다.

    명령원(teleop) -> /cmd_vel -> [fault_injector] -> /wheel_cmd -> 브리지 -> Gazebo

mode:
    normal : 그대로 전달
    stuck  : 속도 0 (바퀴 고착, 브레이크 미해제)
    weak   : 선속도/각속도 40% (모터 출력 저하)
    drop   : 50% 확률로 메시지 버림 (명령 전달 손실)
    drift  : 각속도 +0.3 rad/s (한쪽 바퀴 이상)

실행 중 변경: ros2 param set /fault_injector mode stuck
"""

import random

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 고장 모드별 숫자. 한곳에 모아 두면 나중에 바꾸기 쉽다.
KNOWN_MODES = ('normal', 'stuck', 'weak', 'drop', 'drift')
WEAK_RATIO = 0.4        # weak: 명령의 40%만 전달
DROP_PROBABILITY = 0.5  # drop: 50% 확률로 버림
DRIFT_W = 0.3           # drift: 각속도에 더하는 값 [rad/s]


class FaultInjector(Node):
    """/cmd_vel을 mode에 따라 변형해서 /wheel_cmd로 보내는 노드."""

    def __init__(self):
        super().__init__('fault_injector')

        self.declare_parameter('mode', 'normal')
        self.warned_mode = None  # 같은 경고를 매 메시지마다 찍지 않으려고 기억

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.wheel_pub = self.create_publisher(Twist, 'wheel_cmd', 10)
        self.mode_pub = self.create_publisher(String, '~/mode', 10)

        # 1Hz로 현재 mode 알림 (나중에 블랙박스가 기록)
        self.mode_timer = self.create_timer(1.0, self.publish_mode)

        self.get_logger().info(f'fault_injector 시작, mode={self.current_mode()}')

    def current_mode(self):
        """현재 mode 파라미터를 읽는다. 모르는 값이면 경고 후 'normal'."""
        mode = self.get_parameter('mode').get_parameter_value().string_value
        if mode in KNOWN_MODES:
            self.warned_mode = None
            return mode
        if mode != self.warned_mode:
            self.get_logger().warn(
                f"알 수 없는 mode '{mode}' -> normal로 동작. 가능: {', '.join(KNOWN_MODES)}")
            self.warned_mode = mode
        return 'normal'

    def on_cmd_vel(self, msg):
        """/cmd_vel 한 개가 들어올 때마다 호출된다."""
        mode = self.current_mode()

        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = msg.angular.z

        if mode == 'stuck':
            out.linear.x = 0.0
            out.angular.z = 0.0
        elif mode == 'weak':
            out.linear.x *= WEAK_RATIO
            out.angular.z *= WEAK_RATIO
        elif mode == 'drop':
            if random.random() < DROP_PROBABILITY:
                return  # 발행하지 않고 버림
        elif mode == 'drift':
            out.angular.z += DRIFT_W

        self.wheel_pub.publish(out)

    def publish_mode(self):
        """현재 mode를 /fault_injector/mode로 발행."""
        self.mode_pub.publish(String(data=self.current_mode()))


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
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
