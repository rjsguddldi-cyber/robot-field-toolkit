"""
주행 감시 노드 (E401~E403). 판정은 core/nav_check.py.

구독:
    /navigate_to_pose/_action/status    목표 상태 (action_msgs/GoalStatusArray)
    /navigate_to_pose/_action/feedback  남은 거리, 복구 횟수
    /plan                               목표 좌표 = 경로 마지막 점
    /amcl_pose                          위치 공분산 (E402)
    TF map -> base_footprint            현재 위치 (E401 도착 오차)
발행:
    /field_toolkit/arm_work_inhibit     Bool, E401이면 True (safety_interlock이 팔 요청 거부)
    /diagnostics, /field_toolkit/alarms
"""

from dataclasses import fields

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from robot_field_toolkit.core.common import yaw_from_quaternion
from robot_field_toolkit.core.nav_check import NavChecker, NavThresholds
from robot_field_toolkit.nodes.alarm_reporter import AlarmReporter
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(q):
    return yaw_from_quaternion(q.x, q.y, q.z, q.w)


class NavMonitor(Node):
    """Nav2 목표 도착 정확도, 위치 추정, 경로 진척을 감시한다."""

    def __init__(self):
        super().__init__('nav_monitor')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('alarm_codes_file', '')
        defaults = NavThresholds()
        values = {}
        for f in fields(NavThresholds):
            self.declare_parameter(f.name, getattr(defaults, f.name))
            values[f.name] = self.get_parameter(f.name).value
        self.checker = NavChecker(NavThresholds(**values))
        self.reporter = AlarmReporter(self, 'nav', self.get_parameter('alarm_codes_file').value)

        action = self.get_parameter('action_name').value
        self.create_subscription(
            GoalStatusArray, f'{action}/_action/status', self.on_status, 10)
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage, f'{action}/_action/feedback',
            self.on_feedback, 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)
        self.inhibit_pub = self.create_publisher(Bool, 'field_toolkit/arm_work_inhibit', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.evaluate)
        self.get_logger().info('nav_monitor 시작')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        if not msg.status_list:
            return
        latest = max(msg.status_list,
                     key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
        self.checker.update_status(self.now_sec(), goal_id, latest.status)

    def on_feedback(self, msg):
        fb = msg.feedback
        self.checker.update_feedback(
            self.now_sec(), fb.distance_remaining, fb.number_of_recoveries)

    def on_plan(self, msg):
        if msg.poses:
            p = msg.poses[-1].pose
            self.checker.set_goal(p.position.x, p.position.y, _yaw(p.orientation))

    def on_amcl(self, msg):
        c = msg.pose.covariance
        self.checker.update_covariance(c[0], c[7], c[35])

    def update_pose_from_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('global_frame').value,
                self.get_parameter('base_frame').value, rclpy.time.Time())
        except TransformException:
            return
        tr = tf.transform.translation
        self.checker.update_pose(tr.x, tr.y, _yaw(tf.transform.rotation))

    def evaluate(self):
        t = self.now_sec()
        if t <= 0.0:
            return
        self.update_pose_from_tf()
        self.reporter.update(t, self.checker.evaluate(t))
        self.inhibit_pub.publish(Bool(data=self.checker.work_inhibit()))


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
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
