"""
주행·팔 상호 인터락 (E301~E303) + 속도 오버라이드. rclpy 없이 순수 파이썬.

    il = SafetyInterlock()
    il.update_arm_state(t, pose, status)       # /arm/state
    il.update_odom(t, v, w)                    # /odom
    v, w, blocked = il.filter_drive(t, v, w)   # /cmd_vel -> /cmd_vel_checked
    ok, reason = il.request_arm(t, pose)       # /arm/pose_request -> /arm/pose_command
    results = il.evaluate(t)                   # [Judgement E301, E302, E303]
"""

from dataclasses import dataclass

from robot_field_toolkit.core.arm_pose import STOWED
from robot_field_toolkit.core.common import Debounce, Judgement

CODES = ('E301', 'E302', 'E303')


@dataclass
class InterlockThresholds:
    """인터락 기준값. 의미는 config/safety_interlock.yaml 주석 참고."""

    cmd_v_min: float = 0.05      # 이보다 큰 |v| 명령 = 주행 명령 [m/s]
    cmd_w_min: float = 0.1       # 이보다 큰 |w| 명령 = 주행 명령 [rad/s]
    moving_v: float = 0.02       # 실제 |v|가 이보다 크면 주행 중 [m/s]
    moving_w: float = 0.05       # 실제 |w|가 이보다 크면 주행 중 [rad/s]
    arm_timeout: float = 8.0     # E303: 팔이 이 시간 안에 목표에 도달 못 하면 [s]
    reject_hold: float = 3.0     # E302: 거부 후 알람을 보여 주는 시간 [s]
    clear_time: float = 1.0      # E301: 조건이 이만큼 사라져야 해제 [s]
    arm_state_timeout: float = 1.0  # 팔 상태가 이보다 오래 안 오면 '모름' = 접힘 아님 [s]
    odom_timeout: float = 0.5    # odom이 이보다 오래 안 오면 실제 속도는 0으로 봄 [s]


class SafetyInterlock:
    """주행 명령과 팔 요청 사이의 인터락."""

    def __init__(self, thresholds=None):
        self.th = thresholds or InterlockThresholds()
        self.speed_override = 100.0   # [%]
        self.work_inhibit = False     # 팔 작업 금지 (E401 도착 오차, E105 헛돎 등)
        self.inhibit_reason = ''
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.odom = (0.0, 0.0)
        self.odom_t = None
        self.arm_pose = None
        self.arm_status = None
        self.arm_elapsed = 0.0
        self.arm_t = None
        self.last_reject_t = None
        self.last_reject_reason = ''
        self._e301 = Debounce(0.0, self.th.clear_time)

    # ---------- 입력 ----------
    def update_arm_state(self, t, pose, status, elapsed=0.0):
        self.arm_pose = pose
        self.arm_status = status
        self.arm_elapsed = elapsed
        self.arm_t = t

    def update_odom(self, t, v, w):
        self.odom = (v, w)
        self.odom_t = t

    def set_work_inhibit(self, inhibit, reason=''):
        """팔 작업 금지 설정. reason 예: 'E401', 'E105'."""
        self.work_inhibit = bool(inhibit)
        self.inhibit_reason = reason if inhibit else ''

    def set_speed_override(self, percent):
        """속도 오버라이드 [%]. 0~100으로 자른다."""
        self.speed_override = min(100.0, max(0.0, float(percent)))

    # ---------- 상태 ----------
    def arm_known(self, t):
        return self.arm_t is not None and t - self.arm_t <= self.th.arm_state_timeout

    def arm_stowed(self, t):
        """팔이 접힘 자세에 도달해 있는가. 상태를 모르면 False (안전 쪽)."""
        return self.arm_known(t) and self.arm_pose == STOWED and self.arm_status == 'reached'

    def drive_commanded(self):
        return abs(self.cmd_v) > self.th.cmd_v_min or abs(self.cmd_w) > self.th.cmd_w_min

    def actually_moving(self, t):
        if self.odom_t is None or t - self.odom_t > self.th.odom_timeout:
            return False
        v, w = self.odom
        return abs(v) > self.th.moving_v or abs(w) > self.th.moving_w

    # ---------- 동작 ----------
    def filter_drive(self, t, v, w):
        """
        주행 명령 통과. (v_out, w_out, blocked)를 돌려준다.

        팔이 접힘이 아니면 0으로 막는다(E301). 통과하면 speed_override(%)를 곱한다.
        마지막 명령을 기억한다 (teleop은 키 누를 때만 보내고 로봇은 마지막 명령을 유지).
        """
        self.cmd_v, self.cmd_w = v, w
        if self.drive_commanded() and not self.arm_stowed(t):
            return 0.0, 0.0, True
        k = self.speed_override / 100.0
        return v * k, w * k, False

    def request_arm(self, t, pose):
        """
        팔 자세 요청. (허용 여부, 거부 이유)를 돌려준다.

        stowed(접기)는 위험을 줄이는 동작이라 항상 허용한다.
        주행 중이면 거부(E302). E401 작업 금지 중이면 거부.
        """
        if pose == STOWED:
            return True, ''
        if self.drive_commanded() or self.actually_moving(t):
            self.last_reject_t = t
            self.last_reject_reason = f'주행 중 팔 요청({pose}) 거부'
            return False, 'E302'
        if self.work_inhibit:
            return False, f'{self.inhibit_reason or "작업 금지"}로 팔 작업 금지'
        return True, ''

    # ---------- 판정 ----------
    def evaluate(self, t):
        th = self.th
        e301_cond = self.drive_commanded() and not self.arm_stowed(t)
        arm_txt = (f'팔 {self.arm_pose}/{self.arm_status}' if self.arm_known(t)
                   else '팔 상태 모름')
        e302 = self.last_reject_t is not None and t - self.last_reject_t < th.reject_hold
        e303 = (self.arm_known(t) and self.arm_status == 'moving'
                and self.arm_elapsed > th.arm_timeout)
        return [
            Judgement('E301', self._e301.update(t, e301_cond),
                      f'{arm_txt}, 명령 v {self.cmd_v:+.2f} w {self.cmd_w:+.2f}'),
            Judgement('E302', e302, self.last_reject_reason if e302 else ''),
            Judgement('E303', e303, f'{arm_txt}, {self.arm_elapsed:.1f}s 경과'
                      f' (기준 {th.arm_timeout:.1f}s)'),
        ]
