"""
바퀴 헛돎 판정 (E105). rclpy 없이 순수 파이썬.

바퀴 기반 odom과 LiDAR를 교차 확인한다.
odom은 일정 거리 이상 움직였다고 하는데 스캔(주변까지 거리)이 거의 그대로면
바퀴만 돌고 로봇은 안 움직인 것(벽에 막힘, 바닥 미끄러움)으로 본다.

E101(명령 대비 odom 속도)과 E401(TF 위치)은 둘 다 바퀴 정보를 믿기 때문에
헛돎을 못 잡는다 (DEVLOG 13번). 그래서 서로 다른 센서로 비교한다.

헛돎이 한 번 생기면 위치(odom, TF)가 이미 틀어졌으므로,
위치 재설정(/initialpose, rviz2 2D Pose Estimate) 전까지 알람을 유지한다.
"""

from collections import deque
from dataclasses import dataclass
import math
import statistics

from robot_field_toolkit.core.common import Judgement, normalize_angle

CODES = ('E105',)


def scan_change(ranges_a, ranges_b, range_min, range_max):
    """
    두 스캔에서 같은 빔끼리 거리 변화의 중앙값 [m]과 비교한 빔 비율.

    두 스캔 모두 유효한 빔만 쓴다 (가려진 빔, 측정 범위 밖은 뺀다).
    """
    diffs = [abs(a - b) for a, b in zip(ranges_a, ranges_b)
             if math.isfinite(a) and math.isfinite(b)
             and range_min <= a <= range_max and range_min <= b <= range_max]
    n = max(len(ranges_a), 1)
    if not diffs:
        return None, 0.0
    return statistics.median(diffs), len(diffs) / n


@dataclass
class SlipThresholds:
    """헛돎 판정 기준값. 의미는 config/motion_monitor.yaml 주석 참고."""

    slip_window: float = 2.0        # 이 시간 동안의 변화를 비교 [s]
    odom_dist_min: float = 0.15     # odom이 이만큼 이상 움직였다고 할 때만 판정 [m]
    scan_change_max: float = 0.03   # 스캔 변화 중앙값이 이보다 작으면 '안 움직임' [m]
    slip_yaw_max: float = 0.1       # 그동안 odom 회전이 이보다 크면 판정 안 함 [rad]
    scan_valid_min: float = 0.3     # 비교 가능한 빔 비율이 이보다 작으면 판정 안 함


class SlipChecker:
    """odom 이동량과 LiDAR 스캔 변화량을 비교한다."""

    def __init__(self, thresholds=None):
        self.th = thresholds or SlipThresholds()
        self._odom = deque()   # (t, x, y, yaw)
        self._scans = deque()  # (t, ranges, range_min, range_max)
        self.latched = False
        self.latch_detail = ''
        self.last_detail = '판정 전'

    def update_odom(self, t, x, y, yaw):
        """바퀴 기반 odom 위치 (odom 좌표계)."""
        self._odom.append((t, x, y, yaw))
        self._trim(self._odom, t)

    def update_scan(self, t, ranges, range_min, range_max):
        self._scans.append((t, tuple(ranges), range_min, range_max))
        self._trim(self._scans, t)

    def reset(self):
        """위치 재설정 후 호출: 알람과 지난 기록을 지운다."""
        self.latched = False
        self.latch_detail = ''
        self._odom.clear()
        self._scans.clear()

    def _trim(self, buf, t):
        while buf and buf[0][0] < t - self.th.slip_window - 1.0:
            buf.popleft()

    @staticmethod
    def _at_or_before(buf, t):
        """시각 t 이전의 가장 최근 기록."""
        best = None
        for item in buf:
            if item[0] <= t:
                best = item
            else:
                break
        return best

    def evaluate(self, t):
        if not self.latched:
            self._check(t)
        detail = self.latch_detail if self.latched else self.last_detail
        return [Judgement('E105', self.latched, detail)]

    def _check(self, t):
        th = self.th
        if not self._odom or not self._scans:
            self.last_detail = 'odom 또는 스캔 없음'
            return
        o_now, s_now = self._odom[-1], self._scans[-1]
        o_old = self._at_or_before(self._odom, o_now[0] - th.slip_window)
        s_old = self._at_or_before(self._scans, s_now[0] - th.slip_window)
        if o_old is None or s_old is None:
            self.last_detail = '기록 쌓는 중'
            return
        dist = math.hypot(o_now[1] - o_old[1], o_now[2] - o_old[2])
        dyaw = abs(normalize_angle(o_now[3] - o_old[3]))
        change, valid = scan_change(s_old[1], s_now[1], s_now[2], s_now[3])
        change_txt = '-' if change is None else f'{change * 100:.1f}cm'
        self.last_detail = (f'odom {dist * 100:.0f}cm 이동, 스캔 변화 {change_txt}'
                            f' (최근 {th.slip_window:.0f}s)')
        if dist < th.odom_dist_min or dyaw > th.slip_yaw_max:
            return
        if change is None or valid < th.scan_valid_min:
            return
        if change < th.scan_change_max:
            self.latched = True
            self.latch_detail = (self.last_detail
                                 + ' -> 바퀴만 돌고 로봇은 안 움직임. 위치 재설정 필요')
