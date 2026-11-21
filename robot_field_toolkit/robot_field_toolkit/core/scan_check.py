"""
LiDAR 판정 (E201 가림, E202 끊김)과 고장 주입 계산. rclpy 없이 순수 파이썬.

    sc = ScanChecker()
    sc.update_scan(t, ranges, range_min, range_max)   # /scan 수신
    results = sc.evaluate(t)                          # [Judgement E201, E202]
"""

from collections import deque
from dataclasses import dataclass
import math

from robot_field_toolkit.core.common import Debounce, Judgement

CODES = ('E201', 'E202')


def valid_ratio(ranges, range_min, range_max):
    """유효 거리값(유한하고 range_min~range_max 안) 비율. 빈 스캔은 0."""
    if not ranges:
        return 0.0
    ok = sum(1 for r in ranges if math.isfinite(r) and range_min <= r <= range_max)
    return ok / len(ranges)


def occlude(ranges, angle_min, angle_increment, center, width):
    """
    center를 중심으로 width[rad] 구간의 값을 +inf(측정 없음)로 바꾼 새 목록.

    센서 창이 오염·가림된 상황을 흉내 낸다.
    """
    half = width / 2.0
    out = list(ranges)
    for i in range(len(out)):
        a = angle_min + i * angle_increment
        diff = math.atan2(math.sin(a - center), math.cos(a - center))
        if abs(diff) <= half:
            out[i] = math.inf
    return out


@dataclass
class ScanThresholds:
    """LiDAR 판정 기준값. 의미는 config/sensor_monitor.yaml 주석 참고."""

    valid_ratio_min: float = 0.6   # E201: 유효 거리값 비율이 이보다 작으면
    occlusion_time: float = 2.0    # E201: 지속 시간 [s]
    rate_window: float = 2.0       # E202: 수신 주기를 재는 구간 [s]
    min_rate_hz: float = 5.0       # E202: 수신 주기가 이보다 낮으면 [Hz] (정상 약 10Hz)
    stale_timeout: float = 1.0     # E202: 이 시간 넘게 하나도 안 오면 [s]
    clear_time: float = 1.0        # 조건이 이만큼 사라져야 해제 [s]


class ScanChecker:
    """스캔 내용과 수신 주기를 판정한다."""

    def __init__(self, thresholds=None):
        self.th = thresholds or ScanThresholds()
        self.times = deque()
        self.last_ratio = None
        self.start_t = None
        self._e201 = Debounce(self.th.occlusion_time, self.th.clear_time)
        self._e202 = Debounce(0.0, self.th.clear_time)

    def update_scan(self, t, ranges, range_min, range_max):
        self.times.append(t)
        self.last_ratio = valid_ratio(ranges, range_min, range_max)

    def rate(self, t):
        """최근 rate_window 동안의 수신 주기 [Hz]."""
        while self.times and self.times[0] < t - self.th.rate_window:
            self.times.popleft()
        return len(self.times) / self.th.rate_window

    def evaluate(self, t):
        th = self.th
        if self.start_t is None:
            self.start_t = t
        warmed_up = t - self.start_t >= th.rate_window  # 시작 직후에는 주기 판정 안 함
        rate = self.rate(t)
        last_t = self.times[-1] if self.times else self.start_t
        stale = t - last_t > th.stale_timeout
        e202 = warmed_up and (rate < th.min_rate_hz or stale)

        fresh = not stale and self.last_ratio is not None
        e201 = fresh and self.last_ratio < th.valid_ratio_min
        ratio_txt = '스캔 없음' if self.last_ratio is None else f'유효 {self.last_ratio * 100:.0f}%'
        return [
            Judgement('E201', self._e201.update(t, e201),
                      f'{ratio_txt} (기준 {th.valid_ratio_min * 100:.0f}%)'),
            Judgement('E202', self._e202.update(t, e202),
                      f'{rate:.1f}Hz, 마지막 수신 {t - last_t:.1f}s 전'
                      f' (기준 {th.min_rate_hz:.1f}Hz)'),
        ]
