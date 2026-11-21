"""
판정 공통 도구. rclpy 없이 순수 파이썬.

Judgement: 코드 하나의 판정 결과
Debounce: 조건이 일정 시간 계속돼야 켜고, 일정 시간 계속 사라져야 끈다
"""

from dataclasses import dataclass
import math


@dataclass
class Judgement:
    """코드 하나의 판정 결과."""

    code: str
    active: bool
    detail: str = ''


class Debounce:
    """조건이 on_time 동안 계속 참이면 켜고, off_time 동안 계속 거짓이면 끈다."""

    def __init__(self, on_time, off_time):
        self.on_time = on_time
        self.off_time = off_time
        self.active = False
        self._since = None  # 현재 상태와 반대인 조건이 시작된 시각

    def update(self, t, condition):
        """시각 t에 조건 값을 넣고 현재 켜짐 여부를 돌려준다."""
        if condition == self.active:
            self._since = None
            return self.active
        if self._since is None:
            self._since = t
        hold = self.off_time if self.active else self.on_time
        if t - self._since >= hold:
            self.active = condition
            self._since = None
        return self.active

    def reset(self):
        """꺼진 상태로 되돌린다."""
        self.active = False
        self._since = None


def normalize_angle(a):
    """각도를 -pi ~ pi로."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    """쿼터니언에서 yaw(z축 회전) [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
