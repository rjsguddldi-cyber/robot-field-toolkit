"""core/scan_check.py 테스트."""

import math

import pytest
from robot_field_toolkit.core.scan_check import occlude, ScanChecker, valid_ratio

N = 360
RANGES = [2.0] * N
A_MIN = -math.pi
A_INC = 2 * math.pi / N


def feed(sc, start, duration, hz=10.0, ranges=RANGES):
    t = start
    result = None
    steps = int(round(duration * 10))  # 10Hz로 판정
    next_scan = start
    for i in range(steps):
        t = start + (i + 1) * 0.1
        while hz > 0 and next_scan <= t:
            sc.update_scan(next_scan, ranges, 0.15, 12.0)
            next_scan += 1.0 / hz
        result = {j.code: j.active for j in sc.evaluate(t)}
    return result, t


def test_valid_ratio():
    assert valid_ratio([1.0, math.inf, 0.1, 5.0], 0.15, 12.0) == pytest.approx(0.5)
    assert valid_ratio([], 0.15, 12.0) == 0.0


def test_occlude_half():
    out = occlude(RANGES, A_MIN, A_INC, 0.0, math.pi)
    assert valid_ratio(out, 0.15, 12.0) == pytest.approx(0.5, abs=0.01)
    assert math.isinf(out[N // 2])       # 정면(0 rad)은 가려짐
    assert out[0] == 2.0                 # 뒤쪽(-pi)은 그대로


def test_occlude_wraps_around_back():
    out = occlude(RANGES, A_MIN, A_INC, math.pi, math.radians(20))
    assert math.isinf(out[0]) and math.isinf(out[-1])


def test_normal_no_alarm():
    r, _ = feed(ScanChecker(), 0.0, 5.0)
    assert not any(r.values())


def test_e201_occlusion_raises_after_time():
    sc = ScanChecker()
    feed(sc, 0.0, 3.0)
    blocked = occlude(RANGES, A_MIN, A_INC, 0.0, math.radians(180))
    r, t = feed(sc, 3.0, 1.5, ranges=blocked)
    assert not r['E201'], '2초 전에는 안 뜸'
    r, t = feed(sc, t, 1.0, ranges=blocked)
    assert r['E201']
    assert not r['E202']


def test_e201_small_occlusion_no_alarm():
    blocked = occlude(RANGES, A_MIN, A_INC, 0.0, math.radians(90))  # 25%만 가림
    r, _ = feed(ScanChecker(), 0.0, 5.0, ranges=blocked)
    assert not r['E201']


def test_e202_low_rate():
    sc = ScanChecker()
    feed(sc, 0.0, 3.0)
    r, _ = feed(sc, 3.0, 3.0, hz=1.0)
    assert r['E202']


def test_e202_stopped():
    sc = ScanChecker()
    feed(sc, 0.0, 3.0)
    r, _ = feed(sc, 3.0, 2.0, hz=0.0)
    assert r['E202']
    assert not r['E201'], '끊겼을 때 옛 스캔으로 가림 판정하지 않음'


def test_no_e202_right_after_start():
    """시작 직후(rate_window 전)에는 주기 판정을 하지 않는다."""
    r, _ = feed(ScanChecker(), 0.0, 1.0, hz=0.0)
    assert not r['E202']


def test_e202_clears_after_recovery():
    sc = ScanChecker()
    feed(sc, 0.0, 3.0)
    r, t = feed(sc, 3.0, 3.0, hz=0.0)
    assert r['E202']
    r, _ = feed(sc, t, 4.0, hz=10.0)
    assert not r['E202']
