"""core/slip_check.py 테스트. 가상의 사각형 방에서 LiDAR 스캔을 계산해 넣는다."""

import math

from robot_field_toolkit.core.slip_check import scan_change, SlipChecker

N = 360
HALF = 5.0      # 10m x 10m 방
RMIN, RMAX = 0.15, 12.0


def room_scan(x, y, yaw):
    """(x, y, yaw)에서 본 사각형 방 벽까지 거리."""
    out = []
    for i in range(N):
        a = yaw - math.pi + i * 2 * math.pi / N
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        if abs(dx) > 1e-9:
            ts += [(HALF - x) / dx, (-HALF - x) / dx]
        if abs(dy) > 1e-9:
            ts += [(HALF - y) / dy, (-HALF - y) / dy]
        out.append(min(t for t in ts if t > 0))
    return out


def run(checker, start_t, duration, odom_from, odom_speed, real_speed, real_from=None):
    """
    odom은 odom_speed로, 실제 로봇은 real_speed로 +x 방향 이동 (10Hz).

    real_from: 실제 로봇 시작 x (기본: odom과 같음).
    """
    real_x = odom_from if real_from is None else real_from
    odom_x = odom_from
    t = start_t
    r = None
    for _ in range(int(duration * 10)):
        t += 0.1
        odom_x += odom_speed * 0.1
        real_x += real_speed * 0.1
        checker.update_odom(t, odom_x, 0.0, 0.0)
        checker.update_scan(t, room_scan(real_x, 0.3, 0.0), RMIN, RMAX)
        r = checker.evaluate(t)[0]
    return r, t, odom_x, real_x


def test_scan_change_zero_for_same_scan():
    s = room_scan(0, 0, 0)
    change, valid = scan_change(s, s, RMIN, RMAX)
    assert change == 0.0 and valid == 1.0


def test_scan_change_reflects_motion():
    change, _ = scan_change(room_scan(0, 0, 0), room_scan(0.15, 0, 0), RMIN, RMAX)
    assert change > 0.05


def test_normal_driving_no_e105():
    r, *_ = run(SlipChecker(), 0.0, 5.0, 0.0, 0.25, 0.25)
    assert not r.active


def test_stopped_no_e105():
    r, *_ = run(SlipChecker(), 0.0, 5.0, 0.0, 0.0, 0.0)
    assert not r.active


def test_wheel_spin_against_wall_raises_e105():
    """odom은 0.25m/s로 가는데 실제로는 벽에 막혀 그대로."""
    r, *_ = run(SlipChecker(), 0.0, 3.0, 0.0, 0.25, 0.0)
    assert r.active
    assert '위치 재설정' in r.detail


def test_partial_slip_not_flagged():
    """실제로 절반 속도라도 움직이면(스캔이 바뀜) E105 아님."""
    r, *_ = run(SlipChecker(), 0.0, 5.0, 0.0, 0.25, 0.12)
    assert not r.active


def test_e105_latched_until_reset():
    c = SlipChecker()
    r, t, ox, rx = run(c, 0.0, 3.0, 0.0, 0.25, 0.0)
    assert r.active
    r, t, *_ = run(c, t, 3.0, ox, 0.0, 0.0, real_from=rx)  # 멈춰도 유지
    assert r.active
    c.reset()  # 2D Pose Estimate
    r, *_ = run(c, t, 3.0, 0.0, 0.0, 0.0)
    assert not r.active


def test_rotation_not_judged():
    """제자리 회전 중에는 판정하지 않는다."""
    c = SlipChecker()
    t = 0.0
    for i in range(30):
        t += 0.1
        c.update_odom(t, 0.02 * i, 0.0, 0.5 * t)
        c.update_scan(t, room_scan(0.0, 0.0, 0.0), RMIN, RMAX)
        r = c.evaluate(t)[0]
    assert not r.active


def test_mostly_occluded_scan_not_judged():
    """비교 가능한 빔이 적으면(가림) 판정하지 않는다 (E201이 담당)."""
    c = SlipChecker()
    t = 0.0
    blocked = [math.inf] * 300 + room_scan(0, 0.3, 0)[300:]
    for i in range(30):
        t += 0.1
        c.update_odom(t, 0.025 * i, 0.0, 0.0)
        c.update_scan(t, blocked, RMIN, RMAX)
        r = c.evaluate(t)[0]
    assert not r.active
