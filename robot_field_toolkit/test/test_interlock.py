"""core/interlock.py 테스트. 인터락이 막은 명령 때문에 E101이 뜨지 않는지도 확인한다."""

import pytest
from robot_field_toolkit.core.interlock import InterlockThresholds, SafetyInterlock
from robot_field_toolkit.core.motion_check import MotionChecker

DT = 0.05


def active(il, t):
    return {j.code: j.active for j in il.evaluate(t)}


def stowed(il, t):
    il.update_arm_state(t, 'stowed', 'reached')


def extended(il, t, status='reached', elapsed=0.0):
    il.update_arm_state(t, 'work', status, elapsed)


# ---------- E301 ----------
def test_drive_passes_when_stowed():
    il = SafetyInterlock()
    stowed(il, 0.0)
    v, w, blocked = il.filter_drive(0.0, 0.3, 0.2)
    assert (v, w, blocked) == (0.3, 0.2, False)
    assert not active(il, 0.0)['E301']


def test_e301_blocks_drive_when_arm_extended():
    il = SafetyInterlock()
    extended(il, 0.0)
    v, w, blocked = il.filter_drive(0.0, 0.3, 0.0)
    assert (v, w, blocked) == (0.0, 0.0, True)
    assert active(il, 0.0)['E301']


def test_e301_blocks_when_arm_state_unknown():
    """팔 상태를 모르면 안전 쪽(막음)."""
    il = SafetyInterlock()
    _, _, blocked = il.filter_drive(0.0, 0.3, 0.0)
    assert blocked


def test_e301_blocks_when_arm_state_stale():
    il = SafetyInterlock()
    stowed(il, 0.0)
    _, _, blocked = il.filter_drive(2.0, 0.3, 0.0)  # 팔 상태가 2초 전 것
    assert blocked


def test_stop_command_not_blocked_and_no_e301():
    il = SafetyInterlock()
    extended(il, 0.0)
    v, w, blocked = il.filter_drive(0.0, 0.0, 0.0)
    assert not blocked
    assert not active(il, 0.0)['E301']


def test_e301_clears_after_clear_time():
    il = SafetyInterlock()
    extended(il, 0.0)
    il.filter_drive(0.0, 0.3, 0.0)
    assert active(il, 0.0)['E301']
    stowed(il, 0.5)
    il.filter_drive(0.5, 0.3, 0.0)
    assert active(il, 0.5)['E301'], '해제는 clear_time 뒤'
    stowed(il, 1.6)
    assert not active(il, 1.6)['E301']


# ---------- 속도 오버라이드 ----------
def test_speed_override_scales_drive():
    il = SafetyInterlock()
    stowed(il, 0.0)
    il.set_speed_override(30)
    v, w, _ = il.filter_drive(0.0, 0.2, 1.0)
    assert v == pytest.approx(0.06) and w == pytest.approx(0.3)


def test_speed_override_clamped():
    il = SafetyInterlock()
    il.set_speed_override(150)
    assert il.speed_override == 100.0
    il.set_speed_override(-5)
    assert il.speed_override == 0.0


# ---------- E302 ----------
def test_e302_rejects_arm_request_while_commanded():
    il = SafetyInterlock()
    stowed(il, 0.0)
    il.filter_drive(0.0, 0.3, 0.0)
    ok, reason = il.request_arm(0.0, 'approach')
    assert not ok and reason == 'E302'
    assert active(il, 0.1)['E302']
    assert not active(il, 3.5)['E302'], 'reject_hold(3s) 뒤 해제'


def test_e302_rejects_arm_request_while_actually_moving():
    """명령은 0인데 관성으로 움직이는 중이어도 거부."""
    il = SafetyInterlock()
    stowed(il, 0.0)
    il.filter_drive(0.0, 0.0, 0.0)
    il.update_odom(0.0, 0.1, 0.0)
    ok, _ = il.request_arm(0.0, 'work')
    assert not ok


def test_arm_request_allowed_when_stopped():
    il = SafetyInterlock()
    stowed(il, 0.0)
    il.filter_drive(0.0, 0.0, 0.0)
    il.update_odom(0.0, 0.0, 0.0)
    assert il.request_arm(0.0, 'approach') == (True, '')


def test_stowed_request_always_allowed():
    """접기는 위험을 줄이는 동작: E301로 막힌 상태에서 빠져나올 수 있어야 한다."""
    il = SafetyInterlock()
    extended(il, 0.0)
    il.filter_drive(0.0, 0.3, 0.0)
    assert il.request_arm(0.0, 'stowed') == (True, '')


def test_work_inhibit_rejects_non_stowed():
    il = SafetyInterlock()
    stowed(il, 0.0)
    il.set_work_inhibit(True, 'E401')
    ok, reason = il.request_arm(0.0, 'approach')
    assert not ok and 'E401' in reason
    assert il.request_arm(0.0, 'stowed')[0]


# ---------- E303 ----------
def test_e303_arm_timeout():
    il = SafetyInterlock(InterlockThresholds(arm_timeout=8.0))
    extended(il, 10.0, status='moving', elapsed=9.0)
    assert active(il, 10.0)['E303']


def test_e303_not_raised_before_timeout_or_when_reached():
    il = SafetyInterlock(InterlockThresholds(arm_timeout=8.0))
    extended(il, 0.0, status='moving', elapsed=3.0)
    assert not active(il, 0.0)['E303']
    extended(il, 0.0, status='reached', elapsed=12.0)
    assert not active(il, 0.0)['E303']


# ---------- 인터락 + 동작 감시 ----------
def test_blocked_command_does_not_raise_e101():
    """
    팔이 펼쳐진 채 주행 명령 -> 인터락이 0으로 막음 -> 로봇은 서 있음.

    motion_check가 인터락 통과 명령(/cmd_vel_checked)을 보면 E101이 안 떠야 한다.
    (막기 전 /cmd_vel을 보면 E101이 뜬다 -> 비교 기준을 바꾼 이유)
    """
    il = SafetyInterlock()
    checked, raw = MotionChecker(), MotionChecker()
    t = 0.0
    for _ in range(60):  # 3초
        t += DT
        extended(il, t)
        v, w, blocked = il.filter_drive(t, 0.3, 0.0)
        assert blocked
        for mc, cmd in ((checked, (v, w)), (raw, (0.3, 0.0))):
            mc.update_command(t, *cmd)
            mc.update_wheel_cmd(t)
            mc.update_odom(t, 0.0, 0.0)
        r_checked = {j.code: j.active for j in checked.evaluate(t)}
        r_raw = {j.code: j.active for j in raw.evaluate(t)}
    assert not r_checked['E101']
    assert r_raw['E101']
