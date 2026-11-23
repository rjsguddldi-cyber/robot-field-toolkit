"""core/nav_check.py 테스트."""

import math

import pytest
from robot_field_toolkit.core.nav_check import (
    NavChecker, pose_error, STATUS_ABORTED, STATUS_EXECUTING, STATUS_SUCCEEDED)


def act(nc, t):
    return {j.code: j.active for j in nc.evaluate(t)}


def arrive(nc, actual, t_success=10.0, goal=(1.0, 2.0, 0.0)):
    nc.set_goal(*goal)
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    nc.update_pose(*actual)
    nc.update_covariance(0.01, 0.01, 0.01)
    nc.update_status(t_success, 'g1', STATUS_SUCCEEDED)


def test_pose_error():
    d, a = pose_error((0.0, 0.0, math.pi - 0.05), (0.3, 0.4, -math.pi + 0.05))
    assert d == pytest.approx(0.5)
    assert a == pytest.approx(0.1)  # -pi/pi 경계를 넘어도 짧은 쪽


# ---------- E401 ----------
def test_e401_not_raised_when_accurate():
    nc = NavChecker()
    arrive(nc, (1.03, 2.02, 0.05))
    assert not act(nc, 11.5)['E401']
    assert not nc.work_inhibit()


def test_e401_raised_when_far_and_inhibits_arm():
    nc = NavChecker()
    arrive(nc, (1.2, 2.0, 0.0))  # 20cm 오차 (Nav2 기본 허용 25cm 안이라 '성공'으로 끝남)
    assert not act(nc, 10.5)['E401'], 'settle_time 전에는 재지 않음'
    assert act(nc, 11.5)['E401']
    assert nc.work_inhibit()


def test_e401_raised_on_yaw_error():
    nc = NavChecker()
    arrive(nc, (1.0, 2.0, 0.3))
    assert act(nc, 11.5)['E401']


def test_e401_cleared_by_new_goal():
    nc = NavChecker()
    arrive(nc, (1.2, 2.0, 0.0))
    assert act(nc, 11.5)['E401']
    nc.update_status(12.0, 'g2', STATUS_EXECUTING)
    assert not act(nc, 12.1)['E401']
    assert not nc.work_inhibit()


# ---------- E402 ----------
def test_e402_lost_after_time():
    nc = NavChecker()
    nc.update_covariance(0.5, 0.5, 0.01)  # 표준편차 0.71m
    assert not act(nc, 0.0)['E402']
    assert not act(nc, 1.5)['E402']
    assert act(nc, 2.1)['E402']


def test_e402_not_raised_when_confident():
    nc = NavChecker()
    nc.update_covariance(0.01, 0.01, 0.01)
    act(nc, 0.0)
    assert not act(nc, 5.0)['E402']


# ---------- E403 ----------
def feedback_run(nc, start, end, dist_fn, rec=0):
    t = start
    r = None
    while t <= end:
        nc.update_feedback(t, dist_fn(t), rec)
        r = act(nc, t)
        t += 0.5
    return r


def test_e403_no_progress():
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    r = feedback_run(nc, 0.0, 14.0, lambda t: 3.0)
    assert not r['E403']
    r = feedback_run(nc, 14.5, 16.0, lambda t: 3.0)
    assert r['E403']


def test_e403_not_raised_while_progressing():
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    r = feedback_run(nc, 0.0, 30.0, lambda t: 6.0 - 0.1 * t)
    assert not r['E403']


def test_e403_detour_with_temporary_increase_ok():
    """돌아가느라 남은 거리가 잠깐 늘어도, 결국 줄면 E403 아님."""
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    r = feedback_run(nc, 0.0, 30.0, lambda t: 3.0 + (0.5 if 5 < t < 10 else 0.0) - 0.05 * t)
    assert not r['E403']


def test_e403_ignores_zero_distance_before_plan():
    """경로 계산 전 남은 거리 0 피드백 뒤 10.2m에서 잘 줄어들면 E403 아님 (11/23 버그)."""
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    nc.update_feedback(0.0, 0.0, 0)
    r = feedback_run(nc, 0.5, 40.0, lambda t: 10.2 - 0.25 * t)
    assert not r['E403']


def test_e403_still_raised_when_blocked_after_zero_feedback():
    """0 피드백이 먼저 와도 실제로 막히면 E403은 뜬다."""
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    nc.update_feedback(0.0, 0.0, 0)
    r = feedback_run(nc, 0.5, 17.0, lambda t: 3.0)
    assert r['E403']


def test_e403_recoveries():
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    nc.update_feedback(0.0, 3.0, 3)
    assert act(nc, 0.1)['E403']


def test_e403_kept_after_abort_cleared_by_new_goal():
    nc = NavChecker()
    nc.update_status(0.0, 'g1', STATUS_EXECUTING)
    nc.update_feedback(0.0, 3.0, 3)
    act(nc, 0.1)
    nc.update_status(1.0, 'g1', STATUS_ABORTED)
    assert act(nc, 1.1)['E403'], '실패 후에도 다음 목표까지 남김'
    nc.update_status(2.0, 'g2', STATUS_EXECUTING)
    assert not act(nc, 2.1)['E403']
