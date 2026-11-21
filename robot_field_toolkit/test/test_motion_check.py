"""core/motion_check.py 테스트. 가짜 시간을 넣어서 판정을 확인한다."""

from robot_field_toolkit.core.motion_check import MotionChecker, MotionThresholds

DT = 0.05  # 20Hz


def run(checker, duration, cmd=(0.0, 0.0), odom=(0.0, 0.0), wheel_every=1, start=0.0):
    """
    duration초 동안 20Hz로 명령·odom을 넣고 마지막 판정을 돌려준다.

    wheel_every=2이면 wheel_cmd를 두 번에 한 번만 보낸다 (drop 흉내).
    """
    t = start
    steps = int(round(duration / DT))
    result = None
    for i in range(steps):
        t = start + (i + 1) * DT
        checker.update_command(t, *cmd)
        if i % wheel_every == 0:
            checker.update_wheel_cmd(t)
        checker.update_odom(t, *odom)
        result = {j.code: j.active for j in checker.evaluate(t)}
    return result, t


# ---------- 정상 ----------
def test_normal_no_alarm():
    r, _ = run(MotionChecker(), 5.0, cmd=(0.3, 0.0), odom=(0.3, 0.0))
    assert not any(r.values())


def test_stop_command_no_alarm():
    """정지 명령이고 로봇도 서 있으면 오알람 없음."""
    r, _ = run(MotionChecker(), 5.0, cmd=(0.0, 0.0), odom=(0.0, 0.0))
    assert not any(r.values())


def test_tiny_command_no_alarm():
    """기준보다 작은 명령(0.03 m/s)은 '움직이라는 명령'이 아니므로 E101 없음."""
    r, _ = run(MotionChecker(), 5.0, cmd=(0.03, 0.05), odom=(0.0, 0.0))
    assert not r['E101']


# ---------- E101 구동 정지 ----------
def test_e101_stuck_raises():
    r, _ = run(MotionChecker(), 1.5, cmd=(0.3, 0.0), odom=(0.0, 0.0))
    assert r['E101']
    assert not r['E102']


def test_e101_needs_duration():
    """1.0초가 안 되면 아직 안 뜬다 (출발 직후 가속 구간 오알람 방지)."""
    r, _ = run(MotionChecker(), 0.8, cmd=(0.3, 0.0), odom=(0.0, 0.0))
    assert not r['E101']


def test_e101_rotation_only_stuck():
    """제자리 회전 명령도 못 하면 E101."""
    r, _ = run(MotionChecker(), 1.5, cmd=(0.0, 0.5), odom=(0.0, 0.0))
    assert r['E101']


def test_e101_clears_after_stop_command():
    c = MotionChecker()
    r, t = run(c, 1.5, cmd=(0.3, 0.0), odom=(0.0, 0.0))
    assert r['E101']
    r, t = run(c, 0.5, cmd=(0.0, 0.0), odom=(0.0, 0.0), start=t)
    assert r['E101'], '해제는 clear_time(1.0s) 뒤'
    r, _ = run(c, 1.0, cmd=(0.0, 0.0), odom=(0.0, 0.0), start=t)
    assert not r['E101']


def test_e101_reverse_motion_counts_as_zero():
    """앞으로 가라는데 뒤로 가면 비율 0으로 본다."""
    r, _ = run(MotionChecker(), 1.5, cmd=(0.3, 0.0), odom=(-0.3, 0.0))
    assert r['E101']


# ---------- E102 출력 저하 ----------
def test_e102_weak_raises():
    r, _ = run(MotionChecker(), 2.5, cmd=(0.3, 0.5), odom=(0.12, 0.2))  # 40%
    assert r['E102']
    assert not r['E101']


def test_e102_needs_duration():
    r, _ = run(MotionChecker(), 1.5, cmd=(0.3, 0.0), odom=(0.12, 0.0))
    assert not r['E102']


def test_e102_not_raised_at_80_percent():
    r, _ = run(MotionChecker(), 5.0, cmd=(0.3, 0.0), odom=(0.24, 0.0))
    assert not r['E102']


# ---------- E103 명령 전달 손실 ----------
def test_e103_drop_raises():
    r, _ = run(MotionChecker(), 2.5, cmd=(0.3, 0.0), odom=(0.3, 0.0), wheel_every=2)
    assert r['E103']
    assert not r['E101']


def test_e103_all_delivered_no_alarm():
    r, _ = run(MotionChecker(), 5.0, cmd=(0.3, 0.0), odom=(0.3, 0.0), wheel_every=1)
    assert not r['E103']


def test_e103_too_few_commands_no_judgement():
    """명령이 너무 적으면(키 한 번 누름 등) 판정하지 않는다."""
    c = MotionChecker()
    c.update_command(0.0, 0.3, 0.0)
    c.update_odom(0.0, 0.3, 0.0)
    r = {j.code: j.active for j in c.evaluate(0.1)}
    assert not r['E103']


def test_e103_stuck_is_not_loss():
    """stuck은 0 명령이 전달은 되므로 E103이 아니다."""
    r, _ = run(MotionChecker(), 2.5, cmd=(0.3, 0.0), odom=(0.0, 0.0), wheel_every=1)
    assert not r['E103']


# ---------- E104 방향 이탈 ----------
def test_e104_drift_raises():
    r, _ = run(MotionChecker(), 2.0, cmd=(0.3, 0.0), odom=(0.3, 0.3))
    assert r['E104']
    assert not r['E101'] and not r['E102']


def test_e104_turn_command_no_alarm():
    """회전 명령 중의 회전은 정상."""
    r, _ = run(MotionChecker(), 3.0, cmd=(0.3, 0.5), odom=(0.3, 0.5))
    assert not r['E104']


def test_e104_small_wobble_no_alarm():
    r, _ = run(MotionChecker(), 3.0, cmd=(0.3, 0.0), odom=(0.3, 0.1))
    assert not r['E104']


def test_e104_needs_duration():
    r, _ = run(MotionChecker(), 1.2, cmd=(0.3, 0.0), odom=(0.3, 0.3))
    assert not r['E104']


# ---------- 기타 ----------
def test_no_odom_no_motion_alarm():
    """odom이 안 오면 E101/E102/E104는 판정 보류 (오알람 방지)."""
    c = MotionChecker()
    for i in range(40):
        t = (i + 1) * DT
        c.update_command(t, 0.3, 0.0)
        c.update_wheel_cmd(t)
        r = {j.code: j.active for j in c.evaluate(t)}
    assert not r['E101'] and not r['E102'] and not r['E104']


def test_custom_thresholds():
    """기준값을 바꾸면 판정이 바뀐다 (stall_time 0.5초)."""
    c = MotionChecker(MotionThresholds(stall_time=0.5))
    r, _ = run(c, 0.8, cmd=(0.3, 0.0), odom=(0.0, 0.0))
    assert r['E101']
