"""core/footprint.py 테스트."""

from robot_field_toolkit.core.footprint import footprint_for


def test_stowed_empty_is_compact_circle():
    name, pts = footprint_for('stowed', 'reached')
    assert name == 'compact' and len(pts) == 16
    assert max(abs(x) for x, _ in pts) <= 0.2001


def test_arm_out_or_loaded_or_unknown_is_extended():
    assert footprint_for('work', 'reached')[0] == 'extended'
    assert footprint_for('stowed', 'moving')[0] == 'extended', '접는 중'
    assert footprint_for('stowed', 'reached', 'loaded_0.5kg')[0] == 'extended'
    assert footprint_for(None, None)[0] == 'extended', '모르면 보수적으로'
    _, pts = footprint_for('work', 'reached')
    assert max(x for x, _ in pts) > 0.3, '팔이 앞으로 뻗는 쪽이 더 큼'
