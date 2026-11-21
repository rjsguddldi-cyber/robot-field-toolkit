"""core/commissioning.py 테스트."""

import json
import math

import pytest
from robot_field_toolkit.core.commissioning import (
    build_plan, check_points, CycleRecord, summarize, to_json, to_markdown)


def test_plan_speeds_and_alternating_points():
    plan = build_plan(['A', 'B'], speeds=(30, 60, 100), cycles_per_speed=3)
    assert len(plan) == 9
    assert [p.speed for p in plan[:4]] == [30, 30, 30, 60]
    assert [p.point for p in plan[:4]] == ['A', 'B', 'A', 'B']
    assert [p.cycle for p in plan[:4]] == [1, 2, 3, 1]


def test_plan_needs_points():
    with pytest.raises(ValueError):
        build_plan([])


def test_check_points_rejects_empty_coordinates():
    with pytest.raises(ValueError):
        check_points({'A': {'x': None, 'y': 1.0, 'yaw': 0.0}})
    check_points({'A': {'x': 0.5, 'y': 1.0, 'yaw': 0.0}})


def records():
    return [
        CycleRecord(30, 1, 'A', True, 40.0, 0.04, 0.05, 12.0, []),
        CycleRecord(30, 2, 'B', True, 44.0, 0.08, 0.10, 12.0, ['E102']),
        CycleRecord(60, 1, 'A', False, 20.0, 0.18, 0.02, 0.0, ['E401'], 'E401 도착 오차'),
    ]


def test_summary():
    s = summarize(records())
    s30, s60 = s
    assert s30['speed'] == 30 and s30['success'] == 2 and s30['success_rate'] == 1.0
    assert s30['dist_mean'] == pytest.approx(0.06)
    assert s30['dist_max'] == pytest.approx(0.08)
    assert s30['cycle_time_mean'] == pytest.approx(42.0)
    assert s60['success_rate'] == 0.0
    assert s60['cycle_time_mean'] is None, '성공한 사이클만 사이클 타임 평균'


def test_markdown_has_basis_and_tables():
    md = to_markdown(records(), summarize(records()), {'started': 'now'})
    assert 'Gazebo 시뮬레이션 기준' in md.splitlines()[2]
    assert '| 30 | 1 | A | O |' in md
    assert 'E401 도착 오차' in md
    assert f'{math.degrees(0.10):.1f}' in md


def test_json_roundtrip():
    data = json.loads(to_json(records(), summarize(records()), {'note': 'x'}))
    assert data['basis'] == 'Gazebo simulation'
    assert len(data['cycles']) == 3
    assert data['cycles'][2]['alarms'] == ['E401']
