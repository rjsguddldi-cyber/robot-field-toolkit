"""core/arm_pose.py 테스트."""

from pathlib import Path

import pytest
from robot_field_toolkit.core.arm_pose import ArmPoseTracker
import yaml

POSES_YAML = Path(__file__).resolve().parents[1] / 'config' / 'arm_poses.yaml'
CFG = yaml.safe_load(POSES_YAML.read_text(encoding='utf-8'))
JOINTS = CFG['joint_names']


def make():
    return ArmPoseTracker(CFG['poses'], JOINTS, CFG['tolerance'])


def test_yaml_has_pick_and_place_poses():
    for name in ('stowed', 'approach', 'work', 'retreat'):
        assert name in CFG['poses']


def test_unknown_joint_state_is_moving():
    assert make().status() == 'moving'


def test_reached_when_within_tolerance():
    tr = make()
    goal = tr.request('approach', 0.0)
    tr.update_joints({j: v + 0.02 for j, v in goal.items()})
    assert tr.status() == 'reached'


def test_moving_when_far():
    tr = make()
    tr.request('work', 0.0)
    tr.update_joints({j: 0.0 for j in JOINTS})
    assert tr.status() == 'moving'


def test_unknown_pose_raises():
    with pytest.raises(KeyError):
        make().request('dance', 0.0)


def test_is_stowed_only_when_reached():
    tr = make()
    tr.update_joints(CFG['poses']['stowed'])
    assert tr.is_stowed()
    tr.request('approach', 1.0)
    assert not tr.is_stowed()


def test_elapsed():
    tr = make()
    tr.request('work', 2.0)
    assert tr.elapsed(5.5) == pytest.approx(3.5)


def test_missing_joint_in_pose_rejected():
    with pytest.raises(ValueError):
        ArmPoseTracker({'bad': {'shoulder_joint': 0.0}}, JOINTS)


def test_test_pose_is_beyond_shoulder_limit():
    """E303 시험용 자세는 URDF 어깨 한계(1.57)를 넘어야 한다."""
    assert CFG['poses']['test_beyond_limit']['shoulder_joint'] > 1.57
