"""
팔 상태·하중별 Nav2 풋프린트 (PHM 추가 4). 순수 파이썬.

팔이 접힘(stowed)이 아니거나 짐을 들면 로봇이 실제로 차지하는 공간이 커진다(E301 설명과 같은 이유).
그때 Nav2 costmap 풋프린트를 큰 다각형으로 바꿔 경로·장애물 여유를 키운다. ※ 값은 11/23 초안.
"""

import math

STOWED_RADIUS = 0.20     # 평상시: nav2_params robot_radius와 같은 원 [m]
EXTENDED_BOX = (-0.20, 0.32, -0.22, 0.22)   # 팔 펼침·적재: x_min, x_max, y_min, y_max [m]
# 근거: 팔 기둥 x=-0.06, 팔 길이 0.15 + 0.12 + 툴 0.03 = 0.30 → 앞으로 최대 약 0.24 + 여유 0.08


def circle(r, n=16):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def box(x0, x1, y0, y1):
    return [(x1, y1), (x0, y1), (x0, y0), (x1, y0)]


def is_compact(arm_pose, arm_status, load='empty'):
    """팔이 접힘에 도달했고 짐이 없을 때만 작은 풋프린트. 팔 상태를 모르면 큰 쪽(보수적)."""
    return arm_pose == 'stowed' and arm_status == 'reached' and load == 'empty'


def footprint_for(arm_pose, arm_status, load='empty'):
    """(이름, 꼭짓점 목록 [(x, y)])."""
    if is_compact(arm_pose, arm_status, load):
        return 'compact', circle(STOWED_RADIUS)
    return 'extended', box(*EXTENDED_BOX)
