"""
팔 자세 추적. rclpy 없이 순수 파이썬.

이름 붙은 자세(stowed, approach, work, retreat ...)를 관절 각도로 바꾸고,
현재 관절 각도를 받아서 목표에 도달했는지(moving/reached) 판단한다.
"""

STOWED = 'stowed'


class ArmPoseTracker:
    """자세 요청과 관절 상태로 현재 자세 이름과 상태를 계산한다."""

    def __init__(self, poses, joint_names, tolerance=0.05, initial_pose=STOWED):
        """
        poses: {자세 이름: {관절 이름: 각도[rad]}}.

        joint_names: 관절 이름 순서. tolerance: 도달 판정 허용 오차 [rad].
        """
        for name, joints in poses.items():
            missing = [j for j in joint_names if j not in joints]
            if missing:
                raise ValueError(f'자세 {name}: 관절 {missing} 각도가 없음')
        self.poses = poses
        self.joint_names = list(joint_names)
        self.tolerance = tolerance
        self.current = {}          # 관절 이름 -> 현재 각도
        self.target_name = initial_pose
        self.request_t = None      # 마지막 요청 시각

    def request(self, name, t):
        """자세 요청. 관절 목표 {관절: 각도}를 돌려준다. 모르는 이름이면 KeyError."""
        if name not in self.poses:
            raise KeyError(name)
        self.target_name = name
        self.request_t = t
        return dict(self.poses[name])

    def target(self):
        """현재 목표 자세의 관절 각도."""
        return dict(self.poses[self.target_name])

    def update_joints(self, positions):
        """관절 상태 갱신. positions: {관절 이름: 각도}. 모르는 관절은 무시."""
        for name, value in positions.items():
            if name in self.joint_names:
                self.current[name] = value

    def max_error(self):
        """목표와 현재의 최대 각도 차이. 관절 상태를 아직 모르면 None."""
        if any(j not in self.current for j in self.joint_names):
            return None
        goal = self.poses[self.target_name]
        return max(abs(goal[j] - self.current[j]) for j in self.joint_names)

    def status(self):
        """'reached' 또는 'moving'."""
        err = self.max_error()
        return 'reached' if err is not None and err <= self.tolerance else 'moving'

    def elapsed(self, t):
        """마지막 요청 뒤 지난 시간 [s]. 요청이 없었으면 0."""
        return 0.0 if self.request_t is None else t - self.request_t

    def is_stowed(self):
        """접힘 자세에 도달해 있는가."""
        return self.target_name == STOWED and self.status() == 'reached'
