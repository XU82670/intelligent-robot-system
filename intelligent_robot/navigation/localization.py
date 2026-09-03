"""导航层：定位。对应 PPT「定位导航模块-定位与建图」。

OdometryLocalizer 基于轮式里程计积分位姿，是纯标准库实现的基础定位器。
真实系统中可替换为 AMCL/SLAM（如 ROS 2 的 nav2_amcl），接口保持一致。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from .controller import normalize_angle


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # 朝向 (rad)

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def distance_to(self, point: Tuple[float, float]) -> float:
        return math.hypot(self.x - point[0], self.y - point[1])


class OdometryLocalizer:
    """轮式里程计定位器：根据左右轮位移积分位姿。

    真实系统中编码器脉冲 -> 轮位移；此处直接接收左右轮位移 (m)。
    """

    def __init__(self, wheel_base: float = 0.30) -> None:
        self.wheel_base = wheel_base
        self.pose = Pose()

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.pose = Pose(x, y, theta)

    def update(self, left_dist: float, right_dist: float) -> Pose:
        """根据左右轮位移更新位姿（差分驱动运动学）。"""
        d = (left_dist + right_dist) / 2.0
        dtheta = (right_dist - left_dist) / self.wheel_base
        # 中点近似
        self.pose.x += d * math.cos(self.pose.theta + dtheta / 2.0)
        self.pose.y += d * math.sin(self.pose.theta + dtheta / 2.0)
        self.pose.theta = normalize_angle(self.pose.theta + dtheta)
        return self.pose

    def update_velocity(self, v: float, w: float, dt: float) -> Pose:
        """根据 (v, w) 和时间步长更新位姿。"""
        if abs(w) < 1e-6:
            self.pose.x += v * dt * math.cos(self.pose.theta)
            self.pose.y += v * dt * math.sin(self.pose.theta)
        else:
            r = v / w
            self.pose.x += r * (math.sin(self.pose.theta + w * dt) - math.sin(self.pose.theta))
            self.pose.y -= r * (math.cos(self.pose.theta + w * dt) - math.cos(self.pose.theta))
            self.pose.theta = normalize_angle(self.pose.theta + w * dt)
        return self.pose
