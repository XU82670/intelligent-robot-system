"""执行层：差分驱动运动学。对应 PPT「运动控制模块-机械结构/电机驱动」。

DifferentialDrive 提供：
  - to_wheel_speeds(v, w) -> (left, right)  逆运动学
  - integrate(pose, v, w, dt) -> new_pose   正运动学积分
"""
from __future__ import annotations

import math
from typing import Tuple

from ..navigation.controller import normalize_angle


class DifferentialDrive:
    """两轮差分驱动运动学模型。

    wheel_radius: 轮半径 (m)
    wheel_base:   左右轮间距 (m)
    """

    def __init__(self, wheel_radius: float = 0.05, wheel_base: float = 0.30) -> None:
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base

    def to_wheel_speeds(self, v: float, w: float) -> Tuple[float, float]:
        """逆运动学：由期望线速度 v (m/s) 和角速度 w (rad/s) 计算左右轮角速度 (rad/s)。"""
        left = (2 * v - w * self.wheel_base) / (2 * self.wheel_radius)
        right = (2 * v + w * self.wheel_base) / (2 * self.wheel_radius)
        return left, right

    def from_wheel_speeds(self, left: float, right: float) -> Tuple[float, float]:
        """正运动学：由左右轮角速度 (rad/s) 计算机器人 (v, w)。"""
        v = self.wheel_radius * (left + right) / 2.0
        w = self.wheel_radius * (right - left) / self.wheel_base
        return v, w

    def integrate(
        self, pose: Tuple[float, float, float], v: float, w: float, dt: float
    ) -> Tuple[float, float, float]:
        """正运动学积分：给定 (v, w) 和 dt，返回新位姿。"""
        x, y, theta = pose
        if abs(w) < 1e-6:
            x += v * dt * math.cos(theta)
            y += v * dt * math.sin(theta)
        else:
            r = v / w
            x += r * (math.sin(theta + w * dt) - math.sin(theta))
            y -= r * (math.cos(theta + w * dt) - math.cos(theta))
            theta = normalize_angle(theta + w * dt)
        return x, y, theta
