"""感知层：激光雷达扫描。对应 PPT「感知层-环境感知」。

LidarSimulator 基于占用栅格地图进行射线投射，返回距离数组，纯标准库实现。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class LidarScan:
    angles: List[float]          # 各射线的角度 (rad)，相对机器人朝向
    ranges: List[float]          # 各射线探测到的距离 (m)
    timestamp: float = 0.0

    def min_range(self) -> float:
        return min(self.ranges, default=float("inf"))

    def closest_angle(self) -> Optional[float]:
        """返回最近障碍物相对角度；无有效数据返回 None。"""
        if not self.ranges:
            return None
        idx = min(range(len(self.ranges)), key=lambda i: self.ranges[i])
        return self.angles[idx]


class LidarSimulator:
    """基于栅格地图的 2D 激光雷达模拟器（射线投射）。

    grid: navigation.GridMap 实例
    """

    def __init__(
        self,
        grid,
        rays: int = 36,
        max_range: float = 5.0,
        fov: float = 2 * math.pi,
    ) -> None:
        self.grid = grid
        self.rays = max(4, int(rays))
        self.max_range = max_range
        self.fov = fov

    def scan(self, pose, timestamp: float = 0.0) -> LidarScan:
        """在给定位姿 (x, y, theta) 下执行一次扫描。"""
        x, y, theta = pose
        angles: List[float] = []
        ranges: List[float] = []
        step = self.fov / self.rays
        for i in range(self.rays):
            a = theta - self.fov / 2 + i * step
            angles.append(a)
            ranges.append(self._cast(x, y, a))
        return LidarScan(angles=angles, ranges=ranges, timestamp=timestamp)

    def _cast(self, x: float, y: float, angle: float) -> float:
        """沿 angle 方向射线投射，返回碰到的障碍距离。"""
        dx = math.cos(angle)
        dy = math.sin(angle)
        res = self.grid.resolution
        steps = int(self.max_range / res) if res > 0 else 100
        for s in range(1, steps + 1):
            d = s * res
            wx = x + dx * d
            wy = y + dy * d
            if not self.grid.is_free_xy(wx, wy):
                return d
        return self.max_range
