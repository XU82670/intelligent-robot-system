"""导航层：局部避障与路径跟踪控制器。对应 PPT「定位导航模块-避障策略」。

PotentialFieldController 基于人工势场法：目标吸引 + 障碍排斥，
输出期望速度 (v, w)，供运动控制层执行。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from ..perception.lidar import LidarScan


def normalize_angle(a: float) -> float:
    """把角度归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


class PotentialFieldController:
    """人工势场避障控制器。

    输入：当前位姿、目标点、激光雷达扫描
    输出：(v, w) 期望线速度/角速度
    """

    def __init__(
        self,
        k_att: float = 1.0,
        k_rep: float = 3.0,
        rep_range: float = 0.8,
        max_v: float = 0.8,
        max_w: float = 1.5,
    ) -> None:
        self.k_att = k_att
        self.k_rep = k_rep
        self.rep_range = rep_range
        self.max_v = max_v
        self.max_w = max_w

    def compute(
        self,
        pose: Tuple[float, float, float],
        goal: Tuple[float, float],
        scan: Optional[LidarScan] = None,
    ) -> Tuple[float, float]:
        x, y, theta = pose
        # 吸引力：指向目标
        dx = goal[0] - x
        dy = goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0
        att_x = self.k_att * dx / dist
        att_y = self.k_att * dy / dist

        # 排斥力：来自激光雷达障碍点
        rep_x, rep_y = 0.0, 0.0
        if scan is not None:
            for angle, rng in zip(scan.angles, scan.ranges):
                if rng >= self.rep_range:
                    continue
                # 障碍点世界坐标
                ox = x + rng * math.cos(angle)
                oy = y + rng * math.sin(angle)
                # 从障碍指向机器人的方向
                vx = x - ox
                vy = y - oy
                d = math.hypot(vx, vy)
                if d < 1e-6:
                    continue
                strength = self.k_rep * (1.0 / rng - 1.0 / self.rep_range) / (rng ** 2)
                rep_x += strength * vx / d
                rep_y += strength * vy / d

        fx = att_x + rep_x
        fy = att_y + rep_y
        force = math.hypot(fx, fy)
        if force < 1e-6:
            return 0.0, 0.0

        # 期望方向
        desired_theta = math.atan2(fy, fx)
        angle_err = normalize_angle(desired_theta - theta)

        # 大角度差时原地转向，小角度差时前进
        if abs(angle_err) > math.pi / 3:
            v = 0.0
        else:
            v = min(self.max_v, force * 0.5) * (1.0 - abs(angle_err) / (math.pi / 3))
        w = max(-self.max_w, min(self.max_w, 2.0 * angle_err))
        return v, w
