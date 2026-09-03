"""2D 世界仿真器：在带障碍的栅格世界中运行机器人，验证完整系统。

对应 PPT「系统调试-仿真调试」。纯标准库实现，可直接运行：
    python -m simulator.run_simulation
"""
from __future__ import annotations

import math
import os
import sys
from typing import List, Tuple

# 允许从项目根直接运行
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.robot import Robot  # noqa: E402


def build_world(robot: Robot) -> None:
    """构建一个带障碍的测试世界（单位：米）。"""
    # 地图 5m x 5m（resolution=0.1 -> 50x50 栅格）
    # 中间一道墙，留一个缺口
    robot.add_wall(2.0, 0.0, 2.2, 3.0)   # 竖墙（下方到 y=3）
    robot.add_wall(2.0, 4.0, 2.2, 5.0)   # 竖墙（上方，留 y=3~4 缺口）
    # 右上角一个方块障碍
    robot.add_wall(3.5, 3.5, 4.2, 4.2)


def run(
    start: Tuple[float, float] = (0.5, 0.5),
    goal: Tuple[float, float] = (4.5, 4.5),
    max_steps: int = 500,
    dt: float = 0.1,
    verbose: bool = True,
) -> dict:
    """运行一次完整仿真，返回结果统计。"""
    robot = Robot()
    build_world(robot)
    robot.start()
    robot.localizer.reset(start[0], start[1], 0.0)

    ok = robot.goto(goal[0], goal[1])
    if not ok:
        return {"success": False, "reason": "路径规划失败", "steps": 0}

    trajectory: List[Tuple[float, float]] = [start]
    for step in range(max_steps):
        robot.update(dt)
        pose = robot.localizer.pose
        trajectory.append((round(pose.x, 3), round(pose.y, 3)))
        if verbose and step % 20 == 0:
            print(
                f"step={step:3d}  state={robot.fsm.state:8s}  "
                f"pose=({pose.x:.2f},{pose.y:.2f})  v={robot.last_v:.2f} w={robot.last_w:.2f}"
            )
        if robot.fsm.state == "DONE":
            break

    final = robot.localizer.pose
    dist_to_goal = math.hypot(final.x - goal[0], final.y - goal[1])
    success = dist_to_goal < 0.3
    result = {
        "success": success,
        "steps": len(trajectory) - 1,
        "start": start,
        "goal": goal,
        "final_pose": (round(final.x, 3), round(final.y, 3), round(final.theta, 3)),
        "distance_to_goal": round(dist_to_goal, 3),
        "trajectory_length": len(trajectory),
        "trajectory": trajectory,
    }
    if verbose:
        print("-" * 60)
        print(f"仿真结束: success={success}, steps={result['steps']}, "
              f"final=({final.x:.2f},{final.y:.2f}), dist_to_goal={dist_to_goal:.3f}m")
    return result


if __name__ == "__main__":
    run()
