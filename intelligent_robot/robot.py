"""系统集成：Robot 顶层类，对应 PPT「系统集成-软硬件联调」。

Robot 把感知、导航、运动控制、决策、HMI、数据管理各模块
通过消息总线组装在一起，提供 start/stop/goto/update/status 统一接口。
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

from .core.config import load_config
from .core.logger import setup_logger
from .core.message_bus import MessageBus
from .data_manager.recorder import Recorder
from .decision.mission import Mission
from .decision.state_machine import (
    AVOID,
    DONE,
    IDLE,
    NAVIGATE,
    DecisionContext,
    StateMachine,
)
from .motion_control.kinematics import DifferentialDrive
from .motion_control.pid import PID
from .navigation.controller import PotentialFieldController, normalize_angle
from .navigation.grid_map import GridMap
from .navigation.localization import OdometryLocalizer, Pose
from .navigation.path_planner import AStarPlanner
from .perception.detector import build_detector
from .perception.lidar import LidarSimulator


class Robot:
    """智能机器人系统顶层集成类。

    Example:
        robot = Robot()
        robot.start()
        robot.goto(2.0, 2.0)
        for _ in range(100):
            robot.update(0.1)
        print(robot.status())
    """

    def __init__(self, config_path: Optional[str] = None, config: Optional[dict] = None) -> None:
        self.cfg = config if config is not None else load_config(config_path)
        self.log = setup_logger("robot")
        self.bus = MessageBus()

        # 数据支撑层
        self.recorder = Recorder(self.cfg["data"]["log_dir"])

        # 导航层
        nav_cfg = self.cfg["navigation"]
        self.grid = GridMap(
            width=nav_cfg["map_size"],
            height=nav_cfg["map_size"],
            resolution=nav_cfg["grid_resolution"],
        )
        self.planner = AStarPlanner(self.grid)
        self.localizer = OdometryLocalizer(wheel_base=self.cfg["motion"]["wheel_base"])
        self.avoid_ctrl = PotentialFieldController(
            rep_range=nav_cfg["avoid_range"],
            max_v=self.cfg["motion"]["max_linear_speed"],
            max_w=self.cfg["motion"]["max_angular_speed"],
        )

        # 执行层
        mot_cfg = self.cfg["motion"]
        self.drive = DifferentialDrive(
            wheel_radius=mot_cfg["wheel_radius"], wheel_base=mot_cfg["wheel_base"]
        )
        self.pid = PID(
            kp=mot_cfg["pid"]["kp"],
            ki=mot_cfg["pid"]["ki"],
            kd=mot_cfg["pid"]["kd"],
            output_limits=(-mot_cfg["max_angular_speed"], mot_cfg["max_angular_speed"]),
        )

        # 感知层
        per_cfg = self.cfg["perception"]
        self.detector = build_detector(per_cfg["detector"])
        self.lidar = LidarSimulator(
            self.grid, rays=per_cfg["lidar_rays"], max_range=per_cfg["lidar_max_range"]
        )

        # 决策层
        self.fsm = StateMachine(initial=IDLE)
        self.ctx = DecisionContext()
        self.mission: Optional[Mission] = None
        self.waypoint_index = 0
        self.path: List[Tuple[float, float]] = []
        self.goal: Optional[Tuple[float, float]] = None

        self.started = False
        self.last_v = 0.0
        self.last_w = 0.0

    # ---- 生命周期 ----
    def start(self) -> None:
        if self.started:
            return
        self.started = True
        self.fsm.reset()
        self.localizer.reset()
        self.log.info("机器人系统启动")
        self.bus.publish("system/start", {"timestamp": time.time()})

    def stop(self) -> None:
        self.started = False
        self.last_v = 0.0
        self.last_w = 0.0
        self.fsm.reset()
        self.recorder.close()
        self.log.info("机器人系统停止")

    # ---- 任务接口 ----
    def goto(self, x: float, y: float) -> bool:
        """导航到目标点。返回是否规划成功。"""
        if not self.started:
            self.start()
        start_xy = (self.localizer.pose.x, self.localizer.pose.y)
        # 规划时使用膨胀地图（障碍外扩 0.15m），使路径主动远离墙壁
        inflated = self.grid.inflate(0.15)
        planner = AStarPlanner(inflated)
        path = planner.plan_xy(start_xy, (x, y))
        if path is None:
            self.log.warning(f"无法规划到 ({x}, {y}) 的路径")
            return False
        self.path = path
        self.goal = (x, y)
        self.waypoint_index = 0
        self.pid.reset()
        self.ctx.has_goal = True
        self.ctx.goal_reached = False
        self.fsm.transition(NAVIGATE)
        self.log.info(f"目标 ({x}, {y})，路径 {len(path)} 点")
        self.recorder.record("navigation/goal", {"x": x, "y": y, "path_len": len(path)})
        return True

    def load_mission(self, mission: Mission) -> None:
        """加载多航点任务。"""
        self.mission = mission
        self.waypoint_index = 0
        if mission.waypoints:
            wp = mission.waypoints[0]
            self.goto(wp.x, wp.y)

    # ---- 主循环 ----
    def update(self, dt: float = 0.1) -> Dict:
        """执行一个控制周期：感知 -> 决策 -> 规划 -> 控制 -> 运动学积分。"""
        if not self.started:
            return self.status()

        pose = self.localizer.pose

        # 1. 感知：激光雷达扫描
        scan = self.lidar.scan(pose.as_tuple(), timestamp=time.time())
        nearest = scan.min_range()
        self.ctx.obstacle_near = nearest < self.cfg["navigation"]["avoid_range"]
        self.ctx.obstacle_cleared = nearest >= self.cfg["navigation"]["avoid_range"] * 1.2
        self.bus.publish("perception/lidar", {"nearest": nearest, "ranges": scan.ranges})

        # 目标检测（模拟/可选）
        detections = self.detector.detect(None)
        if detections:
            self.bus.publish("perception/detections", [d.to_dict() for d in detections])

        # 2. 决策：状态机推进
        self.ctx.goal_reached = (
            self.goal is not None and pose.distance_to(self.goal) < self.cfg["decision"]["goal_tolerance"]
        )
        self.ctx.at_home = pose.distance_to((0.0, 0.0)) < self.cfg["decision"]["goal_tolerance"]
        state = self.fsm.update(self.ctx)

        # 3. 规划与控制
        v, w = 0.0, 0.0
        if state == NAVIGATE:
            v, w = self._navigate(pose, dt)
        elif state == AVOID:
            # 避障时用下一个路径点作为吸引力（而非最终目标），避免被墙后目标吸向障碍
            target = self.path[self.waypoint_index] if self.waypoint_index < len(self.path) else (self.goal or pose.as_tuple()[:2])
            v, w = self.avoid_ctrl.compute(pose.as_tuple(), target, scan)
        elif state == DONE:
            self.ctx.has_goal = False

        # 4. 执行：运动学积分
        self.localizer.update_velocity(v, w, dt)
        self.last_v, self.last_w = v, w
        self.recorder.record(
            "motion/state",
            {
                "pose": list(pose.as_tuple()),
                "v": v,
                "w": w,
                "state": state,
            },
        )
        return self.status()

    def _navigate(self, pose: Pose, dt: float) -> Tuple[float, float]:
        """路径跟踪：比例航向控制（P-control）。

        1. 跳过容差范围内的路径点
        2. 跟踪下一个路径点，计算航向误差
        3. 比例控制转向角速度，大角度差时平滑减速
        4. 接近最终目标时减速
        """
        if not self.path:
            return 0.0, 0.0

        max_v = self.cfg["motion"]["max_linear_speed"]
        max_w = self.cfg["motion"]["max_angular_speed"]

        # 1. 跳过已到达的路径点（最后一个点即目标点，不跳过，确保持续追踪到真正到达）
        wp_tol = 0.25
        while self.waypoint_index < len(self.path) - 1:
            if pose.distance_to(self.path[self.waypoint_index]) < wp_tol:
                self.waypoint_index += 1
            else:
                break
        if self.waypoint_index >= len(self.path):
            return 0.0, 0.0

        # 2. 跟踪下一个路径点
        target = self.path[self.waypoint_index]
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            return 0.0, 0.0
        desired_theta = math.atan2(dy, dx)
        angle_err = normalize_angle(desired_theta - pose.theta)

        # 3. 比例航向控制（无积分项，避免饱和振荡）
        Kp = 2.5
        w = max(-max_w, min(max_w, Kp * angle_err))

        # 4. 速度：大角度差平滑减速（最低保持 15% 速度，避免卡死）+ 接近目标减速
        angle_factor = max(0.15, 1.0 - abs(angle_err) / math.pi)
        dist_to_goal = pose.distance_to(self.goal) if self.goal else 0.0
        goal_factor = min(1.0, dist_to_goal / 0.5)
        v = max_v * angle_factor * goal_factor
        return v, w

    # ---- 状态查询 ----
    def status(self) -> Dict:
        pose = self.localizer.pose
        return {
            "state": self.fsm.state,
            "started": self.started,
            "pose": {"x": pose.x, "y": pose.y, "theta": pose.theta},
            "goal": list(self.goal) if self.goal else None,
            "waypoints_remaining": max(0, len(self.path) - self.waypoint_index),
            "nearest_obstacle": self._nearest_obstacle(),
            "velocity": {"v": self.last_v, "w": self.last_w},
        }

    def _nearest_obstacle(self) -> float:
        try:
            scan = self.lidar.scan(self.localizer.pose.as_tuple())
            return round(scan.min_range(), 3)
        except Exception:
            return float("inf")

    # ---- 地图编辑（供仿真/测试使用）----
    def add_wall(self, x0: float, y0: float, x1: float, y1: float) -> None:
        self.grid.set_rect_obstacle(x0, y0, x1, y1)
