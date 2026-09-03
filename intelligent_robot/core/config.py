"""配置管理：加载 JSON 配置文件，提供默认配置兜底。

对应 PPT「数据支撑层-数据存储/配置管理」。保持纯标准库实现。
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict

DEFAULT_CONFIG: Dict[str, Any] = {
    # ---- 运动控制 ----
    "motion": {
        "wheel_radius": 0.05,   # 轮半径 (m)
        "wheel_base": 0.30,     # 轮距 (m)
        "max_linear_speed": 0.8,  # 最大线速度 (m/s)
        "max_angular_speed": 1.5,  # 最大角速度 (rad/s)
        "pid": {"kp": 1.2, "ki": 0.05, "kd": 0.15},
    },
    # ---- 感知 ----
    "perception": {
        "detector": "color",     # color | mock | yolo
        "lidar_rays": 36,        # 激光雷达射线数
        "lidar_max_range": 5.0,  # 激光雷达最大量程 (m)
    },
    # ---- 导航 ----
    "navigation": {
        "grid_resolution": 0.1,  # 栅格分辨率 (m/cell)
        "map_size": 50,          # 地图尺寸 (cells)，默认 50x50
        "avoid_range": 0.25,      # 避障触发范围 (m)
    },
    # ---- 决策 ----
    "decision": {
        "goal_tolerance": 0.15,  # 到达目标容差 (m)
        "max_retries": 3,        # 导航失败最大重试次数
    },
    # ---- 数据管理 ----
    "data": {
        "log_dir": "logs",
        "dataset_dir": "data/datasets",
    },
    # ---- 应用层 ----
    "hmi": {
        "http_host": "127.0.0.1",
        "http_port": 8080,
    },
}


def load_config(path: str | None = None) -> Dict[str, Any]:
    """加载配置文件；未提供路径或文件缺失时使用默认配置。

    配置为深合并：文件中的键覆盖默认配置中对应键。
    """
    cfg = _deep_copy(DEFAULT_CONFIG)
    if path and os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            user_cfg = json.load(fh)
        _deep_merge(cfg, user_cfg)
    return cfg


def _deep_copy(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _deep_copy(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_deep_copy(v) for v in value]
    return value


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> None:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
