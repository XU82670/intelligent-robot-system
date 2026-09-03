"""决策层：任务/航点解析。对应 PPT「决策层-任务调度」。

Mission 描述一次任务：一系列航点 + 每个航点的动作。
支持从 JSON/dict 加载，便于 HMI 下发任务。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class Waypoint:
    x: float
    y: float
    action: str = "none"   # none | inspect | pick | place | wait
    label: str = ""

    def to_dict(self) -> dict:
        return {"x": self.x, "y": self.y, "action": self.action, "label": self.label}


@dataclass
class Mission:
    name: str = "mission"
    waypoints: List[Waypoint] = field(default_factory=list)
    home: Tuple[float, float] = (0.0, 0.0)

    @classmethod
    def from_dict(cls, data: dict) -> "Mission":
        wps = [
            Waypoint(
                x=float(w["x"]),
                y=float(w["y"]),
                action=w.get("action", "none"),
                label=w.get("label", ""),
            )
            for w in data.get("waypoints", [])
        ]
        home = tuple(data.get("home", [0.0, 0.0]))
        return cls(name=data.get("name", "mission"), waypoints=wps, home=home)  # type: ignore

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "home": list(self.home),
            "waypoints": [w.to_dict() for w in self.waypoints],
        }

    def current(self, index: int) -> Optional[Waypoint]:
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None
