"""导航层：A* 路径规划。对应 PPT「定位导航模块-路径规划」。

AStarPlanner 在 GridMap 上规划从起点到目标的最短路径（栅格坐标），
纯标准库实现，可直接单测。
"""
from __future__ import annotations

import heapq
from typing import List, Optional, Tuple

from .grid_map import Cell, GridMap


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Cell, b: Cell) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


class AStarPlanner:
    """A* 路径规划器。

    Example:
        planner = AStarPlanner(grid)
        path = planner.plan(start_cell, goal_cell)
    """

    def __init__(self, grid: GridMap, heuristic: str = "manhattan") -> None:
        self.grid = grid
        self.heuristic = manhattan if heuristic == "manhattan" else euclidean

    def plan(self, start: Cell, goal: Cell) -> Optional[List[Cell]]:
        """规划从 start 到 goal 的路径（含起点和终点）。无路径返回 None。"""
        if not self.grid.is_free(start) or not self.grid.is_free(goal):
            return None

        # (f, counter, cell)
        counter = 0
        open_heap: List[Tuple[float, int, Cell]] = [(0.0, counter, start)]
        g_score: dict = {start: 0.0}
        parent: dict = {}
        closed: set = set()

        while open_heap:
            f, _, current = heapq.heappop(open_heap)
            if current == goal:
                return self._reconstruct(parent, start, goal)
            if current in closed:
                continue
            closed.add(current)

            for nxt in self.grid.neighbors(current):
                # 对角移动时检查两侧格，避免穿墙
                if self._diagonal_cut_corner(current, nxt):
                    continue
                step = 1.0 if (nxt[0] == current[0] or nxt[1] == current[1]) else 1.41421356
                tentative = g_score[current] + step
                if tentative < g_score.get(nxt, float("inf")):
                    g_score[nxt] = tentative
                    parent[nxt] = current
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (tentative + self.heuristic(nxt, goal), counter, nxt),
                    )
        return None

    def plan_xy(
        self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]
    ) -> Optional[List[Tuple[float, float]]]:
        """世界坐标版本：返回世界坐标路径点列表。"""
        start = self.grid.to_cell(*start_xy)
        goal = self.grid.to_cell(*goal_xy)
        path = self.plan(start, goal)
        if path is None:
            return None
        return [self.grid.to_world(c) for c in path]

    def _diagonal_cut_corner(self, current: Cell, nxt: Cell) -> bool:
        dc = nxt[0] - current[0]
        dr = nxt[1] - current[1]
        if dc != 0 and dr != 0:
            # 对角移动：两侧至少一个为障碍则禁止
            if not self.grid.is_free((current[0] + dc, current[1])):
                return True
            if not self.grid.is_free((current[0], current[1] + dr)):
                return True
        return False

    @staticmethod
    def _reconstruct(parent: dict, start: Cell, goal: Cell) -> List[Cell]:
        path = [goal]
        current = goal
        while current != start:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path
