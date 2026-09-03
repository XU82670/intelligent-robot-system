"""导航层：占用栅格地图。对应 PPT「定位导航模块-建图」。

GridMap 用二维列表存储占用状态（0=空闲, 1=障碍），
提供世界坐标与栅格坐标互转、邻居查询、障碍物膨胀等。
"""
from __future__ import annotations

import math
from typing import List, Tuple


Cell = Tuple[int, int]  # (col, row)


class GridMap:
    """2D 占用栅格地图。

    width/height 为栅格数量，resolution 为每格代表的米数。
    世界坐标原点在地图左下角 (0,0)。
    """

    FREE = 0
    OBSTACLE = 1

    def __init__(self, width: int = 50, height: int = 50, resolution: float = 0.1) -> None:
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        # grid[row][col]
        self.grid: List[List[int]] = [
            [self.FREE for _ in range(self.width)] for _ in range(self.height)
        ]

    # ---- 坐标转换 ----
    def to_cell(self, x: float, y: float) -> Cell:
        col = int(round(x / self.resolution))
        row = int(round(y / self.resolution))
        return (col, row)

    def to_world(self, cell: Cell) -> Tuple[float, float]:
        col, row = cell
        return (col * self.resolution, row * self.resolution)

    def in_bounds(self, cell: Cell) -> bool:
        col, row = cell
        return 0 <= col < self.width and 0 <= row < self.height

    def is_free(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return False
        col, row = cell
        return self.grid[row][col] == self.FREE

    def is_free_xy(self, x: float, y: float) -> bool:
        return self.is_free(self.to_cell(x, y))

    # ---- 地图编辑 ----
    def set_obstacle(self, cell: Cell) -> None:
        if self.in_bounds(cell):
            col, row = cell
            self.grid[row][col] = self.OBSTACLE

    def set_obstacle_xy(self, x: float, y: float) -> None:
        self.set_obstacle(self.to_cell(x, y))

    def set_rect_obstacle(self, x0: float, y0: float, x1: float, y1: float) -> None:
        """在世界坐标矩形区域填充障碍。"""
        c0, r0 = self.to_cell(min(x0, x1), min(y0, y1))
        c1, r1 = self.to_cell(max(x0, x1), max(y0, y1))
        for r in range(max(0, r0), min(self.height, r1 + 1)):
            for c in range(max(0, c0), min(self.width, c1 + 1)):
                self.grid[r][c] = self.OBSTACLE

    def inflate(self, radius: float) -> "GridMap":
        """返回膨胀后的新地图（障碍向外膨胀 radius 米），便于安全规划。"""
        cells = int(math.ceil(radius / self.resolution))
        new_map = GridMap(self.width, self.height, self.resolution)
        for r in range(self.height):
            for c in range(self.width):
                if self.grid[r][c] == self.OBSTACLE:
                    for dr in range(-cells, cells + 1):
                        for dc in range(-cells, cells + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < self.height and 0 <= nc < self.width:
                                new_map.grid[nr][nc] = self.OBSTACLE
        return new_map

    # ---- 规划辅助 ----
    def neighbors(self, cell: Cell) -> List[Cell]:
        """8 邻域可通行邻居。"""
        col, row = cell
        result: List[Cell] = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nc, nr = col + dc, row + dr
                if self.is_free((nc, nr)):
                    result.append((nc, nr))
        return result

    def obstacle_cells(self) -> List[Cell]:
        return [
            (c, r)
            for r in range(self.height)
            for c in range(self.width)
            if self.grid[r][c] == self.OBSTACLE
        ]

    def clear(self) -> None:
        for r in range(self.height):
            for c in range(self.width):
                self.grid[r][c] = self.FREE
