"""单元测试：A* 路径规划。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.navigation.grid_map import GridMap
from intelligent_robot.navigation.path_planner import AStarPlanner


class TestAStar(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap(width=20, height=20, resolution=1.0)
        self.planner = AStarPlanner(self.grid)

    def test_open_map_path(self):
        path = self.planner.plan((0, 0), (10, 10))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (10, 10))

    def test_no_path_when_goal_blocked(self):
        self.grid.set_obstacle((5, 5))
        # 目标被障碍包围
        for c in range(4, 7):
            for r in range(4, 7):
                self.grid.set_obstacle((c, r))
        path = self.planner.plan((0, 0), (5, 5))
        self.assertIsNone(path)

    def test_path_avoids_obstacle(self):
        # 中间一道墙
        for r in range(0, 15):
            self.grid.set_obstacle((10, r))
        path = self.planner.plan((5, 5), (15, 5))
        self.assertIsNotNone(path)
        # 路径不应经过障碍
        for cell in path:
            self.assertTrue(self.grid.is_free(cell), f"路径经过障碍 {cell}")

    def test_world_coordinate_path(self):
        grid = GridMap(width=50, height=50, resolution=0.1)
        planner = AStarPlanner(grid)
        path = planner.plan_xy((0.5, 0.5), (2.0, 2.0))
        self.assertIsNotNone(path)
        self.assertTrue(len(path) > 1)


if __name__ == "__main__":
    unittest.main()
