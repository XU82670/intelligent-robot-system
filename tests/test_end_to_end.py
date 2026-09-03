"""端到端测试：完整系统仿真（感知->决策->规划->控制->运动）。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator.run_simulation import run


class TestEndToEnd(unittest.TestCase):
    def test_simulation_reaches_goal(self):
        """机器人应能规划路径并到达目标附近。"""
        result = run(start=(0.5, 0.5), goal=(4.5, 4.5), verbose=False)
        self.assertTrue(result["success"], f"未到达目标: {result}")
        self.assertLess(result["distance_to_goal"], 0.3)

    def test_simulation_open_world(self):
        """无障碍世界应快速到达。"""
        from intelligent_robot.robot import Robot

        robot = Robot()
        robot.start()
        robot.localizer.reset(0.5, 0.5, 0.0)
        ok = robot.goto(3.0, 3.0)
        self.assertTrue(ok)
        for _ in range(300):
            robot.update(0.1)
            if robot.fsm.state == "DONE":
                break
        final = robot.localizer.pose
        dist = ((final.x - 3.0) ** 2 + (final.y - 3.0) ** 2) ** 0.5
        self.assertLess(dist, 0.3)


if __name__ == "__main__":
    unittest.main()
