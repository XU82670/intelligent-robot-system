"""单元测试：PID 控制器与差分驱动运动学。"""
import os
import sys
import math
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.motion_control.pid import PID
from intelligent_robot.motion_control.kinematics import DifferentialDrive


class TestPID(unittest.TestCase):
    def test_proportional_response(self):
        pid = PID(kp=2.0, ki=0, kd=0, dt=0.1)
        out = pid.update(1.0)
        self.assertAlmostEqual(out, 2.0)

    def test_output_limits(self):
        pid = PID(kp=10.0, ki=0, kd=0, dt=0.1, output_limits=(-1.0, 1.0))
        out = pid.update(5.0)
        self.assertLessEqual(out, 1.0)
        self.assertGreaterEqual(out, -1.0)

    def test_reset(self):
        pid = PID(kp=1.0, ki=1.0, kd=0, dt=0.1)
        pid.update(1.0)
        pid.reset()
        # reset 后 prev_error 为 None，d 项应为 0
        out = pid.update(1.0)
        self.assertAlmostEqual(out, 1.0 + 0.1)  # p + i


class TestDifferentialDrive(unittest.TestCase):
    def setUp(self):
        self.dd = DifferentialDrive(wheel_radius=0.05, wheel_base=0.30)

    def test_inverse_forward_consistency(self):
        v, w = 0.5, 0.3
        left, right = self.dd.to_wheel_speeds(v, w)
        v2, w2 = self.dd.from_wheel_speeds(left, right)
        self.assertAlmostEqual(v, v2, places=6)
        self.assertAlmostEqual(w, w2, places=6)

    def test_straight_motion(self):
        pose = (0.0, 0.0, 0.0)
        new_pose = self.dd.integrate(pose, v=1.0, w=0.0, dt=1.0)
        self.assertAlmostEqual(new_pose[0], 1.0)
        self.assertAlmostEqual(new_pose[1], 0.0)
        self.assertAlmostEqual(new_pose[2], 0.0)

    def test_rotation_in_place(self):
        pose = (0.0, 0.0, 0.0)
        new_pose = self.dd.integrate(pose, v=0.0, w=math.pi / 2, dt=1.0)
        self.assertAlmostEqual(new_pose[0], 0.0, places=6)
        self.assertAlmostEqual(new_pose[1], 0.0, places=6)
        self.assertAlmostEqual(new_pose[2], math.pi / 2)


if __name__ == "__main__":
    unittest.main()
