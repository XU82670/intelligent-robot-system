"""执行层：PID 控制器。对应 PPT「运动控制模块-运动控制」。

PID 用于航向/速度闭环控制，纯标准库实现，可直接单测。
"""
from __future__ import annotations

from typing import Optional, Tuple


class PID:
    """增量式 PID 控制器。

    Example:
        pid = PID(kp=1.2, ki=0.05, kd=0.15, dt=0.1, output_limits=(-1.5, 1.5))
        output = pid.update(error)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        dt: float = 0.1,
        output_limits: Optional[Tuple[float, float]] = None,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = max(1e-6, dt)
        self.output_limits = output_limits
        self._integral = 0.0
        self._prev_error: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None

    def update(self, error: float) -> float:
        """输入当前误差，返回控制量。"""
        p_term = self.kp * error

        self._integral += error * self.dt
        i_term = self.ki * self._integral

        if self._prev_error is None:
            d_term = 0.0
        else:
            d_term = self.kd * (error - self._prev_error) / self.dt
        self._prev_error = error

        output = p_term + i_term + d_term

        if self.output_limits is not None:
            low, high = self.output_limits
            output = max(low, min(high, output))
            # 抗积分饱和：输出被限幅时冻结积分
            if output != p_term + i_term + d_term:
                self._integral -= error * self.dt
        return output

    def set_params(self, kp: float, ki: float, kd: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
