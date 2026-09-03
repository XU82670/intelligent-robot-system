"""决策层：有限状态机。对应 PPT「决策层-任务决策」。

状态：IDLE(待机) -> NAVIGATE(导航) -> AVOID(避障) -> TASK(执行任务) -> RETURN(返航) -> DONE(完成)
事件驱动转换，每步 update 根据上下文自动推进。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional


# 状态枚举（用字符串便于序列化与日志）
IDLE = "IDLE"
NAVIGATE = "NAVIGATE"
AVOID = "AVOID"
TASK = "TASK"
RETURN = "RETURN"
DONE = "DONE"
ERROR = "ERROR"

ALL_STATES = [IDLE, NAVIGATE, AVOID, TASK, RETURN, DONE, ERROR]


@dataclass
class DecisionContext:
    """状态机决策上下文：各模块共享的运行时信息。"""
    has_goal: bool = False
    goal_reached: bool = False
    obstacle_near: bool = False
    obstacle_cleared: bool = True
    task_ready: bool = False
    task_done: bool = False
    at_home: bool = True
    error: Optional[str] = None
    retries: int = 0
    extra: Dict = field(default_factory=dict)


class StateMachine:
    """任务决策有限状态机。

    支持：
      - transition(event)  事件驱动显式转换
      - update(ctx)         根据上下文自动推进（每控制周期调用）
      - on_enter/on_exit 回调
    """

    # 合法转换表：from_state -> set of to_states
    TRANSITIONS: Dict[str, set] = {
        IDLE: {NAVIGATE, ERROR},
        NAVIGATE: {AVOID, TASK, DONE, RETURN, ERROR, IDLE},
        AVOID: {NAVIGATE, ERROR, IDLE},
        TASK: {RETURN, DONE, ERROR, IDLE},
        RETURN: {DONE, ERROR, IDLE},
        DONE: {IDLE},
        ERROR: {IDLE},
    }

    def __init__(self, initial: str = IDLE) -> None:
        if initial not in ALL_STATES:
            raise ValueError(f"未知初始状态: {initial}")
        self.state = initial
        self.history: List[str] = [initial]
        self._on_enter: Dict[str, Callable[[], None]] = {}
        self._on_exit: Dict[str, Callable[[], None]] = {}

    def on_enter(self, state: str, callback: Callable[[], None]) -> None:
        self._on_enter[state] = callback

    def on_exit(self, state: str, callback: Callable[[], None]) -> None:
        self._on_exit[state] = callback

    def can_transition(self, to: str) -> bool:
        return to in self.TRANSITIONS.get(self.state, set())

    def transition(self, to: str) -> bool:
        """显式转换到目标状态；非法转换返回 False。"""
        if to == self.state:
            return True
        if not self.can_transition(to):
            return False
        if self.state in self._on_exit:
            self._on_exit[self.state]()
        self.state = to
        self.history.append(to)
        if to in self._on_enter:
            self._on_enter[to]()
        return True

    def update(self, ctx: DecisionContext) -> str:
        """根据上下文自动推进状态机，返回当前状态。"""
        if ctx.error:
            self.transition(ERROR)
            return self.state

        if self.state == IDLE:
            if ctx.has_goal and not ctx.goal_reached:
                self.transition(NAVIGATE)

        elif self.state == NAVIGATE:
            if ctx.obstacle_near and not ctx.obstacle_cleared:
                self.transition(AVOID)
            elif ctx.goal_reached:
                if ctx.task_ready:
                    self.transition(TASK)
                else:
                    self.transition(DONE)

        elif self.state == AVOID:
            if ctx.obstacle_cleared:
                self.transition(NAVIGATE)

        elif self.state == TASK:
            if ctx.task_done:
                self.transition(RETURN)

        elif self.state == RETURN:
            if ctx.at_home:
                self.transition(DONE)

        elif self.state == DONE:
            # 完成后可复位到 IDLE 等待新任务
            if not ctx.has_goal:
                self.transition(IDLE)

        return self.state

    def reset(self) -> None:
        self.state = IDLE
        self.history = [IDLE]
