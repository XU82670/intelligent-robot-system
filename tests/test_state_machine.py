"""单元测试：决策状态机。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.decision.state_machine import (
    AVOID,
    DONE,
    IDLE,
    NAVIGATE,
    TASK,
    DecisionContext,
    StateMachine,
)


class TestStateMachine(unittest.TestCase):
    def test_initial_state(self):
        fsm = StateMachine()
        self.assertEqual(fsm.state, IDLE)

    def test_valid_transition(self):
        fsm = StateMachine()
        self.assertTrue(fsm.transition(NAVIGATE))
        self.assertEqual(fsm.state, NAVIGATE)

    def test_invalid_transition(self):
        fsm = StateMachine()
        # IDLE 不能直接到 TASK
        self.assertFalse(fsm.transition(TASK))
        self.assertEqual(fsm.state, IDLE)

    def test_auto_navigate_on_goal(self):
        fsm = StateMachine()
        ctx = DecisionContext(has_goal=True, goal_reached=False)
        fsm.update(ctx)
        self.assertEqual(fsm.state, NAVIGATE)

    def test_avoid_on_obstacle(self):
        fsm = StateMachine(initial=NAVIGATE)
        ctx = DecisionContext(obstacle_near=True, obstacle_cleared=False)
        fsm.update(ctx)
        self.assertEqual(fsm.state, AVOID)

    def test_done_on_goal_reached(self):
        fsm = StateMachine(initial=NAVIGATE)
        ctx = DecisionContext(goal_reached=True, task_ready=False)
        fsm.update(ctx)
        self.assertEqual(fsm.state, DONE)

    def test_history_recorded(self):
        fsm = StateMachine()
        fsm.transition(NAVIGATE)
        fsm.transition(DONE)
        self.assertEqual(fsm.history, [IDLE, NAVIGATE, DONE])


if __name__ == "__main__":
    unittest.main()
