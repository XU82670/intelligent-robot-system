"""单元测试：消息总线。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.core.message_bus import MessageBus


class TestMessageBus(unittest.TestCase):
    def test_publish_subscribe(self):
        bus = MessageBus()
        received = []
        bus.subscribe("test/topic", lambda data: received.append(data))
        bus.publish("test/topic", {"value": 42})
        self.assertEqual(received, [{"value": 42}])

    def test_multiple_subscribers(self):
        bus = MessageBus()
        a, b = [], []
        bus.subscribe("t", lambda d: a.append(d))
        bus.subscribe("t", lambda d: b.append(d))
        bus.publish("t", 1)
        self.assertEqual(a, [1])
        self.assertEqual(b, [1])

    def test_unsubscribe(self):
        bus = MessageBus()
        received = []
        cb = lambda d: received.append(d)
        bus.subscribe("t", cb)
        bus.publish("t", 1)
        bus.unsubscribe("t", cb)
        bus.publish("t", 2)
        self.assertEqual(received, [1])

    def test_subscriber_error_isolated(self):
        bus = MessageBus()
        received = []
        def bad(data):
            raise RuntimeError("boom")
        bus.subscribe("t", bad)
        bus.subscribe("t", lambda d: received.append(d))
        bus.publish("t", 1)  # 不应抛异常
        self.assertEqual(received, [1])


if __name__ == "__main__":
    unittest.main()
