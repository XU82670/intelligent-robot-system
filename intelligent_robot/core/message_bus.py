"""消息总线：模块间发布-订阅通信，对应 PPT「数据支撑层-通信中间件」。

纯标准库实现，线程安全。各模块通过主题解耦，便于独立开发与测试。
"""
from __future__ import annotations

import threading
from typing import Any, Callable, Dict, List


class MessageBus:
    """轻量级发布/订阅消息总线。

    Example:
        bus = MessageBus()
        bus.subscribe("perception/detections", on_detection)
        bus.publish("perception/detections", [det1, det2])
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subscribers: Dict[str, List[Callable[[Any], None]]] = {}

    def subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        """订阅主题，callback 会在 publish 时被同步调用。"""
        with self._lock:
            self._subscribers.setdefault(topic, []).append(callback)

    def unsubscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        with self._lock:
            if topic in self._subscribers and callback in self._subscribers[topic]:
                self._subscribers[topic].remove(callback)

    def publish(self, topic: str, data: Any = None) -> None:
        """向主题广播数据。单个订阅者异常不影响其它订阅者。"""
        with self._lock:
            callbacks = list(self._subscribers.get(topic, []))
        for cb in callbacks:
            try:
                cb(data)
            except Exception as exc:  # noqa: BLE001 - 隔离订阅者异常
                print(f"[MessageBus] subscriber error on '{topic}': {exc!r}")

    def topics(self) -> List[str]:
        with self._lock:
            return sorted(self._subscribers.keys())
