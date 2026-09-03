"""数据支撑层：数据记录器。对应 PPT「数据管理模块-数据记录」。

Recorder 把各主题数据以 JSONL 格式追加写入文件，便于后续分析与回放。
纯标准库实现，线程安全。
"""
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict, Optional


class Recorder:
    """JSONL 数据记录器。

    每个主题一个文件：<out_dir>/<topic>.jsonl
    每行一条 JSON 记录，含 timestamp 与 data。
    """

    def __init__(self, out_dir: str = "logs") -> None:
        self.out_dir = out_dir
        self._lock = threading.Lock()
        self._files: Dict[str, Any] = {}
        os.makedirs(out_dir, exist_ok=True)

    def record(self, topic: str, data: Any, timestamp: Optional[float] = None) -> None:
        """记录一条数据。topic 中非法字符会被替换。"""
        safe_topic = topic.replace("/", "_").replace("\\", "_")
        path = os.path.join(self.out_dir, f"{safe_topic}.jsonl")
        entry = {"timestamp": timestamp if timestamp is not None else time.time(), "data": data}
        line = json.dumps(entry, ensure_ascii=False, default=str)
        with self._lock:
            fh = self._files.get(safe_topic)
            if fh is None or fh.closed:
                fh = open(path, "a", encoding="utf-8")
                self._files[safe_topic] = fh
            fh.write(line + "\n")
            fh.flush()

    def close(self) -> None:
        with self._lock:
            for fh in self._files.values():
                try:
                    fh.close()
                except Exception:
                    pass
            self._files.clear()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
