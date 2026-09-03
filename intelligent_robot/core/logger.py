"""日志工具：统一日志格式。对应 PPT「数据支撑层-日志管理」。"""
from __future__ import annotations

import logging


def setup_logger(name: str = "robot", level: str = "INFO") -> logging.Logger:
    """创建/获取带统一格式的 logger，避免重复添加 handler。"""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(
            logging.Formatter(
                "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                datefmt="%H:%M:%S",
            )
        )
        logger.addHandler(handler)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False
    return logger
