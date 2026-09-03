"""感知层：目标识别检测器。对应 PPT「感知层-目标识别」。

ObjectDetector 为统一接口，实际实现可插拔：
  - SimpleColorDetector : 基于颜色的简单检测（可选依赖 numpy/opencv-python）
  - MockDetector        : 纯标准库模拟检测，用于仿真与测试
  - YoloDetector        : YOLO 深度模型接口占位，接入真实模型时实现

Detection 为统一检测结果结构。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Detection:
    label: str                # 目标类别
    confidence: float         # 置信度 (0~1)
    bbox: Optional[tuple] = None  # (x, y, w, h)，图像坐标系
    position: Optional[tuple] = None  # (x, y) 世界坐标系（可空）

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "bbox": list(self.bbox) if self.bbox else None,
            "position": list(self.position) if self.position else None,
        }


class ObjectDetector:
    """目标检测器抽象基类。"""

    def detect(self, frame) -> List[Detection]:
        raise NotImplementedError


class MockDetector(ObjectDetector):
    """模拟检测器：从帧中按配置返回预置目标，纯标准库，用于仿真/测试。"""

    def __init__(self, targets: Optional[List[dict]] = None) -> None:
        # targets: [{"label": "person", "confidence": 0.9, "position": (x, y)}]
        self._targets: List[dict] = list(targets or [])
        self._cursor = 0

    def set_targets(self, targets: List[dict]) -> None:
        self._targets = list(targets)

    def detect(self, frame=None) -> List[Detection]:
        out: List[Detection] = []
        for t in self._targets:
            pos = tuple(t["position"]) if t.get("position") else None
            out.append(
                Detection(
                    label=t.get("label", "unknown"),
                    confidence=float(t.get("confidence", 1.0)),
                    position=pos,
                )
            )
        return out


class SimpleColorDetector(ObjectDetector):
    """基于颜色的简单目标检测（依赖可选包 numpy + opencv-python）。

    未安装可选依赖时 detect() 返回空列表并给出提示，不影响系统其它部分运行。
    """

    def __init__(self, targets: Optional[List[dict]] = None) -> None:
        # targets: [{"label": "red_ball", "hsv_low": (0,150,100), "hsv_high": (10,255,255)}]
        self._targets = list(targets or [])
        self._cv2 = None
        try:
            import cv2  # type: ignore
            self._cv2 = cv2
        except ImportError:
            self._cv2 = None

    def detect(self, frame) -> List[Detection]:
        if self._cv2 is None or frame is None:
            return []
        cv2 = self._cv2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        out: List[Detection] = []
        for t in self._targets:
            mask = cv2.inRange(
                hsv,
                tuple(t["hsv_low"]),
                tuple(t["hsv_high"]),
            )
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in sorted(contours, key=cv2.contourArea, reverse=True)[:1]:
                x, y, w, h = cv2.boundingRect(cnt)
                if w * h < 100:  # 过滤过小区域
                    continue
                out.append(
                    Detection(
                        label=t.get("label", "object"),
                        confidence=float(min(1.0, (w * h) / 20000.0)),
                        bbox=(x, y, w, h),
                    )
                )
        return out


class YoloDetector(ObjectDetector):
    """YOLO 深度模型接口占位。

    接入真实模型时：在 __init__ 中加载模型（如 ultralytics.YOLO），
    在 detect() 中运行推理并把结果包装为 Detection 列表。
    """

    def __init__(self, model_path: Optional[str] = None, **kwargs) -> None:
        self.model_path = model_path
        self.kwargs = kwargs

    def detect(self, frame) -> List[Detection]:
        raise NotImplementedError(
            "YoloDetector 为占位接口，请按实际模型实现 detect()。"
        )


def build_detector(name: str, **kwargs) -> ObjectDetector:
    """根据配置名称构建检测器。"""
    name = (name or "mock").lower()
    if name == "color":
        return SimpleColorDetector(**kwargs)
    if name == "yolo":
        return YoloDetector(**kwargs)
    return MockDetector(**kwargs)
