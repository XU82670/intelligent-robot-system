"""数据支撑层：数据集构建与划分。对应 PPT「数据资源构建-数据集划分」。

DatasetBuilder 从记录列表构建数据集，并按比例划分为 训练/验证/测试 集，
保证可复现（固定随机种子）。
"""
from __future__ import annotations

import random
from typing import Any, Dict, List, Optional


class DatasetBuilder:
    """数据集构建与划分工具。

    records: 样本列表，每个样本为 dict（含 label 等字段）
    """

    def __init__(self, records: Optional[List[Dict[str, Any]]] = None) -> None:
        self.records: List[Dict[str, Any]] = list(records or [])

    def add(self, record: Dict[str, Any]) -> None:
        self.records.append(record)

    def __len__(self) -> int:
        return len(self.records)

    def split(
        self,
        train: float = 0.7,
        val: float = 0.2,
        test: float = 0.1,
        seed: int = 42,
        stratified: bool = False,
    ) -> Dict[str, List[Dict[str, Any]]]:
        """按比例划分数据集。

        stratified=True 时按 "label" 字段分层抽样，保证各类别比例一致。
        """
        if abs(train + val + test - 1.0) > 1e-6:
            raise ValueError("train + val + test 必须等于 1.0")
        rng = random.Random(seed)
        data = list(self.records)
        rng.shuffle(data)

        if stratified and data and "label" in data[0]:
            return self._stratified_split(data, train, val, test, rng)

        n = len(data)
        n_train = int(n * train)
        n_val = int(n * val)
        return {
            "train": data[:n_train],
            "val": data[n_train : n_train + n_val],
            "test": data[n_train + n_val :],
        }

    @staticmethod
    def _stratified_split(
        data: List[Dict[str, Any]],
        train: float,
        val: float,
        test: float,
        rng: random.Random,
    ) -> Dict[str, List[Dict[str, Any]]]:
        groups: Dict[Any, List[Dict[str, Any]]] = {}
        for rec in data:
            groups.setdefault(rec.get("label"), []).append(rec)
        result: Dict[str, List[Dict[str, Any]]] = {"train": [], "val": [], "test": []}
        for items in groups.values():
            rng.shuffle(items)
            n = len(items)
            n_train = int(n * train)
            n_val = int(n * val)
            result["train"].extend(items[:n_train])
            result["val"].extend(items[n_train : n_train + n_val])
            result["test"].extend(items[n_train + n_val :])
        for key in result:
            rng.shuffle(result[key])
        return result

    def stats(self) -> Dict[str, Any]:
        """返回数据集统计信息。"""
        labels: Dict[Any, int] = {}
        for rec in self.records:
            lbl = rec.get("label", "unknown")
            labels[lbl] = labels.get(lbl, 0) + 1
        return {"total": len(self.records), "labels": labels}
