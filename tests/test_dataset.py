"""单元测试：数据集构建与划分。"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from intelligent_robot.data_manager.dataset import DatasetBuilder


class TestDatasetBuilder(unittest.TestCase):
    def setUp(self):
        self.records = [{"id": i, "label": i % 3, "value": i * 10} for i in range(100)]
        self.builder = DatasetBuilder(self.records)

    def test_total(self):
        self.assertEqual(len(self.builder), 100)

    def test_split_sizes(self):
        result = self.builder.split(train=0.7, val=0.2, test=0.1)
        self.assertEqual(len(result["train"]), 70)
        self.assertEqual(len(result["val"]), 20)
        self.assertEqual(len(result["test"]), 10)

    def test_split_no_overlap(self):
        result = self.builder.split()
        all_ids = set()
        for part in ("train", "val", "test"):
            for rec in result[part]:
                self.assertNotIn(rec["id"], all_ids)
                all_ids.add(rec["id"])
        self.assertEqual(len(all_ids), 100)

    def test_stratified_split(self):
        result = self.builder.split(stratified=True)
        # 每个子集都应包含全部 3 个类别
        for part in ("train", "val", "test"):
            labels = set(rec["label"] for rec in result[part])
            self.assertEqual(labels, {0, 1, 2})

    def test_invalid_ratio(self):
        with self.assertRaises(ValueError):
            self.builder.split(train=0.5, val=0.5, test=0.5)

    def test_stats(self):
        stats = self.builder.stats()
        self.assertEqual(stats["total"], 100)
        self.assertEqual(stats["labels"], {0: 34, 1: 33, 2: 33})


if __name__ == "__main__":
    unittest.main()
