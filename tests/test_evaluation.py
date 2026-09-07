import json
import unittest
from copy import deepcopy
from pathlib import Path

from ksae_2026_autumn.evaluation import (
    any_wheel_outside_corridor,
    capped_time_benefit,
    classify_confusion,
    classify_pair,
    evaluate_pairs,
)


class EvaluationTests(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.records = json.loads((root / "configs/pairs.example.json").read_text())

    def test_truth_table_and_all_confusion_outcomes(self):
        for e, f, outcome in (
            (False, False, "UNNECESSARY"),
            (True, False, "NECESSARY_EFFECTIVE"),
            (True, True, "NECESSARY_INEFFECTIVE"),
            (False, True, "HARMFUL"),
        ):
            self.assertEqual(classify_pair(e, f), outcome)
            positive = outcome == "NECESSARY_EFFECTIVE"
            self.assertEqual(classify_confusion(True, outcome), "TP" if positive else "FP")
            self.assertEqual(classify_confusion(False, outcome), "FN" if positive else "TN")

    def test_example_excludes_invalid_pair(self):
        result = evaluate_pairs(self.records)
        self.assertEqual(result["valid_pairs"], 4)
        self.assertEqual(result["invalid_pairs"], 1)
        self.assertEqual(result["confusion"], {"TP": 1, "TN": 2, "FP": 1, "FN": 0})
        self.assertEqual(result["precision"], 0.5)
        self.assertEqual(result["recall"], 1.0)
        self.assertEqual(result["invalid_reasons"], {"incomplete_run": 1})

    def test_rejects_strings_partial_horizons_and_duplicates(self):
        for key, value in (("collision", "false"), ("duration_s", 2.0), ("duration_s", True)):
            records = deepcopy(self.records)
            records[0]["branch_e"][key] = value
            with self.assertRaises(ValueError):
                evaluate_pairs(records)
        with self.assertRaises(ValueError):
            evaluate_pairs(self.records + [self.records[0]])

    def test_empty_counts_are_undefined_rates(self):
        self.assertIsNone(evaluate_pairs([])["precision"])
        self.assertIsNone(evaluate_pairs([])["recall"])

    def test_wheels_and_capped_benefit(self):
        self.assertTrue(any_wheel_outside_corridor([True, True, False, True]))
        with self.assertRaises(ValueError):
            any_wheel_outside_corridor([True])
        self.assertEqual(capped_time_benefit(1, float("inf")), 2.0)
        with self.assertRaises(ValueError):
            capped_time_benefit(float("nan"), 1)
