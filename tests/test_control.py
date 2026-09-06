import unittest

from ksae_2026_autumn.control import ActionDelayFIFO, candidate_time, proposed_decision


class ControlTests(unittest.TestCase):
    def test_exact_delay_and_snapshot_isolation(self):
        fifo = ActionDelayFIFO(2, {"value": -1})
        action = {"value": 10}
        self.assertEqual(fifo.step(action), {"value": -1})
        saved = fifo.snapshot()
        action["value"] = 999
        self.assertEqual(fifo.step({"value": 20}), {"value": -1})
        self.assertEqual(fifo.step({"value": 30}), {"value": 10})
        fifo.restore(saved)
        saved[1]["value"] = 999
        fifo.step({"value": 20})
        self.assertEqual(fifo.step({"value": 30}), {"value": 10})

    def test_zero_delay_and_bad_delay(self):
        self.assertEqual(ActionDelayFIFO(0, 0).step(42), 42)
        for value in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                ActionDelayFIFO(value, 0)

    def test_candidate_formula(self):
        self.assertAlmostEqual(candidate_time(10, 100), 10.6)

    def test_normal_and_emergency_gates(self):
        gates = dict(
            action_age_gate=True,
            predicted_e2e_risk=True,
            fallback_benefit_gate=True,
            persistence_gate=True,
        )
        self.assertTrue(proposed_decision(**gates))
        for name in gates:
            self.assertFalse(proposed_decision(**(gates | {name: False})))
        gates["persistence_gate"] = False
        self.assertFalse(proposed_decision(**gates, emergency_override=True))
        self.assertTrue(
            proposed_decision(
                **gates,
                emergency_override=True,
                fallback_not_worse=True,
            )
        )
