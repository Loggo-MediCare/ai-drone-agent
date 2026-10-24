"""Benchmark decisions (planning only, no full flights, so it stays fast)."""
import unittest

from physical_ai_drone.benchmark import CONDITIONS, run_benchmark


class BenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = {r["condition"]: r for r in run_benchmark(simulate=False)}

    def test_optimizer_never_needs_more_estimated_energy_than_a_feasible_baseline(self):
        for name, row in self.rows.items():
            base, opt = row["baseline_plan"], row["optimizer_plan"]
            self.assertIsNotNone(opt, name)
            self.assertTrue(opt["feasible"], name)
            if base["feasible"]:
                self.assertLessEqual(opt["estimated_wh"], base["estimated_wh"], name)

    def test_cold_low_battery_baseline_fails_validation_but_a_safe_plan_exists(self):
        row = self.rows["cold, low battery (-8 C, 33 %)"]
        self.assertFalse(row["baseline_plan"]["feasible"])
        self.assertIn("BATTERY", row["baseline_plan"]["why"])
        self.assertTrue(row["optimizer_plan"]["feasible"])

    def test_strategist_unsafe_proposal_is_always_rejected(self):
        for name, row in self.rows.items():
            verdicts = {p["airspeed_kmh"]: p for p in row["strategist_review"]["proposals"]}
            self.assertFalse(verdicts[80.0]["accepted"], name)
            self.assertEqual(verdicts[80.0]["rejected_by"], "INVALID_PARAMS", name)
            self.assertIsNotNone(row["strategist_review"]["chosen"], name)

    def test_all_conditions_were_evaluated(self):
        self.assertEqual(set(self.rows), set(CONDITIONS))


if __name__ == "__main__":
    unittest.main()
