"""各層單元測試：python -m unittest discover -s tests -v"""
import unittest

from drone_agent.constraints import GREEN, RED, STALE, evaluate_envelope
from drone_agent.geometry import Rect, segment_intersects_rect
from drone_agent.knowledge_graph import KnowledgeGraph, propagate_influence
from drone_agent.motion import CLEARANCE_M, MotionPlanner
from drone_agent.servo import KT_N_PER_A, ServoLayer
from drone_agent.world import Environment, Notam

FENCE = Rect(-500, -1500, 7000, 1500)


class GeometryTests(unittest.TestCase):
    def test_segment_rect(self):
        r = Rect(0, 0, 10, 10)
        self.assertTrue(segment_intersects_rect((-5, 5), (15, 5), r))
        self.assertFalse(segment_intersects_rect((-5, 20), (15, 20), r))
        self.assertTrue(segment_intersects_rect((5, 5), (6, 6), r))   # 完全在內部


class ServoTests(unittest.TestCase):
    def test_current_loop_tracks_hover_thrust(self):
        servo = ServoLayer(1000)
        target = 1.5 * 9.81
        for _ in range(500):
            thrust, power = servo.step(target)
        self.assertAlmostEqual(thrust, target, delta=0.3)
        self.assertGreater(power, 0)
        self.assertLess(servo.motors[0].i, 15.0)
        self.assertAlmostEqual(servo.motors[0].i, target / 4 / KT_N_PER_A, delta=0.15)


class MotionTests(unittest.TestCase):
    def test_straight_line_when_free(self):
        mp = MotionPlanner(FENCE)
        self.assertEqual(mp.plan((0, 0), (1000, 0), []), [(0, 0), (1000, 0)])
        self.assertEqual(mp.rrt_runs, 0)

    def test_rrt_star_avoids_obstacle(self):
        mp = MotionPlanner(FENCE, seed=1)
        tower = Rect(450, -60, 550, 60)
        path = mp.plan((0, 0), (1000, 0), [tower])
        self.assertIsNotNone(path)
        self.assertEqual(mp.rrt_runs, 1)
        self.assertTrue(mp.path_valid(path, [tower]))
        self.assertGreater(len(path), 2)

    def test_goal_inside_obstacle(self):
        mp = MotionPlanner(FENCE)
        self.assertIsNone(mp.plan((0, 0), (500, 0), [Rect(450, -60, 550, 60)]))


class EnvelopeTests(unittest.TestCase):
    def _eval(self, env, soc=0.82, path=((0, 0), (1500, 0), (0, 0)), t=0.0, preflight=True):
        return evaluate_envelope(t, env.read_weather(t), env.read_notams(t), env.read_parks(t),
                                 soc, list(path), 60.0, 36.0, FENCE, preflight)

    def test_all_green(self):
        self.assertTrue(self._eval(Environment()).green)

    def test_wind_limit(self):
        e = self._eval(Environment(wind_kmh=25.0))
        self.assertEqual(e.domains["WEATHER"].status, RED)

    def test_stale_vetoes(self):
        e = self._eval(Environment(weather_frozen_at=-1000.0))
        self.assertEqual(e.domains["WEATHER"].status, STALE)
        self.assertFalse(e.green)

    def test_notam_only_when_active_and_crossing(self):
        n = Notam("N1", Rect(500, -100, 700, 100), valid_from=100, valid_until=200, published_at=0)
        env = Environment(notams=[n])
        self.assertEqual(self._eval(env, t=50).domains["AIRSPACE"].status, GREEN)
        self.assertEqual(self._eval(env, t=150).domains["AIRSPACE"].status, RED)
        far = Environment(notams=[Notam("N2", Rect(500, 500, 700, 700), 0, 1e9, 0)])
        self.assertEqual(self._eval(far, t=150).domains["AIRSPACE"].status, GREEN)

    def test_battery_return_reserve(self):
        long_path = ((0, 0), (6000, 0), (0, 0), (6000, 0), (0, 0))
        self.assertEqual(self._eval(Environment(), soc=0.6, path=long_path).domains["BATTERY"].status, RED)


class KnowledgeGraphTests(unittest.TestCase):
    def test_weight_clamped_and_expert_precedence(self):
        kg = KnowledgeGraph()
        self.assertEqual(kg.add_edge("A", "B", "restricts", 3.0, source="llm").weight, 1.0)
        kg.add_edge("A", "C", "affects", 0.4, source="expert")
        self.assertEqual(kg.add_edge("A", "C", "affects", 0.9, source="llm").weight, 0.4)
        self.assertEqual(len(kg.audit_log), 3)

    def test_propagation_attenuates(self):
        kg = KnowledgeGraph()
        kg.add_edge("N", "L", "restricts", 1.0)
        kg.add_edge("L", "M", "part_of", 0.5)
        kg.add_edge("M", "X", "affects", 0.1)
        impacts = propagate_influence(kg, "N")
        self.assertEqual(impacts["M"], (0.5, ["N", "L", "M"]))
        self.assertNotIn("X", impacts)   # 0.05 < 門檻 0.1


if __name__ == "__main__":
    unittest.main()
