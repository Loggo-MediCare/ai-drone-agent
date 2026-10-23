"""任務規劃與幾何規劃分離後的回歸測試：python -m unittest tests.test_architecture -v

涵蓋：轉角切過禁區、安全距離不足、飛行中 NOTAM 變化、重新規劃失敗、
重複航點 ID、假的測量完成、返航也必須經過檢查。
"""
import unittest

from demo import BadOrderPlanner
from drone_agent.constraints import ConstraintEnvelope, DomainStatus, GREEN, DOMAINS
from drone_agent.geometry import Rect, segment_intersects_rect
from drone_agent.motion import MotionPlanner, inflate
from drone_agent.planner import Mission, MissionDecision, MissionRequest, RuleBasedPlanner, validate_decision
from drone_agent.route_planner import RoutePlanner
from drone_agent.safety import SafetyMonitor, WaypointCommand
from drone_agent.supervisor import MissionSupervisor
from drone_agent.world import Environment, Notam

FENCE = Rect(-500, -1500, 7000, 1500)
ZONE = Rect(1800, -300, 2700, 300)
SHORT = dict(survey=[("WP1", (600.0, 0.0)), ("WP2", (1200.0, 0.0))])


def green_envelope(t=0.0, blocked=()):
    return ConstraintEnvelope(t, {d: DomainStatus(GREEN) for d in DOMAINS}, list(blocked))


def cmd(x, y, label="P", airspeed=36.0):
    return WaypointCommand(x, y, 60.0, airspeed, label)


def fly(env, mission=None, planner=None, soc=0.82):
    sup = MissionSupervisor(env, mission or Mission(**SHORT), planner or RuleBasedPlanner(), soc=soc)
    return sup.run(max_time_s=1500), sup


class SegmentValidationTests(unittest.TestCase):
    def test_corner_clipping_rejected_although_both_waypoints_are_outside(self):
        a, b = (1700.0, 350.0), (2800.0, 250.0)          # 兩個端點都在禁區外
        self.assertFalse(ZONE.contains(a) or ZONE.contains(b))
        v = SafetyMonitor(FENCE, clearance_m=0).validate(cmd(*b), a, green_envelope(blocked=[ZONE]), 0.0)
        self.assertFalse(v.is_safe)
        self.assertIn("enters restricted area", v.reason)

    def test_insufficient_clearance_is_configurable(self):
        a, b = (1500.0, 350.0), (3000.0, 350.0)           # 平行禁區上緣，距離 50 m
        env = green_envelope(blocked=[ZONE])
        tight = SafetyMonitor(FENCE, clearance_m=100).validate(cmd(*b), a, env, 0.0)
        self.assertFalse(tight.is_safe)
        self.assertIn("within 100 m", tight.reason)
        self.assertTrue(SafetyMonitor(FENCE, clearance_m=30).validate(cmd(*b), a, env, 0.0).is_safe)

    def test_leaving_from_inside_clearance_band_is_allowed_but_not_entering_zone(self):
        s = SafetyMonitor(FENCE, clearance_m=100)
        env = green_envelope(blocked=[ZONE])
        self.assertTrue(s.validate(cmd(1000, 350), (1750, 350), env, 0.0).is_safe)    # 往外離開
        self.assertFalse(s.validate(cmd(2800, 0), (1750, 0), env, 0.0).is_safe)       # 穿過禁區


class RoutePlannerTests(unittest.TestCase):
    def setUp(self):
        self.mission = Mission(survey=[("WP1", (1500.0, 0.0)), ("WP2", (3000.0, 0.0))], exclusion_clearance_m=100)
        self.rp = RoutePlanner(MotionPlanner(FENCE, seed=3))

    def test_every_segment_keeps_clearance(self):
        cmds, why = self.rp.plan((0.0, 0.0), ["WP1", "WP2"], self.mission, [ZONE], [])
        self.assertIsNotNone(cmds, why)
        padded = inflate([ZONE], 100)[0]
        pts = [(0.0, 0.0)] + [c.xy for c in cmds]
        for a, b in zip(pts, pts[1:]):
            self.assertFalse(segment_intersects_rect(a, b, padded), (a, b))
        self.assertGreaterEqual(self.rp.motion.rrt_runs, 1)
        self.assertEqual([c.survey_id for c in cmds if c.survey_id], ["WP1", "WP2"])

    def test_waypoint_inside_clearance_cannot_be_routed(self):
        near = Rect(1550, -50, 1700, 50)                   # WP1 距離禁區 50 m < 100 m
        cmds, why = self.rp.plan((0.0, 0.0), ["WP1", "WP2"], self.mission, [near], [])
        self.assertIsNone(cmds)
        self.assertIn("WP1", why)


class DecisionValidationTests(unittest.TestCase):
    def test_duplicate_unknown_and_omitted_ids(self):
        ids = ["WP1", "WP2", "WP3"]
        self.assertIsNone(validate_decision(MissionDecision(["WP3", "WP1", "WP2"]), ids))
        self.assertIn("duplicate", validate_decision(MissionDecision(["WP1", "WP1", "WP2", "WP3"]), ids))
        self.assertIn("unknown", validate_decision(MissionDecision(["WP1", "WP2", "WP3", "WP9"]), ids))
        # 漏掉 WP1 等於宣稱已完成：拒絕
        self.assertIn("omits", validate_decision(MissionDecision(["WP2", "WP3"]), ids))
        self.assertIsNone(validate_decision(MissionDecision([], return_home=True), ids))

    def test_duplicate_waypoint_ids_in_mission_never_arm(self):
        m = Mission(survey=[("WP1", (600.0, 0.0)), ("WP1", (1200.0, 0.0))])
        r, sup = fly(Environment(obstacles=[]), mission=m)
        self.assertEqual(r.status, "NO_GO")
        self.assertIn("duplicate waypoint id", r.reason)
        self.assertEqual(r.layer_ticks["伺服層"], 0)

    def test_bad_planner_order_is_rejected_and_vehicle_still_surveys_wp1(self):
        m = Mission(survey=[("WP1", (600.0, 0.0)), ("WP2", (1200.0, 0.0)),
                            ("WP3", (1200.0, 300.0)), ("WP4", (600.0, 300.0))])
        r, sup = fly(Environment(obstacles=[]), mission=m, planner=BadOrderPlanner())
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertEqual(r.surveyed, ["WP1", "WP2", "WP3", "WP4"])
        self.assertTrue(any("決策被拒絕" in line and "duplicate" in line for line in r.log))

    def test_always_invalid_planner_falls_back_to_default_order(self):
        class AlwaysBad(RuleBasedPlanner):
            name = "always-bad"

            def decide(self, req):
                return MissionDecision(["WP9"])

        r, sup = fly(Environment(obstacles=[]), planner=AlwaysBad())
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertTrue(any("改用預設任務順序" in line for line in r.log))


class MidFlightNotamTests(unittest.TestCase):
    def test_notam_near_route_without_crossing_triggers_clearance_reroute(self):
        near = Notam("NEAR", Rect(800, 40, 1000, 200), valid_from=30, valid_until=1e9, published_at=30)
        r, sup = fly(Environment(notams=[near], obstacles=[]))
        self.assertEqual(r.status, "COMPLETED_REROUTED", r.log)
        self.assertEqual(r.zone_violations, 0)
        self.assertTrue(any("within 100 m" in line for line in r.log), r.log)

    def test_failed_replanning_returns_home_without_crediting_unreached_points(self):
        over_wp2 = Notam("OVER-WP2", Rect(1100, -100, 1300, 100), valid_from=30, valid_until=1e9, published_at=30)
        r, sup = fly(Environment(notams=[over_wp2], obstacles=[]))
        self.assertEqual(r.status, "RETURNED_HOME", r.log)
        self.assertNotIn("WP2", r.surveyed)
        self.assertTrue(any("航線規劃失敗" in line for line in r.log), r.log)
        self.assertEqual(r.zone_violations, 0)


class ReturnHomeValidationTests(unittest.TestCase):
    def test_rth_route_detours_around_zone_between_vehicle_and_home(self):
        ahead = Notam("AHEAD", Rect(1000, -2000, 1100, 2000), valid_from=70, valid_until=1e9, published_at=70)
        behind = Notam("BEHIND", Rect(200, -100, 300, 100), valid_from=70, valid_until=1e9, published_at=70)
        r, sup = fly(Environment(notams=[ahead, behind], obstacles=[]))
        self.assertEqual(r.status, "RETURNED_HOME", r.log)
        self.assertEqual(r.zone_violations, 0)
        self.assertTrue(any("返航航線" in line and "已通過安全檢查" in line for line in r.log), r.log)
        self.assertGreaterEqual(r.rrt_runs, 1)

    def _airborne(self, env, x, soc):
        sup = MissionSupervisor(env, Mission(**SHORT), RuleBasedPlanner(), soc=soc)
        sup.phase = "FLY"
        sup.body.p = [x, 0.0, 60.0]
        return sup

    def test_rth_not_attempted_when_battery_projection_insufficient(self):
        sup = self._airborne(Environment(obstacles=[]), 1200.0, soc=0.06)
        sup._rth(100.0, "test")
        self.assertEqual(sup.phase, "LAND")
        self.assertTrue(any("返航預估抵達電量" in line for line in sup.log), sup.log)

    def test_rth_not_assumed_safe_when_home_is_inside_restricted_airspace(self):
        home_zone = Notam("HOME-TFR", Rect(-100, -100, 100, 100), valid_from=0, valid_until=1e9, published_at=0)
        sup = self._airborne(Environment(notams=[home_zone], obstacles=[]), 1200.0, soc=0.8)
        sup._rth(100.0, "test")
        self.assertEqual(sup.phase, "LAND")
        self.assertTrue(any("找不到安全的返航航線" in line for line in sup.log), sup.log)


class LLMPlannerContractTests(unittest.TestCase):
    def test_llm_output_is_order_only(self):
        try:
            from drone_agent.llm_planner import LLMPlanner, _Decision
        except ImportError:
            self.skipTest("langchain-openai / pydantic not installed")

        class StubLLM:
            def invoke(self, prompt):
                self.prompt = prompt
                return _Decision(order=["WP2", "WP1"], return_home=False, reason="shorter")

        planner = object.__new__(LLMPlanner)        # 不呼叫 __init__，不需要 API 金鑰
        planner.instruction, planner._llm = "test", StubLLM()
        req = MissionRequest(Mission(**SHORT), (0.0, 0.0), 0.8, list(SHORT["survey"]))
        d = planner.decide(req)
        self.assertEqual((d.order, d.return_home), (["WP2", "WP1"], False))
        self.assertEqual(set(_Decision.model_fields), {"order", "return_home", "reason"})   # 沒有座標欄位
        self.assertIn("do not plan geometry", planner._llm.prompt)


if __name__ == "__main__":
    unittest.main()
