"""任務整合測試：用 2.4 km 的短任務跑完整四層（伺服層 1 kHz）。"""
import unittest

from demo import OverEagerRoutePlanner
from drone_agent.geometry import Rect, dist
from drone_agent.planner import Mission, RuleBasedPlanner
from drone_agent.route_planner import RoutePlanner
from drone_agent.safety import WaypointCommand
from drone_agent.supervisor import MissionSupervisor
from drone_agent.world import Environment, Gust, Notam

SHORT = dict(survey=[("WP1", (600.0, 0.0)), ("WP2", (1200.0, 0.0))])
TOWER = [Rect(850, -40, 950, 40)]


def fly(env=None, soc=0.82, planner=None, route_planner=None, mission=None):
    env = env or Environment(obstacles=TOWER)
    sup = MissionSupervisor(env, mission or Mission(**SHORT), planner or RuleBasedPlanner(), soc=soc)
    if route_planner:
        sup.route_planner = route_planner(sup.motion)
    return sup.run(max_time_s=1500), sup


class MissionTests(unittest.TestCase):
    def test_nominal_with_rrt_detour(self):
        r, sup = fly()
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertEqual(r.surveyed, ["WP1", "WP2"])
        self.assertGreaterEqual(r.rrt_runs, 1)
        self.assertLess(dist(sup.body.xy, (0, 0)), 10)
        self.assertGreaterEqual(r.soc_end, r.projected_home_soc)   # 事前估算是保守的
        # 多速率：伺服 = 10 × 軌跡
        self.assertEqual(r.layer_ticks["伺服層"], 10 * r.layer_ticks["軌跡層"])

    def test_no_go_never_arms(self):
        for env, soc in [(Environment(temperature_c=-12), 0.82), (Environment(), 0.25),
                         (Environment(weather_frozen_at=-1200.0), 0.82)]:
            r, sup = fly(env, soc)
            self.assertEqual(r.status, "NO_GO")
            self.assertEqual(r.layer_ticks["伺服層"], 0)

    def test_notam_reroute(self):
        n = Notam("TFR", Rect(250, -100, 450, 100), valid_from=20, valid_until=1e9, published_at=20)
        r, sup = fly(Environment(notams=[n], obstacles=TOWER))
        self.assertEqual(r.status, "COMPLETED_REROUTED", r.log)
        self.assertEqual(r.reroutes, 1)
        self.assertEqual(r.zone_violations, 0)

    def test_notam_blocking_corridor_returns_home(self):
        n = Notam("TFR", Rect(250, -2000, 450, 2000), valid_from=60, valid_until=1e9, published_at=60)
        r, sup = fly(Environment(notams=[n], obstacles=TOWER))
        self.assertEqual(r.status, "RETURNED_HOME", r.log)
        self.assertLess(dist(sup.body.xy, (0, 0)), 10)

    def test_gust_hold(self):
        r, sup = fly(Environment(gusts=[Gust(40, 60, 32.0)], obstacles=TOWER))
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertEqual(r.holds, 1)

    def test_survey_credit_requires_reaching_the_point(self):
        class ForgingRoutePlanner(RoutePlanner):
            """返航航線的第一點偽造成 WP1（survey_id=WP1，但座標是目前位置）。"""
            def plan(self, start, order, mission, exclusion_zones, obstacles):
                cmds, why = super().plan(start, order, mission, exclusion_zones, obstacles)
                if cmds and not order:
                    cmds.insert(0, WaypointCommand(start[0] + 1, start[1], mission.altitude_m,
                                                   mission.airspeed_kmh, "WP1", survey_id="WP1"))
                return cmds, why

        n = Notam("TFR", Rect(250, -2000, 450, 2000), valid_from=10, valid_until=1e9, published_at=10)
        r, sup = fly(Environment(notams=[n], obstacles=TOWER), route_planner=ForgingRoutePlanner)
        self.assertEqual(r.status, "RETURNED_HOME", r.log)
        self.assertEqual(r.surveyed, [])
        self.assertTrue(any("忽略未經驗證的測量完成" in line for line in r.log), r.log)

    def test_unsafe_plan_rejected_then_replanned(self):
        r, sup = fly(route_planner=OverEagerRoutePlanner)
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertTrue(any("安全層拒絕" in line for line in r.log))


if __name__ == "__main__":
    unittest.main()
