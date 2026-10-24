"""The 11 MCP tools, called through the gateway, over the real drone_agent backend.

Mission: virtual drone flies from A (home, origin) to B (2000, 0) and back. A
50 m-wide tower sits on the straight line. Written before the implementation.
"""
import math
import unittest

from drone_agent.geometry import Rect, segment_intersects_rect
from physical_ai_drone.adapters.drone_agent_sim import DroneAgentSimAdapter
from physical_ai_drone.gateway import GatewayRequest
from physical_ai_drone.scenarios import TOWER, default_scenario
from physical_ai_drone.tools import build_gateway

TOKEN = "test-token-0123456789"
SPEC = {"goal": [2000.0, 0.0], "altitude_m": 60.0, "airspeed_kmh": 36.0}


class Client:
    """Signs each call with a fresh nonce and the next sequence number."""

    def __init__(self, gateway, client_id="planner", token=TOKEN):
        self.gw, self.client_id, self.token, self.seq = gateway, client_id, token, 0

    def call(self, action, params=None):
        self.seq += 1
        return self.gw.execute(GatewayRequest(self.client_id, action, params or {}, f"{self.client_id}-{self.seq}",
                                              self.seq, self.gw._clock(), self.token))


def make(scenario=None, scopes=("read", "simulate")):
    adapter = DroneAgentSimAdapter(scenario or default_scenario())
    gw = build_gateway(adapter, clients={"planner": (TOKEN, set(scopes))})
    return Client(gw), adapter


def path_of(waypoints):
    return [(0.0, 0.0)] + [(w["x"], w["y"]) for w in waypoints]


class ReadToolTests(unittest.TestCase):
    def test_drone_state_starts_disarmed_at_home(self):
        c, _ = make()
        r = c.call("get_drone_state")
        self.assertTrue(r.ok, r.reason)
        self.assertEqual(r.result["position_m"], [0.0, 0.0, 0.0])
        self.assertFalse(r.result["armed"])
        self.assertEqual(r.result["frame"], "ENU, metres, origin = home (A)")

    def test_world_model_lists_obstacles_geofence_and_environment(self):
        c, _ = make()
        r = c.call("get_world_model")
        self.assertTrue(r.ok, r.reason)
        w = r.result
        self.assertEqual(w["schema"], "physical_ai_drone.world_model/v1")
        self.assertIn({"name": "tower", "kind": "obstacle", "rect": [TOWER.xmin, TOWER.ymin, TOWER.xmax, TOWER.ymax]},
                      w["zones"])
        self.assertEqual(w["geofence"], [-500.0, -800.0, 2500.0, 800.0])
        self.assertIn("battery", w)
        self.assertIn("weather", w)

    def test_battery_status(self):
        c, _ = make()
        b = c.call("get_battery_status").result
        self.assertAlmostEqual(b["soc"], 0.9)
        self.assertAlmostEqual(b["available_wh"], 0.9 * 355.0)
        self.assertEqual((b["reserve_soc"], b["departure_min_soc"]), (0.15, 0.30))

    def test_weather_constraints_green_then_red_in_strong_wind(self):
        c, _ = make()
        self.assertEqual(c.call("get_weather_constraints").result["status"], "GREEN")
        c2, _ = make(default_scenario(wind_kmh=30.0))
        w = c2.call("get_weather_constraints").result
        self.assertEqual(w["status"], "RED")
        self.assertIn("wind", w["detail"])

    def test_airspace_constraints_report_notams(self):
        sc = default_scenario(notams=[("NOTAM-1", Rect(1200, -400, 1600, 400), "Temporary flight restriction")])
        c, _ = make(sc)
        a = c.call("get_airspace_constraints").result
        self.assertEqual(a["notams"][0]["id"], "NOTAM-1")
        self.assertTrue(a["notams"][0]["active"])
        self.assertEqual(a["notams"][0]["text"], {"untrusted": True, "value": "Temporary flight restriction"})


class PlanAndValidateTests(unittest.TestCase):
    def test_plan_reaches_b_returns_home_and_avoids_the_tower(self):
        c, _ = make()
        r = c.call("plan_mission", SPEC)
        self.assertTrue(r.ok, r.reason)
        wps = r.result["waypoints"]
        self.assertEqual((wps[-1]["x"], wps[-1]["y"]), (0.0, 0.0))
        self.assertTrue(any(math.dist((w["x"], w["y"]), (2000.0, 0.0)) < 1.0 for w in wps))
        self.assertGreater(len(wps), 2, "straight line is blocked by the tower, so RRT* must add a detour")
        pts = path_of(wps)
        self.assertFalse(any(segment_intersects_rect(a, b, TOWER) for a, b in zip(pts, pts[1:])))
        self.assertTrue(c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps}).result["valid"])

    def test_validate_rejects_a_straight_line_through_the_tower(self):
        c, _ = make()
        wps = [{"x": 2000.0, "y": 0.0, "altitude_m": 60.0, "airspeed_kmh": 36.0},
               {"x": 0.0, "y": 0.0, "altitude_m": 60.0, "airspeed_kmh": 36.0}]
        v = c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps}).result
        self.assertFalse(v["valid"])
        self.assertTrue(any("obstacle" in reason for reason in v["reasons"]), v["reasons"])

    def test_validate_enforces_speed_altitude_geofence_return_home_and_goal(self):
        c, _ = make()
        plan = c.call("plan_mission", SPEC).result["waypoints"]

        def with_(**changes):
            return [{**w, **changes} for w in plan]

        cases = {
            "airspeed": with_(airspeed_kmh=80.0),
            "altitude": with_(altitude_m=500.0),
            "geofence": plan[:-1] + [{**plan[-1], "x": 0.0, "y": 5000.0}, plan[-1]],
            "home": plan[:-1],
            "goal": [w for w in plan if math.dist((w["x"], w["y"]), (2000.0, 0.0)) >= 1.0],
        }
        for word, wps in cases.items():
            v = c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps}).result
            self.assertFalse(v["valid"], word)
            self.assertTrue(any(word in reason for reason in v["reasons"]), (word, v["reasons"]))

    def test_validate_enforces_the_acceleration_limit(self):
        c, _ = make()
        wps = c.call("plan_mission", SPEC).result["waypoints"]
        wps = [dict(w) for w in wps]
        # 1 m after the first waypoint, slow to 5 km/h: 36 -> 5 km/h over 1 m is about 48 m/s^2.
        wps.insert(1, {**wps[0], "x": wps[0]["x"] + 1.0, "airspeed_kmh": 5.0})
        v = c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps}).result
        self.assertFalse(v["valid"])
        self.assertTrue(any("acceleration" in reason for reason in v["reasons"]), v["reasons"])

    def test_validate_rejects_a_route_that_would_end_below_reserve(self):
        c, _ = make(default_scenario(soc=0.30, temperature_c=-9.0))  # cold raises consumption by 18 %
        wps = c.call("plan_mission", SPEC).result["waypoints"]
        v = c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps}).result
        self.assertFalse(v["valid"])
        # Must come from the envelope check itself, not only from the final SafetyMonitor.
        self.assertTrue(any(reason.startswith("BATTERY RED") for reason in v["reasons"]), v["reasons"])


class EnergyTests(unittest.TestCase):
    def test_energy_estimate_is_positive_and_consistent(self):
        c, _ = make()
        wps = c.call("plan_mission", SPEC).result["waypoints"]
        e = c.call("estimate_mission_energy", {"spec": SPEC, "waypoints": wps}).result
        self.assertGreater(e["estimated_wh"], 0)
        self.assertAlmostEqual(e["estimated_wh"], (0.9 - e["projected_home_soc"]) * 355.0, places=6)
        self.assertGreater(e["duration_s"], sum(math.dist(a, b) for a, b in zip(path_of(wps), path_of(wps)[1:])) / 10.0)
        self.assertTrue(e["reserve_ok"])

    def test_energy_estimate_does_not_depend_on_starting_charge(self):
        # Regression: the envelope stops projecting at the first waypoint where the battery
        # check fails, so a low starting charge used to truncate the estimate.
        estimates = []
        for soc in (0.9, 0.31):
            c, _ = make(default_scenario(soc=soc, temperature_c=-8.0))
            wps = c.call("plan_mission", SPEC).result["waypoints"]
            e = c.call("estimate_mission_energy", {"spec": SPEC, "waypoints": wps}).result
            estimates.append(e)
        self.assertAlmostEqual(estimates[0]["estimated_wh"], estimates[1]["estimated_wh"], places=6)
        self.assertTrue(estimates[0]["reserve_ok"])
        self.assertFalse(estimates[1]["reserve_ok"])

    def test_faster_cruise_uses_less_estimated_energy_under_this_model(self):
        c, _ = make()
        result = {}
        for v in (20.0, 36.0, 50.0):
            spec = {**SPEC, "airspeed_kmh": v}
            wps = c.call("plan_mission", spec).result["waypoints"]
            result[v] = c.call("estimate_mission_energy", {"spec": spec, "waypoints": wps}).result["estimated_wh"]
        self.assertGreater(result[20.0], result[36.0])
        self.assertGreater(result[36.0], result[50.0])


class SimulationTests(unittest.TestCase):
    def test_simulated_mission_completes_safely_and_is_evaluated(self):
        c, _ = make()
        s = c.call("simulate_mission", {**SPEC, "mission_id": "m-1"})
        self.assertTrue(s.ok, s.reason)
        t = s.result
        self.assertEqual(t["status"], "COMPLETED", t["reason"])
        self.assertEqual(t["zone_violations"], 0)
        self.assertTrue(t["reached_goal"])
        self.assertGreater(t["battery_wh_used"], 0)

        self.assertEqual(c.call("get_mission_telemetry", {"mission_id": "m-1"}).result["status"], "COMPLETED")
        perf = c.call("evaluate_mission_performance", {"mission_id": "m-1"}).result
        self.assertTrue(perf["completed"])
        self.assertEqual(perf["zone_violations"], 0)
        self.assertGreaterEqual(perf["estimate_error_pct"], -50)
        self.assertLessEqual(perf["estimate_error_pct"], 50)
        self.assertTrue(perf["reserve_kept"])

    def test_no_go_when_weather_is_red(self):
        c, _ = make(default_scenario(wind_kmh=30.0))
        t = c.call("simulate_mission", {**SPEC, "mission_id": "windy"}).result
        self.assertEqual(t["status"], "NO_GO")
        self.assertEqual(t["battery_wh_used"], 0)

    def test_unknown_mission_telemetry_is_invalid(self):
        c, _ = make()
        r = c.call("get_mission_telemetry", {"mission_id": "never-ran"})
        self.assertEqual((r.ok, r.code), (False, "INVALID_PARAMS"))


class SchemaTests(unittest.TestCase):
    def test_bad_specs_are_rejected_before_reaching_the_simulator(self):
        c, _ = make()
        bad = [
            {**SPEC, "goal": [math.nan, 0.0]},
            {**SPEC, "goal": [math.inf, 0.0]},
            {**SPEC, "goal": "2000,0"},
            {**SPEC, "goal": [2000.0]},
            {**SPEC, "altitude_m": "60"},
            {**SPEC, "altitude_m": True},
            {**SPEC, "airspeed_kmh": -5.0},
            {**SPEC, "start": [100.0, 0.0]},
            {**SPEC, "unexpected": 1},
            {"goal": [2000.0, 0.0]},
            {**SPEC, "mission_id": "x" * 100},
        ]
        for spec in bad:
            r = c.call("plan_mission", spec)
            self.assertEqual((r.ok, r.code), (False, "INVALID_PARAMS"), spec)

    def test_waypoint_lists_are_bounded_and_typed(self):
        c, _ = make()
        w = {"x": 1.0, "y": 1.0, "altitude_m": 60.0, "airspeed_kmh": 36.0}
        for wps in ([], [w] * 501, [{**w, "x": "1"}], [{**w, "extra": 1}], "not-a-list", [{**w, "label": "x" * 100}]):
            r = c.call("validate_trajectory", {"spec": SPEC, "waypoints": wps})
            self.assertEqual((r.ok, r.code), (False, "INVALID_PARAMS"), str(wps)[:60])


if __name__ == "__main__":
    unittest.main()
