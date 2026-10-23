"""資料輸入層測試（不連網、不需金鑰、不需飛控）：python -m unittest tests.test_inputs -v"""
import json
import os
import tempfile
import unittest
from datetime import datetime, timedelta, timezone
from unittest import mock

from drone_agent.constraints import active_notams, evaluate_envelope
from drone_agent.geometry import segment_intersects_rect
from drone_agent.inputs import airspace, cwa_weather, telemetry
from drone_agent.inputs.base import ERROR, MISSING, OK, STALE, SourceResult
from drone_agent.inputs.builder import load_mission_input
from drone_agent.inputs.geo import LocalFrame
from drone_agent.supervisor import MissionSupervisor
from drone_agent.planner import RuleBasedPlanner

HERE = os.path.dirname(__file__)
ROOT = os.path.dirname(HERE)
CWA_FIXTURE = os.path.join(HERE, "fixtures", "cwa_O-A0001-001.json")
TW = timezone(timedelta(hours=8))
NOW = datetime(2026, 10, 23, 15, 0, tzinfo=TW).astimezone(timezone.utc)   # 測站觀測後 10 分鐘


def write_json(d, obj):
    path = os.path.join(d, f"{len(os.listdir(d))}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False)
    return path


def ok_telemetry(*args, **kwargs):
    return SourceResult("TELEMETRY", "MAVSDK", OK, {"battery_soc": 0.9}, NOW, True, "fixture")


class GeoTests(unittest.TestCase):
    def test_round_trip_and_scale(self):
        f = LocalFrame(22.62, 120.31)
        x, y = f.to_xy(22.63, 120.32)
        self.assertAlmostEqual(y, 1105.4, delta=1)
        self.assertAlmostEqual(x, 1027.6, delta=2)
        lat, lon = f.to_latlon(x, y)
        self.assertAlmostEqual(lat, 22.63, places=6)
        self.assertAlmostEqual(lon, 120.32, places=6)

    def test_circle_rect_is_conservative(self):
        r = LocalFrame(22.62, 120.31).circle_rect(22.62, 120.31, 100)
        self.assertEqual((r.xmin, r.xmax), (-100, 100))


class CwaTests(unittest.TestCase):
    def setUp(self):
        with open(CWA_FIXTURE, encoding="utf-8") as f:
            self.payload = json.load(f)

    def test_nearest_station_and_units(self):
        r = cwa_weather.parse_station_payload(self.payload, 22.62, 120.31)
        self.assertEqual(r.status, OK)
        self.assertIn("TEST01", r.data["station"])
        self.assertAlmostEqual(r.data["wind_speed_kmh"], 3.2 * 3.6)
        self.assertAlmostEqual(r.data["peak_gust_kmh"], 6.1 * 3.6)
        self.assertEqual(r.data["temperature_c"], 27.4)
        self.assertFalse(r.data["precipitation_active"])
        self.assertEqual(r.observed_at, datetime(2026, 10, 23, 6, 50, tzinfo=timezone.utc))

    def test_missing_values_are_errors_not_guesses(self):
        r = cwa_weather.parse_station_payload(self.payload, 22.62, 120.31, station_id="TEST02")
        self.assertEqual(r.status, ERROR)
        self.assertIn("WindSpeed", r.detail)

    def test_unexpected_format(self):
        self.assertEqual(cwa_weather.parse_station_payload({"oops": 1}, 22.6, 120.3).status, ERROR)

    def test_fetch_without_key(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(cwa_weather.fetch(22.6, 120.3).status, MISSING)


class AirspaceFileTests(unittest.TestCase):
    def test_caa_geojson_and_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = write_json(d, {"type": "FeatureCollection", "retrieved_at": "2026-10-23T09:00:00+08:00",
                                  "verified": True, "features": [{"type": "Feature", "properties": {"name": "Z"},
                                  "geometry": {"type": "Polygon", "coordinates": [[[120.31, 22.62], [120.32, 22.63], [120.31, 22.63]]]}}]})
            r = airspace.load_caa_geojson(path)
        self.assertEqual((r.status, r.verified), (OK, True))
        self.assertEqual(r.data["zones"][0]["area"]["coordinates"][0], [22.62, 120.31])   # 已轉成 [lat, lon]
        self.assertEqual(airspace.load_caa_geojson("nope.geojson").status, MISSING)

    def test_bad_notam_entry(self):
        with tempfile.TemporaryDirectory() as d:
            path = write_json(d, {"notams": [{"id": "X"}]})
            self.assertEqual(airspace.load_notam_file(path).status, ERROR)


class TelemetryTests(unittest.TestCase):
    def test_missing_mavsdk_is_reported(self):
        with mock.patch.dict("sys.modules", {"mavsdk": None}):
            self.assertEqual(telemetry.fetch().status, MISSING)


class GateTests(unittest.TestCase):
    def real_input(self, d, weather_file=CWA_FIXTURE, notam_time="2026-10-23T14:30:00+08:00",
                   verified=True, notam_area=None):
        caa = write_json(d, {"type": "FeatureCollection", "retrieved_at": "2026-10-22T09:00:00+08:00",
                             "verified": verified, "features": []})
        notam = write_json(d, {"retrieved_at": notam_time, "verified": verified, "notams": [
            {"id": "TEST-1", "valid_from": "2026-10-23T14:00:00+08:00", "valid_until": "2026-10-23T18:00:00+08:00",
             "area": notam_area or {"type": "circle", "center": [22.9, 120.9], "radius_m": 500}}]})
        return write_json(d, {
            "simulation": False,
            "mission": {"start": [22.62, 120.31], "goal": [22.625, 120.315]},
            "weather": {"source": "CWA_FILE", "file": weather_file},
            "airspace": {"source": "CAA_GEOJSON", "file": caa},
            "notam": {"source": "AIS_FILE", "file": notam},
            "telemetry": {"source": "MAVSDK"},
            "route": {"source": "GIS_SURVEY", "surveyed_at": "2026-10-20T10:00:00+08:00", "verified": verified},
        })

    def load(self, path):
        with mock.patch.object(telemetry, "fetch", ok_telemetry):
            return load_mission_input(path, now=NOW)

    def test_example_simulated_input_passes(self):
        inputs = load_mission_input(os.path.join(ROOT, "data", "mission_input.json"), now=NOW)
        self.assertEqual(inputs.issues, [])
        self.assertAlmostEqual(inputs.soc, 0.85)
        self.assertEqual(len(inputs.environment.obstacles), 1)

    def test_shipped_real_example_is_rejected(self):
        inputs = load_mission_input(os.path.join(ROOT, "data", "mission_input.real.example.json"), now=NOW)
        self.assertGreaterEqual(len(inputs.issues), 4)
        self.assertIsNone(inputs.environment)

    def test_real_mode_all_fresh_and_verified_passes(self):
        with tempfile.TemporaryDirectory() as d:
            inputs = self.load(self.real_input(d))
        self.assertEqual(inputs.issues, [])
        self.assertAlmostEqual(inputs.environment.wind_kmh, 3.2 * 3.6)
        self.assertAlmostEqual(inputs.soc, 0.9)

    def test_real_mode_rejects_unverified(self):
        with tempfile.TemporaryDirectory() as d:
            issues = self.load(self.real_input(d, verified=False)).issues
        self.assertEqual({i.split(":")[0] for i in issues}, {"AIRSPACE", "NOTAM", "GIS"})

    def test_real_mode_rejects_stale_notam(self):
        with tempfile.TemporaryDirectory() as d:
            issues = self.load(self.real_input(d, notam_time="2026-10-23T09:00:00+08:00")).issues
        self.assertEqual(len(issues), 1)
        self.assertTrue(issues[0].startswith("NOTAM: STALE"))

    def test_real_mode_rejects_simulated_source(self):
        with tempfile.TemporaryDirectory() as d:
            path = self.real_input(d)
            with open(path, encoding="utf-8") as f:
                cfg = json.load(f)
            cfg["telemetry"] = {"source": "SIMULATED", "remaining_percent": 90}
            path = write_json(d, cfg)
            issues = load_mission_input(path, now=NOW).issues
        self.assertIn("TELEMETRY: simulated source not allowed in real-data mode", issues)

    def test_simulated_flags_block_takeoff(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.path.join(ROOT, "data", "mission_input.json"), encoding="utf-8") as f:
                cfg = json.load(f)
            cfg["airspace"]["restricted"] = True
            cfg["notam"]["notam_clear"] = False
            issues = load_mission_input(write_json(d, cfg), now=NOW).issues
        self.assertIn("AIRSPACE: airspace marked restricted", issues)
        self.assertIn("NOTAM: NOTAM not clear", issues)

    def test_active_notam_over_route_reaches_constraint_envelope(self):
        area = {"type": "circle", "center": [22.6225, 120.3125], "radius_m": 100}   # 航線中點
        with tempfile.TemporaryDirectory() as d:
            inputs = self.load(self.real_input(d, notam_area=area))
        self.assertEqual(inputs.issues, [])
        env, m = inputs.environment, inputs.mission
        active = active_notams(env.read_notams(0.0), 0.0)
        self.assertEqual([n.alert_id for n in active], ["TEST-1"])
        self.assertTrue(segment_intersects_rect(m.home, m.survey[0][1], active[0].area))
        e = evaluate_envelope(0.0, env.read_weather(0), env.read_notams(0), env.read_parks(0), inputs.soc,
                              [m.home, m.survey[0][1], m.home], m.altitude_m, m.airspeed_kmh, m.geofence, True)
        self.assertEqual(e.domains["AIRSPACE"].status, "RED")


class EndToEndTests(unittest.TestCase):
    def test_simulated_mission_input_flies(self):
        inputs = load_mission_input(os.path.join(ROOT, "data", "mission_input.json"), now=NOW)
        r = MissionSupervisor(inputs.environment, inputs.mission, RuleBasedPlanner(), soc=inputs.soc).run()
        self.assertEqual(r.status, "COMPLETED", r.log)
        self.assertGreaterEqual(r.rrt_runs, 1)
        self.assertGreaterEqual(r.soc_end, r.projected_home_soc)


if __name__ == "__main__":
    unittest.main()
