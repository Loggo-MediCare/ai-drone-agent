"""The parser itself must reject bad input, not rely on later layers (found by mutation testing)."""
import math
import unittest

from physical_ai_drone.errors import InvalidParams
from physical_ai_drone.schemas import parse_mission_spec, parse_waypoints

SPEC = {"goal": [2000.0, 0.0], "altitude_m": 60.0, "airspeed_kmh": 36.0}
W = {"x": 1.0, "y": 1.0, "altitude_m": 60.0, "airspeed_kmh": 36.0}


class ParserTests(unittest.TestCase):
    def test_spec_rejects_non_finite_numbers_itself(self):
        for field, value in (("goal", [math.nan, 0.0]), ("goal", [0.0, math.inf]), ("altitude_m", math.nan),
                             ("airspeed_kmh", math.inf), ("max_sim_time_s", math.nan)):
            with self.assertRaisesRegex(InvalidParams, "finite"):
                parse_mission_spec({**SPEC, field: value})

    def test_spec_enforces_the_speed_and_altitude_limits_itself(self):
        for field, value, word in (("airspeed_kmh", 80.0, "airspeed"), ("airspeed_kmh", 50.0001, "airspeed"),
                                   ("airspeed_kmh", 0.0, "airspeed"), ("altitude_m", 500.0, "altitude"),
                                   ("altitude_m", 9.9, "altitude"), ("max_sim_time_s", 5000.0, "max_sim_time_s")):
            with self.assertRaisesRegex(InvalidParams, word):
                parse_mission_spec({**SPEC, field: value})
        self.assertEqual(parse_mission_spec({**SPEC, "airspeed_kmh": 50.0}).airspeed_kmh, 50.0)

    def test_waypoints_reject_non_finite_numbers_itself(self):
        for field in ("x", "y", "altitude_m", "airspeed_kmh"):
            for value in (math.nan, math.inf, -math.inf):
                with self.assertRaisesRegex(InvalidParams, "finite"):
                    parse_waypoints([{**W, field: value}])


if __name__ == "__main__":
    unittest.main()
