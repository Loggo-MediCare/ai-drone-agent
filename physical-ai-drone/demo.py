"""A -> B demonstration through the safety gateway, step by step.

    .venv/bin/python demo.py

1. Observe: world model, battery, weather.
2. Plan A -> B -> home with RRT* around the tower.
3. Validate the plan and estimate its energy.
4. Show the gateway rejecting a straight line through the tower and an unsafe request.
5. Fly the mission in the four-layer simulation and evaluate it.
"""
import time

import physical_ai_drone  # noqa: F401  (puts drone_agent on the path)
from physical_ai_drone.adapters.drone_agent_sim import DroneAgentSimAdapter
from physical_ai_drone.gateway import GatewayRequest
from physical_ai_drone.scenarios import default_scenario
from physical_ai_drone.tools import build_gateway

TOKEN = "demo-token-0123456789"
SPEC = {"goal": [2000.0, 0.0], "altitude_m": 60.0, "airspeed_kmh": 50.0, "mission_id": "demo"}


class Client:
    def __init__(self, gateway):
        self.gw, self.seq = gateway, 0

    def call(self, action, params=None):
        self.seq += 1
        r = self.gw.execute(GatewayRequest("demo", action, params or {}, f"demo-{self.seq}", self.seq, time.time(), TOKEN))
        status = "ALLOW" if r.ok else f"DENY {r.code}"
        print(f"  -> {action:<28} {status}" + ("" if r.ok else f"  ({r.reason})"))
        return r


def main():
    c = Client(build_gateway(DroneAgentSimAdapter(default_scenario()), clients={"demo": (TOKEN, {"read", "simulate"})}))

    print("1. Observe")
    world = c.call("get_world_model").result
    print(f"     zones: {[z['name'] for z in world['zones']]}, geofence {world['geofence']}")
    print(f"     battery {world['battery']['soc']:.0%}, weather {world['weather']['status']} "
          f"({world['weather']['temperature_c']} C, wind {world['weather']['wind_kmh']} km/h)")

    print("2. Plan A (0, 0) -> B (2000, 0) -> home")
    plan = c.call("plan_mission", SPEC).result
    for w in plan["waypoints"]:
        print(f"     {w['label']:<10} ({w['x']:7.1f}, {w['y']:7.1f})  {w['airspeed_kmh']:.0f} km/h")

    print("3. Validate and estimate")
    request = {"spec": SPEC, "waypoints": plan["waypoints"]}
    print(f"     valid: {c.call('validate_trajectory', request).result['valid']}")
    e = c.call("estimate_mission_energy", request).result
    print(f"     estimated {e['estimated_wh']:.1f} Wh, {e['duration_s']:.0f} s, home at {e['projected_home_soc']:.0%}")

    print("4. What the gateway refuses")
    straight = [{"x": 2000.0, "y": 0.0, "altitude_m": 60.0, "airspeed_kmh": 50.0},
                {"x": 0.0, "y": 0.0, "altitude_m": 60.0, "airspeed_kmh": 50.0}]
    v = c.call("validate_trajectory", {"spec": SPEC, "waypoints": straight}).result
    print(f"     straight line through the tower: valid={v['valid']}; {v['reasons'][0]}")
    c.call("set_motor_pwm", {"duty": 100})
    c.call("plan_mission", {**SPEC, "airspeed_kmh": 120.0})

    print("5. Simulate and evaluate (four-layer simulation, about 3 s)")
    t = c.call("simulate_mission", SPEC).result
    p = c.call("evaluate_mission_performance", {"mission_id": "demo"}).result
    print(f"     {t['status']}: {t['battery_wh_used']:.1f} Wh used (estimate {p['estimated_wh']:.1f} Wh, "
          f"{p['estimate_error_pct']:+.0f} %), {t['sim_time_s']:.0f} s, {t['distance_m']:.0f} m, "
          f"zone violations {t['zone_violations']}, reserve kept {p['reserve_kept']}")

    print(f"\nAudit log: {len(c.gw.audit_log())} entries, "
          f"{sum(1 for e in c.gw.audit_log() if e['decision'] != 'ALLOW')} denied")


if __name__ == "__main__":
    main()
