"""Energy benchmark: deterministic baseline vs. deterministic optimizer vs. mock strategist.

  baseline   fixed 36 km/h cruise (the repository default), RRT* route
  optimizer  sweeps cruise speeds; keeps plans that pass validate_trajectory and
             keep the reserve; picks the least estimated Wh (ties: shortest time)
  strategist MockStrategist proposals (one deliberately unsafe) reviewed through
             the gateway; picks the safe proposal with the least estimated Wh

Every choice is then flown in the full four-layer simulation, which reports the
energy actually drawn. All calls go through the SafetyGateway.
"""
import time

from .adapters.drone_agent_sim import DroneAgentSimAdapter, completed
from .gateway import GatewayRequest
from .scenarios import default_scenario
from .strategist import MockStrategist, review_proposals
from .tools import build_gateway

TOKEN = "benchmark-token-0123456789"
GOAL = [2000.0, 0.0]
ALTITUDE_M = 60.0
BASELINE_KMH = 36.0
SWEEP_KMH = (20.0, 25.0, 30.0, 36.0, 40.0, 45.0, 50.0)
STRATEGIST_PROPOSALS = [
    {"airspeed_kmh": 80.0, "rationale": "Fly as fast as possible to save time."},
    {"airspeed_kmh": 45.0, "rationale": "Balance speed and battery."},
    {"airspeed_kmh": 30.0, "rationale": "Fly gently to be safe in wind."},
]
CONDITIONS = {
    "calm": dict(),
    "breezy (20 km/h wind)": dict(wind_kmh=20.0),
    "cold, low battery (-8 C, 33 %)": dict(temperature_c=-8.0, soc=0.33),
}


class _Client:
    def __init__(self, gateway):
        self.gw, self.seq = gateway, 0

    def call(self, action, params):
        self.seq += 1
        return self.gw.execute(GatewayRequest("benchmark", action, params, f"b-{self.seq}", self.seq, time.time(), TOKEN))


def _spec(speed, mission_id="bench"):
    return {"goal": GOAL, "altitude_m": ALTITUDE_M, "airspeed_kmh": speed, "mission_id": mission_id}


def assess(client, speed):
    """plan -> validate -> estimate for one cruise speed; returns a dict with 'feasible'."""
    started = time.perf_counter()
    plan = client.call("plan_mission", _spec(speed))
    if not plan.ok or not plan.result["feasible"]:
        return {"airspeed_kmh": speed, "feasible": False, "why": plan.reason or plan.result.get("reason", "")}
    request = {"spec": _spec(speed), "waypoints": plan.result["waypoints"]}
    verdict = client.call("validate_trajectory", request).result
    energy = client.call("estimate_mission_energy", request).result
    return {
        "airspeed_kmh": speed,
        "feasible": verdict["valid"] and energy["reserve_ok"],
        "why": "; ".join(verdict["reasons"]),
        "estimated_wh": energy["estimated_wh"],
        "duration_s": energy["duration_s"],
        "decision_latency_ms": (time.perf_counter() - started) * 1000,
    }


def choose(client):
    baseline = assess(client, BASELINE_KMH)
    sweep = [assess(client, v) for v in SWEEP_KMH]
    feasible = [s for s in sweep if s["feasible"]]
    optimized = min(feasible, key=lambda s: (s["estimated_wh"], s["duration_s"])) if feasible else None
    review = review_proposals(client, _spec(BASELINE_KMH), MockStrategist(STRATEGIST_PROPOSALS).propose(None))
    return baseline, optimized, sweep, review


def fly(client, speed, mission_id):
    if speed is None:
        return None
    t = client.call("simulate_mission", _spec(speed, mission_id)).result
    p = client.call("evaluate_mission_performance", {"mission_id": mission_id}).result
    return {"airspeed_kmh": speed, "status": t["status"], "completed": p["completed"],
            "battery_wh_used": t["battery_wh_used"], "flight_time_s": t["sim_time_s"],
            "zone_violations": t["zone_violations"], "estimate_error_pct": p["estimate_error_pct"],
            "sim_wall_s": t["wall_time_s"]}


def run_benchmark(conditions=None, simulate=True):
    rows = []
    for name, overrides in (conditions or CONDITIONS).items():
        gateway = build_gateway(DroneAgentSimAdapter(default_scenario(**overrides)),
                                clients={"benchmark": (TOKEN, {"read", "simulate"})})
        client = _Client(gateway)
        baseline, optimized, sweep, review = choose(client)
        chosen_ai = review["chosen"]["airspeed_kmh"] if review["chosen"] else None
        row = {"condition": name, "baseline_plan": baseline, "optimizer_plan": optimized, "sweep": sweep,
               "strategist_review": review}
        if simulate:
            # Like a real system, only a plan that passed validation is flown.
            row["baseline"] = fly(client, BASELINE_KMH, "baseline") if baseline["feasible"] else \
                {"airspeed_kmh": BASELINE_KMH, "status": "NOT_FLOWN (failed validation)", "completed": False}
            row["optimizer"] = fly(client, optimized["airspeed_kmh"], "optimizer") if optimized else None
            row["strategist"] = fly(client, chosen_ai, "strategist")
        rows.append(row)
    return rows


def _fmt(run):
    if not run:
        return "no feasible plan"
    if "battery_wh_used" not in run:
        return f"{run['airspeed_kmh']:.0f} km/h  {run['status']}"
    return (f"{run['airspeed_kmh']:.0f} km/h  {run['status']:<10} {run['battery_wh_used']:5.1f} Wh  "
            f"{run['flight_time_s']:4.0f} s  violations {run['zone_violations']}")


def main():
    rows = run_benchmark()
    for row in rows:
        print(f"\n=== {row['condition']}")
        for label in ("baseline", "optimizer", "strategist"):
            print(f"  {label:<11} {_fmt(row[label])}")
        rejected = [p for p in row["strategist_review"]["proposals"] if not p["accepted"]]
        for p in rejected:
            print(f"  strategist proposal {p['airspeed_kmh']} km/h rejected: {p['rejected_by']}")
    flown = [row[k] for row in rows for k in ("baseline", "optimizer", "strategist")]
    for label in ("baseline", "optimizer", "strategist"):
        runs = [row[label] for row in rows]
        done = sum(1 for r in runs if r and r.get("completed"))
        print(f"\n{label:<11} completion {done}/{len(runs)}", end="")
        energies = [r["battery_wh_used"] for r in runs if r and r.get("completed")]
        if energies:
            print(f", mean energy {sum(energies) / len(energies):.1f} Wh over completed missions", end="")
    print(f"\n\nzone violations across all flown missions: {sum(r.get('zone_violations', 0) for r in flown if r)}")


if __name__ == "__main__":
    main()
