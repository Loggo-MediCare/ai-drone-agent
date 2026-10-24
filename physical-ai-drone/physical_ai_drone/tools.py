"""The 11 tools. Each one parses its input strictly, checks that the backend's
state is physically possible, and returns plain JSON-like data. They are bound
to the SafetyGateway, which authenticates, authorizes and audits every call.

All tools are read-only or simulation-only.
"""
import math
import time

from drone_agent.constraints import SOC_DEPARTURE, SOC_RESERVE
from drone_agent.energy import BATTERY_WH

from .adapters.drone_agent_sim import completed
from .errors import InconsistentState, InvalidParams
from .gateway import ALLOWED_ACTIONS, SafetyGateway
from .schemas import FRAME, WORLD_MODEL_SCHEMA, parse_mission_ref, parse_mission_spec, parse_plan_request


def _no_params(params):
    if params:
        raise InvalidParams(f"this tool takes no parameters, got {sorted(params)}")


class DroneToolService:
    def __init__(self, adapter):
        self.adapter = adapter
        self._missions = {}  # mission_id -> {"telemetry", "estimate"}

    # ---------- state checks ----------
    def _soc(self):
        soc = self.adapter.battery_soc()
        if isinstance(soc, bool) or not isinstance(soc, (int, float)) or not math.isfinite(soc) or not 0 <= soc <= 1:
            raise InconsistentState(f"battery state of charge {soc!r} is not within 0..1")
        return float(soc)

    def _vector(self, values, name):
        if not isinstance(values, list) or len(values) != 3 or not all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            raise InconsistentState(f"{name} {values!r} is not a finite 3-vector")
        return [float(v) for v in values]

    def _checked_state(self):
        return self._soc(), self._vector(self.adapter.drone_position(), "position"), \
            self._vector(self.adapter.drone_velocity(), "velocity")

    # ---------- read tools ----------
    def get_drone_state(self, params):
        _no_params(params)
        soc, position, velocity = self._checked_state()
        return {"frame": FRAME, "position_m": position, "velocity_mps": velocity, "soc": soc, "armed": False,
                "mode": "SIMULATION"}

    def get_battery_status(self, params):
        _no_params(params)
        soc = self._soc()
        return {"soc": soc, "capacity_wh": BATTERY_WH, "available_wh": soc * BATTERY_WH,
                "reserve_soc": SOC_RESERVE, "departure_min_soc": SOC_DEPARTURE,
                "can_depart": soc >= SOC_DEPARTURE}

    def get_weather_constraints(self, params):
        _no_params(params)
        return self.adapter.weather()

    def get_airspace_constraints(self, params):
        _no_params(params)
        return self.adapter.airspace()

    def get_world_model(self, params):
        _no_params(params)
        return {
            "schema": WORLD_MODEL_SCHEMA,
            "frame": FRAME,
            "observed_at": time.time(),
            "backend": self.adapter.backend_name,
            "drone": self.get_drone_state({}),
            "battery": self.get_battery_status({}),
            "weather": self.adapter.weather(),
            "geofence": self.adapter.geofence(),
            "zones": self.adapter.zones(),
        }

    # ---------- simulation tools ----------
    def plan_mission(self, params):
        spec = parse_mission_spec(params)
        self._checked_state()
        waypoints, why = self.adapter.plan(spec)
        if waypoints is None:
            return {"mission_id": spec.mission_id, "feasible": False, "reason": why, "waypoints": []}
        return {"mission_id": spec.mission_id, "feasible": True, "planner": "rrt-star",
                "waypoints": [w.to_dict() for w in waypoints]}

    def validate_trajectory(self, params):
        spec, waypoints = parse_plan_request(params)
        self._checked_state()
        reasons = self.adapter.validate(spec, waypoints)
        return {"valid": not reasons, "reasons": reasons}

    def estimate_mission_energy(self, params):
        spec, waypoints = parse_plan_request(params)
        self._checked_state()
        return self.adapter.estimate(spec, waypoints)

    def simulate_mission(self, params):
        spec = parse_mission_spec(params)
        self._checked_state()
        estimate = None
        waypoints, _ = self.adapter.plan(spec)
        if waypoints:
            estimate = self.adapter.estimate(spec, waypoints)
        telemetry = self.adapter.simulate(spec)
        self._missions[spec.mission_id] = {"telemetry": telemetry, "estimate": estimate}
        return telemetry

    def get_mission_telemetry(self, params):
        return self._mission(params)["telemetry"]

    def evaluate_mission_performance(self, params):
        record = self._mission(params)
        t, est = record["telemetry"], record["estimate"]
        used = t["battery_wh_used"]
        error = None
        if est and used > 0:
            error = (est["estimated_wh"] - used) / used * 100
        return {
            "mission_id": t["mission_id"],
            "status": t["status"],
            "completed": completed(t["status"]),
            "reached_goal": t["reached_goal"],
            "zone_violations": t["zone_violations"],
            "battery_wh_used": used,
            "estimated_wh": est["estimated_wh"] if est else None,
            "estimate_error_pct": error,
            "wh_per_km": used / (t["distance_m"] / 1000) if t["distance_m"] else None,
            "flight_time_s": t["sim_time_s"],
            "reserve_kept": t["soc_end"] >= SOC_RESERVE,
            "holds": t["holds"],
            "reroutes": t["reroutes"],
        }

    def _mission(self, params):
        mission_id = parse_mission_ref(params)
        if mission_id not in self._missions:
            raise InvalidParams(f"no simulated mission with id {mission_id!r}")
        return self._missions[mission_id]


def build_gateway(adapter, clients, clock=time.time, handler_timeout_s=60.0):
    """clients: {client_id: (token, scopes)}."""
    service = DroneToolService(adapter)
    gateway = SafetyGateway(clock=clock, handler_timeout_s=handler_timeout_s)
    for client_id, (token, scopes) in clients.items():
        gateway.register_client(client_id, token, scopes)
    for action in ALLOWED_ACTIONS:
        gateway.bind(action, getattr(service, action))
    gateway.service = service
    return gateway
