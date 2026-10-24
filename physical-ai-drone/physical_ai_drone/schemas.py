"""Typed tool contracts and strict input parsing.

Every parameter coming from a tool call (and so possibly from an LLM) is parsed
here before it reaches the simulator. Parsing is strict: unknown keys, strings
where numbers are expected, booleans, NaN and infinity are all rejected with
InvalidParams.

Physical limits are imported from drone_agent so there is one source of truth.
"""
import math
from dataclasses import dataclass

from drone_agent.constraints import MAX_ALT_M, MIN_ALT_M
from drone_agent.safety import MAX_AIRSPEED_KMH

from .errors import InvalidParams

WORLD_MODEL_SCHEMA = "physical_ai_drone.world_model/v1"
FRAME = "ENU, metres, origin = home (A)"

MAX_ACCEL_MPS2 = 3.0          # horizontal speed change allowed between consecutive waypoints
MAX_WAYPOINTS = 500
MAX_LABEL_LEN = 64
MAX_MISSION_ID_LEN = 64
GOAL_TOLERANCE_M = 1.0        # a plan must contain a waypoint this close to B
HOME_TOLERANCE_M = 1.0        # and end this close to A
MAX_SIM_TIME_S = 1800.0


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise InvalidParams(f"{name} must be a finite number")
    return float(value)


def _point(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise InvalidParams(f"{name} must be [x, y]")
    return (_number(value[0], f"{name}[0]"), _number(value[1], f"{name}[1]"))


def _only(params, allowed, required, where):
    if not isinstance(params, dict):
        raise InvalidParams(f"{where} must be an object")
    unknown = set(params) - set(allowed)
    if unknown:
        raise InvalidParams(f"{where}: unknown fields {sorted(unknown)}")
    missing = set(required) - set(params)
    if missing:
        raise InvalidParams(f"{where}: missing fields {sorted(missing)}")


def _mission_id(value):
    if not isinstance(value, str) or not 0 < len(value) <= MAX_MISSION_ID_LEN:
        raise InvalidParams(f"mission_id must be a non-empty string up to {MAX_MISSION_ID_LEN} chars")
    return value


@dataclass(frozen=True)
class MissionSpec:
    """Fly from A (home, the origin) to B and return home."""

    goal: tuple
    altitude_m: float
    airspeed_kmh: float
    mission_id: str = "default"
    max_sim_time_s: float = MAX_SIM_TIME_S

    @property
    def start(self):
        return (0.0, 0.0)


def parse_mission_spec(params) -> MissionSpec:
    _only(params, {"goal", "altitude_m", "airspeed_kmh", "start", "mission_id", "max_sim_time_s"},
          {"goal", "altitude_m", "airspeed_kmh"}, "mission spec")
    if "start" in params and _point(params["start"], "start") != (0.0, 0.0):
        raise InvalidParams("start (A) must be home [0, 0]; this backend always departs from home")
    altitude = _number(params["altitude_m"], "altitude_m")
    if not MIN_ALT_M <= altitude <= MAX_ALT_M:
        raise InvalidParams(f"altitude_m must be within {MIN_ALT_M:.0f}..{MAX_ALT_M:.0f} m")
    airspeed = _number(params["airspeed_kmh"], "airspeed_kmh")
    if not 0 < airspeed <= MAX_AIRSPEED_KMH:
        raise InvalidParams(f"airspeed_kmh must be within (0, {MAX_AIRSPEED_KMH:.0f}] km/h")
    sim_time = _number(params.get("max_sim_time_s", MAX_SIM_TIME_S), "max_sim_time_s")
    if not 0 < sim_time <= MAX_SIM_TIME_S:
        raise InvalidParams(f"max_sim_time_s must be within (0, {MAX_SIM_TIME_S:.0f}]")
    return MissionSpec(
        goal=_point(params["goal"], "goal"),
        altitude_m=altitude,
        airspeed_kmh=airspeed,
        mission_id=_mission_id(params.get("mission_id", "default")),
        max_sim_time_s=sim_time,
    )


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    altitude_m: float
    airspeed_kmh: float
    label: str = ""

    def to_dict(self):
        return {"x": self.x, "y": self.y, "altitude_m": self.altitude_m, "airspeed_kmh": self.airspeed_kmh,
                "label": self.label}


def parse_waypoints(value) -> list:
    """Structure and types only. Whether the values are safe is decided by validate_trajectory."""
    if not isinstance(value, list) or not 0 < len(value) <= MAX_WAYPOINTS:
        raise InvalidParams(f"waypoints must be a list of 1..{MAX_WAYPOINTS} items")
    out = []
    for i, w in enumerate(value):
        _only(w, {"x", "y", "altitude_m", "airspeed_kmh", "label"}, {"x", "y", "altitude_m", "airspeed_kmh"},
              f"waypoint {i}")
        label = w.get("label", "")
        if not isinstance(label, str) or len(label) > MAX_LABEL_LEN:
            raise InvalidParams(f"waypoint {i}: label must be a string up to {MAX_LABEL_LEN} chars")
        out.append(Waypoint(_number(w["x"], f"waypoint {i}.x"), _number(w["y"], f"waypoint {i}.y"),
                            _number(w["altitude_m"], f"waypoint {i}.altitude_m"),
                            _number(w["airspeed_kmh"], f"waypoint {i}.airspeed_kmh"), label))
    return out


def parse_plan_request(params):
    _only(params, {"spec", "waypoints"}, {"spec", "waypoints"}, "request")
    return parse_mission_spec(params["spec"]), parse_waypoints(params["waypoints"])


def parse_mission_ref(params) -> str:
    _only(params, {"mission_id"}, {"mission_id"}, "request")
    return _mission_id(params["mission_id"])


def untrusted(text: str) -> dict:
    """Wraps externally sourced text (NOTAMs, LLM rationales) so no consumer mistakes it for instructions."""
    return {"untrusted": True, "value": str(text)}


def rect_list(r) -> list:
    return [float(r.xmin), float(r.ymin), float(r.xmax), float(r.ymax)]
