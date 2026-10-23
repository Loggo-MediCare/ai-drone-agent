# 摘錄自第十六章 Listing 16.6，原文照錄。
# 已知問題（本專案已修正）：
#   1. 安全只靠提示詞中的 Protocol；dispatch_waypoint_command 本身不做任何檢查，
#      LLM 若跳過 check_flight_safety 仍可下達航點。
#   2. check_flight_safety 只檢查起飛電量 30%，缺少航點 20% 與返航 15% 的下限。
# Listing 16.6: Embodied drone agent with safety-constrained mission execution
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DroneFlightState:
    """Current belief state b(s) over the drone's physical and environmental state."""
    position_lat: float = 0.0       # Current GPS latitude
    position_lon: float = 0.0       # Current GPS longitude
    altitude_m: float = 0.0         # Current altitude (metres AGL)
    battery_soc: float = 1.0        # State of Charge, 0.0-1.0
    wind_speed_kmh: float = 0.0     # Wind speed at mission altitude
    temperature_c: float = 0.0      # Ambient temperature
    precipitation_active: bool = False  # Any active precip in corridor
    constraint_envelope_green: bool = False  # Unified Constraint Envelope status


# --- Tool layer: maps to control hierarchy layers (the Embodied Intelligence Agent section) ---
@tool
def check_flight_safety(state_json: str) -> dict:
    """Safety enforcement layer: validate current state against the Unified Constraint
    Envelope (the Constraint Formalization section). Maps to A_safe(s) from the Embodied Intelligence Agent section.
    Returns go/no-go with per-domain status."""
    import json
    s = json.loads(state_json)
    checks = {
        "temperature": s["temperature_c"] > -10.0,       # Author-defined operational limit
        "wind": s["wind_speed_kmh"] < 25.0,              # Mission-specific operational limit
        "precipitation": not s["precipitation_active"],
        "battery_departure": s["battery_soc"] >= 0.30,   # SoC floor: 30% at departure
    }
    envelope_green = all(checks.values())
    return {
        "envelope_green": envelope_green,
        "domain_status": checks,
        "safe_to_fly": envelope_green
    }


@tool
def query_flight_state(drone_id: str) -> dict:
    """World-model query layer: retrieve current telemetry belief state b(s)
    from drone sensors and environmental APIs. Maps to task-planning layer."""
    # Production: query live telemetry + weather API + battery management system
    return {
        "position_lat": 45.3490,
        "position_lon": -75.7544,
        "altitude_m": 0.0,
        "battery_soc": 0.82,
        "wind_speed_kmh": 18.5,
        "temperature_c": -6.2,
        "precipitation_active": False,
        "constraint_envelope_green": False   # Not yet evaluated
    }


@tool
def dispatch_waypoint_command(
    waypoint_lat: float,
    waypoint_lon: float,
    altitude_m: float,
    airspeed_kmh: float
) -> dict:
    """Motion-planning interface: submit a validated waypoint command to the
    deterministic flight controller. The controller executes at 50-200 Hz.
    This tool may only be called after check_flight_safety returns envelope_green=True."""
    # Production: serialize to MAVLink protocol and send to autopilot firmware
    return {
        "command_accepted": True,
        "waypoint": {
            "lat": waypoint_lat,
            "lon": waypoint_lon,
            "alt": altitude_m,
            "airspeed": airspeed_kmh
        },
        "estimated_duration_s": 120
    }


# Listing 16.6 (continued): Mission Supervisor agent construction and execution loop
# --- Mission Supervisor agent (task-planning layer, 0.1-1 Hz) ---
mission_supervisor = create_react_agent(
    ChatOpenAI(model="gpt-4o", temperature=0.0),
    [
        check_flight_safety,
        query_flight_state,
        dispatch_waypoint_command
    ]
)


def execute_mission(operator_instruction: str) -> str:
    """Entry point: translate operator intent into authorized flight actions.
    The agent must call check_flight_safety and receive envelope_green=True
    before any call to dispatch_waypoint_command is permitted."""
    result = mission_supervisor.invoke({
        "messages": [
            {
                "role": "user",
                "content": (
                    f"{operator_instruction}\n\n"
                    "Protocol: (1) call query_flight_state to retrieve current b(s). "
                    "(2) call check_flight_safety with the state JSON. "
                    "(3) Only if envelope_green is True, call dispatch_waypoint_command. "
                    "(4) If envelope_green is False, report which domains failed and do not arm."
                )
            }
        ]
    })
    return result['messages'][-1].content
