"""Interface every simulation backend implements.

The tools and the gateway only talk to this interface, so the same safety
gateway and MCP tools can later front other backends: PX4 SITL + Gazebo, an
Isaac Sim scene driven through PX4, an AMR or a manipulator. A backend reports
state and simulates; it never exposes a path to real actuators.
"""
from typing import Protocol


class SimulationAdapter(Protocol):
    backend_name: str

    # --- observable state (read) ---
    def battery_soc(self) -> float: ...
    def drone_position(self) -> list: ...          # [x, y, z] metres, ENU, origin = home
    def drone_velocity(self) -> list: ...
    def zones(self) -> list: ...                   # [{"name", "kind", "rect", ("text")}]
    def geofence(self) -> list: ...                # [xmin, ymin, xmax, ymax]
    def weather(self) -> dict: ...
    def airspace(self) -> dict: ...

    # --- planning and analysis (simulation only) ---
    def plan(self, spec) -> list: ...              # list of schemas.Waypoint, ending at home
    def validate(self, spec, waypoints) -> list: ...   # reasons; empty list means valid
    def estimate(self, spec, waypoints) -> dict: ...
    def simulate(self, spec) -> dict: ...          # telemetry
