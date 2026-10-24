"""Simulation scenarios for the A -> B demonstration.

A = home (origin). B = (2000, 0). A 100 m x 120 m tower stands on the straight
line between them, so the planner must detour. Mild autumn weather by default.
"""
from dataclasses import dataclass, field

from drone_agent.geometry import Rect
from drone_agent.planner import Mission
from drone_agent.world import Environment, Notam

TOWER = Rect(950.0, -60.0, 1050.0, 60.0)
GEOFENCE = Rect(-500.0, -800.0, 2500.0, 800.0)


@dataclass
class Scenario:
    env: Environment
    soc: float
    geofence: Rect
    notam_texts: dict = field(default_factory=dict)   # NOTAM id -> free text from the issuer (untrusted)

    def mission(self, spec):
        return Mission(
            name=f"A -> B ({spec.goal[0]:.0f}, {spec.goal[1]:.0f})",
            home=(0.0, 0.0),
            survey=[("B", spec.goal)],
            altitude_m=spec.altitude_m,
            airspeed_kmh=spec.airspeed_kmh,
            geofence=self.geofence,
        )


def default_scenario(soc=0.9, temperature_c=15.0, wind_kmh=10.0, notams=(), weather_age_s=0.0):
    """notams: iterable of (id, Rect, text); they are active from t=0."""
    env = Environment(
        temperature_c=temperature_c,
        wind_kmh=wind_kmh,
        greenbelt=None,
        obstacles=[TOWER],
        notams=[Notam(nid, area, 0.0, 1e9, 0.0) for nid, area, _ in notams],
        weather_frozen_at=-weather_age_s if weather_age_s else None,
    )
    return Scenario(env=env, soc=soc, geofence=GEOFENCE, notam_texts={nid: text for nid, _, text in notams})
