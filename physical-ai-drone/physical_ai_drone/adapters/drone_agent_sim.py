"""Backend adapter over the repository's pure-Python drone_agent simulation.

Planning uses drone_agent's RRT* RoutePlanner, feasibility uses its five-domain
evaluate_envelope, the final authority is its SafetyMonitor, and simulate()
flies the full four-layer MissionSupervisor (mission, motion, trajectory PID,
1 kHz servo). Nothing here is mocked; what is simplified is listed in
docs/ARCHITECTURE.md ("Simulation-to-reality gaps").
"""
import copy
import math
import time

from drone_agent.constraints import MAX_ALT_M, MIN_ALT_M, SOC_RESERVE, blocked_areas, evaluate_envelope
from drone_agent.energy import BATTERY_WH, PROJECTION_MARGIN, estimated_power_w, soc_drain, soc_for_distance
from drone_agent.geometry import dist, segment_intersects_rect
from drone_agent.motion import CLEARANCE_M, MotionPlanner, inflate
from drone_agent.route_planner import RoutePlanner
from drone_agent.safety import MAX_AIRSPEED_KMH, SafetyMonitor, WaypointCommand
from drone_agent.supervisor import MissionSupervisor
from drone_agent.trajectory import CLIMB_MS, DESCENT_MS

from ..errors import InvalidParams
from ..schemas import (GOAL_TOLERANCE_M, HOME_TOLERANCE_M, MAX_ACCEL_MPS2, Waypoint, rect_list, untrusted)

NOW = 0.0  # simulation time at which the world is observed (preflight)
COMPLETED = ("COMPLETED", "COMPLETED_REROUTED")


class DroneAgentSimAdapter:
    backend_name = "drone_agent (pure-Python four-layer simulation)"

    def __init__(self, scenario):
        self.sc = scenario

    # ---------- observable state ----------
    def battery_soc(self):
        return self.sc.soc

    def drone_position(self):
        return [0.0, 0.0, 0.0]

    def drone_velocity(self):
        return [0.0, 0.0, 0.0]

    def geofence(self):
        return rect_list(self.sc.geofence)

    def _readings(self):
        env = self.sc.env
        return env.read_weather(NOW), env.read_notams(NOW), env.read_parks(NOW)

    def _envelope(self, path, altitude_m, airspeed_kmh, preflight=True):
        weather, airspace, parks = self._readings()
        return evaluate_envelope(NOW, weather, airspace, parks, self.battery_soc(), path, altitude_m, airspeed_kmh,
                                 self.sc.geofence, preflight)

    def zones(self):
        out = [{"name": f"obstacle-{i}" if i else "tower", "kind": "obstacle", "rect": rect_list(r)}
               for i, r in enumerate(self.sc.env.obstacles)]
        for n in self.sc.env.read_notams(NOW).data["notams"]:
            out.append({"name": n.alert_id, "kind": "no_fly", "rect": rect_list(n.area),
                        "active": n.valid_from <= NOW <= n.valid_until,
                        "text": untrusted(self.sc.notam_texts.get(n.alert_id, ""))})
        for name, rect, authorized in self.sc.env.read_parks(NOW).data["areas"]:
            out.append({"name": name, "kind": "protected", "rect": rect_list(rect), "authorized": authorized})
        return out

    def weather(self):
        weather, _, _ = self._readings()
        status = self._envelope([(0.0, 0.0), (0.0, 0.0)], 60.0, 36.0, preflight=False).domains["WEATHER"]
        d = weather.data
        return {
            "temperature_c": d["temperature_c"],
            "wind_kmh": d["wind_speed_kmh"],
            "wind_toward_deg": self.sc.env.wind_toward_deg,
            "precipitation": d["precipitation_active"],
            "data_age_s": NOW - weather.timestamp,
            "status": status.status,
            "detail": status.reason,
        }

    def airspace(self):
        _, airspace, _ = self._readings()
        notams = [{"id": n.alert_id, "rect": rect_list(n.area), "active": n.valid_from <= NOW <= n.valid_until,
                   "text": untrusted(self.sc.notam_texts.get(n.alert_id, ""))}
                  for n in airspace.data["notams"]]
        return {"notams": notams, "data_age_s": NOW - airspace.timestamp}

    # ---------- planning ----------
    def _mission(self, spec):
        return self.sc.mission(spec)

    def _blocked(self):
        _, airspace, parks = self._readings()
        return blocked_areas(airspace, parks, NOW)

    def plan(self, spec):
        mission = self._mission(spec)
        if not self.sc.geofence.contains(spec.goal):
            raise InvalidParams("goal B is outside the geofence")
        planner = RoutePlanner(MotionPlanner(self.sc.geofence))
        cmds, why = planner.plan((0.0, 0.0), ["B"], mission, self._blocked(), self.sc.env.obstacles)
        if cmds is None:
            return None, why
        return [Waypoint(c.x, c.y, c.altitude_m, c.airspeed_kmh, c.label) for c in cmds], ""

    def validate(self, spec, waypoints):
        """All reasons the plan is unsafe; an empty list means valid."""
        reasons = []
        path = [(0.0, 0.0)] + [(w.x, w.y) for w in waypoints]
        fence = self.sc.geofence

        for i, w in enumerate(waypoints):
            if not 0 < w.airspeed_kmh <= MAX_AIRSPEED_KMH:
                reasons.append(f"waypoint {i} airspeed {w.airspeed_kmh} km/h outside (0, {MAX_AIRSPEED_KMH:.0f}]")
            if not MIN_ALT_M <= w.altitude_m <= MAX_ALT_M:
                reasons.append(f"waypoint {i} altitude {w.altitude_m} m outside {MIN_ALT_M:.0f}..{MAX_ALT_M:.0f} m")
            if not fence.contains((w.x, w.y)):
                reasons.append(f"waypoint {i} ({w.x:.0f}, {w.y:.0f}) is outside the geofence")

        for i, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
            d = dist((a.x, a.y), (b.x, b.y))
            va, vb = a.airspeed_kmh / 3.6, b.airspeed_kmh / 3.6
            if va != vb:
                accel = math.inf if d == 0 else abs(vb * vb - va * va) / (2 * d)
                if accel > MAX_ACCEL_MPS2:
                    reasons.append(f"leg {i + 1} needs acceleration {accel:.1f} m/s^2 > {MAX_ACCEL_MPS2} m/s^2")

        if dist(path[-1], (0.0, 0.0)) > HOME_TOLERANCE_M:
            reasons.append("route does not return home (A)")
        if not any(dist(p, spec.goal) <= GOAL_TOLERANCE_M for p in path[1:]):
            reasons.append("route never reaches goal B")

        obstacles = inflate(self.sc.env.obstacles, CLEARANCE_M)
        names = ["tower"] + [f"obstacle-{i}" for i in range(1, len(obstacles))]
        clearance = self._mission(spec).exclusion_clearance_m
        restricted = self._blocked()
        for i, (a, b) in enumerate(zip(path, path[1:])):
            for name, r in zip(names, obstacles):
                if segment_intersects_rect(a, b, r):
                    reasons.append(f"leg {i + 1} passes within {CLEARANCE_M:.0f} m of obstacle {name}")
            for r in inflate(restricted, clearance):
                if segment_intersects_rect(a, b, r):
                    reasons.append(f"leg {i + 1} enters a restricted area or its {clearance:.0f} m clearance")

        envelope = self._envelope(path, spec.altitude_m, spec.airspeed_kmh, preflight=True)
        for domain, s in envelope.domains.items():
            if s.status != "GREEN":
                reasons.append(f"{domain} {s.status}: {s.reason}")

        # Independent final authority: drone_agent's SafetyMonitor.
        if not reasons:
            cmds = [WaypointCommand(w.x, w.y, w.altitude_m, w.airspeed_kmh, w.label) for w in waypoints]
            verdict = SafetyMonitor(fence, clearance).validate_plan(cmds, (0.0, 0.0), envelope, NOW)
            if not verdict.is_safe:
                reasons.append(f"SafetyMonitor: {verdict.reason}")
        return reasons

    def estimate(self, spec, waypoints):
        path = [(0.0, 0.0)] + [(w.x, w.y) for w in waypoints]
        envelope = self._envelope(path, spec.altitude_m, spec.airspeed_kmh, preflight=True)
        soc = self.battery_soc()
        projected = soc - self._mission_soc(path, spec)
        legs = [dist(a, b) for a, b in zip(path, path[1:])]
        cruise_s = sum(d / (w.airspeed_kmh / 3.6) for d, w in zip(legs, waypoints) if w.airspeed_kmh > 0)
        duration = spec.altitude_m / CLIMB_MS + cruise_s + spec.altitude_m / DESCENT_MS
        distance = sum(legs)
        wh = (soc - projected) * BATTERY_WH
        return {
            "estimated_wh": wh,
            "duration_s": duration,
            "distance_m": distance,
            "wh_per_km": wh / (distance / 1000) if distance else 0.0,
            "projected_home_soc": projected,
            "reserve_soc": SOC_RESERVE,
            "reserve_ok": projected >= SOC_RESERVE,
            "domains": {k: v.status for k, v in envelope.domains.items()},
            "basis": "drone_agent estimated_power_w with x1.2 projection margin and cold-weather factor",
        }

    def _mission_soc(self, path, spec):
        """State of charge the whole flight needs: same formula as drone_agent's envelope
        (climb and descent hover plus every leg, x1.2 margin, cold factor), but over the entire
        path. The envelope stops at the first waypoint that fails its battery check, so its
        projection is not a full-mission estimate."""
        weather = self.sc.env.read_weather(NOW).data
        wind, temp = weather["wind_speed_kmh"], weather["temperature_c"]
        hover_s = spec.altitude_m / CLIMB_MS + spec.altitude_m / DESCENT_MS
        need = soc_drain(estimated_power_w(0.0, wind), hover_s, temp) * PROJECTION_MARGIN
        for a, b in zip(path, path[1:]):
            need += soc_for_distance(dist(a, b), spec.airspeed_kmh, wind, temp)
        return need

    # ---------- simulation ----------
    def simulate(self, spec):
        mission = self._mission(spec)
        if not self.sc.geofence.contains(spec.goal):
            raise InvalidParams("goal B is outside the geofence")
        env = copy.deepcopy(self.sc.env)
        started = time.perf_counter()
        report = MissionSupervisor(env, mission, soc=self.battery_soc()).run(max_time_s=spec.max_sim_time_s)
        wall = time.perf_counter() - started
        return {
            "mission_id": spec.mission_id,
            "backend": self.backend_name,
            "status": report.status,
            "reason": untrusted(report.reason),
            "sim_time_s": report.t_end,
            "wall_time_s": wall,
            "soc_start": report.soc_start,
            "soc_end": report.soc_end,
            "battery_wh_used": (report.soc_start - report.soc_end) * BATTERY_WH,
            "distance_m": report.distance_m,
            "reached_goal": "B" in report.surveyed,
            "zone_violations": report.zone_violations,
            "holds": report.holds,
            "reroutes": report.reroutes,
            "max_track_err_m": report.max_track_err_m,
            "log_tail": [untrusted(line) for line in report.log[-8:]],
        }


def completed(status):
    return status in COMPLETED

