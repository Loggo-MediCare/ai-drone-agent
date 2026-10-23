"""任務規劃層（0.1–1 Hz）：提出航點，不負責判斷是否安全。"""
from dataclasses import dataclass, field

from .geometry import Rect, segment_intersects_rect
from .safety import WaypointCommand

DETOUR_MARGIN_M = 100.0


@dataclass
class Mission:
    name: str = "Centrepointe → Ottawa River survey"
    home: tuple = (0.0, 0.0)
    survey: list = field(default_factory=lambda: [
        ("WP1", (1500.0, 0.0)), ("WP2", (3000.0, 0.0)),
        ("WP3", (4500.0, 0.0)), ("WP4", (6000.0, 0.0)),
    ])
    altitude_m: float = 60.0
    airspeed_kmh: float = 36.0
    geofence: Rect = Rect(-500, -1500, 7000, 1500)


@dataclass
class PlanRequest:
    start: tuple
    mission: Mission
    remaining_survey: list            # [(label, (x, y)), ...]
    blocked_areas: list
    feedback: str = None


class RuleBasedPlanner:
    """確定性規劃器：依序飛完剩餘測量點後返航，遇到禁區就從上方或下方繞過。"""
    name = "rule-based"

    def plan(self, req):
        m = req.mission
        stops = list(req.remaining_survey) + [("HOME", m.home)]
        for label, p in stops:
            if any(a.contains(p) for a in req.blocked_areas):
                return None   # 測量點本身在禁區內，無法完成
        cmds, pos = [], req.start
        for label, p in stops:
            legs = self._route_leg(pos, p, req.blocked_areas, m.geofence)
            if legs is None:
                return None
            for i, q in enumerate(legs[:-1]):
                cmds.append(WaypointCommand(q[0], q[1], m.altitude_m, m.airspeed_kmh, f"DETOUR-{label}-{i + 1}"))
            cmds.append(WaypointCommand(p[0], p[1], m.altitude_m, m.airspeed_kmh, label))
            pos = p
        return cmds

    def _route_leg(self, a, b, blocked, geofence, depth=0):
        hit = next((r for r in blocked if segment_intersects_rect(a, b, r)), None)
        if hit is None:
            return [b]
        if depth >= 3:
            return None
        xs = (hit.xmin - DETOUR_MARGIN_M, hit.xmax + DETOUR_MARGIN_M)
        if b[0] < a[0]:
            xs = xs[::-1]
        options = []
        for y in (hit.ymax + DETOUR_MARGIN_M, hit.ymin - DETOUR_MARGIN_M):
            p1, p2 = (xs[0], y), (xs[1], y)
            if geofence.contains(p1) and geofence.contains(p2):
                options.append((abs(y - a[1]), p1, p2))
        for _, p1, p2 in sorted(options):
            route, pos = [], a
            for q in (p1, p2, b):
                sub = self._route_leg(pos, q, blocked, geofence, depth + 1)
                if sub is None:
                    break
                route += sub
                pos = q
            else:
                return route
        return None
