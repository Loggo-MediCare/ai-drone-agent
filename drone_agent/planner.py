"""任務規劃層（0.1–1 Hz）：只決定「做什麼」，不產生任何幾何路徑。

規劃器（規則式或 GPT）輸出 MissionDecision：
    - order：剩餘測量點 ID 的飛行順序（必須剛好是剩餘 ID 的一個排列）
    - return_home：是否建議返航
    - reason：理由

航線幾何（含繞行禁航區）一律由 route_planner.RoutePlanner 用 RRT* 計算；
任務完成狀態一律由監督器依實際位置判定。規劃器的輸出無法影響這兩者。
"""
from dataclasses import dataclass, field

from .geometry import Rect

DEFAULT_EXCLUSION_CLEARANCE_M = 100.0


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
    exclusion_clearance_m: float = DEFAULT_EXCLUSION_CLEARANCE_M   # 禁航區外擴安全距離

    def point(self, wid):
        return dict(self.survey).get(wid)

    def validate(self):
        issues = []
        ids = [wid for wid, _ in self.survey]
        dupes = sorted({wid for wid in ids if ids.count(wid) > 1})
        if dupes:
            issues.append(f"duplicate waypoint id: {', '.join(dupes)}")
        if any(not wid or wid == "HOME" for wid in ids):
            issues.append("waypoint id must be non-empty and not 'HOME'")
        if not ids:
            issues.append("mission has no survey waypoints")
        if self.exclusion_clearance_m < 0:
            issues.append("exclusion clearance must be ≥ 0")
        return issues


@dataclass
class MissionRequest:
    """給任務規劃器的資訊；規劃器看得到狀態，但改不了狀態。"""
    mission: Mission
    position: tuple
    soc: float
    remaining: list                  # [(id, (x, y)), …]
    envelope_summary: str = ""
    feedback: str = None


@dataclass
class MissionDecision:
    order: list
    return_home: bool = False
    reason: str = ""


def validate_decision(decision, remaining_ids):
    """回傳 None 表示可接受；否則回傳拒絕原因（會回饋給規劃器）。"""
    if decision.return_home:
        return None
    order = list(decision.order)
    dupes = sorted({w for w in order if order.count(w) > 1})
    if dupes:
        return f"duplicate waypoint id in order: {', '.join(dupes)}"
    unknown = [w for w in order if w not in remaining_ids]
    if unknown:
        return f"unknown or already-completed waypoint id: {', '.join(unknown)}"
    missing = [w for w in remaining_ids if w not in order]
    if missing:
        return f"order omits remaining waypoint(s): {', '.join(missing)} (completion is decided by the vehicle, not the planner)"
    return None


class RuleBasedPlanner:
    """確定性任務規劃器：照任務定義的順序飛完剩餘測量點。"""
    name = "rule-based"

    def decide(self, req):
        return MissionDecision([wid for wid, _ in req.remaining], False, "mission order")
