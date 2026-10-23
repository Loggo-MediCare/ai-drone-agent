"""安全執行層 A_safe(s)。

每一個航點命令在送到自動駕駛儀之前都必須經過 validate()；這是程式碼強制的，
不是提示詞中的「協定」。規劃器（包含 LLM）無法繞過。
"""
from dataclasses import dataclass

from .constraints import MAX_ALT_M, MIN_ALT_M
from .geometry import Rect, segment_intersects_rect

MAX_AIRSPEED_KMH = 50.0
ENVELOPE_MAX_AGE_S = 35.0     # 約束包絡最多可以多舊（每 30 秒更新一次）
GUST_HOLD_KMH = 25.0          # 機載量測陣風超過此值 → 懸停
GUST_CALM_S = 5.0             # 陣風低於門檻持續多久才恢復
SOC_HARD_FLOOR = 0.12         # 低於此電量立即返航，不等下一次包絡更新


@dataclass(frozen=True)
class WaypointCommand:
    x: float
    y: float
    altitude_m: float
    airspeed_kmh: float
    label: str = ""             # 只用於紀錄顯示，不影響任何判定
    survey_id: str = None       # 由 RoutePlanner 依任務定義設定；抵達後仍須通過位置驗證

    @property
    def xy(self):
        return (self.x, self.y)


@dataclass
class Validation:
    is_safe: bool
    reason: str = ""


class SafetyMonitor:
    def __init__(self, geofence, clearance_m=100.0):
        self.geofence = geofence
        self.clearance_m = clearance_m

    def _geometry(self, cmd, from_pos, blocked):
        """檢查整段直線（不只端點）：不得進入禁區，也不得進入禁區外擴的安全距離。"""
        if not self.geofence.contains(cmd.xy):
            return Validation(False, f"{cmd.label} ({cmd.x:.0f}, {cmd.y:.0f}) outside geofence")
        for area in blocked:
            if segment_intersects_rect(from_pos, cmd.xy, area):
                return Validation(False, f"leg ({from_pos[0]:.0f},{from_pos[1]:.0f})→({cmd.x:.0f},{cmd.y:.0f}) "
                                         f"to {cmd.label} enters restricted area")
            c = self.clearance_m
            padded = Rect(area.xmin - c, area.ymin - c, area.xmax + c, area.ymax + c)
            # 起點已在安全距離內時（例如剛生效的 NOTAM），只要求不進入禁區本身，允許離開
            if not padded.contains(from_pos) and segment_intersects_rect(from_pos, cmd.xy, padded):
                return Validation(False, f"leg to {cmd.label} passes within {c:.0f} m of restricted area")
        return Validation(True)

    def validate(self, cmd, from_pos, envelope, now):
        if envelope is None or not envelope.green:
            failed = ", ".join(envelope.failed) if envelope else "none"
            return Validation(False, f"constraint envelope not green ({failed})")
        if now - envelope.t > ENVELOPE_MAX_AGE_S:
            return Validation(False, f"constraint envelope too old ({now - envelope.t:.0f}s)")
        if not MIN_ALT_M <= cmd.altitude_m <= MAX_ALT_M:
            return Validation(False, f"{cmd.label} altitude {cmd.altitude_m} m out of range")
        if not 0 < cmd.airspeed_kmh <= MAX_AIRSPEED_KMH:
            return Validation(False, f"{cmd.label} airspeed {cmd.airspeed_kmh} km/h > {MAX_AIRSPEED_KMH}")
        return self._geometry(cmd, from_pos, envelope.blocked_areas)

    def validate_plan(self, cmds, start, envelope, now):
        pos = start
        for cmd in cmds:
            v = self.validate(cmd, pos, envelope, now)
            if not v.is_safe:
                return v
            pos = cmd.xy
        return Validation(True)

    def validate_contingency(self, cmd, from_pos, blocked):
        """返航等應變動作：包絡已非全綠，仍檢查地理圍欄、禁航區與安全距離（不假設返航必然安全）。"""
        return self._geometry(cmd, from_pos, blocked)

    def validate_contingency_route(self, cmds, start, blocked):
        pos = start
        for cmd in cmds:
            v = self.validate_contingency(cmd, pos, blocked)
            if not v.is_safe:
                return v
            pos = cmd.xy
        return Validation(True)
