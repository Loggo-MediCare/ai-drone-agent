"""統一約束包絡（Unified Constraint Envelope）：五個領域的保守融合。

與 Listing 16.7 的差別：書中由 LLM 讀完各領域資料後輸出 JSON 宣告是否全綠；
這裡改由確定性程式碼計算，任何一個領域非 GREEN（包含 STALE）就否決整個包絡。
LLM 只能提出航線，不能宣告「可以飛」。
"""
from dataclasses import dataclass, field

from .energy import PROJECTION_MARGIN, estimated_power_w, soc_drain, soc_for_distance
from .geometry import dist, path_intersects
from .trajectory import CLIMB_MS, DESCENT_MS

GREEN, RED, STALE = "GREEN", "RED", "STALE"
DOMAINS = ("WEATHER", "AIRSPACE", "BATTERY", "PARKS", "MISSION")
ROUTE_DOMAINS = {"AIRSPACE", "PARKS", "MISSION"}   # 可透過改道解決的領域

# 書中的作業限制
MIN_TEMP_C = -10.0
MAX_WIND_KMH = 25.0
SOC_DEPARTURE = 0.30
SOC_WAYPOINT = 0.20
SOC_RESERVE = 0.15
MIN_ALT_M, MAX_ALT_M = 10.0, 120.0

# 資料過期門檻（秒）：書中說天氣可容忍 15 分鐘，NOTAM 要短得多
STALENESS_S = {"WEATHER": 900, "AIRSPACE": 60, "PARKS": 3600}


@dataclass
class DomainStatus:
    status: str
    reason: str = ""


@dataclass
class ConstraintEnvelope:
    t: float
    domains: dict
    blocked_areas: list = field(default_factory=list)
    projected_home_soc: float = None

    @property
    def green(self):
        return all(d.status == GREEN for d in self.domains.values())

    @property
    def failed(self):
        return {k: v for k, v in self.domains.items() if v.status != GREEN}

    def summary(self):
        return ", ".join(f"{k}={v.status}" + (f"({v.reason})" if v.reason else "")
                         for k, v in self.domains.items())


def _stale(reading, now):
    limit = STALENESS_S.get(reading.domain)
    return limit is not None and now - reading.timestamp > limit


def active_notams(airspace_reading, now):
    return [n for n in airspace_reading.data["notams"] if n.valid_from <= now <= n.valid_until]


def blocked_areas(airspace_reading, parks_reading, now):
    """規劃器必須避開的區域：生效中的 NOTAM＋未授權的綠帶。"""
    areas = [n.area for n in active_notams(airspace_reading, now)]
    if not parks_reading.data["overflight_authorized"]:
        areas.append(parks_reading.data["greenbelt"])
    return areas


def evaluate_envelope(now, weather, airspace, parks, soc, path, altitude_m, airspeed_kmh,
                      geofence, preflight):
    """path：從目前位置開始、到返航點結束的平面點序列。"""
    d = {}

    # WEATHER
    w = weather.data
    if _stale(weather, now):
        d["WEATHER"] = DomainStatus(STALE, f"data age {now - weather.timestamp:.0f}s")
    else:
        bad = []
        if not w["temperature_c"] > MIN_TEMP_C:
            bad.append(f"temp {w['temperature_c']}°C ≤ {MIN_TEMP_C}")
        if not w["wind_speed_kmh"] < MAX_WIND_KMH:
            bad.append(f"wind {w['wind_speed_kmh']} km/h ≥ {MAX_WIND_KMH}")
        if w["precipitation_active"]:
            bad.append("precipitation")
        d["WEATHER"] = DomainStatus(RED, "; ".join(bad)) if bad else DomainStatus(GREEN)

    # AIRSPACE
    if _stale(airspace, now):
        d["AIRSPACE"] = DomainStatus(STALE, f"data age {now - airspace.timestamp:.0f}s")
    else:
        hits = [n.alert_id for n in active_notams(airspace, now) if path_intersects(path, n.area)]
        d["AIRSPACE"] = DomainStatus(RED, "route crosses " + ", ".join(hits)) if hits else DomainStatus(GREEN)

    # PARKS
    p = parks.data
    if _stale(parks, now):
        d["PARKS"] = DomainStatus(STALE, f"data age {now - parks.timestamp:.0f}s")
    elif path_intersects(path, p["greenbelt"]) and not p["overflight_authorized"]:
        d["PARKS"] = DomainStatus(RED, "greenbelt overflight not authorized")
    else:
        d["PARKS"] = DomainStatus(GREEN)

    # BATTERY：逐點預估電量（含起飛爬升與降落的懸停時間）
    bad = []
    if preflight and soc < SOC_DEPARTURE:
        bad.append(f"departure SoC {soc:.0%} < {SOC_DEPARTURE:.0%}")
    hover_s = altitude_m / DESCENT_MS + (altitude_m / CLIMB_MS if preflight else 0.0)
    projected = soc - soc_drain(estimated_power_w(0.0, w["wind_speed_kmh"]), hover_s,
                                w["temperature_c"]) * PROJECTION_MARGIN
    for i, (a, b) in enumerate(zip(path, path[1:])):
        projected -= soc_for_distance(dist(a, b), airspeed_kmh, w["wind_speed_kmh"], w["temperature_c"])
        is_home = i == len(path) - 2
        floor = SOC_RESERVE if is_home else SOC_WAYPOINT
        if projected < floor:
            where = "home" if is_home else f"waypoint {i + 1}"
            bad.append(f"projected {projected:.0%} at {where} < {floor:.0%}")
            break
    d["BATTERY"] = DomainStatus(RED, "; ".join(bad)) if bad else DomainStatus(GREEN)

    # MISSION geometry
    bad = []
    if len(path) < 2:
        bad.append("empty route")
    if any(not geofence.contains(pt) for pt in path):
        bad.append("route leaves geofence")
    if not MIN_ALT_M <= altitude_m <= MAX_ALT_M:
        bad.append(f"altitude {altitude_m} m outside {MIN_ALT_M}-{MAX_ALT_M} m")
    d["MISSION"] = DomainStatus(RED, "; ".join(bad)) if bad else DomainStatus(GREEN)

    return ConstraintEnvelope(
        t=now,
        domains=d,
        blocked_areas=blocked_areas(airspace, parks, now),
        projected_home_soc=projected,
    )
