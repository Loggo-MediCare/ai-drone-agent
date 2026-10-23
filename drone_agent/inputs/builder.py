"""統一任務輸入：讀 data/mission_input.json → 各轉接器 → 起飛前資料閘門 → Environment / Mission。

兩種模式（由 JSON 的 "simulation" 決定）：
- 模擬模式：允許 SIMULATED 來源，用於練習與測試。
- 真實資料模式：每個關鍵來源都必須是真實來源、狀態 OK、在新鮮度內、且已核實；
  否則拒絕起飛。即使通過，飛行本身仍是模擬，不會驅動任何真實馬達。
"""
import json
from dataclasses import dataclass, field

from ..geometry import Rect
from ..planner import Mission
from ..world import Environment, Notam
from . import airspace, cwa_weather, telemetry
from .base import (BLOCKED, OK, SIMULATED, SourceResult, check_freshness, parse_time, utcnow)
from .geo import LocalFrame

CRITICAL = ("WEATHER", "AIRSPACE", "NOTAM", "TELEMETRY", "GIS")

# 真實資料模式的新鮮度上限（秒）。CWA 測站每 10 分鐘觀測、發布另有延遲，所以給 30 分鐘。
MAX_AGE_S = {
    "WEATHER": 1800,
    "AIRSPACE": 7 * 24 * 3600,   # 禁限航區變動較少，但至少每週重新查詢核實
    "NOTAM": 3600,               # NOTAM 可能隨時發布，最多 1 小時前查詢
    "TELEMETRY": 5,
}
GEOFENCE_MARGIN_M = 1500.0


@dataclass
class MissionInputs:
    simulation: bool
    frame: LocalFrame
    sources: list
    mission: Mission = None
    environment: Environment = None
    soc: float = None
    issues: list = field(default_factory=list)

    def table(self):
        rows = []
        for s in self.sources:
            mark = "已核實" if s.verified else "未核實"
            rows.append(f"  {s.domain:<9} {s.source:<12} {s.status:<10} {mark}  {s.detail}")
        return "\n".join(rows)


# ---------- 各領域：模擬或真實 ----------
def _weather(cfg, lat, lon, now):
    if cfg.get("source", "SIMULATED") == "SIMULATED":
        wind_kmh = cfg["wind_speed_mps"] * 3.6 if "wind_speed_mps" in cfg else cfg.get("wind_speed_kmh", 0.0)
        return SourceResult("WEATHER", "SIMULATED", SIMULATED, {
            "wind_speed_kmh": wind_kmh,
            "temperature_c": cfg.get("temperature_c", 25.0),
            "precipitation_active": bool(cfg.get("rain", False)),
        }, now, False, "simulated weather")
    if cfg["source"] == "CWA":
        return cwa_weather.fetch(lat, lon, cfg.get("station_id"))
    if cfg["source"] == "CWA_FILE":   # 離線測試：讀存下來的 CWA 回應
        with open(cfg["file"], encoding="utf-8") as f:
            return cwa_weather.parse_station_payload(json.load(f), lat, lon, cfg.get("station_id"))
    return SourceResult("WEATHER", cfg["source"], "ERROR", detail="unknown weather source")


def _airspace(cfg, now):
    if cfg.get("source", "SIMULATED") == "SIMULATED":
        r = SourceResult("AIRSPACE", "SIMULATED", SIMULATED, {"zones": cfg.get("zones", [])},
                         now, bool(cfg.get("verified", False)), "simulated airspace")
        if cfg.get("restricted"):
            r.status, r.detail = BLOCKED, "airspace marked restricted"
        return r
    if cfg["source"] == "CAA_GEOJSON":
        return airspace.load_caa_geojson(cfg["file"])
    return SourceResult("AIRSPACE", cfg["source"], "ERROR", detail="unknown airspace source")


def _notam(cfg, now):
    if cfg.get("source", "SIMULATED") == "SIMULATED":
        r = SourceResult("NOTAM", "SIMULATED", SIMULATED, {"notams": cfg.get("notams", [])},
                         now, bool(cfg.get("verified", False)), "simulated NOTAM")
        if cfg.get("notam_clear") is False:
            r.status, r.detail = BLOCKED, "NOTAM not clear"
        return r
    if cfg["source"] == "AIS_FILE":
        return airspace.load_notam_file(cfg["file"])
    return SourceResult("NOTAM", cfg["source"], "ERROR", detail="unknown NOTAM source")


def _telemetry(cfg, now):
    if cfg.get("source", "SIMULATED") == "SIMULATED":
        return SourceResult("TELEMETRY", "SIMULATED", SIMULATED,
                            {"battery_soc": cfg["remaining_percent"] / 100.0}, now, False, "simulated battery")
    if cfg["source"] == "MAVSDK":
        return telemetry.fetch(cfg.get("address", "udp://:14540"))
    return SourceResult("TELEMETRY", cfg["source"], "ERROR", detail="unknown telemetry source")


def _gis(cfg, now):
    source = cfg.get("source", "SIMULATED")
    status = SIMULATED if source == "SIMULATED" else OK
    observed = parse_time(cfg.get("surveyed_at")) if source != "SIMULATED" else now
    return SourceResult("GIS", source, status, {
        "obstacles": cfg.get("obstacles", []),
        "protected_areas": cfg.get("protected_areas", cfg.get("green_corridor", [])),
    }, observed, bool(cfg.get("verified", False)), cfg.get("note", "") or f"{source.lower()} route data")


# ---------- 起飛前資料閘門 ----------
def preflight_data_gate(inputs, now):
    issues = []
    by_domain = {s.domain: s for s in inputs.sources}
    for domain in CRITICAL:
        s = by_domain.get(domain)
        if s is None:
            issues.append(f"{domain}: no source configured")
            continue
        if s.status == BLOCKED:
            issues.append(f"{domain}: {s.detail}")
            continue
        if s.status == OK and domain in MAX_AGE_S:   # 真實來源在兩種模式下都檢查新鮮度
            check_freshness(s, now, MAX_AGE_S[domain])
        if inputs.simulation:
            if s.status not in (OK, SIMULATED):
                issues.append(f"{domain}: {s.status} {s.detail}".rstrip())
            continue
        # 真實資料模式：不接受模擬來源、要求新鮮且已核實
        if s.status == SIMULATED:
            issues.append(f"{domain}: simulated source not allowed in real-data mode")
            continue
        if s.status != OK:
            issues.append(f"{domain}: {s.status} {s.detail}".rstrip())
        elif not s.verified:
            what = "on site" if domain == "GIS" else "against official publication"
            issues.append(f"{domain}: not verified {what}")
    return issues


# ---------- 組成模擬器輸入 ----------
def load_mission_input(path, now=None):
    now = now or utcnow()
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    m = cfg["mission"]
    lat0, lon0 = m["start"]
    frame = LocalFrame(lat0, lon0)
    simulation = bool(cfg.get("simulation", True))

    sources = [
        _weather(cfg.get("weather", {}), lat0, lon0, now),
        _airspace(cfg.get("airspace", {}), now),
        _notam(cfg.get("notam", cfg.get("airspace_notam", {})), now),
        _telemetry(cfg.get("telemetry", cfg.get("battery", {})), now),
        _gis(cfg.get("route", {}), now),
    ]
    inputs = MissionInputs(simulation, frame, sources)
    inputs.issues = preflight_data_gate(inputs, now)
    if inputs.issues:
        return inputs

    by = {s.domain: s for s in sources}
    # 任務：起點 + 航點（或單一 goal）
    points = m.get("waypoints") or [m["goal"]]
    survey = [(f"WP{i + 1}", frame.to_xy(lat, lon)) for i, (lat, lon) in enumerate(points)]
    xs = [0.0] + [p[0] for _, p in survey]
    ys = [0.0] + [p[1] for _, p in survey]
    geofence = Rect(min(xs) - GEOFENCE_MARGIN_M, min(ys) - GEOFENCE_MARGIN_M,
                    max(xs) + GEOFENCE_MARGIN_M, max(ys) + GEOFENCE_MARGIN_M)
    inputs.mission = Mission(name=m.get("name", "mission_input.json"), survey=survey,
                             altitude_m=m.get("altitude_m", 60.0),
                             airspeed_kmh=m.get("airspeed_kmh", 36.0), geofence=geofence)

    # 環境：禁限航區當作永久限制；NOTAM 生效時間換算成以起飛為 0 的模擬秒數
    w = by["WEATHER"].data
    notams = [Notam(f"CAA:{z['name']}", frame.area_rect(z["area"]), -1e9, 1e9, -1e9)
              for z in by["AIRSPACE"].data.get("zones", [])]
    for n in by["NOTAM"].data.get("notams", []):
        vf, vu = parse_time(n["valid_from"]), parse_time(n["valid_until"])
        notams.append(Notam(n["id"], frame.area_rect(n["area"]),
                            (vf - now).total_seconds(), (vu - now).total_seconds(), -1e9))
    g = by["GIS"].data
    inputs.environment = Environment(
        temperature_c=w["temperature_c"],
        wind_kmh=w["wind_speed_kmh"],
        precipitation=w["precipitation_active"],
        notams=notams,
        greenbelt=None,
        protected_areas=[(a.get("name", "protected"), frame.area_rect(a["area"]), bool(a.get("authorized")))
                         for a in g["protected_areas"]],
        obstacles=[frame.area_rect(o["area"]) for o in g["obstacles"]],
    )
    inputs.soc = by["TELEMETRY"].data["battery_soc"]
    return inputs
