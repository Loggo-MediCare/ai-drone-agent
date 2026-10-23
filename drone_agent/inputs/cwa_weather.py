"""中央氣象署（CWA）開放資料：自動氣象站觀測 O-A0001-001。

需要先到 https://opendata.cwa.gov.tw 申請會員授權碼，放在 .env 的 CWA_API_KEY。
選擇距離任務起點最近的測站，取風速、氣溫、降雨與觀測時間。

欄位依 CWA O-A0001-001 資料格式；CWA 以 -99 表示缺值。
若 CWA 調整格式，parse_station_payload() 會回傳 ERROR，不會猜測數值。
"""
import json
import math
import os
import urllib.parse
import urllib.request

from .base import ERROR, MISSING, OK, SourceResult, parse_time

API_URL = "https://opendata.cwa.gov.tw/api/v1/rest/datastore/O-A0001-001"
MISSING_VALUE = -99


def _num(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return None if v <= MISSING_VALUE else v


def _station_latlon(station):
    for c in station.get("GeoInfo", {}).get("Coordinates", []):
        if c.get("CoordinateName") == "WGS84":
            return float(c["StationLatitude"]), float(c["StationLongitude"])
    return None


def parse_station_payload(payload, lat, lon, station_id=None):
    """從 CWA 回應中挑選測站並轉成 SourceResult。"""
    try:
        stations = payload["records"]["Station"]
    except (KeyError, TypeError):
        return SourceResult("WEATHER", "CWA", ERROR, detail="unexpected CWA payload format")
    if station_id:
        stations = [s for s in stations if s.get("StationId") == station_id]
    candidates = [(s, _station_latlon(s)) for s in stations]
    candidates = [(s, p) for s, p in candidates if p]
    if not candidates:
        return SourceResult("WEATHER", "CWA", MISSING, detail="no station found")
    station, (slat, slon) = min(
        candidates, key=lambda sp: math.hypot(sp[1][0] - lat, (sp[1][1] - lon) * math.cos(math.radians(lat))))

    we = station.get("WeatherElement", {})
    wind = _num(we.get("WindSpeed"))
    temp = _num(we.get("AirTemperature"))
    precip = _num(we.get("Now", {}).get("Precipitation"))
    gust = _num(we.get("GustInfo", {}).get("PeakGustSpeed"))
    missing = [n for n, v in (("WindSpeed", wind), ("AirTemperature", temp), ("Precipitation", precip)) if v is None]
    observed = parse_time(station.get("ObsTime", {}).get("DateTime"))
    distance_km = math.hypot(slat - lat, (slon - lon) * math.cos(math.radians(lat))) * 111.0
    data = {
        "station": f"{station.get('StationName', '')} ({station.get('StationId', '')})",
        "station_distance_km": round(distance_km, 1),
        "wind_speed_kmh": wind * 3.6 if wind is not None else None,
        "peak_gust_kmh": gust * 3.6 if gust is not None else None,
        "temperature_c": temp,
        "precipitation_active": precip is not None and precip > 0,
    }
    if missing:
        return SourceResult("WEATHER", "CWA", ERROR, data, observed, True,
                            f"missing values: {', '.join(missing)}")
    # 官方觀測資料：視為已核實；新鮮度由 builder 檢查
    return SourceResult("WEATHER", "CWA", OK, data, observed, True, data["station"])


def fetch(lat, lon, station_id=None, api_key=None, timeout_s=10):
    api_key = api_key or os.getenv("CWA_API_KEY")
    if not api_key:
        return SourceResult("WEATHER", "CWA", MISSING, detail="CWA_API_KEY not set")
    query = {"Authorization": api_key, "format": "JSON"}
    if station_id:
        query["StationId"] = station_id
    url = f"{API_URL}?{urllib.parse.urlencode(query)}"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as r:
            payload = json.loads(r.read().decode("utf-8"))
    except Exception as e:  # 網路、授權碼錯誤、逾時
        return SourceResult("WEATHER", "CWA", ERROR, detail=f"request failed: {type(e).__name__}")
    return parse_station_payload(payload, lat, lon, station_id)
