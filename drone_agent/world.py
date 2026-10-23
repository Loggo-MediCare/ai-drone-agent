"""模擬環境資料來源：天氣、NOTAM、綠帶、實體障礙物。

真實系統中：天氣來自氣象 API、NOTAM 來自民航主管機關、無人機狀態來自
MAVLink 遙測。每筆讀數都帶時間戳，約束組裝器據此判斷資料是否過期（STALE）。
"""
import math
from dataclasses import dataclass, field

from .geometry import Rect


@dataclass
class Reading:
    domain: str
    data: dict
    timestamp: float


@dataclass
class Notam:
    alert_id: str
    area: Rect
    valid_from: float      # 模擬時間（秒）
    valid_until: float
    published_at: float    # 推播送達時間


@dataclass
class Gust:
    start: float
    end: float
    peak_kmh: float


@dataclass
class Environment:
    temperature_c: float = -6.2           # 書中渥太華一月案例
    wind_kmh: float = 18.5                # 持續風速（氣象 API 回報）
    wind_toward_deg: float = -45.0        # 西北風 → 吹向東南
    precipitation: bool = False
    gusts: list = field(default_factory=list)
    notams: list = field(default_factory=list)
    greenbelt: Rect = Rect(3500, -1500, 4200, 1500)
    greenbelt_authorized: bool = True
    greenbelt_auth_ref: str = "NCC-UAV-2025-0017"
    obstacles: list = field(default_factory=lambda: [Rect(4950, -60, 5050, 60)])  # 通訊塔
    weather_frozen_at: float = None       # 設定後天氣資料停在這個時間戳（模擬 API 斷線）

    def read_weather(self, t):
        ts = t if self.weather_frozen_at is None else self.weather_frozen_at
        return Reading("WEATHER", {
            "temperature_c": self.temperature_c,
            "wind_speed_kmh": self.wind_kmh,
            "precipitation_active": self.precipitation,
        }, ts)

    def read_notams(self, t):
        published = [n for n in self.notams if n.published_at <= t]
        return Reading("AIRSPACE", {"notams": published}, t)

    def read_parks(self, t):
        return Reading("PARKS", {
            "greenbelt": self.greenbelt,
            "overflight_authorized": self.greenbelt_authorized,
            "authorization_reference": self.greenbelt_auth_ref if self.greenbelt_authorized else None,
        }, t)

    def notam_pushed(self, t_prev, t):
        """這段時間內是否收到新的 NOTAM 推播。"""
        return any(t_prev < n.published_at <= t for n in self.notams)

    def wind_speed_kmh(self, t):
        """機載量測的瞬間風速（含陣風）。"""
        for g in self.gusts:
            if g.start <= t <= g.end:
                return g.peak_kmh
        return self.wind_kmh

    def wind_vector(self, t):
        v = self.wind_speed_kmh(t) / 3.6
        a = math.radians(self.wind_toward_deg)
        return (v * math.cos(a), v * math.sin(a))
