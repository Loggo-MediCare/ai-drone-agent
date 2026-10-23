"""WGS84 經緯度 ↔ 任務平面座標（以起降點為原點的公尺座標，x 向東、y 向北）。

使用等距矩形近似，在數公里範圍內誤差遠小於規劃用的安全距離。
多邊形與圓形區域一律轉成「外接矩形」：會比實際區域大，屬於保守處理。
"""
import math

from ..geometry import Rect

M_PER_DEG_LAT = 110_540.0
M_PER_DEG_LON_EQUATOR = 111_320.0


class LocalFrame:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self._kx = M_PER_DEG_LON_EQUATOR * math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        return ((lon - self.lon0) * self._kx, (lat - self.lat0) * M_PER_DEG_LAT)

    def to_latlon(self, x, y):
        return (self.lat0 + y / M_PER_DEG_LAT, self.lon0 + x / self._kx)

    def polygon_rect(self, latlon_points):
        pts = [self.to_xy(lat, lon) for lat, lon in latlon_points]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return Rect(min(xs), min(ys), max(xs), max(ys))

    def circle_rect(self, lat, lon, radius_m):
        x, y = self.to_xy(lat, lon)
        return Rect(x - radius_m, y - radius_m, x + radius_m, y + radius_m)

    def area_rect(self, area):
        """area：{"type": "polygon", "coordinates": [[lat, lon], …]} 或
                 {"type": "circle", "center": [lat, lon], "radius_m": 500}"""
        if area.get("type") == "circle":
            lat, lon = area["center"]
            return self.circle_rect(lat, lon, float(area["radius_m"]))
        return self.polygon_rect(area["coordinates"])
