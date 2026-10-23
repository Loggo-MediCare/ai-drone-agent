"""平面幾何：任務座標以起降點為原點，單位公尺（x 向東、y 向北）。"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Rect:
    """軸對齊矩形區域（禁航區、綠帶、地理圍欄）。"""
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, p):
        return self.xmin <= p[0] <= self.xmax and self.ymin <= p[1] <= self.ymax


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(points):
    return sum(dist(a, b) for a, b in zip(points, points[1:]))


def segment_intersects_rect(p, q, r):
    """Liang–Barsky 線段裁切：線段 p→q 是否穿過矩形 r。"""
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for pk, qk in ((-dx, p[0] - r.xmin), (dx, r.xmax - p[0]),
                   (-dy, p[1] - r.ymin), (dy, r.ymax - p[1])):
        if pk == 0:
            if qk < 0:
                return False
        else:
            t = qk / pk
            if pk < 0:
                t0 = max(t0, t)
            else:
                t1 = min(t1, t)
            if t0 > t1:
                return False
    return True


def path_intersects(points, rect):
    return any(segment_intersects_rect(a, b, rect) for a, b in zip(points, points[1:]))
