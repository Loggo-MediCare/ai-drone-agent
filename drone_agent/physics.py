"""機體動力學：質點＋線性空氣阻力，與伺服層同頻率積分。"""
from dataclasses import dataclass, field

MASS_KG = 1.5
G = 9.81
C_DRAG = 0.3          # N/(m/s)，相對空速的線性阻力


@dataclass
class Body:
    p: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # x, y, z（z = 離地高度）
    v: list = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def step(self, dt, thrust, direction, wind):
        """thrust：伺服層實際產生的總推力；direction：軌跡層要求的推力方向（單位向量）。"""
        a = [0.0, 0.0, 0.0]
        for k in range(3):
            w = wind[k] if k < 2 else 0.0
            a[k] = (thrust * direction[k] + C_DRAG * (w - self.v[k])) / MASS_KG
        a[2] -= G
        for k in range(3):
            self.v[k] += a[k] * dt
            self.p[k] += self.v[k] * dt
        if self.p[2] <= 0.0:          # 地面
            self.p[2] = 0.0
            self.v = [0.0, 0.0, max(0.0, self.v[2])]

    @property
    def xy(self):
        return (self.p[0], self.p[1])
