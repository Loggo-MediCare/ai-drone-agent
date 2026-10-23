"""軌跡控制層（50–200 Hz）：把運動規劃層的路徑按時間參數化成設定點，
再用 PID 計算所需推力向量，交給伺服層。
"""
import math

from .geometry import dist
from .physics import C_DRAG, G, MASS_KG

KP_XY, KD_XY, KI_XY = 0.8, 1.6, 0.05
KP_Z, KD_Z, KI_Z = 2.0, 2.5, 0.3
MAX_TILT = math.radians(30)
CLIMB_MS, DESCENT_MS = 3.0, 2.0
LEAD_LIMIT_M = 15.0          # 追蹤誤差超過時暫停推進設定點
INTEG_LIMIT = 20.0


class TrajectoryController:
    def __init__(self, hz):
        self.dt = 1.0 / hz
        self.path = []
        self.seg_len = []
        self.s = 0.0
        self.speed = 0.0
        self.alt_target = 0.0
        self.z_sp = 0.0
        self.hold = False
        self.integ = [0.0, 0.0, 0.0]
        self.p_sp = (0.0, 0.0)
        self.max_err_m = 0.0

    def set_path(self, path, altitude, speed_ms):
        self.path = list(path)
        self.seg_len = [dist(a, b) for a, b in zip(path, path[1:])]
        self.s = 0.0
        self.speed = speed_ms
        self.alt_target = altitude
        self.p_sp = path[0]

    @property
    def length(self):
        return sum(self.seg_len)

    def _point(self, s):
        for (a, b), L in zip(zip(self.path, self.path[1:]), self.seg_len):
            if s <= L and L > 0:
                f = s / L
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])), ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
            s -= L
        return self.path[-1], (0.0, 0.0)

    def update(self, body):
        """回傳 (總推力需求 N, 推力方向單位向量)。"""
        # 高度設定點：以爬升/下降率逼近目標
        dz = self.alt_target - self.z_sp
        rate = CLIMB_MS if dz > 0 else DESCENT_MS
        self.z_sp += max(-rate * self.dt, min(rate * self.dt, dz))

        # 水平設定點：沿路徑推進（爬到巡航高度附近、未懸停、追蹤誤差小時才推進）
        p_now, tangent = self._point(self.s)
        lead = dist(p_now, body.xy)
        cruising = abs(body.p[2] - self.alt_target) < 5.0 and self.alt_target > 0
        advancing = cruising and not self.hold and lead < LEAD_LIMIT_M and self.s < self.length
        if advancing:
            self.s = min(self.length, self.s + self.speed * self.dt)
        self.p_sp, tangent = self._point(self.s)
        v_sp = (tangent[0] * self.speed, tangent[1] * self.speed) if advancing else (0.0, 0.0)

        # PID
        sp = (self.p_sp[0], self.p_sp[1], self.z_sp)
        vs = (v_sp[0], v_sp[1], 0.0)
        a = [0.0, 0.0, 0.0]
        for k in range(3):
            kp, kd, ki = (KP_XY, KD_XY, KI_XY) if k < 2 else (KP_Z, KD_Z, KI_Z)
            e = sp[k] - body.p[k]
            self.integ[k] = max(-INTEG_LIMIT, min(INTEG_LIMIT, self.integ[k] + e * self.dt))
            a[k] = kp * e + kd * (vs[k] - body.v[k]) + ki * self.integ[k]
        self.max_err_m = max(self.max_err_m, dist(self.p_sp, body.xy))

        # 期望力 = m(a + g) + 阻力前饋；限制傾角
        f = [MASS_KG * a[0] + C_DRAG * vs[0], MASS_KG * a[1] + C_DRAG * vs[1], MASS_KG * (a[2] + G)]
        f[2] = max(f[2], 0.1)
        horiz = math.hypot(f[0], f[1])
        max_h = f[2] * math.tan(MAX_TILT)
        if horiz > max_h:
            f[0] *= max_h / horiz
            f[1] *= max_h / horiz
        mag = math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)
        return mag, (f[0] / mag, f[1] / mag, f[2] / mag)

    def remaining_path(self):
        """目前設定點之後尚未飛過的路徑（給運動層檢查是否仍無碰撞）。"""
        out, s = [self.p_sp], self.s
        acc = 0.0
        for pt, L in zip(self.path[1:], self.seg_len):
            acc += L
            if acc > s:
                out.append(pt)
        return out

    def arrived(self, body):
        return (self.s >= self.length and dist(self.path[-1], body.xy) < 3.0
                and math.hypot(body.v[0], body.v[1]) < 1.0)
