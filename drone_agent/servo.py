"""伺服控制層（1–10 kHz）：調節四顆馬達的電流。

每顆馬達是一個 RL 電路加反電動勢：L·di/dt = u − R·i − e_back
PI 電流控制器加前饋（R·i_ref + e_back），抑制匯流排電壓漣波等高頻電氣干擾。
推力 ≈ KT·i（準靜態近似）。參數為示意值，非特定馬達規格。
"""
import math
import random

R_OHM = 0.1           # 線圈電阻
L_H = 0.002           # 線圈電感（電氣時間常數 20 ms）
KT_N_PER_A = 0.6      # 每安培推力
KF = 1e-5             # 推力係數 T = KF·ω²
KE = 0.02             # 反電動勢常數 V·s/rad
V_BUS = 22.2          # 6S 電池
I_MAX = 15.0          # 電流上限（推力上限 9 N/顆）
RIPPLE_V = 0.3        # 匯流排電壓漣波（干擾）振幅


class MotorCurrentLoop:
    def __init__(self, hz, bandwidth_hz=100.0, seed=0):
        self.dt = 1.0 / hz
        wc = 2 * math.pi * bandwidth_hz
        self.kp = L_H * wc
        self.ki = R_OHM * wc
        self.alpha = math.exp(-self.dt * R_OHM / L_H)   # 精確離散化，大步長也穩定
        self.i = 0.0
        self.integ = 0.0
        self.rng = random.Random(seed)
        self.err_sq_sum = 0.0
        self.ticks = 0

    def step(self, i_ref):
        """回傳 (推力 N, 電功率 W)。"""
        i_ref = max(0.0, min(I_MAX, i_ref))
        err = i_ref - self.i
        e_back = KE * math.sqrt(max(KT_N_PER_A * self.i, 0.0) / KF)
        u = self.kp * err + self.ki * self.integ + R_OHM * i_ref + e_back
        if 0.0 < u < V_BUS:
            self.integ += err * self.dt         # 飽和時停止積分（anti-windup）
        u = max(0.0, min(V_BUS, u))
        disturbance = self.rng.uniform(-RIPPLE_V, RIPPLE_V)
        self.i = self.i * self.alpha + (1 - self.alpha) * (u + disturbance - e_back) / R_OHM
        self.i = max(0.0, self.i)
        self.err_sq_sum += err * err
        self.ticks += 1
        return KT_N_PER_A * self.i, u * self.i

    @property
    def rms_error_a(self):
        return math.sqrt(self.err_sq_sum / self.ticks) if self.ticks else 0.0


class ServoLayer:
    """四顆馬達；本模擬中姿態迴路視為理想，四顆分擔相同推力。"""

    def __init__(self, hz):
        self.hz = hz
        self.motors = [MotorCurrentLoop(hz, seed=k) for k in range(4)]

    def step(self, thrust_total_ref):
        i_ref = thrust_total_ref / 4 / KT_N_PER_A
        thrust, power = 0.0, 0.0
        for m in self.motors:
            t, p = m.step(i_ref)
            thrust += t
            power += p
        return thrust, power

    @property
    def rms_error_a(self):
        return max(m.rms_error_a for m in self.motors)
