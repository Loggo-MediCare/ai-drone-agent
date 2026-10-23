"""任務主管（Listing 16.6 擴充）：把四層控制串成一個多速率迴圈。

    任務規劃層   0.1–1 Hz   約束包絡每 30 s 更新，或收到 NOTAM 推播立即更新；改道／返航
    運動規劃層   2 Hz       RRT* 無碰撞路徑；檢查路徑是否仍有效，失效就重算
    軌跡控制層   100 Hz     路徑時間參數化＋PID → 推力向量；陣風懸停、電量硬下限
    伺服控制層   1 kHz      四顆馬達電流 PI 迴路 → 實際推力與電功率

修正 Listing 16.3 的問題：每個動作只執行一次；被拒絕後使用新計畫重新開始驗證；
航點在送到自動駕駛儀前由程式碼呼叫 SafetyMonitor，而不是依賴提示詞。
"""
from dataclasses import dataclass, field

from .constraints import (ROUTE_DOMAINS, SOC_RESERVE, blocked_areas, active_notams,
                          evaluate_envelope)
from .energy import soc_drain
from .geometry import dist, segment_intersects_rect
from .knowledge_graph import KnowledgeGraph, propagate_influence
from .motion import MotionPlanner
from .physics import Body
from .planner import PlanRequest
from .safety import (ENVELOPE_MAX_AGE_S, GUST_CALM_S, GUST_HOLD_KMH, SOC_HARD_FLOOR,
                     SafetyMonitor, WaypointCommand)
from .servo import ServoLayer
from .trajectory import TrajectoryController

SERVO_HZ = 1000
TRAJ_HZ = 100
MOTION_HZ = 2
ENVELOPE_REFRESH_S = 30.0


@dataclass
class MissionReport:
    status: str
    reason: str = ""
    t_end: float = 0.0
    soc_start: float = 0.0
    soc_end: float = 0.0
    distance_m: float = 0.0
    holds: int = 0
    reroutes: int = 0
    surveyed: list = field(default_factory=list)
    rrt_runs: int = 0
    layer_ticks: dict = field(default_factory=dict)
    max_track_err_m: float = 0.0
    servo_rms_a: float = 0.0
    projected_home_soc: float = None
    log: list = field(default_factory=list)

    def print(self, show_log=True):
        if show_log:
            for line in self.log:
                print("   " + line)
        print(f"   結果：{self.status}" + (f"（{self.reason}）" if self.reason else ""))
        if self.t_end:
            print(f"   飛行 {self.t_end:.0f} s、距離 {self.distance_m / 1000:.2f} km、"
                  f"電量 {self.soc_start:.0%} → {self.soc_end:.0%}、測量點 {','.join(self.surveyed) or '無'}")
            print(f"   懸停 {self.holds} 次、改道 {self.reroutes} 次、RRT* {self.rrt_runs} 次、"
                  f"最大追蹤誤差 {self.max_track_err_m:.1f} m、電流追蹤 RMS {self.servo_rms_a:.3f} A")
            print("   各層執行次數：" + "、".join(f"{k} {v}" for k, v in self.layer_ticks.items()))


class MissionSupervisor:
    def __init__(self, env, mission, planner, soc=0.82,
                 servo_hz=SERVO_HZ, traj_hz=TRAJ_HZ, motion_hz=MOTION_HZ):
        self.env = env
        self.m = mission
        self.planner = planner
        self.soc = soc
        self.soc_start = soc
        self.servo_hz, self.traj_hz, self.motion_hz = servo_hz, traj_hz, motion_hz
        self.safety = SafetyMonitor(mission.geofence)
        self.motion = MotionPlanner(mission.geofence)
        self.servo = ServoLayer(servo_hz)
        self.traj = TrajectoryController(traj_hz)
        self.body = Body()
        self.log = []
        self.phase = "PREFLIGHT"
        self.route, self.idx = [], 0
        self.envelope = None
        self.remaining_survey = list(mission.survey)
        self.surveyed = []
        self.holds = self.reroutes = 0
        self.aborted = False
        self.exempt = []
        self.distance = 0.0
        self.ticks = {"任務層": 0, "運動層": 0, "軌跡層": 0, "伺服層": 0}

    # ---------- 任務規劃層 ----------
    def _log(self, t, msg):
        self.log.append(f"[{t:7.1f}s] {msg}")

    def _envelope(self, t, cmds, preflight=False):
        self.ticks["任務層"] += 1
        path = [self.body.xy] + [c.xy for c in cmds]
        e = self.env
        return evaluate_envelope(t, e.read_weather(t), e.read_notams(t), e.read_parks(t), self.soc,
                                 path, self.m.altitude_m, self.m.airspeed_kmh, self.m.geofence, preflight)

    def _blocked(self, t):
        areas = blocked_areas(self.env.read_notams(t), self.env.read_parks(t), t)
        return [a for a in areas if a not in self.exempt]

    def _plan(self, t, survey, feedback, preflight=False):
        """規劃器提案 → 約束包絡 → 安全層；被拒就把原因回饋給規劃器，最多 3 次。"""
        envelope = None
        for attempt in range(1, 4):
            cmds = self.planner.plan(PlanRequest(self.body.xy, self.m, survey, self._blocked(t), feedback))
            if not cmds:
                return None, envelope, f"{self.planner.name} 規劃器找不到可行航線"
            envelope = self._envelope(t, cmds, preflight)
            if not envelope.green:
                feedback = envelope.summary()
                self._log(t, f"規劃第 {attempt} 次被約束包絡否決：{feedback}")
                if set(envelope.failed) <= ROUTE_DOMAINS:
                    continue
                return None, envelope, feedback
            v = self.safety.validate_plan(cmds, self.body.xy, envelope, t)
            if not v.is_safe:
                feedback = v.reason
                self._log(t, f"規劃第 {attempt} 次被安全層拒絕：{feedback}")
                continue
            return cmds, envelope, ""
        return None, envelope, feedback

    def _cascade(self, t):
        """Listing 16.5：NOTAM 對任務的影響傳播。"""
        kg = KnowledgeGraph()
        path = [self.body.xy] + [c.xy for c in self.route[self.idx:]]
        labels = [c.label for c in self.route[self.idx:]]
        kg.add_edge("Battery", "FlightDuration", "constrains", self.soc - SOC_RESERVE, t=t)
        kg.add_edge("FlightDuration", "MissionEnvelope", "part_of", 0.8, t=t)
        for n in active_notams(self.env.read_notams(t), t):
            for (a, b), label in zip(zip(path, path[1:]), labels):
                if segment_intersects_rect(a, b, n.area):
                    kg.add_edge(n.alert_id, f"Leg→{label}", "restricts", 1.0, t=t)
                    kg.add_edge(f"Leg→{label}", "MissionEnvelope", "part_of", 0.9, t=t)
            impacts = propagate_influence(kg, n.alert_id)
            for node, (strength, p) in impacts.items():
                if node != n.alert_id:
                    self._log(t, f"  影響傳播 {' → '.join(p)}：{strength:.2f}")

    def _handle_failure(self, t, envelope, reason):
        failed = set(envelope.failed) if envelope else set()
        self.traj.hold = True
        if failed and failed <= ROUTE_DOMAINS:
            if "AIRSPACE" in failed:
                self._cascade(t)
            cmds, env2, why = self._plan(t, self.remaining_survey, reason)
            if cmds:
                self.route, self.idx, self.envelope = cmds, 0, env2
                self.reroutes += 1
                self.traj.hold = False
                self._log(t, "改道成功：" + " → ".join(c.label for c in cmds))
                self._dispatch(t)
                return
            reason = why
        self._rth(t, reason)

    def _rth(self, t, reason):
        self.aborted = True
        self.phase = "RTH"
        self.traj.hold = False
        self._log(t, f"返航（RTH）：{reason}")
        # 禁航區在無人機已經身處其中時才生效：唯一合理的動作是離開，該區域對返航豁免
        self.exempt = [a for a in blocked_areas(self.env.read_notams(t), self.env.read_parks(t), t)
                       if a.contains(self.body.xy)]
        if self.exempt:
            self._log(t, "目前位置在禁航區內，返航時以直線離開該區域")
        cmds = self.planner.plan(PlanRequest(self.body.xy, self.m, [], self._blocked(t)))
        if not cmds:
            self._emergency_land(t, "找不到返航路線")
            return
        self.route, self.idx = cmds, 0
        self._dispatch(t)

    def _emergency_land(self, t, reason):
        self.aborted = True
        self.phase = "LAND"
        self._log(t, f"原地降落：{reason}")
        self.traj.set_path([self.body.xy], 0.0, 0.0)

    # ---------- 運動規劃層 ----------
    def _obstacles(self, t):
        return list(self.env.obstacles) + self._blocked(t)

    def _dispatch(self, t):
        cmd = self.route[self.idx]
        if self.phase == "FLY":
            if self.envelope is None or t - self.envelope.t > ENVELOPE_MAX_AGE_S - 5:
                self.envelope = self._envelope(t, self.route[self.idx:])
            v = self.safety.validate(cmd, self.body.xy, self.envelope, t)
            if not v.is_safe:
                self._log(t, f"安全層拒絕航點 {cmd.label}：{v.reason}")
                self._handle_failure(t, self.envelope, v.reason)
                return
        else:
            v = self.safety.validate_contingency(cmd, self.body.xy, self._blocked(t))
            if not v.is_safe:
                self._emergency_land(t, v.reason)
                return
        path = self.motion.plan(self.body.xy, cmd.xy, self._obstacles(t))
        if path is None:
            self._log(t, f"RRT* 找不到到 {cmd.label} 的無碰撞路徑")
            if self.phase == "FLY":
                self._rth(t, "運動規劃失敗")
            else:
                self._emergency_land(t, "運動規劃失敗")
            return
        via = f"（RRT* 繞障，{len(path) - 2} 個中繼點）" if len(path) > 2 else ""
        self._log(t, f"下達航點 {cmd.label} ({cmd.x:.0f}, {cmd.y:.0f}) 高度 {cmd.altitude_m:.0f} m{via}")
        self.traj.set_path(path, cmd.altitude_m, cmd.airspeed_kmh / 3.6)

    # ---------- 主迴圈 ----------
    def run(self, max_time_s=4000):
        m, t = self.m, 0.0
        base = [WaypointCommand(p[0], p[1], m.altitude_m, m.airspeed_kmh, label) for label, p in m.survey]
        base.append(WaypointCommand(m.home[0], m.home[1], m.altitude_m, m.airspeed_kmh, "HOME"))
        env0 = self._envelope(0.0, base, preflight=True)
        self.projected_home_soc = env0.projected_home_soc
        self._log(0.0, f"起飛前約束包絡：{env0.summary()}；預估返航電量 {env0.projected_home_soc:.0%}")
        if not env0.green and not set(env0.failed) <= ROUTE_DOMAINS:
            return self._report("NO_GO", env0.summary(), 0.0)
        cmds, envelope, why = self._plan(0.0, list(m.survey), None if env0.green else env0.summary(), True)
        if not cmds:
            return self._report("NO_GO", why if env0.green else f"{why}；{env0.summary()}", 0.0)
        self.route, self.idx, self.envelope = cmds, 0, envelope
        self.phase = "FLY"
        self._log(0.0, f"解鎖起飛（{self.planner.name} 規劃器）：" + " → ".join(c.label for c in cmds))
        self._dispatch(0.0)

        dt = 1.0 / self.servo_hz
        traj_every = self.servo_hz // self.traj_hz
        motion_every = self.servo_hz // self.motion_hz
        dt_traj = 1.0 / self.traj_hz
        thrust_ref, direction = 0.0, (0.0, 0.0, 1.0)
        next_refresh = ENVELOPE_REFRESH_S
        calm = 0.0
        k = 0
        while t < max_time_s:
            # 伺服層＋機體物理（每 tick）
            thrust, power = self.servo.step(thrust_ref)
            prev = self.body.xy
            self.body.step(dt, thrust, direction, self.env.wind_vector(t))
            self.distance += dist(prev, self.body.xy)
            self.soc -= soc_drain(power, dt, self.env.temperature_c)
            self.ticks["伺服層"] += 1
            k += 1
            t = k * dt
            if k % traj_every:
                continue

            # 軌跡層（100 Hz）＋快速安全檢查
            self.ticks["軌跡層"] += 1
            if self.phase in ("FLY", "RTH"):
                gust = self.env.wind_speed_kmh(t)
                if gust > GUST_HOLD_KMH and not self.traj.hold:
                    self.traj.hold = True
                    self.holds += 1
                    calm = 0.0
                    self._log(t, f"陣風 {gust:.0f} km/h > {GUST_HOLD_KMH:.0f}：拒絕推進，原地懸停")
                elif self.traj.hold:
                    calm = calm + dt_traj if gust <= GUST_HOLD_KMH else 0.0
                    if calm >= GUST_CALM_S:
                        self.traj.hold = False
                        self._log(t, "陣風減弱，恢復航線")
                if self.soc < SOC_HARD_FLOOR and self.phase == "FLY":
                    self._rth(t, f"電量 {self.soc:.0%} 低於硬下限 {SOC_HARD_FLOOR:.0%}")
            thrust_ref, direction = self.traj.update(self.body)

            if self.phase == "LAND":
                if self.body.p[2] < 0.05:
                    self._log(t, f"降落完成，電量 {self.soc:.0%}")
                    return self._final(t)
            elif self.traj.arrived(self.body) and not self.traj.hold:
                cmd = self.route[self.idx]
                self._log(t, f"抵達 {cmd.label}，電量 {self.soc:.0%}")
                # 只有真的飛到測量點座標才算完成（規劃器可能把其他點也標成 WP1）
                for item in list(self.remaining_survey):
                    if item[0] == cmd.label and dist(item[1], self.body.xy) < 10:
                        self.remaining_survey.remove(item)
                        self.surveyed.append(cmd.label)
                self.idx += 1
                if self.idx >= len(self.route):
                    self.phase = "LAND"
                    self.traj.alt_target = 0.0
                else:
                    self._dispatch(t)

            # 任務層：定期或 NOTAM 推播時更新約束包絡（先於運動層，讓法規限制走改道/返航流程）
            if self.phase == "FLY" and (t >= next_refresh or self.env.notam_pushed(t - dt_traj, t)):
                next_refresh = t + ENVELOPE_REFRESH_S
                pushed = self.env.notam_pushed(t - dt_traj, t)
                self.envelope = self._envelope(t, self.route[self.idx:])
                if pushed:
                    self._log(t, f"收到 NOTAM 推播，約束包絡：{self.envelope.summary()}")
                if not self.envelope.green:
                    if not pushed:
                        self._log(t, f"約束包絡轉為非全綠：{self.envelope.summary()}")
                    self._handle_failure(t, self.envelope, self.envelope.summary())

            # 運動層（2 Hz）：路徑仍然無碰撞嗎？
            if k % motion_every == 0 and self.phase in ("FLY", "RTH") and not self.traj.hold:
                self.ticks["運動層"] += 1
                if not self.motion.path_valid(self.traj.remaining_path(), self._obstacles(t)):
                    self._log(t, "運動層：剩餘路徑已不安全，重新規劃")
                    self._dispatch(t)
        return self._report("TIMEOUT", "超過模擬時間上限", t)

    def _final(self, t):
        if not self.remaining_survey and not self.aborted:
            return self._report("COMPLETED_REROUTED" if self.reroutes else "COMPLETED", "", t)
        at_home = dist(self.body.xy, self.m.home) < 10
        return self._report("RETURNED_HOME" if at_home else "EMERGENCY_LANDED",
                            "任務中止" if self.aborted else "", t)

    def _report(self, status, reason, t):
        if status == "NO_GO":
            self._log(t, "不解鎖馬達")
        return MissionReport(
            status=status, reason=reason, t_end=t, soc_start=self.soc_start, soc_end=self.soc,
            distance_m=self.distance, holds=self.holds, reroutes=self.reroutes,
            surveyed=list(self.surveyed), rrt_runs=self.motion.rrt_runs,
            layer_ticks=dict(self.ticks), max_track_err_m=self.traj.max_err_m,
            servo_rms_a=self.servo.rms_error_a, log=list(self.log),
            projected_home_soc=getattr(self, "projected_home_soc", None),
        )
