"""執行所有任務情境。

python demo.py            # 規則式任務規劃器，不需任何套件或 API 金鑰
python demo.py 6          # 只跑第 6 個情境
python demo.py --llm      # 改由 GPT 決定測量順序（需 requirements-llm.txt 與 .env 的 OPENAI_API_KEY）
"""
import sys
import time

from drone_agent.geometry import Rect
from drone_agent.planner import Mission, MissionDecision, RuleBasedPlanner
from drone_agent.route_planner import RoutePlanner
from drone_agent.safety import WaypointCommand
from drone_agent.supervisor import MissionSupervisor
from drone_agent.world import Environment, Gust, Notam


class OverEagerRoutePlanner(RoutePlanner):
    """模擬設定錯誤的航線產生器：第一次把空速設成 80 km/h，交給安全層攔截。"""

    def __init__(self, motion=None):
        super().__init__(motion)
        self.calls = 0

    def plan(self, start, order, mission, exclusion_zones, obstacles):
        self.calls += 1
        cmds, why = super().plan(start, order, mission, exclusion_zones, obstacles)
        if self.calls == 1 and cmds:
            cmds = [WaypointCommand(c.x, c.y, c.altitude_m, 80.0, c.label, c.survey_id) for c in cmds]
        return cmds, why


class BadOrderPlanner(RuleBasedPlanner):
    """模擬出錯的 GPT：第一次回傳重複的航點 ID，並漏掉 WP1（彷彿宣稱已完成）。"""
    name = "bad-order"

    def __init__(self):
        self.calls = 0

    def decide(self, req):
        self.calls += 1
        if self.calls == 1:
            return MissionDecision(["WP2", "WP2", "WP3", "WP4"], False, "WP1 already done")
        return super().decide(req)


def scenarios():
    return [
        ("1. 正常任務（書中渥太華一月條件，含通訊塔 RRT* 繞障）", {}),
        ("2. 氣溫 -12°C，低於 -10°C 作業下限", {"env": Environment(temperature_c=-12.0)}),
        ("3. 起飛電量 25%", {"soc": 0.25}),
        ("4. 綠帶未取得飛越授權", {"env": Environment(greenbelt_authorized=False)}),
        ("5. 天氣資料 20 分鐘未更新（STALE）", {"env": Environment(weather_frozen_at=-1200.0)}),
        ("6. 飛行中收到 NOTAM，WP1–WP2 之間臨時禁航 → RRT* 改道",
         {"env": Environment(notams=[Notam("NOTAM-TFR-0042", Rect(1800, -300, 2700, 300), 60, 1e9, 60)])}),
        ("7. 飛行中收到 NOTAM，禁航區橫跨整條走廊 → 返航",
         {"env": Environment(notams=[Notam("NOTAM-TFR-0099", Rect(1800, -2000, 2700, 2000), 60, 1e9, 60)])}),
        ("8. 陣風 32 km/h 持續 40 秒 → 懸停後繼續",
         {"env": Environment(gusts=[Gust(300, 340, 32.0)])}),
        ("9. 航線產生器給出 80 km/h 超速航線 → 安全層拒絕、重新規劃",
         {"route_planner": OverEagerRoutePlanner}),
        ("10. 任務規劃器回傳重複 ID、漏掉 WP1 → 拒絕、重試", {"planner": BadOrderPlanner()}),
    ]


def run(name, env=None, soc=0.82, planner=None, route_planner=None, show_log=True):
    sup = MissionSupervisor(env or Environment(), Mission(), planner or RuleBasedPlanner(), soc=soc)
    if route_planner:
        sup.route_planner = route_planner(sup.motion)
    t0 = time.perf_counter()
    report = sup.run()
    print(f"\n=== {name}  （模擬耗時 {time.perf_counter() - t0:.1f} s）")
    report.print(show_log)
    return report


def main():
    args = sys.argv[1:]
    planner_factory = None
    if "--llm" in args:
        args.remove("--llm")
        try:
            from dotenv import load_dotenv
            load_dotenv()
        except ImportError:
            pass
        from drone_agent.llm_planner import LLMPlanner
        planner_factory = LLMPlanner
    chosen = {int(a) for a in args if a.isdigit()}
    for i, (name, kwargs) in enumerate(scenarios(), 1):
        if chosen and i not in chosen:
            continue
        if planner_factory and "planner" not in kwargs:
            kwargs = {**kwargs, "planner": planner_factory()}
        run(name, **kwargs)


if __name__ == "__main__":
    main()
