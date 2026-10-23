"""執行所有任務情境。

python demo.py            # 規則式規劃器，不需任何套件或 API 金鑰
python demo.py 6          # 只跑第 6 個情境
python demo.py --llm      # 改用 LLM 規劃器（需 requirements-llm.txt 與 .env 的 OPENAI_API_KEY）
"""
import sys
import time

from drone_agent.geometry import Rect
from drone_agent.planner import Mission, RuleBasedPlanner
from drone_agent.safety import WaypointCommand
from drone_agent.supervisor import MissionSupervisor
from drone_agent.world import Environment, Gust, Notam


class OverEagerPlanner(RuleBasedPlanner):
    """模擬會出錯的規劃器（例如 LLM）：第一次提出 80 km/h 的超速航線。"""
    name = "over-eager"

    def __init__(self):
        self.calls = 0

    def plan(self, req):
        self.calls += 1
        cmds = super().plan(req)
        if self.calls == 1 and cmds:
            return [WaypointCommand(c.x, c.y, c.altitude_m, 80.0, c.label) for c in cmds]
        return cmds


def scenarios():
    return [
        ("1. 正常任務（書中渥太華一月條件，含通訊塔 RRT* 繞障）", {}),
        ("2. 氣溫 -12°C，低於 -10°C 作業下限", {"env": Environment(temperature_c=-12.0)}),
        ("3. 起飛電量 25%", {"soc": 0.25}),
        ("4. 綠帶未取得飛越授權", {"env": Environment(greenbelt_authorized=False)}),
        ("5. 天氣資料 20 分鐘未更新（STALE）", {"env": Environment(weather_frozen_at=-1200.0)}),
        ("6. 飛行中收到 NOTAM，WP1–WP2 之間臨時禁航 → 改道",
         {"env": Environment(notams=[Notam("NOTAM-TFR-0042", Rect(1800, -300, 2700, 300), 60, 1e9, 60)])}),
        ("7. 飛行中收到 NOTAM，禁航區橫跨整條走廊 → 返航",
         {"env": Environment(notams=[Notam("NOTAM-TFR-0099", Rect(1800, -2000, 2700, 2000), 60, 1e9, 60)])}),
        ("8. 陣風 32 km/h 持續 40 秒 → 懸停後繼續",
         {"env": Environment(gusts=[Gust(300, 340, 32.0)])}),
        ("9. 規劃器提出超速航線 → 安全層拒絕、回饋後重新規劃", {"planner": OverEagerPlanner()}),
    ]


def run(name, env=None, soc=0.82, planner=None, show_log=True):
    sup = MissionSupervisor(env or Environment(), Mission(), planner or RuleBasedPlanner(), soc=soc)
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
