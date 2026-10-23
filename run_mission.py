"""從統一任務輸入檔執行任務：讀資料 → 起飛前資料閘門 → 四層模擬飛行。

python run_mission.py                                  # 預設 data/mission_input.json（模擬模式）
python run_mission.py data/mission_input.real.example.json
python run_mission.py data/mission_input.json --llm

不會連接或控制任何真實無人機的馬達；真實資料模式只是用真實資料「判斷能不能飛」。
"""
import sys

from drone_agent.inputs.builder import load_mission_input
from drone_agent.planner import RuleBasedPlanner
from drone_agent.supervisor import MissionSupervisor


def main():
    args = sys.argv[1:]
    use_llm = "--llm" in args
    args = [a for a in args if a != "--llm"]
    path = args[0] if args else "data/mission_input.json"
    try:
        from dotenv import load_dotenv
        load_dotenv()
    except ImportError:
        pass

    inputs = load_mission_input(path)
    mode = "模擬模式（SIMULATED，不可作為真實飛行依據）" if inputs.simulation else "真實資料模式"
    print(f"任務輸入：{path}\n模式：{mode}\n資料來源：\n{inputs.table()}")
    if inputs.issues:
        print("\n起飛前資料閘門：拒絕起飛")
        for issue in inputs.issues:
            print(f"  ✗ {issue}")
        return 1
    print("\n起飛前資料閘門：通過")

    if use_llm:
        from drone_agent.llm_planner import LLMPlanner
        planner = LLMPlanner()
    else:
        planner = RuleBasedPlanner()
    report = MissionSupervisor(inputs.environment, inputs.mission, planner, soc=inputs.soc).run()
    print(f"\n=== {inputs.mission.name}")
    report.print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
