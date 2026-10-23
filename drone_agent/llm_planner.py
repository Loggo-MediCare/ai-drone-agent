"""LLM 規劃器（選用）：讓 GPT 依操作員指令提出航點。

需要：pip install -r requirements-llm.txt，以及 .env 中的 OPENAI_API_KEY。
LLM 只「提案」；約束包絡與 SafetyMonitor 仍由程式碼決定能不能飛，
提案不合格時把拒絕原因回饋給 LLM 重新規劃（Listing 16.3 的回饋迴圈）。
"""
import os

from pydantic import BaseModel, Field

from .safety import WaypointCommand


class _Waypoint(BaseModel):
    label: str = Field(description="WP1..WP4, HOME, or DETOUR-n")
    x_m: float
    y_m: float
    altitude_m: float
    airspeed_kmh: float


class _Plan(BaseModel):
    waypoints: list[_Waypoint]


class LLMPlanner:
    name = "llm"

    def __init__(self, model=None, instruction="Survey the corridor waypoints in order, then return home."):
        from langchain_openai import ChatOpenAI
        self.instruction = instruction
        self._llm = ChatOpenAI(
            model=model or os.getenv("OPENAI_MODEL", "gpt-4o"), temperature=0
        ).with_structured_output(_Plan)

    def plan(self, req):
        m = req.mission
        areas = [f"x {a.xmin}..{a.xmax}, y {a.ymin}..{a.ymax}" for a in req.blocked_areas] or ["none"]
        prompt = (
            f"You plan drone waypoints on a flat x/y metre grid. Operator: {self.instruction}\n"
            f"Start: {req.start}. Home: {m.home}.\n"
            f"Remaining survey waypoints (must visit in order): {req.remaining_survey}.\n"
            f"Altitude {m.altitude_m} m, airspeed {m.airspeed_kmh} km/h.\n"
            f"Geofence: x {m.geofence.xmin}..{m.geofence.xmax}, y {m.geofence.ymin}..{m.geofence.ymax}.\n"
            f"Restricted areas (no straight leg may cross them): {'; '.join(areas)}.\n"
            "End with HOME."
        )
        if req.feedback:
            prompt += f"\nYour previous plan was rejected: {req.feedback}. Replan within the constraints."
        try:
            plan = self._llm.invoke(prompt)
        except Exception as e:  # 網路、金鑰、配額錯誤：當作無法規劃
            print(f"[llm] planning failed: {e}")
            return None
        return [WaypointCommand(w.x_m, w.y_m, w.altitude_m, w.airspeed_kmh, w.label) for w in plan.waypoints]
