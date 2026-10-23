"""GPT 任務規劃器（選用）：只決定測量點順序、任務優先順序與是否建議返航。

GPT **不產生座標、航線或馬達指令**，也不能宣告測量完成：
- 回傳的順序必須剛好是剩餘測量點 ID 的一個排列（validate_decision 檢查）。
- 航線幾何由 RoutePlanner（RRT*）計算；安全層獨立檢查每一段。
- 測量完成由監督器依實際位置判定。

需要：pip install -r requirements-llm.txt，以及 .env 中的 OPENAI_API_KEY。
"""
import os

from pydantic import BaseModel, Field

from .planner import MissionDecision


class _Decision(BaseModel):
    order: list[str] = Field(description="All remaining survey waypoint IDs, each exactly once, in flight order")
    return_home: bool = Field(description="True only if continuing the mission is unwise")
    reason: str


class LLMPlanner:
    name = "llm"

    def __init__(self, model=None, instruction="Survey all remaining waypoints efficiently, then return home."):
        from langchain_openai import ChatOpenAI
        self.instruction = instruction
        self._llm = ChatOpenAI(
            model=model or os.getenv("OPENAI_MODEL", "gpt-4o"), temperature=0
        ).with_structured_output(_Decision)

    def build_prompt(self, req):
        remaining = ", ".join(f"{wid} at ({p[0]:.0f}, {p[1]:.0f}) m" for wid, p in req.remaining)
        prompt = (
            "You are the mission-planning layer of a survey drone. You only decide the ORDER of the "
            "remaining survey waypoints and whether to recommend returning home. You do not plan geometry: "
            "a separate deterministic planner computes every route and detour, and an independent safety "
            "supervisor checks it.\n"
            f"Operator: {self.instruction}\n"
            f"Vehicle position: ({req.position[0]:.0f}, {req.position[1]:.0f}) m. Home: {req.mission.home}. "
            f"Battery: {req.soc:.0%}.\n"
            f"Remaining survey waypoints: {remaining}.\n"
            f"Constraint envelope: {req.envelope_summary or 'not yet evaluated'}.\n"
            "Return every remaining waypoint ID exactly once."
        )
        if req.feedback:
            prompt += f"\nThe previous plan was rejected: {req.feedback}"
        return prompt

    def decide(self, req):
        d = self._llm.invoke(self.build_prompt(req))
        return MissionDecision(list(d.order), bool(d.return_home), d.reason)
