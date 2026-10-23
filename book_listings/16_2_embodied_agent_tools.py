# 摘錄自第十六章 Listing 16.2，原文照錄。
# Listing 16.2: Embodied agent with four-responsibility tool decomposition
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent


# --- Model Maintenance: query the world model ---
@tool
def query_world_model(query: str) -> dict:
    """Query the robot's world model for object poses,
    collision status, or grasp difficulty estimates."""
    return world_model.query(query)


# --- Real-Time Control: dispatch validated commands ---
@tool
def dispatch_motion_command(target: str, action: str) -> dict:
    """Submit a motion command through the control interface.
    Safety validation occurs before dispatch."""
    return control_interface.execute(target, action)


# --- Safety Enforcement: validate before execution ---
@tool
def check_safety_constraints(action: str, target: str) -> dict:
    """Validate an action against the admissible action set.
    Checks workspace bounds, force limits, velocity caps."""
    return safety_monitor.validate(action, target)


# --- Strategic Reasoning: LLM agent with tool access ---
llm = ChatOpenAI(model="gpt-4o", temperature=0.0)
tools = [
    query_world_model,
    dispatch_motion_command,
    check_safety_constraints
]
embodied_agent = create_react_agent(llm, tools)
