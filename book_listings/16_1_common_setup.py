# 摘錄自第十六章 Listing 16.1，原文照錄。
# Listing 16.1: Common setup: shared interface stubs for all chapter listings
"""Common setup for Chapter 16 listings.
These stubs define the interface contracts that all subsequent
listings reference.  They are architectural scaffolds, not
production implementations."""
from langchain_core.messages import HumanMessage


class WorldModel:
    """Maintains belief state b(s) over robot and environment."""
    def query(self, query: str) -> dict:
        raise NotImplementedError

    def get_current_state(self) -> dict:
        raise NotImplementedError

    def update(self, observations: dict) -> None:
        raise NotImplementedError


class ControlInterface:
    """Dispatches validated commands to the deterministic
    controller (50-200 Hz loop)."""
    def execute(self, *args, **kwargs) -> object:
        raise NotImplementedError


class SafetyMonitor:
    """Validates actions against A_safe(s) before execution."""
    def validate(self, action, target_or_state) -> object:
        raise NotImplementedError

    def halt(self, reason: str = "") -> None:
        raise NotImplementedError


def extract_actions(plan) -> list:
    """Parse proposed actions from agent plan output."""
    raise NotImplementedError


# --- Stubs for Domain-Transforming Integration Agent (Listings 16.3-16.4) ---
class _EnergyAPI:
    def get_substations(self, region: str) -> list:
        raise NotImplementedError


class _TrafficAPI:
    def get_network(self, region: str) -> list:
        raise NotImplementedError


class _KnowledgeGraph:
    def add_edge(self, src, tgt, rel, weight) -> dict:
        raise NotImplementedError


world_model = WorldModel()
control_interface = ControlInterface()
safety_monitor = SafetyMonitor()
energy_api = _EnergyAPI()
traffic_api = _TrafficAPI()
knowledge_graph = _KnowledgeGraph()
