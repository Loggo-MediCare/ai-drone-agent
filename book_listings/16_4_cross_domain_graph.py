# 摘錄自第十六章 Listing 16.4，原文照錄。
# Listing 16.4: Cross-domain knowledge graph construction via tool-calling
from langchain_core.tools import tool
from langgraph.prebuilt import create_react_agent


# --- Domain-specific data source tools ---
@tool
def query_energy_grid(region: str) -> list:
    """Query substations, capacity, and load for a region."""
    return energy_api.get_substations(region)


@tool
def query_traffic_network(region: str) -> list:
    """Query intersections, controllers, and throughput."""
    return traffic_api.get_network(region)


@tool
def register_cross_domain_edge(
    source_id: str, target_id: str,
    relation: str, weight: float
) -> dict:
    """Register a typed, weighted edge between two
    entities in different domains."""
    return knowledge_graph.add_edge(
        source_id, target_id, relation, weight
    )


# --- Graph construction agent ---
graph_builder = create_react_agent(
    ChatOpenAI(model="gpt-4o", temperature=0.0),
    [
        query_energy_grid,
        query_traffic_network,
        register_cross_domain_edge
    ]
)
