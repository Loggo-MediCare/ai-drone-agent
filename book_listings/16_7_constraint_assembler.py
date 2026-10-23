# 摘錄自第十六章 Listing 16.7，原文照錄。
# 已知問題（本專案已修正）：
#   1. 由 LLM 輸出 JSON 來決定 envelope 是否全綠；LLM 可能判斷錯誤。
#   2. ConstraintEnvelope、ValidationError、knowledge_graph.add_constraint_node 未定義。
#   3. 書中文字描述的 STALE（資料過期）規則沒有實作在程式中。
# Listing 16.7: Cross-domain constraint assembler with NOTAM integration
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent
from typing import Optional
import json


# --- Domain query tools (extend Listing 16.4 tool pattern) ---
@tool
def query_weather_constraints(corridor_id: str) -> dict:
    """Query real-time weather data for the specified flight corridor.
    Returns constraint-relevant fields: temperature, wind, precipitation."""
    # Production: query Environment Canada API or equivalent
    return {
        "temperature_c": -6.2,
        "wind_speed_kmh": 18.5,
        "precipitation_active": False,
        "temperature_constraint_met": True,   # > -10C
        "wind_constraint_met": True,          # < 25 km/hr
        "precip_constraint_met": True
    }


@tool
def query_airspace_notams(corridor_id: str) -> dict:
    # NOTAM validity check (pseudocode):
    # 1. Temporal validity: discard NOTAMs where mission_time is outside
    #    [valid_from, valid_until]; only active windows constrain the plan.
    #    relevant_notams = [n for n in notams
    #                       if n.valid_from <= mission_time <= n.valid_until]
    #
    # 2. Geospatial overlap: check whether the restricted polygon
    #    intersects the planned corridor geometry (Shapely or equivalent).
    #    blocked = any(notam.polygon.intersects(corridor_geometry)
    #                  for notam in relevant_notams)
    #    airspace_constraint_met = not blocked
    """Query Transport Canada NOTAM feed for active flight restrictions.
    NOTAM: Notice to Air Missions, the standard Transport Canada format for
    real-time airspace restriction alerts issued under CARs Part IX."""
    # Production: query Transport Canada DroneZone API
    # Mock NOTAM response structure reflecting Transport Canada alert format
    return {
        "notams": [
            {
                "alert_id": "NOTAM-CYOW-2025-0042",
                "corridor": "CENTERPOINTE_OTTAWARIVER",
                "restriction_type": "NONE",    # NONE | TEMPORARY | PERMANENT | TFR
                "valid_from": "2025-01-15T08:00:00Z",
                "valid_until": "2025-01-15T20:00:00Z",
                "authority": "Transport Canada"
            }
        ],
        "airspace_constraint_met": True   # No active restrictions in corridor
    }


@tool
def query_battery_state(drone_id: str) -> dict:
    """Query battery management system for current State of Charge and
    projected SoC at each planned waypoint."""
    return {
        "current_soc": 0.82,
        "projected_min_soc": 0.34,     # Minimum projected during mission
        "return_reserve_soc": 0.21,    # Projected SoC at return-to-home
        "battery_constraint_met": True  # SoC floor 30% met at departure
    }


@tool
def query_parks_restrictions(route_geojson: str) -> dict:
    """Query Parks Canada and National Capital Commission databases for
    overflight restrictions along the planned route geometry."""
    # Production: intersect route geometry with NCC Greenbelt restriction polygons
    return {
        "greenbelt_intersection": True,     # Route crosses protected land
        "overflight_authorized": True,      # Authorization on file
        "authorization_reference": "NCC-UAV-2025-0017",
        "parks_constraint_met": True
    }


@tool
def register_constraint_node(
    domain: str,
    node_id: str,
    constraint_met: bool,
    weight: float
) -> dict:
    """Register a typed constraint node in the knowledge graph G=(V,E).
    Extends the register_cross_domain_edge pattern from Listing 16.4.
    domain: one of WEATHER, AIRSPACE, BATTERY, PARKS, MISSION."""
    return knowledge_graph.add_constraint_node(
        domain=domain,
        node_id=node_id,
        status='GREEN' if constraint_met else 'RED',
        weight=weight
    )


# --- Constraint assembler agent (Domain-Transforming Integration Agent) ---
constraint_assembler = create_react_agent(
    ChatOpenAI(model="gpt-4o", temperature=0.0),
    [
        query_weather_constraints,
        query_airspace_notams,
        query_battery_state,
        query_parks_restrictions,
        register_constraint_node
    ]
)


def assemble_unified_constraint_envelope(
    corridor_id: str,
    drone_id: str,
    route_geojson: str
) -> dict:
    """Assemble the Unified Constraint Envelope by querying all domain agents
    and applying conservative constraint fusion: all domains must report
    constraint_met=True for envelope_green to be returned as True."""
    result = constraint_assembler.invoke({
        "messages": [
            {
                "role": "user",
                "content": (
                    f"Corridor: {corridor_id}. Drone: {drone_id}.\n"
                    "Query all four constraint domains (weather, airspace, battery, parks). "
                    "Register each result as a typed constraint node in the knowledge graph. "
                    "Apply influence propagation if any NOTAM restriction is active: "
                    "call propagate_influence with the NOTAM node as source to estimate cascade. "
                    "Return unified_envelope_green=True only if ALL domains report constraint_met=True."
                )
            }
        ]
    })
    # Validate output structure before returning to Mission Supervisor
    raw = result['messages'][-1].content
    try:
        envelope = ConstraintEnvelope.model_validate_json(raw)
    except ValidationError as e:
        raise RuntimeError(f"Constraint assembler returned invalid schema: {e}")
    return envelope.model_dump()
