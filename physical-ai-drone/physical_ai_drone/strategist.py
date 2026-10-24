"""Strategic reasoning layer: proposes, never decides.

MockStrategist is a deterministic stand-in for an LLM. It returns canned
proposals (some deliberately unsafe or malformed) so the review loop can be
tested without an API key. A real LLM strategist would implement the same
propose() method; its output gets exactly the same treatment: every proposal
goes through the gateway's plan_mission, validate_trajectory and
estimate_mission_energy, and its free-text rationale is kept as untrusted data.
"""
import math

from .schemas import untrusted


class MockStrategist:
    name = "mock-strategist (deterministic stand-in for an LLM)"

    def __init__(self, proposals):
        self._proposals = proposals

    def propose(self, spec):
        return [dict(p) for p in self._proposals]


def review_proposals(client, base_spec, proposals):
    """Checks every proposal through the gateway and picks the safe one with the least estimated energy.

    client: anything with .call(action, params) returning a GatewayResponse.
    """
    reviewed = []
    for p in proposals:
        speed = p.get("airspeed_kmh")
        entry = {"airspeed_kmh": speed, "rationale": untrusted(p.get("rationale", "")), "accepted": False}
        spec = {**base_spec, "airspeed_kmh": speed}
        plan = client.call("plan_mission", spec)
        if not plan.ok or not plan.result["feasible"]:
            entry["rejected_by"] = plan.code if not plan.ok else "plan_mission: infeasible"
            reviewed.append(entry)
            continue
        request = {"spec": spec, "waypoints": plan.result["waypoints"]}
        verdict = client.call("validate_trajectory", request)
        if not verdict.ok or not verdict.result["valid"]:
            entry["rejected_by"] = verdict.code if not verdict.ok else "; ".join(verdict.result["reasons"])
            reviewed.append(entry)
            continue
        energy = client.call("estimate_mission_energy", request)
        if not energy.ok:
            entry["rejected_by"] = energy.code
            reviewed.append(entry)
            continue
        entry.update(accepted=True, estimated_wh=energy.result["estimated_wh"], duration_s=energy.result["duration_s"])
        reviewed.append(entry)

    accepted = [e for e in reviewed if e["accepted"]]
    chosen = min(accepted, key=lambda e: e["estimated_wh"]) if accepted else None
    for e in reviewed:  # keep the report JSON-safe
        if isinstance(e["airspeed_kmh"], float) and not math.isfinite(e["airspeed_kmh"]):
            e["airspeed_kmh"] = repr(e["airspeed_kmh"])
    return {"proposals": reviewed, "chosen": chosen}
