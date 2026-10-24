"""Deterministic safety gateway in front of every tool.

Order of checks (any failure denies and is audited):

  1. well-formed request          BAD_REQUEST
  2. authenticated client         UNAUTHENTICATED   (constant-time token compare)
  3. allowlisted action           ACTION_NOT_ALLOWED
  4. client holds the scope       FORBIDDEN
  5. fresh issued_at              STALE_REQUEST     (window: 5 s old, 1 s ahead)
  6. not a replay                 REPLAY            (unseen nonce and strictly increasing seq)
  7. handler bound                ACTION_NOT_AVAILABLE
  8. handler runs in time         TIMEOUT / INTERNAL_ERROR
                                  INVALID_PARAMS / INCONSISTENT_STATE (raised by tools)
  9. output is finite and JSON-like  UNSAFE_OUTPUT

The nonce and sequence are consumed only after authentication succeeds, so an
attacker without the token cannot burn a client's sequence. They are consumed
before the handler runs, so a failing handler cannot be retried with the same
request. The allowlist contains only simulation and read-only tools, and
binding anything else raises: there is no path from here to motors, PWM or a
flight controller.

The patterns (nonce, monotonic sequence, deadlines, fail-closed gateway) follow
the blockchain-iot-door gateway, but are re-implemented here so the two
projects stay isolated.
"""
import concurrent.futures
import hashlib
import hmac
import math
import time
from dataclasses import dataclass
from typing import Any, Callable

from .errors import InconsistentState, InvalidParams

READ = "read"
SIMULATE = "simulate"

# action -> scope required. Nothing here can move hardware.
ALLOWED_ACTIONS = {
    "get_drone_state": READ,
    "get_world_model": READ,
    "get_battery_status": READ,
    "get_weather_constraints": READ,
    "get_airspace_constraints": READ,
    "get_mission_telemetry": READ,
    "evaluate_mission_performance": READ,
    "plan_mission": SIMULATE,
    "estimate_mission_energy": SIMULATE,
    "validate_trajectory": SIMULATE,
    "simulate_mission": SIMULATE,
}

MAX_REQUEST_AGE_S = 5.0
MAX_CLOCK_SKEW_S = 1.0
MAX_ID_LENGTH = 128
MAX_REMEMBERED_NONCES = 10_000


@dataclass(frozen=True)
class GatewayRequest:
    client_id: str
    action: str
    params: Any
    request_id: Any
    seq: Any
    issued_at: Any
    token: Any


@dataclass(frozen=True)
class GatewayResponse:
    ok: bool
    code: str
    result: Any = None
    reason: str = ""


def _digest(token: str) -> bytes:
    return hashlib.sha256(token.encode("utf-8")).digest()


def _finite_json(value, depth=0) -> bool:
    """True if value is made of dicts, lists, strings, bools, None and finite numbers."""
    if depth > 20:
        return False
    if value is None or isinstance(value, (str, bool)):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, (list, tuple)):
        return all(_finite_json(v, depth + 1) for v in value)
    if isinstance(value, dict):
        return all(isinstance(k, str) and _finite_json(v, depth + 1) for k, v in value.items())
    return False


class SafetyGateway:
    def __init__(self, clock: Callable[[], float] = time.time, handler_timeout_s: float = 30.0):
        self._clock = clock
        self._timeout = handler_timeout_s
        self._clients = {}  # client_id -> {"digest", "scopes", "last_seq", "nonces"}
        self._handlers = {}
        self._audit = []
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="gateway")

    # ---------- configuration ----------
    def register_client(self, client_id: str, token: str, scopes: set):
        if not token or len(token) < 12:
            raise ValueError("token must be at least 12 characters")
        unknown = set(scopes) - {READ, SIMULATE}
        if unknown:
            raise ValueError(f"unknown scopes: {sorted(unknown)}")
        self._clients[client_id] = {"digest": _digest(token), "scopes": frozenset(scopes), "last_seq": None, "nonces": set()}

    def bind(self, action: str, handler: Callable[[dict], Any]):
        if action not in ALLOWED_ACTIONS:
            raise ValueError(f"{action!r} is not an allowlisted simulation or read-only action")
        self._handlers[action] = handler

    def audit_log(self):
        return [dict(e) for e in self._audit]

    # ---------- request handling ----------
    def execute(self, request: GatewayRequest) -> GatewayResponse:
        try:
            response = self._execute(request)
        except Exception as err:  # fail closed on any bug in the checks themselves
            response = GatewayResponse(False, "INTERNAL_ERROR", None, f"gateway error: {type(err).__name__}")
        self._record(request, response)
        return response

    def _execute(self, r: GatewayRequest) -> GatewayResponse:
        if not isinstance(r.request_id, str) or not 0 < len(r.request_id) <= MAX_ID_LENGTH:
            return GatewayResponse(False, "BAD_REQUEST", reason="request_id must be a non-empty string up to 128 chars")
        if not isinstance(r.seq, int) or isinstance(r.seq, bool):
            return GatewayResponse(False, "BAD_REQUEST", reason="seq must be an integer")
        if not isinstance(r.params, dict):
            return GatewayResponse(False, "BAD_REQUEST", reason="params must be an object")

        client = self._clients.get(r.client_id) if isinstance(r.client_id, str) else None
        presented = _digest(r.token) if isinstance(r.token, str) else b""
        if client is None or not hmac.compare_digest(presented, client["digest"]):
            return GatewayResponse(False, "UNAUTHENTICATED", reason="unknown client or bad token")

        if not isinstance(r.action, str) or r.action not in ALLOWED_ACTIONS:
            return GatewayResponse(False, "ACTION_NOT_ALLOWED", reason=f"action {r.action!r} is not allowlisted")
        if ALLOWED_ACTIONS[r.action] not in client["scopes"]:
            return GatewayResponse(False, "FORBIDDEN", reason=f"client lacks scope {ALLOWED_ACTIONS[r.action]!r}")

        now = self._clock()
        ts = r.issued_at
        if not isinstance(ts, (int, float)) or isinstance(ts, bool) or not math.isfinite(ts):
            return GatewayResponse(False, "STALE_REQUEST", reason="issued_at must be a finite number")
        if ts <= now - MAX_REQUEST_AGE_S or ts > now + MAX_CLOCK_SKEW_S:
            return GatewayResponse(False, "STALE_REQUEST", reason=f"issued_at outside the {MAX_REQUEST_AGE_S:.0f} s window")

        if r.request_id in client["nonces"]:
            return GatewayResponse(False, "REPLAY", reason="request_id already used")
        if client["last_seq"] is not None and r.seq <= client["last_seq"]:
            return GatewayResponse(False, "REPLAY", reason="seq must strictly increase")
        # Consume before running the handler: a failed call cannot be replayed.
        client["last_seq"] = r.seq
        client["nonces"].add(r.request_id)
        if len(client["nonces"]) > MAX_REMEMBERED_NONCES:
            client["nonces"].clear()  # safe: seq monotonicity still rejects every older request

        handler = self._handlers.get(r.action)
        if handler is None:
            return GatewayResponse(False, "ACTION_NOT_AVAILABLE", reason=f"{r.action} has no handler")

        future = self._pool.submit(handler, dict(r.params))
        try:
            result = future.result(timeout=self._timeout)
        except concurrent.futures.TimeoutError:
            return GatewayResponse(False, "TIMEOUT", reason=f"{r.action} exceeded {self._timeout:.1f} s")
        except (InvalidParams, InconsistentState) as err:
            return GatewayResponse(False, err.code, reason=str(err))
        except Exception as err:
            return GatewayResponse(False, "INTERNAL_ERROR", reason=f"{r.action} failed: {type(err).__name__}: {err}")

        if not _finite_json(result):
            return GatewayResponse(False, "UNSAFE_OUTPUT", reason="handler output contains non-finite or non-JSON values")
        return GatewayResponse(True, "ALLOW", result)

    def _record(self, r: GatewayRequest, response: GatewayResponse):
        self._audit.append({
            "ts": self._clock(),
            "client_id": r.client_id if isinstance(r.client_id, str) else repr(r.client_id),
            "action": r.action if isinstance(r.action, str) else repr(r.action),
            "request_id": r.request_id if isinstance(r.request_id, str) else repr(r.request_id),
            "seq": r.seq if isinstance(r.seq, int) else repr(r.seq),
            "decision": response.code,
            "reason": response.reason,
        })
