"""Safety gateway: authentication, allowlist, scopes, freshness, replay, fail-closed, audit.

Written before physical_ai_drone/gateway.py, test-first.
"""
import math
import time
import unittest

from physical_ai_drone.gateway import (
    ALLOWED_ACTIONS,
    GatewayRequest,
    SafetyGateway,
)


class FakeClock:
    def __init__(self, t=1_800_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def make_gateway(clock=None, handlers=None, timeout_s=2.0):
    clock = clock or FakeClock()
    gw = SafetyGateway(clock=clock, handler_timeout_s=timeout_s)
    gw.register_client("planner", token="t0ken-planner-123", scopes={"read", "simulate"})
    gw.register_client("viewer", token="t0ken-viewer-456", scopes={"read"})
    for action, fn in (handlers or {}).items():
        gw.bind(action, fn)
    return gw, clock


def req(gw, clock, action="get_drone_state", params=None, client="planner", token="t0ken-planner-123", **kw):
    seq = kw.pop("seq", None)
    if seq is None:
        req.counter = getattr(req, "counter", 0) + 1
        seq = req.counter
    return GatewayRequest(
        client_id=client,
        action=action,
        params=params or {},
        request_id=kw.pop("request_id", f"nonce-{seq}-{action}"),
        seq=seq,
        issued_at=kw.pop("issued_at", clock()),
        token=token,
    )


OK_HANDLERS = {name: (lambda params, name=name: {"tool": name, "params": params}) for name in ALLOWED_ACTIONS}


class AllowlistTests(unittest.TestCase):
    def test_allowlist_is_exactly_the_eleven_simulation_and_read_only_tools(self):
        self.assertEqual(
            set(ALLOWED_ACTIONS),
            {
                "get_drone_state", "get_world_model", "get_battery_status", "get_weather_constraints",
                "get_airspace_constraints", "plan_mission", "estimate_mission_energy", "validate_trajectory",
                "simulate_mission", "get_mission_telemetry", "evaluate_mission_performance",
            },
        )

    def test_no_actuator_or_hardware_action_exists(self):
        forbidden = ("motor", "pwm", "arm", "actuat", "throttle", "servo", "takeoff", "hardware", "mavlink")
        for name in ALLOWED_ACTIONS:
            self.assertFalse(any(word in name for word in forbidden), name)

    def test_unknown_or_physical_actions_are_denied(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for action in ("set_motor_pwm", "arm", "send_mavlink", "GET_DRONE_STATE", "get_drone_state ", ""):
            r = gw.execute(req(gw, clock, action=action))
            self.assertEqual((r.ok, r.code), (False, "ACTION_NOT_ALLOWED"), action)

    def test_binding_a_non_allowlisted_action_is_refused(self):
        gw, _ = make_gateway()
        with self.assertRaises(ValueError):
            gw.bind("set_motor_pwm", lambda p: None)


class AuthTests(unittest.TestCase):
    def test_valid_token_and_scope_succeeds(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        r = gw.execute(req(gw, clock))
        self.assertTrue(r.ok, r.reason)
        self.assertEqual(r.result["tool"], "get_drone_state")

    def test_wrong_missing_or_other_clients_token_is_denied(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for token in ("wrong", "", None, "t0ken-viewer-456"):
            r = gw.execute(req(gw, clock, token=token))
            self.assertEqual((r.ok, r.code), (False, "UNAUTHENTICATED"), token)

    def test_unknown_client_is_denied(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        r = gw.execute(req(gw, clock, client="intruder"))
        self.assertEqual(r.code, "UNAUTHENTICATED")

    def test_read_only_client_cannot_plan_or_simulate(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for action in ("plan_mission", "estimate_mission_energy", "validate_trajectory", "simulate_mission"):
            r = gw.execute(req(gw, clock, action=action, client="viewer", token="t0ken-viewer-456"))
            self.assertEqual((r.ok, r.code), (False, "FORBIDDEN"), action)
        r = gw.execute(req(gw, clock, action="get_world_model", client="viewer", token="t0ken-viewer-456"))
        self.assertTrue(r.ok, r.reason)


class FreshnessAndReplayTests(unittest.TestCase):
    def test_stale_request_is_rejected(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        r = gw.execute(req(gw, clock, issued_at=clock() - 5.0))
        self.assertEqual(r.code, "STALE_REQUEST")
        r = gw.execute(req(gw, clock, issued_at=clock() - 4.999))
        self.assertTrue(r.ok, r.reason)

    def test_request_from_the_future_is_rejected(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        r = gw.execute(req(gw, clock, issued_at=clock() + 2.0))
        self.assertEqual(r.code, "STALE_REQUEST")

    def test_non_finite_timestamps_are_rejected(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for ts in (math.nan, math.inf, -math.inf, "now", None):
            r = gw.execute(req(gw, clock, issued_at=ts))
            self.assertEqual(r.code, "STALE_REQUEST", ts)

    def test_replayed_nonce_is_rejected_even_with_a_higher_sequence(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        self.assertTrue(gw.execute(req(gw, clock, request_id="n-1", seq=1)).ok)
        r = gw.execute(req(gw, clock, request_id="n-1", seq=2))
        self.assertEqual(r.code, "REPLAY")

    def test_sequence_must_strictly_increase(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        self.assertTrue(gw.execute(req(gw, clock, request_id="a", seq=10)).ok)
        for seq in (10, 9, 0, -1):
            r = gw.execute(req(gw, clock, request_id=f"b{seq}", seq=seq))
            self.assertEqual(r.code, "REPLAY", seq)
        self.assertTrue(gw.execute(req(gw, clock, request_id="c", seq=11)).ok)

    def test_failed_authentication_does_not_burn_the_clients_sequence(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        gw.execute(req(gw, clock, request_id="x", seq=50, token="wrong"))
        self.assertTrue(gw.execute(req(gw, clock, request_id="y", seq=5)).ok)

    def test_malformed_ids_are_rejected(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for rid in ("", None, 123, "x" * 200):
            r = gw.execute(req(gw, clock, request_id=rid))
            self.assertEqual(r.code, "BAD_REQUEST", rid)
        for seq in (1.5, "7", True):
            r = gw.execute(req(gw, clock, seq=seq, request_id=f"s-{seq}"))
            self.assertEqual(r.code, "BAD_REQUEST", seq)


class FailClosedTests(unittest.TestCase):
    def test_handler_exception_is_denied_without_partial_result(self):
        def boom(params):
            raise RuntimeError("simulator crashed")

        gw, clock = make_gateway(handlers={**OK_HANDLERS, "simulate_mission": boom})
        r = gw.execute(req(gw, clock, action="simulate_mission"))
        self.assertEqual((r.ok, r.code, r.result), (False, "INTERNAL_ERROR", None))

    def test_slow_handler_times_out(self):
        def slow(params):
            time.sleep(1.0)
            return {"late": True}

        gw, clock = make_gateway(handlers={**OK_HANDLERS, "simulate_mission": slow}, timeout_s=0.2)
        started = time.monotonic()
        r = gw.execute(req(gw, clock, action="simulate_mission"))
        self.assertEqual((r.ok, r.code, r.result), (False, "TIMEOUT", None))
        self.assertLess(time.monotonic() - started, 0.9)

    def test_allowlisted_action_with_no_handler_is_denied(self):
        gw, clock = make_gateway()
        r = gw.execute(req(gw, clock))
        self.assertEqual(r.code, "ACTION_NOT_AVAILABLE")

    def test_non_dict_params_are_rejected(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        for params in ([1, 2], "x", 5):
            r = gw.execute(GatewayRequest("planner", "get_drone_state", params, f"p-{params!r}", 900 + len(str(params)), clock(), "t0ken-planner-123"))
            self.assertEqual(r.code, "BAD_REQUEST", params)

    def test_handler_returning_non_finite_numbers_is_denied(self):
        gw, clock = make_gateway(handlers={**OK_HANDLERS, "get_battery_status": lambda p: {"soc": math.nan}})
        r = gw.execute(req(gw, clock, action="get_battery_status"))
        self.assertEqual((r.ok, r.code), (False, "UNSAFE_OUTPUT"))


class AuditTests(unittest.TestCase):
    def test_every_request_is_audited_without_secrets(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        gw.execute(req(gw, clock, request_id="good", seq=1))
        gw.execute(req(gw, clock, request_id="bad", seq=2, token="wrong"))
        gw.execute(req(gw, clock, request_id="nope", seq=3, action="set_motor_pwm"))
        log = gw.audit_log()
        self.assertEqual([e["decision"] for e in log], ["ALLOW", "UNAUTHENTICATED", "ACTION_NOT_ALLOWED"])
        for entry in log:
            self.assertEqual(set(entry), {"ts", "client_id", "action", "request_id", "seq", "decision", "reason"})
            self.assertNotIn("t0ken", repr(entry))

    def test_audit_log_is_a_copy(self):
        gw, clock = make_gateway(handlers=OK_HANDLERS)
        gw.execute(req(gw, clock))
        gw.audit_log().clear()
        self.assertEqual(len(gw.audit_log()), 1)


if __name__ == "__main__":
    unittest.main()
