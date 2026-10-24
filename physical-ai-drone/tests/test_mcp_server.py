"""MCP server: tool list, annotations, calls through the gateway, scope enforcement, and a
real stdio round trip with an MCP client.
"""
import asyncio
import os
import sys
import unittest

from mcp.server.mcpserver.exceptions import ToolError
from physical_ai_drone.gateway import ALLOWED_ACTIONS
from physical_ai_drone.mcp_server import create_server

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(coro):
    return asyncio.run(coro)


class InProcessTests(unittest.TestCase):
    def test_exposes_exactly_the_allowlisted_tools_with_safe_annotations(self):
        tools = run(create_server().list_tools())
        self.assertEqual({t.name for t in tools}, set(ALLOWED_ACTIONS))
        for t in tools:
            self.assertTrue(t.annotations.read_only_hint, t.name)
            self.assertFalse(t.annotations.destructive_hint, t.name)
            self.assertFalse(t.annotations.open_world_hint, t.name)

    def test_calls_go_through_the_gateway_and_are_audited(self):
        server = create_server()
        r = run(server.call_tool("get_battery_status", {}))
        self.assertFalse(r.is_error)
        self.assertAlmostEqual(r.structured_content["soc"], 0.9)
        r = run(server.call_tool("plan_mission", {"goal": [2000.0, 0.0], "altitude_m": 60.0, "airspeed_kmh": 36.0}))
        self.assertTrue(r.structured_content["feasible"])
        log = server.gateway.audit_log()
        self.assertEqual([e["action"] for e in log], ["get_battery_status", "plan_mission"])
        self.assertEqual([e["seq"] for e in log], [1, 2])
        self.assertEqual(len({e["request_id"] for e in log}), 2)

    def test_read_only_scope_blocks_simulation_tools(self):
        server = create_server(scopes={"read"})
        self.assertFalse(run(server.call_tool("get_drone_state", {})).is_error)
        with self.assertRaisesRegex(ToolError, "FORBIDDEN"):
            run(server.call_tool("simulate_mission", {"goal": [2000.0, 0.0], "altitude_m": 60.0, "airspeed_kmh": 36.0}))

    def test_invalid_parameters_are_refused(self):
        server = create_server()
        with self.assertRaisesRegex(ToolError, "INVALID_PARAMS"):
            run(server.call_tool("plan_mission", {"goal": [2000.0, 0.0], "altitude_m": 500.0, "airspeed_kmh": 36.0}))
        with self.assertRaises(ToolError):
            run(server.call_tool("plan_mission", {"goal": "B", "altitude_m": 60.0, "airspeed_kmh": 36.0}))

    def test_unknown_tool_is_refused(self):
        with self.assertRaises(ToolError):
            run(create_server().call_tool("set_motor_pwm", {"duty": 100}))


class StdioRoundTripTests(unittest.TestCase):
    def test_a_real_mcp_client_can_list_and_call_tools_over_stdio(self):
        from mcp import ClientSession, StdioServerParameters
        from mcp.client.stdio import stdio_client

        async def go():
            params = StdioServerParameters(command=sys.executable, args=["-m", "physical_ai_drone.mcp_server"], cwd=HERE)
            async with stdio_client(params) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    names = {t.name for t in (await session.list_tools()).tools}
                    state = await session.call_tool("get_drone_state", {})
                    denied = await session.call_tool("plan_mission", {"goal": [2000.0, 0.0], "altitude_m": 500.0,
                                                                      "airspeed_kmh": 36.0})
                    return names, state, denied

        names, state, denied = run(go())
        self.assertEqual(names, set(ALLOWED_ACTIONS))
        self.assertFalse(state.is_error)
        self.assertEqual(state.structured_content["mode"], "SIMULATION")
        self.assertTrue(denied.is_error)
        self.assertIn("INVALID_PARAMS", denied.content[0].text)

    def test_launcher_works_from_any_working_directory(self):
        from mcp import ClientSession, StdioServerParameters
        from mcp.client.stdio import stdio_client

        async def go():
            params = StdioServerParameters(command=sys.executable, args=[os.path.join(HERE, "run_mcp_server.py")],
                                           cwd=os.path.expanduser("~"))
            async with stdio_client(params) as (read, write):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    return await session.call_tool("get_battery_status", {})

        result = run(go())
        self.assertFalse(result.is_error)
        self.assertAlmostEqual(result.structured_content["soc"], 0.9)

    def test_http_transport_is_refused(self):
        import subprocess

        p = subprocess.run([sys.executable, "-m", "physical_ai_drone.mcp_server", "streamable-http"], cwd=HERE,
                           capture_output=True, text=True, timeout=60)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("Only stdio", p.stderr)


if __name__ == "__main__":
    unittest.main()
