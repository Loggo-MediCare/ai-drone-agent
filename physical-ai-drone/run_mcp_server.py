"""Starts the MCP server from any working directory (MCP clients may launch it from anywhere).

    claude mcp add physical-ai-drone -- /abs/path/.venv/bin/python /abs/path/run_mcp_server.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from physical_ai_drone.mcp_server import main  # noqa: E402

if __name__ == "__main__":
    main()
