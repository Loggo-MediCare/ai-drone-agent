"""Simulation-first autonomous drone agent (Chapter 16) with a deterministic safety gateway and MCP tools.

The simulation backend is the existing `drone_agent` package in the repository root.
"""
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
