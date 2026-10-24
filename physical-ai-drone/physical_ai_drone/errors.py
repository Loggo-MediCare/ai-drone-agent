"""Errors that tools raise on purpose. The gateway maps them to their own codes."""


class InvalidParams(ValueError):
    """The caller's parameters failed validation. Nothing was executed."""

    code = "INVALID_PARAMS"


class InconsistentState(RuntimeError):
    """The backend reported a state that cannot be true (NaN, SoC outside 0..1, ...)."""

    code = "INCONSISTENT_STATE"
