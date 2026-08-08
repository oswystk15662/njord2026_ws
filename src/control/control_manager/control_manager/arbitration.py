"""ROS-independent command selection policy."""

from __future__ import annotations

from enum import IntEnum


class Source(IntEnum):
    ZERO = 0
    MANUAL = 1
    AUTO = 2


def select_source(
    *, emergency_stop: bool, effective_source: int, auto_permitted: bool,
    manual_command_fresh: bool, nav_command_fresh: bool,
) -> Source:
    """Return the only source Command Arbiter is permitted to forward."""
    if emergency_stop:
        return Source.ZERO
    if effective_source == Source.MANUAL and manual_command_fresh:
        return Source.MANUAL
    if effective_source == Source.AUTO and auto_permitted and nav_command_fresh:
        return Source.AUTO
    return Source.ZERO
