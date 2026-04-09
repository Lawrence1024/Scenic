"""Canonical ego control staging for the racing domain (matches driving Steers).

The driving domain stages per-step commands via the :class:`~scenic.domains.driving.actions.Steers`
protocol: :meth:`setThrottle`, :meth:`setBraking`, :meth:`setSteering` populate
``agent._control_state`` with keys ``throttle``, ``braking``, ``steering``.

Racing behaviors and helpers should prefer those methods over writing ``_control_state``
directly so ego output stays structurally identical to the driving domain.
"""

from __future__ import annotations

from typing import Any


def apply_steering_command(obj: Any, steer: float) -> None:
    """Apply a steering command the same way as :class:`~scenic.domains.driving.actions.SetSteerAction`.

    Uses :meth:`setSteering` when available (populates ``_control_state['steering']`` on
    dSPACE and other Steers implementations); otherwise writes ``_control_state`` directly.
    """
    if hasattr(obj, "setSteering"):
        obj.setSteering(float(steer))
    else:
        if not hasattr(obj, "_control_state"):
            obj._control_state = {}
        obj._control_state["steering"] = float(steer)
