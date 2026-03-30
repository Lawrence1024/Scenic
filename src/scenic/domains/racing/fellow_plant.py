"""Fellow plant-mode helpers for simulators that drive traffic via (v, d) signals.

The :obj:`FellowConstantSpeedTrackOffsetBehavior` in ``behaviors.scenic`` is intended
primarily for **dSPACE** (External_Signals: speed from ``speed_mph``, converted to km/h
for the plant; lateral offset fixed from placement). Other simulators may ignore it unless
they implement the same contract.
"""

from __future__ import annotations

from typing import Any, Optional

# Must match the Scenic ``behavior FellowConstantSpeedTrackOffsetBehavior`` name.
FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS = "FellowConstantSpeedTrackOffsetBehavior"

# International mile (exact): 1 mi = 1.609344 km
_MPH_TO_KMH = 1.609344


def fellow_constant_speed_kmh_from_behavior(obj: Any) -> Optional[float]:
    """Return target speed in km/h for the dSPACE plant if ``obj`` uses constant-speed fellow mode.

    Reads **speed_mph** from :class:`FellowConstantSpeedTrackOffsetBehavior` and converts
    to km/h for ``Const_v_Fellows_External``.

    Returns:
        Speed in km/h, or ``None`` if not that behavior.
    """
    b = getattr(obj, "behavior", None)
    if b is None:
        return None
    if b.__class__.__name__ != FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS:
        return None
    try:
        mph = float(getattr(b, "speed_mph"))
    except (TypeError, ValueError):
        mph = 31.0
    return mph * _MPH_TO_KMH


def is_fellow_constant_speed_track_offset_behavior(obj: Any) -> bool:
    """True if ``obj`` has :class:`FellowConstantSpeedTrackOffsetBehavior`."""
    b = getattr(obj, "behavior", None)
    return b is not None and b.__class__.__name__ == FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS
