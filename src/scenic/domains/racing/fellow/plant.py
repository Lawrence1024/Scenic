"""Fellow plant-mode helpers for simulators that drive traffic via (v, d) signals.

Behaviors in ``behaviors.scenic`` for **dSPACE** fellow (v, d) plants update
``_fellow_plant_v_kmh`` and ``_fellow_plant_d_m`` each step; the dSPACE controller
writes those to External_Signals (see :mod:`scenic.domains.racing.fellow.commands`).

- :obj:`FellowConstantSpeedTrackOffsetBehavior` — constant ``speed_mph`` and lateral offset
  from placement.
- :obj:`FellowFollowTTLGeometricBehavior` — constant ``speed_mph`` and lateral ``d`` from
  feedforward δ(s) on the main centerline (optimal TTL vs ``ttl_main_road``), with waypoint
  index updates via shared racing helpers.

Other simulators may ignore these unless they implement the same contract.
"""

from __future__ import annotations

from typing import Any, Optional

# Must match the Scenic ``behavior FellowConstantSpeedTrackOffsetBehavior`` name.
FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS = "FellowConstantSpeedTrackOffsetBehavior"
# Must match ``behavior FellowFollowTTLGeometricBehavior``.
FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS = "FellowFollowTTLGeometricBehavior"

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


def fellow_follow_ttl_geometric_speed_kmh(obj: Any) -> Optional[float]:
    """Target speed in km/h for :class:`FellowFollowTTLGeometricBehavior`, or ``None``."""
    b = getattr(obj, "behavior", None)
    if b is None or b.__class__.__name__ != FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS:
        return None
    try:
        mph = float(getattr(b, "speed_mph"))
    except (TypeError, ValueError):
        mph = 31.0
    return mph * _MPH_TO_KMH


def is_fellow_follow_ttl_geometric_behavior(obj: Any) -> bool:
    """True if ``obj`` has :class:`FellowFollowTTLGeometricBehavior`."""
    b = getattr(obj, "behavior", None)
    return b is not None and b.__class__.__name__ == FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS
