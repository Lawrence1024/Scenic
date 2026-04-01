"""Traffic fellow helpers for closed-circuit racing (dSPACE (v, d) plant, TTL geometry).

This subpackage holds Python used by racing behaviors and the dSPACE simulator for
fellow vehicles driven via external speed/lateral commands—not ego control.

Submodules
----------

* :mod:`scenic.domains.racing.fellow.plant` — detect fellow plant behaviors and read
  target speed from Scenic behavior instances.
* :mod:`scenic.domains.racing.fellow.commands` — compute ``_fellow_plant_v_kmh`` and
  ``_fellow_plant_d_m`` each step for ``FellowConstantSpeedTrackOffsetBehavior`` and
  ``FellowFollowTTLGeometricBehavior`` (see ``behaviors.scenic``).
"""

from __future__ import annotations

from scenic.domains.racing.fellow.commands import (
    get_fellow_placed_lateral_deviation,
    update_fellow_constant_speed_track_offset_plant,
    update_fellow_follow_ttl_geometric_plant,
)
from scenic.domains.racing.fellow.plant import (
    FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS,
    FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS,
    fellow_constant_speed_kmh_from_behavior,
    fellow_follow_ttl_geometric_speed_kmh,
    is_fellow_constant_speed_track_offset_behavior,
    is_fellow_follow_ttl_geometric_behavior,
)

__all__ = [
    "FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS",
    "FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS",
    "fellow_constant_speed_kmh_from_behavior",
    "fellow_follow_ttl_geometric_speed_kmh",
    "get_fellow_placed_lateral_deviation",
    "is_fellow_constant_speed_track_offset_behavior",
    "is_fellow_follow_ttl_geometric_behavior",
    "update_fellow_constant_speed_track_offset_plant",
    "update_fellow_follow_ttl_geometric_plant",
]
