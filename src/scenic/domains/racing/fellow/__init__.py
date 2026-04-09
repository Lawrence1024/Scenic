"""Traffic fellow helpers for closed-circuit racing (dSPACE (v, d) plant, TTL geometry).

This subpackage holds Python used by racing behaviors and the dSPACE simulator for
fellow vehicles driven via external speed/lateral commands—not ego control.

Submodules
----------

* :mod:`scenic.domains.racing.fellow.plant` — detect fellow plant behaviors and read
  target speed from Scenic behavior instances.
* :mod:`scenic.domains.racing.fellow.commands` — compute ``_fellow_plant_state`` (``v_kmh``,
  ``d_m``) each step for plant behaviors (constant offset, TTL geometric,
  sudden-stop, swerve-out-of-control). Scenario behaviors: ``FellowSuddenStopIntervalBehavior``
  (``examples/combined/fellow_sudden_stop.scenic``) and ``FellowSwerveOutOfControlBehavior``
  (defaults in ``behaviors.scenic`` match ``examples/combined/fellow_swerve_out_of_control.scenic``).
"""

from __future__ import annotations

from scenic.domains.racing.fellow.commands import (
    get_fellow_placed_lateral_deviation,
    get_fellow_plant_d_m,
    get_fellow_plant_v_kmh,
    set_fellow_plant_d_m,
    set_fellow_plant_v_kmh,
    update_fellow_constant_speed_track_offset_plant,
    update_fellow_follow_ttl_geometric_plant,
    update_fellow_sudden_stop_interval_plant,
    update_fellow_swerve_out_of_control_plant,
)
from scenic.domains.racing.fellow.plant import (
    FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS,
    FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS,
    FELLOW_SUDDEN_STOP_INTERVAL_CLASS,
    FELLOW_SWERVE_OUT_OF_CONTROL_CLASS,
    fellow_constant_speed_kmh_from_behavior,
    fellow_follow_ttl_geometric_speed_kmh,
    is_fellow_constant_speed_track_offset_behavior,
    is_fellow_follow_ttl_geometric_behavior,
    is_fellow_sudden_stop_interval_behavior,
    is_fellow_swerve_out_of_control_behavior,
)

__all__ = [
    "FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS",
    "FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS",
    "FELLOW_SUDDEN_STOP_INTERVAL_CLASS",
    "FELLOW_SWERVE_OUT_OF_CONTROL_CLASS",
    "fellow_constant_speed_kmh_from_behavior",
    "fellow_follow_ttl_geometric_speed_kmh",
    "get_fellow_placed_lateral_deviation",
    "get_fellow_plant_d_m",
    "get_fellow_plant_v_kmh",
    "set_fellow_plant_d_m",
    "set_fellow_plant_v_kmh",
    "is_fellow_constant_speed_track_offset_behavior",
    "is_fellow_follow_ttl_geometric_behavior",
    "is_fellow_sudden_stop_interval_behavior",
    "is_fellow_swerve_out_of_control_behavior",
    "update_fellow_constant_speed_track_offset_plant",
    "update_fellow_follow_ttl_geometric_plant",
    "update_fellow_sudden_stop_interval_plant",
    "update_fellow_swerve_out_of_control_plant",
]
