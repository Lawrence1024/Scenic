"""Traffic fellow helpers for closed-circuit racing (dSPACE (v, d) plant, TTL geometry).

This subpackage holds Python used by racing behaviors and the dSPACE simulator for
fellow vehicles driven via external speed/lateral commands—not ego control.

Submodules
----------

* :mod:`scenic.domains.racing.fellow.plant` — detect fellow (v, d) plant behaviors
  (:func:`is_fellow_vd_plant_behavior`).
* :mod:`scenic.domains.racing.fellow.commands` — compute fellow plant commands (``compute_*``)
  and optional ``update_fellow_*_plant`` writers for non–Action call sites.
"""

from __future__ import annotations

from scenic.domains.racing.fellow.commands import (
    compute_constant_offset_plant_command,
    compute_fellow_swerve_out_of_control_command,
    compute_fellow_ttl_geometric_d_m,
    compute_follow_ttl_geometric_plant_command,
    compute_sudden_stop_plant_command,
    get_fellow_placed_lateral_deviation,
    get_fellow_plant_d_m,
    get_fellow_plant_v_kmh,
    mph_to_kmh,
    set_fellow_plant_d_m,
    set_fellow_plant_v_kmh,
    sudden_stop_v_kmh,
    update_fellow_constant_speed_track_offset_plant,
    update_fellow_follow_ttl_geometric_plant,
    update_fellow_sudden_stop_interval_plant,
    update_fellow_swerve_out_of_control_plant,
)
from scenic.domains.racing.fellow.plant import is_fellow_vd_plant_behavior

__all__ = [
    "compute_constant_offset_plant_command",
    "compute_fellow_swerve_out_of_control_command",
    "compute_fellow_ttl_geometric_d_m",
    "compute_follow_ttl_geometric_plant_command",
    "compute_sudden_stop_plant_command",
    "get_fellow_placed_lateral_deviation",
    "get_fellow_plant_d_m",
    "get_fellow_plant_v_kmh",
    "is_fellow_vd_plant_behavior",
    "mph_to_kmh",
    "set_fellow_plant_d_m",
    "set_fellow_plant_v_kmh",
    "sudden_stop_v_kmh",
    "update_fellow_constant_speed_track_offset_plant",
    "update_fellow_follow_ttl_geometric_plant",
    "update_fellow_sudden_stop_interval_plant",
    "update_fellow_swerve_out_of_control_plant",
]
