"""Traffic fellow helpers for closed-circuit racing (dSPACE (v, d) plant, TTL geometry).

This subpackage holds Python used by racing behaviors and the dSPACE simulator for
fellow vehicles driven via external speed/lateral commands—not ego control.

Submodules
----------

* :mod:`scenic.domains.racing.fellow.commands` — compute fellow plant commands (``compute_*``)
  and low-level helpers for staged plant state.
"""

from __future__ import annotations

from scenic.domains.racing.fellow.commands import (
    compute_always_faster_plant_command,
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
)
__all__ = [
    "compute_always_faster_plant_command",
    "compute_constant_offset_plant_command",
    "compute_fellow_swerve_out_of_control_command",
    "compute_fellow_ttl_geometric_d_m",
    "compute_follow_ttl_geometric_plant_command",
    "compute_sudden_stop_plant_command",
    "get_fellow_placed_lateral_deviation",
    "get_fellow_plant_d_m",
    "get_fellow_plant_v_kmh",
    "mph_to_kmh",
    "set_fellow_plant_d_m",
    "set_fellow_plant_v_kmh",
    "sudden_stop_v_kmh",
]
