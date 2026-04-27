"""Geometry module for dSPACE: parsing, projection, and coordinate transformations."""

from .xodr_parser import build_xodr_sec_points
from .rd_parser import build_rd_road_index
from .projection import project_world_to_st, find_road_id_for_position
# CC-2 (2026-04-26): coordinate_transform.py deleted; was unused
# (_coordinate_transform = None in simulator.py:656).
from .utils import (
    clear_collection,
    ensure_two_segments,
    activate_type,
    set_activity_constant,
    make_endless_transition,
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    get_road_name_for_id,
    MAIN_ROAD_NAMES
)

__all__ = [
    'build_xodr_sec_points',
    'build_rd_road_index',
    'project_world_to_st',
    'find_road_id_for_position',
    'clear_collection',
    'ensure_two_segments',
    'activate_type',
    'set_activity_constant',
    'make_endless_transition',
    'configure_seg0_absolute_pose',
    'configure_seg1_motion',
    'get_road_name_for_id',
    'MAIN_ROAD_NAMES',
]

