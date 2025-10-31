"""Geometry module for dSPACE: parsing, projection, and coordinate transformations."""

from .xodr_parser import build_xodr_sec_points
from .rd_parser import build_rd_road_index
from .projection import project_world_to_st, find_road_id_for_position
from .coordinate_transform import build_coordinate_transform, apply_coordinate_transform, save_transform, load_transform
from .utils import (
    clear_collection,
    ensure_two_segments,
    activate_type,
    set_activity_constant,
    make_endless_transition,
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    MAIN_ROAD_NAMES
)

__all__ = [
    'build_xodr_sec_points',
    'build_rd_road_index',
    'project_world_to_st',
    'find_road_id_for_position',
    'build_coordinate_transform',
    'apply_coordinate_transform',
    'save_transform',
    'load_transform',
    'clear_collection',
    'ensure_two_segments',
    'activate_type',
    'set_activity_constant',
    'make_endless_transition',
    'configure_seg0_absolute_pose',
    'configure_seg1_motion',
    'MAIN_ROAD_NAMES',
]

