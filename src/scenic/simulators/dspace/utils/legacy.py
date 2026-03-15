# -*- coding: utf-8 -*-
"""
Legacy compatibility layer for dSPACE utilities.
Re-exports helpers which historically lived in utils.py.
"""

from ..geometry import (
    clear_collection,
    ensure_two_segments,
    activate_type,
    set_activity_constant,
    make_endless_transition,
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    find_road_id_for_position,
    get_road_name_for_id,
    project_world_to_st,
)

__all__ = [
    'clear_collection',
    'ensure_two_segments',
    'activate_type',
    'set_activity_constant',
    'make_endless_transition',
    'configure_seg0_absolute_pose',
    'configure_seg1_motion',
    'find_road_id_for_position',
    'get_road_name_for_id',
    'project_world_to_st',
]


