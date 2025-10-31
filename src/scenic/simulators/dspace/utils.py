# -*- coding: utf-8 -*-
"""
Legacy compatibility layer for dSPACE utilities.
All functions have been moved to dedicated modules:
- Geometry functions: geometry/
- ModelDesk functions: modeldesk/
- ControlDesk functions: controldesk/

This file re-exports for backward compatibility.
"""

# Re-export geometry functions for backward compatibility
from .geometry import (
    clear_collection,
    ensure_two_segments,
    activate_type,
    set_activity_constant,
    make_endless_transition,
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    find_road_id_for_position,
    build_xodr_sec_points,
    project_world_to_st,
    MAIN_ROAD_NAMES,
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
    'build_xodr_sec_points',
    'project_world_to_st',
    'MAIN_ROAD_NAMES',
]
