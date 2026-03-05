"""Racing track segment logic for evaluation and logging.

This package provides OpenDRIVE-based segment mapping: curve/straight segments
from centerline curvature, optional Laguna Seca conventional sections, and
waypoint-to-segment lookup. See segment_map module and README.md for details.
"""

from scenic.domains.racing.segments.segment_map import (
    CURVATURE_THRESHOLD,
    LAGUNA_SECA_SEGMENTS,
    build_waypoint_segment_map,
    build_waypoint_segment_map_from_ttl,
    get_pit_transitions,
    get_ring_segment_ids,
    get_segment_at_waypoint,
    get_segment_at_waypoint_ring_strict,
    get_segment_label,
    get_segment_sequences,
    position_nearest_road_is_pit,
)

__all__ = [
    "CURVATURE_THRESHOLD",
    "LAGUNA_SECA_SEGMENTS",
    "build_waypoint_segment_map",
    "build_waypoint_segment_map_from_ttl",
    "get_pit_transitions",
    "get_ring_segment_ids",
    "get_segment_at_waypoint",
    "get_segment_at_waypoint_ring_strict",
    "get_segment_label",
    "get_segment_sequences",
    "position_nearest_road_is_pit",
]
