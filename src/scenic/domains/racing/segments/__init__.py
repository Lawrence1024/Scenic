"""Racing track segment logic for evaluation and logging.

This package provides:
- OpenDRIVE-based and TTL-based segment mapping (segment_map)
- RacingTrack and createRacingTrack (tracks) — track built from OpenDRIVE
- mainTrack/pitTrack region building from segment centerlines (track_regions),
  with 5 m and 2 m buffer from centerline, from either OpenDRIVE or TTL CSVs
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
from scenic.domains.racing.segments.tracks import (
    RacingTrack,
    createRacingTrack,
    PitLane,
    RacingLine,
)
from scenic.domains.racing.segments.track_regions import (
    create_track_regions,
    build_track_regions_from_opendrive,
    build_track_regions_from_ttl,
    MAIN_TRACK_BUFFER_M,
    PIT_TRACK_BUFFER_M,
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
    "RacingTrack",
    "createRacingTrack",
    "PitLane",
    "RacingLine",
    "create_track_regions",
    "build_track_regions_from_opendrive",
    "build_track_regions_from_ttl",
    "MAIN_TRACK_BUFFER_M",
    "PIT_TRACK_BUFFER_M",
]
