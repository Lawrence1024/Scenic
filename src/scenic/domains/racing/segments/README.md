# Racing track segments

This folder holds all **segmenting logic** for the racing domain: splitting the track into curve/straight (or conventional named) segments for evaluation, logging, and per-segment or per-TTL analysis.

## Purpose

- **Deterministic segment IDs**: For the same OpenDRIVE map and curvature threshold, segment boundaries and IDs are the same every run. That allows comparing runs and TTLs by segment (e.g. “segment 6 curve”).
- **Waypoint-to-segment mapping**: Waypoints are projected onto the road centerline; each waypoint gets a `(segment_id, segment_name)` used in behaviors (e.g. `FollowRacingLineMPC`) and in logs (WAYPOINT HIT, segment labels).

## Contents

| File | Role |
|------|------|
| `segment_map.py` | Core logic: curvature-based curve/straight segments, optional Laguna Seca conventional segments, `build_waypoint_segment_map`, `get_segment_at_waypoint`, `get_segment_label`. |
| `visualize_racing_segments.py` | Standalone script to load the track, build segments, and plot them (matplotlib). Run with `python -m scenic.domains.racing.segments.visualize_racing_segments [--map PATH]`. |
| `__init__.py` | Re-exports the public API from `segment_map` so callers can `from scenic.domains.racing.segments import build_waypoint_segment_map`, etc. |

## Segment modes

1. **Curve/straight (default)**  
   Derived from centerline curvature. Where curvature exceeds `CURVATURE_THRESHOLD` (~0.015 1/m), the track is labeled “curve”; otherwise “straight”. Consecutive same-type regions are merged into segments. This yields many segments for fine-grained analysis.

2. **Conventional Laguna Seca**  
   When `use_curvature_segments` is False and `use_conventional_laguna` is True and the track has two main roads, fixed named sections (Front Straight+T1, Andretti Hairpin, Corkscrew, etc.) are used.

3. **Coarse**  
   If neither of the above applies, one segment per main racing road (segment id only, no name).

## Usage

- **From Scenic (e.g. behaviors)**:  
  `from scenic.domains.racing.segments import build_waypoint_segment_map, get_segment_at_waypoint, get_segment_label`  
  Then call `build_waypoint_segment_map(wp_list, track)` and use `get_segment_at_waypoint(wp_idx, segment_map)` / `get_segment_label(...)` for logging.

- **Visualization**:  
  From repo root:  
  `python -m scenic.domains.racing.segments.visualize_racing_segments [--map PATH] [--threshold FLOAT]`

## Notes

- Only `(x, y)` from waypoints is used; projection onto the nearest main racing road centerline gives arc length `s`, which drives segment lookup.
- Segments are **map-based**: same map and threshold → same segments across runs, enabling by-segment and per-TTL comparison.
