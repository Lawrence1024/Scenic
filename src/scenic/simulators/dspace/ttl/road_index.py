"""Build a road index from TTL reference CSVs for (s,t) projection.

The CSVs `ttl_main_road.csv` / `ttl_pitlane.csv` are the **dSPACE ModelDesk R2 / R1
route reference lines** -- racing-line-shaped polylines baked into the dSPACE project
in RD frame. They are NOT geometric road centerlines (asymmetry stdev ~8 m,
apex-cutting pattern) and NOT the optimal racing line (`ttl_optimal_xodr.csv` is a
different line ~2 m away). Verified 2026-04-26 by constant-d fellow-drive
measurement on LGS_v1 (see `project_frame_calibration` memory + `docs/frames.md`).

When `ttlFolder` is set, these polylines are used as the projection reference so the
arc-length s sent to ModelDesk via `seq.StartPosition` matches the dSPACE-side R1/R2
s=0 origin. t is lateral offset from this ModelDesk reference line, not from the
geometric track center.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

# Same filenames as placement / route preference
TTL_MAIN_ROAD_FILE = "ttl_main_road.csv"
TTL_PITLANE_FILE = "ttl_pitlane.csv"


def _waypoints_to_sec_points(pts: List[Tuple[float, ...]]) -> Optional[List[Tuple[float, float, float]]]:
    """Convert waypoints [(x,y) or (x,y,z), ...] to [(x, y, s), ...] with arc-length s."""
    if not pts or len(pts) < 2:
        return None
    out = []
    s = 0.0
    x0, y0 = float(pts[0][0]), float(pts[0][1])
    out.append((x0, y0, s))
    for i in range(1, len(pts)):
        x1, y1 = float(pts[i][0]), float(pts[i][1])
        seg_len = math.hypot(x1 - x0, y1 - y0)
        s += seg_len
        out.append((x1, y1, s))
        x0, y0 = x1, y1
    return out


def build_road_index_from_ttl(ttl_folder: str) -> Optional[Dict[str, Any]]:
    """Build a road index from TTL centerline CSVs (true track centerline).

    Use this for (s,t) projection when TTL is available so that t is lateral
    offset from the TTL centerline, regardless of how the XODR lanes/reference
    were defined.

    Args:
        ttl_folder: Path to folder containing ttl_main_road.csv and ttl_pitlane.csv.

    Returns:
        road_index dict compatible with project_world_to_st, or None on failure.
        Roads: "MainTrack_TTL" (id=1), "PitTrack_TTL" (id=2).
    """
    try:
        from .loader import load_ttl_region
    except ImportError:
        return None
    folder = str(ttl_folder)
    _, main_pts = load_ttl_region(folder, TTL_MAIN_ROAD_FILE)
    _, pit_pts = load_ttl_region(folder, TTL_PITLANE_FILE)
    if not main_pts or len(main_pts) < 2:
        return None
    main_sec = _waypoints_to_sec_points(main_pts)
    if not main_sec:
        return None
    road_index = {
        "roads": {
            "MainTrack_TTL": {
                "id": 1,
                "name": "MainTrack_TTL",
                "length": main_sec[-1][2],
                "sec_points": [main_sec],
            },
        }
    }
    if pit_pts and len(pit_pts) >= 2:
        pit_sec = _waypoints_to_sec_points(pit_pts)
        if pit_sec:
            road_index["roads"]["PitTrack_TTL"] = {
                "id": 2,
                "name": "PitTrack_TTL",
                "length": pit_sec[-1][2],
                "sec_points": [pit_sec],
            }
    print(
        "[Geometry] Using TTL centerlines for (s,t) projection (true track centerline; "
        "XODR lanes/reference ignored)"
    )
    return road_index
