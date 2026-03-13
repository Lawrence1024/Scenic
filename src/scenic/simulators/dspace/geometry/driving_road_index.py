"""Build road index from the driving domain's OpenDRIVE network.

For (s,t) projection we prefer TTL centerlines (see ttl.road_index) when available.
Otherwise we use the geometric midline (average of left and right boundaries) from
the driving domain so the centerline is the true track center regardless of XODR
lanes/reference.
"""

import math
from typing import Any, Dict, List, Optional, Tuple


def _centerline_to_sec_points(centerline: Any) -> Optional[List[Tuple[float, float, float]]]:
    """Convert a PolylineRegion or lineString to [(x, y, s), ...] with arc-length s."""
    try:
        if hasattr(centerline, "lineString"):
            ls = centerline.lineString
        else:
            return None
        coords = list(ls.coords)
        if len(coords) < 2:
            return None
        pts = []
        s = 0.0
        x0, y0 = float(coords[0][0]), float(coords[0][1])
        pts.append((x0, y0, s))
        for i in range(1, len(coords)):
            x1, y1 = float(coords[i][0]), float(coords[i][1])
            seg_len = math.hypot(x1 - x0, y1 - y0)
            s += seg_len
            pts.append((x1, y1, s))
            x0, y0 = x1, y1
        return pts
    except Exception:
        return None


def build_road_index_from_driving_network(map_path: str, use_cache: bool = True) -> Optional[Dict[str, Any]]:
    """Build a road index from the driving domain Network using geometric midline.

    Uses the same centerline as mainTrack/pitTrack: the midline (average of left
    and right boundaries) when available, so (s,t) projection has t ≈ 0 on the
    true track center regardless of XODR lanes/reference.

    Args:
        map_path: Path to .xodr map file.
        use_cache: Use cached network if available (default True).

    Returns:
        road_index dict compatible with project_world_to_st, or None on failure.
        Structure: {'roads': {road_name: {'id', 'name', 'length', 'sec_points': [[(x,y,s), ...]]}}}
    """
    try:
        from scenic.domains.driving.roads import Network
        from scenic.domains.racing.segments.segment_map import _get_road_centerline
    except ImportError:
        return None
    try:
        path = str(map_path)
        if not path.lower().endswith(".xodr"):
            return None
        # Use cache so we don't re-parse if the scenario already loaded the map
        network = Network.fromFile(path, useCache=use_cache, writeCache=False)
        if not network or not getattr(network, "allRoads", None):
            return None
        roads_list = list(network.allRoads)
        if not roads_list:
            return None

        road_index = {"roads": {}}
        for road in roads_list:
            name = getattr(road, "name", None) or f"Road_{getattr(road, 'id', id(road))}"
            road_id = getattr(road, "id", None)
            # Use same centerline as mainTrack/pitTrack: midline (avg of left/right) when available
            centerline = _get_road_centerline(road)
            if centerline is None:
                continue
            pts = _centerline_to_sec_points(centerline)
            if not pts or len(pts) < 2:
                continue
            length = pts[-1][2]  # s at last point
            road_index["roads"][name] = {
                "id": road_id,
                "name": name,
                "length": length,
                "sec_points": [pts],
            }
        if not road_index["roads"]:
            return None
        print(
            f"[Geometry] Using driving-domain midline (avg left/right boundaries) for (s,t) projection "
            f"({len(road_index['roads'])} roads)"
        )
        return road_index
    except Exception as e:
        print(f"[Geometry] build_road_index_from_driving_network failed: {e}")
        return None
