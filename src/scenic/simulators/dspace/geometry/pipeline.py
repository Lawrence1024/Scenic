"""Build road index from the map (XODR). Map is the single source of geometry (RD-aligned)."""


def build_road_index_and_transform(map_path, utils_module):
    """Build road index from the map file. No separate RD file or coordinate transform.

    Prefer building from the driving domain's Network (lane 0 centerlines) so that
    (s,t) projection uses the same centerline as mainTrack/pitTrack (small |t|).
    Fall back to dSPACE xodr_parser (reference line) if driving-domain build fails.

    Returns (road_index, None). Second value is for API compatibility only.
    """
    if not map_path:
        return None, None
    path_str = str(map_path)
    # Prefer driving-domain lane centerlines so projection matches mainTrack
    if path_str.lower().endswith(".xodr"):
        try:
            from .driving_road_index import build_road_index_from_driving_network
            road_index = build_road_index_from_driving_network(path_str, use_cache=True)
            if road_index is not None:
                return road_index, None
        except Exception as e:
            print(f"[Geometry] Driving-domain road index skipped: {e}")
    # Fallback: dSPACE xodr_parser (reference line only — can give large |t| vs mainTrack)
    try:
        road_index = utils_module.build_xodr_sec_points(path_str)
        if road_index and road_index.get("roads"):
            print("[Geometry] Using map geometry for (s,t) projection (reference line; |t| may be large vs mainTrack)")
        return road_index, None
    except Exception as e:
        print(f"[Error] Failed to parse {map_path}: {e}")
        return None, None
