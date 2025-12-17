import os


def build_road_index_and_transform(map_path, utils_module):
    """Build road index and, if available, XODR→RD coordinate transform.
    
    Returns (road_index, coordinate_transform).
    """
    road_index = None
    coordinate_transform = None
    if not map_path:
        return None, None

    rd_path = map_path.replace('.xodr', '.rd').replace('LagunaSeca', 'Laguna_Seca')
    if os.path.exists(rd_path):
        # Best case: both XODR and RD available
        try:
            from ..geometry import coordinate_transform as ct
            cache_path = rd_path.replace('.rd', '_transform.json')
            if os.path.exists(cache_path):
                print(f"[Transform] Loading cached coordinate transformation")
                coordinate_transform = ct.load_transform(cache_path)
            else:
                print(f"[Transform] Building automatic XODR→RD coordinate transformation...")
                coordinate_transform = ct.build_coordinate_transform(map_path, rd_path, num_samples=100)
                ct.save_transform(coordinate_transform, cache_path)
            # Use RD geometry for projection (after transformation)
            from ..geometry.rd_parser import build_rd_road_index
            road_index = build_rd_road_index(rd_path, step=0.5)
            print(f"[Geometry] Using RD geometry for accurate (s,t) projection")
            print(f"[Status] [OK] Full coordinate transformation pipeline active")
            return road_index, coordinate_transform
        except Exception as e:
            print(f"[Transform] Failed to build transformation: {e}")
            print(f"[Transform] Falling back to XODR-only mode (may have positioning errors)")
            try:
                road_index = utils_module.build_xodr_sec_points(map_path)
                coordinate_transform = None
                print(f"[Geometry] Using XODR geometry")
                return road_index, coordinate_transform
            except Exception as e2:
                print(f"[Error] Failed to parse {map_path}: {e2}")
                return None, None
    else:
        # Fallback: only XODR available
        try:
            road_index = utils_module.build_xodr_sec_points(map_path)
            coordinate_transform = None
            print(f"[Geometry] Using XODR geometry")
            print(f"[Warning] [WARN] No RD file found - coordinate mismatches possible (up to 34m)")
            print(f"[Hint] Place '{os.path.basename(rd_path)}' next to XODR for accurate positioning")
            return road_index, coordinate_transform
        except Exception as e:
            print(f"[Error] Failed to parse {map_path}: {e}")
            return None, None


