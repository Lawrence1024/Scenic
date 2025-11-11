def get_map_path(scene_params):
    """Return map path from typical Scenic params (map/opendrive/xodr)."""
    try:
        prm = scene_params or {}
        for key in ("map", "opendrive", "xodr"):
            if key in prm and prm[key]:
                return str(prm[key])
    except Exception:
        pass
    return None


