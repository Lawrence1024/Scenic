import os
import csv
from scenic.core.regions import PolylineRegion


def get_ttl_config(scene_params):
    """Build TTL configuration from scene params with sensible defaults.
    
    Returns: (ttl_folder, ttl_index, dx, dy, ttl_file_name_or_None)
    """
    params = scene_params or {}
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    default_folder = os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV", "usable")
    ttl_folder = params.get("ttlFolder", default_folder)
    ttl_index = int(params.get("ttlIndex", 17))  # default to 17
    dx = float(params.get("ttlDX", -53.6))
    dy = float(params.get("ttlDY", -15.7))
    ttl_file = params.get("ttlFileName", None)
    return ttl_folder, ttl_index, dx, dy, ttl_file


def load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file_name=None):
    """Load TTL CSV, apply (dx, dy), return (PolylineRegion, waypoints).
    
    If ttl_file_name is provided, use that exact file; otherwise uses ttl_{index}.csv.
    """
    ttl_path = os.path.join(ttl_folder, ttl_file_name) if ttl_file_name else os.path.join(ttl_folder, f"ttl_{ttl_index}.csv")
    if not os.path.exists(ttl_path):
        print(f"[TTL] File not found: {ttl_path}")
        return None, []
    pts = []
    with open(ttl_path, newline="") as f:
        r = csv.reader(f)
        try:
            next(r)  # skip metadata
        except StopIteration:
            pass
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                pts.append((x, y))
            except Exception:
                continue
    if len(pts) < 2:
        print(f"[TTL] Not enough points in {ttl_path}")
        return None, []
    return PolylineRegion(pts), pts


def attach_to_ego(sim, obj):
    """Load TTL based on scene params and attach region/waypoints to ego object."""
    try:
        ttl_folder, ttl_index, dx, dy, ttl_file = get_ttl_config(getattr(sim.scene, "params", {}) or {})
        region, pts = load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file)
        if region is not None:
            setattr(obj, "ttl", region)
            name = ttl_file if ttl_file else f"ttl_{ttl_index}.csv"
            print(f"[TTL] Assigned TTL PolylineRegion to ego vehicle ({name})")
            if pts:
                setattr(obj, "waypoints", list(pts))
                print(f"[TTL] Attached {len(pts)} TTL waypoints to ego")
    except Exception as e:
        print(f"[TTL] Could not assign TTL to ego: {e}")

