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


def attach_ttl(sim, obj, vehicle_type="vehicle"):
    """Load TTL based on scene params or object properties and attach region/waypoints to object.
    
    Args:
        sim: Simulation object
        obj: Scenic object (ego or fellow) to attach TTL to
        vehicle_type: String identifier for logging ("ego", "fellow", etc.)
    
    TTL configuration priority:
    1. Object-specific properties (obj.ttlIndex, obj.ttlDX, obj.ttlDY, obj.ttlFolder, obj.ttlFileName)
    2. Scene parameters (ttlIndex, ttlDX, ttlDY, ttlFolder, ttlFileName)
    3. Default values (index=17, dx=-53.6, dy=-15.7)
    """
    try:
        # Check for object-specific TTL configuration
        scene_params = getattr(sim.scene, "params", {}) or {}
        
        # Priority 1: Object-specific properties
        if hasattr(obj, 'ttlIndex') or hasattr(obj, 'ttlDX') or hasattr(obj, 'ttlDY') or \
           hasattr(obj, 'ttlFolder') or hasattr(obj, 'ttlFileName'):
            # Build config from object properties, falling back to scene params
            ttl_folder = getattr(obj, 'ttlFolder', scene_params.get("ttlFolder", None))
            ttl_index = getattr(obj, 'ttlIndex', scene_params.get("ttlIndex", 17))
            dx = getattr(obj, 'ttlDX', scene_params.get("ttlDX", -53.6))
            dy = getattr(obj, 'ttlDY', scene_params.get("ttlDY", -15.7))
            ttl_file = getattr(obj, 'ttlFileName', scene_params.get("ttlFileName", None))
            
            # If folder not specified on object, use scene param or default
            if ttl_folder is None:
                repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
                ttl_folder = scene_params.get("ttlFolder", 
                    os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV", "usable"))
            
            ttl_folder = str(ttl_folder)
            ttl_index = int(ttl_index)
            dx = float(dx)
            dy = float(dy)
            ttl_file = str(ttl_file) if ttl_file else None
        else:
            # Priority 2: Scene parameters (existing behavior)
            ttl_folder, ttl_index, dx, dy, ttl_file = get_ttl_config(scene_params)
        
        region, pts = load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file)
        if region is not None:
            setattr(obj, "ttl", region)
            name = ttl_file if ttl_file else f"ttl_{ttl_index}.csv"
            print(f"[TTL] Assigned TTL PolylineRegion to {vehicle_type} ({name})")
            # Only set waypoints if they weren't already set manually (e.g., via "with waypoints")
            if pts and not hasattr(obj, "waypoints"):
                setattr(obj, "waypoints", list(pts))
                print(f"[TTL] Attached {len(pts)} TTL waypoints to {vehicle_type}")
            elif hasattr(obj, "waypoints") and obj.waypoints:
                print(f"[TTL] Preserving existing {len(obj.waypoints)} waypoints for {vehicle_type} (not overwriting with TTL CSV)")
    except Exception as e:
        print(f"[TTL] Could not assign TTL to {vehicle_type}: {e}")


def attach_to_ego(sim, obj):
    """Load TTL based on scene params and attach region/waypoints to ego object.
    
    This is a convenience wrapper for backward compatibility.
    For new code, use attach_ttl() directly.
    """
    attach_ttl(sim, obj, vehicle_type="ego")

