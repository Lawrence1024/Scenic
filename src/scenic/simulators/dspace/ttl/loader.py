import os
import csv
from datetime import datetime, timezone

from scenic.core.regions import PolylineRegion


def _default_ttl_folder() -> str:
    """Absolute path to repo assets/ttls/LS_ENU_TTL_CSV (named CSVs only; no indexed ttl_N.csv)."""
    _here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(_here, "..", "..", "..", "..", ".."))
    return os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV")


def get_ttl_config(scene_params):
    """Build TTL configuration from scene params with sensible defaults.

    Returns: (ttl_folder, ttl_file_name) — filename is a basename like ``ttl_main_road.csv``.
    """
    params = scene_params or {}
    ttl_folder = params.get("ttlFolder") or _default_ttl_folder()
    ttl_file = params.get("ttlFileName") or "ttl_main_road.csv"
    return str(ttl_folder), str(ttl_file)


def load_ttl_region(ttl_folder, ttl_file_name):
    """Load TTL CSV and return (PolylineRegion, waypoints). No offset applied (TTL files are in map/XODR coordinates).

    Args:
        ttl_folder: Path to folder containing TTL CSV files
        ttl_file_name: Basename of the CSV (e.g. ``ttl_main_road.csv``). Empty/None defaults to ``ttl_main_road.csv``.

    Returns:
        (PolylineRegion, list_of_waypoints) or (None, []) if loading fails
    """
    if not ttl_file_name:
        ttl_file_name = "ttl_main_road.csv"
    ttl_path = os.path.join(str(ttl_folder), ttl_file_name)
    if not os.path.exists(ttl_path):
        print(f"[TTL] File not found: {ttl_path}")
        return None, []

    pts = []
    with open(ttl_path, newline="") as f:
        r = csv.reader(f)
        # Check if first line is header (x,y,z or similar)
        has_header = False
        try:
            first_line = next(r)
            # If first line looks like a header (contains 'x' or 'X'), skip it
            if len(first_line) > 0 and ('x' in first_line[0].lower() or 'X' in first_line[0]):
                has_header = True
                # Check if header indicates 3D (has 'z' column)
                has_z_column = any('z' in col.lower() for col in first_line)
            else:
                # Not a header, process it as data
                has_header = False
                has_z_column = len(first_line) >= 3  # Assume 3D if 3+ columns
                if len(first_line) >= 2:
                    try:
                        x = float(first_line[0])
                        y = float(first_line[1])
                        if has_z_column and len(first_line) >= 3:
                            z = float(first_line[2])
                            pts.append((x, y, z))
                        else:
                            pts.append((x, y))
                    except (ValueError, IndexError):
                        pass
        except StopIteration:
            has_z_column = False

        # Process remaining rows
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                # Support 3D waypoints if z coordinate is available
                # If CSV has no z column (len(row) < 3), create 2D waypoint (z implicitly 0)
                if len(row) >= 3:
                    z = float(row[2])
                    pts.append((x, y, z))
                else:
                    # No z column: create 2D waypoint (assumes z = 0 for flat surface)
                    pts.append((x, y))
            except (ValueError, IndexError):
                continue

    if len(pts) < 2:
        print(f"[TTL] Not enough points in {ttl_path}")
        return None, []

    print(f"[TTL] Loaded {len(pts)} waypoints from {os.path.basename(ttl_path)}")
    # PolylineRegion expects 2D points to avoid "nonzero Z components" warnings
    pts_2d = [(p[0], p[1]) for p in pts]
    return PolylineRegion(pts_2d), pts


def attach_ttl(sim, obj, vehicle_type="vehicle"):
    """Load TTL based on scene params or object properties and attach region/waypoints to object.

    TTL configuration priority:
    1. Object-specific properties (obj.ttlFolder, obj.ttlFileName)
    2. Scene parameters (ttlFolder, ttlFileName)
    """
    try:
        scene_params = getattr(sim.scene, "params", {}) or {}

        obj_folder = getattr(obj, "ttlFolder", None)
        obj_file = getattr(obj, "ttlFileName", None)
        if obj_folder is not None or obj_file is not None:
            ttl_folder = obj_folder or scene_params.get("ttlFolder") or _default_ttl_folder()
            ttl_file = obj_file or scene_params.get("ttlFileName") or "ttl_main_road.csv"
            ttl_folder = str(ttl_folder)
            ttl_file = str(ttl_file)
        else:
            ttl_folder, ttl_file = get_ttl_config(scene_params)

        region, pts = load_ttl_region(ttl_folder, ttl_file)
        if region is not None:
            setattr(obj, "ttl", region)
            name = os.path.basename(ttl_file)
            print(f"[TTL] Assigned TTL PolylineRegion to {vehicle_type} ({name})")
            # Log run identifier for analysis scripts (TTL, timestamp)
            if vehicle_type == "ego":
                ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                print(f"[RacingRun] TTL={name} run_timestamp={ts}")
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
