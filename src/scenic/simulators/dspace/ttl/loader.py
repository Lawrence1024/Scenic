import os
import csv
from datetime import datetime, timezone
from pathlib import Path

import yaml

from scenic.core.regions import PolylineRegion


def _get_mpc_run_edit_note():
    """Read run_edit_note from vehicle_mpc.yaml so [RacingRun] edit_note identifies tuning for analyze_racing_log."""
    try:
        # Resolve path: this file is in .../simulators/dspace/ttl/loader.py -> scenic pkg root is parent^4
        pkg_root = Path(__file__).resolve().parent.parent.parent.parent
        mpc_yaml = pkg_root / "domains" / "racing" / "mpc" / "vehicle_mpc.yaml"
        if not mpc_yaml.exists():
            return ""
        with open(mpc_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not data:
            return ""
        params = data.get("/**:") or data.get("/**") or data
        if isinstance(params, dict):
            params = params.get("ros__parameters") or params
        note = (params or {}).get("run_edit_note") or ""
        return str(note).strip().replace("\n", " ")[:200] if note else ""
    except Exception:
        return ""


def get_ttl_config(scene_params):
    """Build TTL configuration from scene params with sensible defaults.
    
    Automatically detects if TTL files are in the 'transformed' folder (already in XODR coordinates)
    and sets offset to (0, 0) in that case. Otherwise uses default offset.
    
    Returns: (ttl_folder, ttl_index, dx, dy, ttl_file_name_or_None)
    """
    params = scene_params or {}
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    default_folder = os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV", "transformed")
    ttl_folder = params.get("ttlFolder", default_folder)
    ttl_index = int(params.get("ttlIndex", 17))  # default to 17
    
    # Check if folder is the 'transformed' folder (files are already in XODR coordinates)
    ttl_folder_str = str(ttl_folder).replace("\\", "/")
    is_transformed = "transformed" in ttl_folder_str.lower()
    
    # If in transformed folder, use zero offset (files are already in XODR space)
    # Otherwise, use default offset for files that need transformation
    if is_transformed:
        default_dx = 0.0
        default_dy = 0.0
    else:
        default_dx = -53.6
        default_dy = -15.7
    
    # Allow explicit override via params
    dx = float(params.get("ttlDX", default_dx))
    dy = float(params.get("ttlDY", default_dy))
    ttl_file = params.get("ttlFileName", None)
    
    if is_transformed and (dx != 0.0 or dy != 0.0):
        print(f"[TTL] Note: Using 'transformed' folder but offset is ({dx}, {dy}). "
              f"Files in 'transformed' folder are already in XODR coordinates, "
              f"so offset should typically be (0, 0).")
    
    return ttl_folder, ttl_index, dx, dy, ttl_file


def load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file_name=None):
    """Load TTL CSV, apply (dx, dy) offset, return (PolylineRegion, waypoints).
    
    If ttl_file_name is provided, use that exact file; otherwise uses ttl_{index}.csv.
    
    Note: Files in the 'transformed' folder are already in XODR coordinates and should
    use offset (0, 0). Files in other folders may need offset transformation.
    
    Args:
        ttl_folder: Path to folder containing TTL CSV files
        ttl_index: Index of TTL file (e.g., 17 for ttl_17.csv)
        dx: X offset to apply (typically 0.0 for transformed files, -53.6 for others)
        dy: Y offset to apply (typically 0.0 for transformed files, -15.7 for others)
        ttl_file_name: Optional specific filename (overrides ttl_index)
    
    Returns:
        (PolylineRegion, list_of_waypoints) or (None, []) if loading fails
    """
    ttl_path = os.path.join(ttl_folder, ttl_file_name) if ttl_file_name else os.path.join(ttl_folder, f"ttl_{ttl_index}.csv")
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
                        x = float(first_line[0]) + dx
                        y = float(first_line[1]) + dy
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
                x = float(row[0]) + dx
                y = float(row[1]) + dy
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
    
    # Log coordinate system info
    folder_str = str(ttl_folder).replace("\\", "/")
    if "transformed" in folder_str.lower():
        coord_system = "XODR (already transformed)"
    else:
        coord_system = "ENU/RD (with offset applied)"
    
    print(f"[TTL] Loaded {len(pts)} waypoints from {os.path.basename(ttl_path)} "
          f"(offset: ({dx}, {dy}), coordinate system: {coord_system})")
    
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
    3. Auto-detected defaults:
       - If folder contains 'transformed': offset (0, 0) - files already in XODR coordinates
       - Otherwise: offset (-53.6, -15.7) - files need transformation
       - Default index: 17
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
            ttl_file = getattr(obj, 'ttlFileName', scene_params.get("ttlFileName", None))
            
            # If folder not specified on object, use scene param or default
            if ttl_folder is None:
                repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
                ttl_folder = scene_params.get("ttlFolder", 
                    os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV", "transformed"))
            
            ttl_folder = str(ttl_folder)
            ttl_index = int(ttl_index)
            
            # Auto-detect offset based on folder (transformed = 0,0; others = default)
            folder_str = ttl_folder.replace("\\", "/")
            is_transformed = "transformed" in folder_str.lower()
            
            # Get offset, with auto-detection if not explicitly set
            if hasattr(obj, 'ttlDX'):
                dx = float(getattr(obj, 'ttlDX'))
            else:
                dx = float(scene_params.get("ttlDX", 0.0 if is_transformed else -53.6))
            
            if hasattr(obj, 'ttlDY'):
                dy = float(getattr(obj, 'ttlDY'))
            else:
                dy = float(scene_params.get("ttlDY", 0.0 if is_transformed else -15.7))
            
            ttl_file = str(ttl_file) if ttl_file else None
        else:
            # Priority 2: Scene parameters (existing behavior)
            ttl_folder, ttl_index, dx, dy, ttl_file = get_ttl_config(scene_params)
        
        region, pts = load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file)
        if region is not None:
            setattr(obj, "ttl", region)
            name = ttl_file if ttl_file else f"ttl_{ttl_index}.csv"
            print(f"[TTL] Assigned TTL PolylineRegion to {vehicle_type} ({name})")
            # Log run identifier for analysis scripts (TTL, timestamp, optional edit_note for config/ideas)
            if vehicle_type == "ego":
                ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
                edit_note = str(scene_params.get("edit_note", "") or scene_params.get("editNote", "")).strip().replace("\n", " ")
                if not edit_note:
                    edit_note = _get_mpc_run_edit_note()
                if edit_note:
                    print(f"[RacingRun] TTL={name} run_timestamp={ts} edit_note={edit_note}")
                else:
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

