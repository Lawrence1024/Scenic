import os
import csv
from datetime import datetime, timezone
from typing import Optional, Dict, List, Tuple, Any

from scenic.core.regions import PolylineRegion


# race_common TtlColumn enum (from
# <RACE_COMMON_ROOT>/src/common/target_trajectory_line/include/target_trajectory_line/ttl.hpp).
# 20 columns per data row.
class TtlColumn:
    X = 0
    Y = 1
    Z = 2
    YAW = 3
    CURVATURE = 4
    CURVATURE_RATE = 5
    LON_VEL = 6
    LAT_VEL = 7
    LON_ACC = 8
    LAT_ACC = 9
    YAW_RATE = 10
    LEFT_BOUND_X = 11
    LEFT_BOUND_Y = 12
    RIGHT_BOUND_X = 13
    RIGHT_BOUND_Y = 14
    BANK_ANGLE = 15
    GRADE_ANGLE = 16
    DIST_TO_SF_BWD = 17
    DIST_TO_SF_FWD = 18
    REGION = 19
    NUM_COLS = 20


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


def _detect_race_common_format(first_row: List[str]) -> bool:
    """True if ``first_row`` looks like a race_common TTL metadata row.

    Format: ``id, num_pts, total_arc_length, gps_lat, gps_lon, gps_alt`` -- 6 numeric fields.
    Distinguishable from the simple ``x,y,z`` header (which has 'x' as the first token).
    """
    if not first_row or len(first_row) < 6:
        return False
    try:
        # All six fields must be parseable as floats
        vals = [float(c) for c in first_row[:6]]
    except (ValueError, TypeError):
        return False
    # Heuristic: 4th column is GPS latitude (deg) -- always between -90 and 90.
    # And 5th column is longitude -- between -180 and 180.
    return -90.0 <= vals[3] <= 90.0 and -180.0 <= vals[4] <= 180.0


def load_ttl_full(ttl_folder: str, ttl_file_name: str) -> Optional[Dict[str, Any]]:
    """Load a TTL CSV in race_common 20-column format.

    The race_common format has two header rows:
      Row 0: metadata ``id, num_pts, total_arc_length, gps_lat, gps_lon, gps_alt``
      Row 1: list of "important index" markers (sector boundaries / corner apexes -- ignored here)
      Row 2+: data with 20 columns (see ``TtlColumn``)

    Returns a dict with the racing line, per-point boundaries, and the metadata.
    Returns ``None`` if the file is missing, can't be parsed, or has fewer than 2 data rows.

    For files in the simple ``x,y,z`` header format, this function returns ``None`` so the
    caller can fall back to ``load_ttl_region``. Use ``load_ttl_region`` directly for the
    common case where only the racing line is needed (it auto-detects format and extracts
    just the line).

    Returned dict keys (all 1D arrays of length n_pts unless noted):
      ``waypoints``    : list of (x, y, z) tuples (the racing line)
      ``left_bound``   : list of (x, y) tuples
      ``right_bound``  : list of (x, y) tuples
      ``yaw``          : list of float (rad)
      ``curvature``    : list of float (1/m)
      ``lon_vel``      : list of float (m/s; reference speed)
      ``bank_angle``   : list of float (rad)
      ``grade_angle``  : list of float (rad)
      ``region``       : Scenic PolylineRegion (racing line, 2D)
      ``metadata``     : dict with keys ``ttl_id, num_pts, total_arc_length, gps_lat, gps_lon, gps_alt``
    """
    ttl_path = os.path.join(str(ttl_folder), str(ttl_file_name))
    if not os.path.exists(ttl_path):
        return None

    with open(ttl_path, newline="") as f:
        rd = csv.reader(f)
        rows = list(rd)
    if len(rows) < 3:
        return None

    if not _detect_race_common_format(rows[0]):
        return None

    try:
        meta = {
            "ttl_id": int(float(rows[0][0])),
            "num_pts": int(float(rows[0][1])),
            "total_arc_length": float(rows[0][2]),
            "gps_lat": float(rows[0][3]),
            "gps_lon": float(rows[0][4]),
            "gps_alt": float(rows[0][5]),
        }
    except (ValueError, IndexError):
        return None

    # rows[1] is the sector / important-index list -- ignored.
    data_rows = rows[2:]
    waypoints: List[Tuple[float, float, float]] = []
    left_bound: List[Tuple[float, float]] = []
    right_bound: List[Tuple[float, float]] = []
    yaw: List[float] = []
    curvature: List[float] = []
    lon_vel: List[float] = []
    bank_angle: List[float] = []
    grade_angle: List[float] = []
    for row in data_rows:
        if not row or len(row) < TtlColumn.NUM_COLS:
            continue
        try:
            x = float(row[TtlColumn.X]); y = float(row[TtlColumn.Y]); z = float(row[TtlColumn.Z])
            lbx = float(row[TtlColumn.LEFT_BOUND_X]); lby = float(row[TtlColumn.LEFT_BOUND_Y])
            rbx = float(row[TtlColumn.RIGHT_BOUND_X]); rby = float(row[TtlColumn.RIGHT_BOUND_Y])
            ya = float(row[TtlColumn.YAW]); cu = float(row[TtlColumn.CURVATURE])
            lv = float(row[TtlColumn.LON_VEL])
            ba = float(row[TtlColumn.BANK_ANGLE]); ga = float(row[TtlColumn.GRADE_ANGLE])
        except (ValueError, IndexError):
            continue
        waypoints.append((x, y, z))
        left_bound.append((lbx, lby))
        right_bound.append((rbx, rby))
        yaw.append(ya)
        curvature.append(cu)
        lon_vel.append(lv)
        bank_angle.append(ba)
        grade_angle.append(ga)

    if len(waypoints) < 2:
        return None

    pts_2d = [(p[0], p[1]) for p in waypoints]
    region = PolylineRegion(pts_2d)
    print(f"[TTL] Loaded {len(waypoints)} race_common-format waypoints from "
          f"{os.path.basename(ttl_path)} (with LEFT/RIGHT bounds)")
    return {
        "waypoints": waypoints,
        "left_bound": left_bound,
        "right_bound": right_bound,
        "yaw": yaw,
        "curvature": curvature,
        "lon_vel": lon_vel,
        "bank_angle": bank_angle,
        "grade_angle": grade_angle,
        "region": region,
        "metadata": meta,
    }


def _autodetect_full_ttl_filename(ttl_folder: str, ttl_file_name: str) -> Optional[str]:
    """Given a 3-col TTL filename like ``ttl_optimal_xodr.csv``, return the corresponding
    race_common 20-col file (``ttl_optimal_xodr_full.csv``) if present, else ``None``.

    Lets scenarios continue referencing the 3-col file by name while transparently picking
    up the boundary columns when the ``_full`` sibling exists -- enables corridor MPC
    without scenic-file edits.
    """
    if not ttl_file_name:
        return None
    base, ext = os.path.splitext(ttl_file_name)
    if base.endswith("_full"):
        return None  # already the full file
    candidate = f"{base}_full{ext}"
    if os.path.exists(os.path.join(str(ttl_folder), candidate)):
        return candidate
    return None


def attach_ttl(sim, obj, vehicle_type="vehicle"):
    """Load TTL based on scene params or object properties and attach region/waypoints to object.

    TTL configuration priority:
    1. Object-specific properties (obj.ttlFolder, obj.ttlFileName)
    2. Scene parameters (ttlFolder, ttlFileName)

    When the loaded file (or a sibling ``<name>_full.csv``) is in race_common 20-column
    format, also attaches per-waypoint LEFT/RIGHT boundary distances under
    ``obj.ttl_left_dist_m`` and ``obj.ttl_right_dist_m`` for use by the corridor-aware
    MPC. Backward compat: if neither file has the full format, those attrs stay unset
    and the MPC falls back to standard line-tracking (no barrier cost).
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

        # Prefer the full-format sibling if present -- gives us boundary columns for free.
        full_file = _autodetect_full_ttl_filename(ttl_folder, ttl_file)
        full_data = load_ttl_full(ttl_folder, full_file) if full_file else load_ttl_full(ttl_folder, ttl_file)

        region, pts = load_ttl_region(ttl_folder, ttl_file)
        if region is not None:
            setattr(obj, "ttl", region)
            name = os.path.basename(ttl_file)
            print(f"[TTL] Assigned TTL PolylineRegion to {vehicle_type} ({name})")
            # Attach per-waypoint corridor half-widths for the MPC barrier cost.
            if full_data is not None:
                import math
                wps_full = full_data["waypoints"]
                lbs = full_data["left_bound"]
                rbs = full_data["right_bound"]
                if len(wps_full) == len(lbs) == len(rbs) and len(wps_full) >= 2:
                    left_dist = [math.hypot(wp[0] - lb[0], wp[1] - lb[1])
                                 for wp, lb in zip(wps_full, lbs)]
                    right_dist = [math.hypot(wp[0] - rb[0], wp[1] - rb[1])
                                  for wp, rb in zip(wps_full, rbs)]
                    setattr(obj, "ttl_left_dist_m", left_dist)
                    setattr(obj, "ttl_right_dist_m", right_dist)
                    n = len(left_dist)
                    print(f"[TTL] Attached corridor bounds: left_dist mean={sum(left_dist)/n:.2f}m "
                          f"min={min(left_dist):.2f}m, right_dist mean={sum(right_dist)/n:.2f}m "
                          f"min={min(right_dist):.2f}m (n={n})")
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
