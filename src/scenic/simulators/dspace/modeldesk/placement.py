import math

from .. import utils as dutils
from ..geometry.frame_calibration import xodr_to_rd
from ..geometry.params import get_map_path
from .traffic_object import apply_fellow_traffic_object

# TTL filenames for route preference (distance to main vs pitlane; if similar, prefer main)
TTL_MAIN_ROAD_FILE = "ttl_main_road.csv"
TTL_PITLANE_FILE = "ttl_pitlane.csv"
# If |dist_main - dist_pit| <= this (m), consider "similar" and prefer main road
ROUTE_SIMILAR_TOLERANCE_M = 2.0
# |t| above this (m) is treated as out-of-bounds for logging; we never clamp (s,t), we place as-is
T_OUT_OF_BOUNDS_THRESHOLD_M = 15.0


# Lateral offset (t or d) convention for ego and fellow placement:
#   positive = left of centerline (in road direction)
#   negative = right of centerline
# ModelDesk/ControlDesk use the same convention; we pass through as-is (no sign flip).
def t_for_dspace_lateral(t_val: float) -> float:
    """Lateral offset sent to dSPACE ModelDesk/ControlDesk. Convention: positive=left, negative=right."""
    return float(t_val)


# Tolerance for "is this placement on the TTL polyline?" — the TTL placement
# pipeline samples directly on the polyline (post-SD-19b PolylineRegion), so
# the resolved (x, y) should be within mm of the polyline. 1 m is safe
# margin against any frame-transform / float-precision drift.
_TTL_PROXIMITY_TOLERANCE_M = 1.0


def _placement_is_on_ttl(ttl_folder, ttl_name, x, y, tol_m=_TTL_PROXIMITY_TOLERANCE_M) -> bool:
    """True iff the placed (x, y) lies within ``tol_m`` of the TTL polyline.

    Used by the contradiction warning to recognize "placed via the unified
    `trackRegion(...)` pipeline (or another TTL-aware route)" — in that case
    the placement is consistent with the TTL by construction even when it
    straddles the main/pit polygon boundary at pit entry/exit (where the
    pit TTL legitimately traverses the mainTrack polygon because of the
    main-wins-on-overlap rule).
    """
    if not ttl_folder or not ttl_name:
        return False
    try:
        from scenic.domains.racing.segments.track_regions import (
            create_ttl_region_from_file,
        )
        from shapely.geometry import Point
    except Exception:
        return False
    try:
        polyline = create_ttl_region_from_file(ttl_folder, ttl_name)
        if polyline is None:
            return False
        return float(polyline.lineString.distance(Point(float(x), float(y)))) < tol_m
    except Exception:
        return False


def _maybe_warn_placement_contradiction(sim, obj, x, y, vehicle_label):
    """SD-24c: emit a [Placement] [WARN] line when a car's TTL category and its
    placed (x, y) classification disagree.

    Silent when:
        - The car has no `ttlFileName` set (no implicit context).
        - The placed (x, y) is consistent with the TTL category.
        - The placed (x, y) is *on* the TTL polyline (within
          ``_TTL_PROXIMITY_TOLERANCE_M``). This is the
          unified-pipeline / default-placement case: the position came
          from ``new Point on trackRegion(self.ttlFileName)`` (or
          equivalent) and is therefore consistent with the TTL by
          construction. The pit TTL traverses the mainTrack polygon at
          pit entry/exit due to the main-wins-on-overlap rule; without
          this skip the warning would fire spuriously on every default
          pit-TTL placement that lands in the entry/exit zone.
        - The placed (x, y) lies in neither mainTrack nor pitTrack (off-track
          placement; the BoundsCheck pipeline owns that diagnostic).
        - mainTrackRegion / pitTrackRegion params are unavailable (e.g.
          legacy maps without RacingTrack).

    Emits a `'PlacementContradiction'` record alongside the print so monitors
    can filter on it. See ``docs/scenic_changes_from_presentation.md`` SD-24
    for the full 4-cell contradiction matrix.
    """
    try:
        from scenic.domains.racing.segments.track_regions import ttl_category
    except Exception:
        return  # racing domain unavailable; silently skip

    ttl_name = getattr(obj, "ttlFileName", None)
    cat_ttl = ttl_category(ttl_name)
    if cat_ttl is None:
        return  # no implicit context to contradict

    params = getattr(getattr(sim, "scene", None), "params", None) or {}
    ttl_folder = params.get("ttlFolder")
    if _placement_is_on_ttl(ttl_folder, ttl_name, x, y):
        return  # placement is on the TTL polyline; consistent by construction

    main_region = params.get("mainTrackRegion")
    pit_region = params.get("pitTrackRegion")
    if main_region is None or pit_region is None:
        return

    try:
        from scenic.core.vectors import Vector
        pt = Vector(float(x), float(y))
        in_main = bool(main_region.containsPoint(pt))
        in_pit = bool(pit_region.containsPoint(pt))
    except Exception:
        return

    if cat_ttl == "main" and in_pit and not in_main:
        cat_pos = "pit"
    elif cat_ttl == "pit" and in_main and not in_pit:
        cat_pos = "main"
    else:
        # Consistent (ttl=main, in_main=True) OR off-track entirely
        # (in_main=False AND in_pit=False) OR ambiguous (both true at a
        # junction). None of those should fire the warning.
        return

    print(
        f"[Placement] [WARN] {vehicle_label} with ttlFileName='{ttl_name}' "
        f"(category={cat_ttl}) was placed at ({float(x):.2f}, {float(y):.2f}) "
        f"classified as {cat_pos}. Continuing anyway -- this may be "
        f"intentional for a falsification scenario."
    )
    try:
        sim.records["PlacementContradiction"].append((sim.currentTime, {
            "name": str(vehicle_label),
            "ttl_file_name": str(ttl_name) if ttl_name else None,
            "ttl_category": cat_ttl,
            "placed_x": float(x),
            "placed_y": float(y),
            "placed_category": cat_pos,
        }))
    except Exception:
        pass


def _racing_st_offset_to_deltas(offset):
    """Convert _racing_st_offset to (delta_s, delta_t) in meters relative to ego.
    offset can be:
    - (delta_s, delta_t): used as-is. Convention: ahead = +s, behind = -s; left = +t, right = -t.
    - ('ahead', d) or ('behind', d) or ('left', d) or ('right', d): converted to (ds, dt).
    Returns None if offset is invalid.

    All offsets are relative to ego: fellow (s,t) = ego (s,t) + (delta_s, delta_t).
    E.g. ('right', 3) -> (0, -3) so fellow t = ego_t - 3 (3 m to the right of ego).
    Convention: positive t = left of centerline, negative t = right.
    """
    if offset is None:
        return None
    try:
        if len(offset) == 2:
            first, second = offset[0], offset[1]
            if isinstance(first, (int, float)) and isinstance(second, (int, float)):
                return (float(first), float(second))
            if isinstance(first, str) and isinstance(second, (int, float)):
                d = float(second)
                kind = str(first).strip().lower()
                if kind == 'ahead':
                    return (d, 0.0)
                if kind == 'behind':
                    return (-d, 0.0)
                # positive t = left, negative t = right
                if kind == 'left':
                    return (0.0, d)
                if kind == 'right':
                    return (0.0, -d)
    except (TypeError, ValueError):
        pass
    return None


def _road_direction_deg_at(road_index, px, py):
    """Return road direction at (px, py) in degrees (Scenic convention: 0=North, 90=East). None if not found."""
    if not road_index or not road_index.get("roads"):
        return None
    best_dist2 = float("inf")
    best_heading_rad = None
    for road in road_index["roads"].values():
        sec_list = road.get("sec_points") or []
        for pts in sec_list:
            if not pts or len(pts) < 2:
                continue
            for i in range(len(pts) - 1):
                x0, y0 = pts[i][0], pts[i][1]
                x1, y1 = pts[i + 1][0], pts[i + 1][1]
                vx, vy = x1 - x0, y1 - y0
                seg_len2 = vx * vx + vy * vy
                if seg_len2 <= 1e-12:
                    continue
                wx, wy = px - x0, py - y0
                u = (wx * vx + wy * vy) / seg_len2
                u = max(0.0, min(1.0, u))
                qx = x0 + u * vx
                qy = y0 + u * vy
                dx, dy = px - qx, py - qy
                dist2 = dx * dx + dy * dy
                if dist2 < best_dist2:
                    best_dist2 = dist2
                    # Scenic: 0 = North = +Y, so heading = atan2(vx, vy)
                    best_heading_rad = math.atan2(vx, vy)
    if best_heading_rad is None:
        return None
    return math.degrees(best_heading_rad)


def _min_dist_to_ttl(px, py, points):
    """Minimum distance from (px, py) to any point in points. points: list of (x,y) or (x,y,z)."""
    if not points:
        return float("inf")
    best = float("inf")
    for p in points:
        dx = px - p[0]
        dy = py - p[1]
        d = (dx * dx + dy * dy) ** 0.5
        if d < best:
            best = d
    return best


def _min_dist_to_polyline(px, py, points):
    """Minimum distance from (px, py) to any segment of the polyline. points: list of (x,y) or (x,y,z)."""
    if not points or len(points) < 2:
        return _min_dist_to_ttl(px, py, points)
    best = float("inf")
    for i in range(len(points) - 1):
        x0, y0 = points[i][0], points[i][1]
        x1, y1 = points[i + 1][0], points[i + 1][1]
        dx, dy = x1 - x0, y1 - y0
        qx, qy = px - x0, py - y0
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-18:
            d = math.hypot(qx, qy)
        else:
            t = max(0.0, min(1.0, (qx * dx + qy * dy) / seg_len_sq))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            d = math.hypot(px - proj_x, py - proj_y)
        if d < best:
            best = d
    return best


def _route_pref_from_ttl_distances(sim, xodr_x, xodr_y):
    """
    Prefer route (Lap vs Pit) by distance to main-road TTL vs pitlane TTL.
    If distances are similar (within ROUTE_SIMILAR_TOLERANCE_M), prefer main road (Lap).
    Returns 'Lap' or 'Pit', or None if TTLs cannot be loaded or scenario did not set ttlFolder (caller uses road-based detection).
    """
    try:
        scene_params = getattr(getattr(sim, "scene", None), "params", None) or {}
        if not scene_params.get("ttlFolder"):
            return None  # Scenario uses XODR track; don't use TTL centerlines for route
        from ..ttl.loader import get_ttl_config, load_ttl_region
        ttl_folder, _ = get_ttl_config(scene_params)
        ttl_folder = str(ttl_folder)
        _, main_pts = load_ttl_region(ttl_folder, TTL_MAIN_ROAD_FILE)
        _, pit_pts = load_ttl_region(ttl_folder, TTL_PITLANE_FILE)
        if not main_pts or not pit_pts:
            return None
        # Use polyline (segment) distance so assignment is by closest centerline
        dist_main = _min_dist_to_polyline(xodr_x, xodr_y, main_pts)
        dist_pit = _min_dist_to_polyline(xodr_x, xodr_y, pit_pts)
        # If clearly closer to pit, use Pit; otherwise prefer main road (including when similar)
        if dist_pit < dist_main - ROUTE_SIMILAR_TOLERANCE_M:
            print(f"  [Route] TTL distances: main={dist_main:.2f}m pit={dist_pit:.2f}m -> Pit (closer to pitlane)")
            return "Pit"
        print(f"  [Route] TTL distances: main={dist_main:.2f}m pit={dist_pit:.2f}m -> Lap (main road or similar, prefer main)")
        return "Lap"
    except Exception as e:
        print(f"  [Route] TTL-based route preference skipped: {e}")
        return None


def place_ego(sim, obj):
    """Create/configure the ego vehicle using the Maneuver API.

    Frames: ``obj.position`` is in Scenic XODR-xy (the frame defined by ``param map``).
    Projection onto the empirical centerline (``ttl_main_road.csv``) and the (s, t)
    sent to ModelDesk are in dSPACE RD-xy. ``xodr_to_rd`` applies the calibrated
    translation between the two; identity if no calibration JSON exists for the map
    (e.g. the OLD ``LagunaSeca.xodr`` workflow). See ``docs/frames.md``.
    """
    # 1) Position in map frame (Scenic XODR), then translate to RD frame for projection.
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        _xodr_path = get_map_path(getattr(getattr(sim, "scene", None), "params", None) or {})
        work_x, work_y = xodr_to_rd(scenic_x, scenic_y, _xodr_path)

        # SD-24c: contradiction warning when the car's TTL category disagrees
        # with the polygon classification of its placed (x, y). Fires only
        # for the four mismatch cases (main TTL on pit polygon, or pit TTL
        # on main polygon); silent otherwise. See docstring on
        # _maybe_warn_placement_contradiction for the full predicate.
        _maybe_warn_placement_contradiction(sim, obj, scenic_x, scenic_y, "ego")

        # 2) Determine route: prefer TTL-based (distance to main vs pitlane; if similar, prefer main).
        # _route_pref_from_ttl_distances compares against TTL CSVs which live in RD frame, so
        # it must receive the RD-translated position.
        route_pref = _route_pref_from_ttl_distances(sim, work_x, work_y)
        if not route_pref:
            try:
                # Road-based detection (RD coordinates)
                position_xy = (work_x, work_y)
                track_segment = sim.detectTrackSegment(position_xy)
                if track_segment:
                    route_pref = sim.assignRoute(obj, track_segment)
            except Exception as e:
                print(f"  [Route] Could not detect route from RD coordinates: {e}")
        if not route_pref:
            try:
                route_pref = sim._detect_route_from_road_segment(obj)
            except Exception:
                pass
        if not route_pref:
            route_pref = 'Lap'
        print(f"[Ego] Assigned route: {route_pref} (Lap=main road R2, Pit=pitlane R1)")

        # Use TTL centerlines for projection only when the scenario explicitly set ttlFolder.
        # Otherwise mainTrack may be from XODR; projecting onto TTL would give wrong (s,t) and large |t|.
        scene_params = getattr(getattr(sim, "scene", None), "params", None) or {}
        use_ttl_for_projection = bool(scene_params.get("ttlFolder"))
        if use_ttl_for_projection and not getattr(sim, '_road_index_ttl', None):
            try:
                from ..ttl.loader import get_ttl_config
                from ..ttl.road_index import build_road_index_from_ttl
                ttl_folder, _ = get_ttl_config(scene_params)
                if ttl_folder:
                    ttl_road_index = build_road_index_from_ttl(str(ttl_folder))
                    if ttl_road_index is not None:
                        sim._road_index_ttl = ttl_road_index
            except Exception as e:
                pass  # Fall back to XODR-based index
        road_index_for_projection = getattr(sim, '_road_index_ttl', None) or sim._road_index

        # One-time note: t is lateral offset from projection centerline (positive = left, negative = right)
        if road_index_for_projection and not getattr(sim, '_placement_t_note_logged', False):
            src = "TTL centerline" if getattr(sim, '_road_index_ttl', None) else "XODR centerline"
            print(f"[Placement] t = signed lateral offset from {src} (t>0 = left, t<0 = right in road direction).")
            sim._placement_t_note_logged = True

        # 3) Project RD → (s,t) using route-specific road index
        if road_index_for_projection:
            from ..geometry.route_projection import project_world_to_st_route_specific
            s_val, t_val = project_world_to_st_route_specific(
                road_index_for_projection,
                (work_x, work_y),
                route_preference=route_pref
            )
        else:
            s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))
        # We never clamp (s,t) to track bounds; out-of-bounds positions are sent to ModelDesk as-is
        if abs(t_val) > T_OUT_OF_BOUNDS_THRESHOLD_M:
            print(f"[Placement] Ego: t={t_val:.2f} m (out of track bounds; placing as-is, no clamping)")
    else:
        s_val, t_val = 0.0, 0.0
        route_pref = 'Lap'  # Default route
        print(f"[Ego] Assigned route: {route_pref} (Lap=main road R2, Pit=pitlane R1) [no position]")

    # 2) Get velocity
    base_v = 0.0

    # 3) Access the ego maneuver
    try:
        maneuver_collection = sim.ts.Maneuver
        if maneuver_collection.Count == 0:
            print("[Ego] Warning: No ego maneuver found in scenario - cannot configure ego")
            return None

        ego_maneuver = maneuver_collection.Item(0)

        # Ensure Maneuver is enabled/active if such a property exists
        try:
            ego_maneuver.Enabled = True
        except:
            pass

        # Access sequences
        sequences = ego_maneuver.Sequences
        if sequences.Count == 0:
            print("[Ego] Warning: No sequences in ego maneuver - cannot configure")
            return None
        seq = sequences.Item(0)

        # 4) Set route FIRST (before setting position) to ensure s-coordinate is interpreted correctly
        # Map route preference to ModelDesk route names (same as fellows)
        route_name_map = {
            'Pit': 'R1',
            'Lap': 'R2'
        }
        modeldesk_route = route_name_map.get(route_pref, 'R2')
        
        try:
            route_sel = seq.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0  # Direct
            route_sel.Activate(modeldesk_route)
            print(f"[Ego] Route set to: {modeldesk_route} (from preference: {route_pref})")
        except Exception as e:
            # Fallback to helper method if direct activation fails
            print(f"[Ego] Direct route activation failed, trying fallback: {e}")
            try:
                sim._set_fellow_route_via_sequence(seq, obj)
            except Exception:
                pass

        # 5) Configure ego vehicle position and properties
        if getattr(obj, "position", None) is not None:
            obj._route_s_t = (s_val, t_val)
            obj._route = route_pref

        # STRATEGY: Set StartPosition on Sequence AND Segments to force update
        seq.StartPosition = float(s_val)
        seq.InitialLongitudinalVelocity = float(base_v)

        # Set lateral offset (t) on sequence - UI "Additional lateral offset" (convert to dSPACE sign)
        ego_lat_set = False
        t_dspace = t_for_dspace_lateral(t_val)
        for lat_prop in ('AdditionalLateralOffset', 'InitialLateralOffset', 'LateralOffset', 'AdditionalLateralPosition'):
            if hasattr(seq, lat_prop):
                try:
                    setattr(seq, lat_prop, float(t_dspace))
                    ego_lat_set = True
                    if abs(t_val) > 0.01:
                        print(f"[Ego] Set {lat_prop}={t_dspace:.3f} m (Additional lateral offset)")
                    break
                except Exception:
                    continue
        if not ego_lat_set and abs(t_val) > 0.01:
            pass  # Fall back to segment lateral below (t_dspace used there)

        # Iterate through segments to ensure they don't override the start pos
        for i in range(seq.Segments.Count):
            seg = seq.Segments.Item(i)
            # Some versions allow setting start S on segment 0 explicitly
            if i == 0 and hasattr(seg, 'StartPosition'):
                try:
                    seg.StartPosition = float(s_val)
                except:
                    pass

        # Orientation conversion
        # Transform from Scenic ENU (North=0°) to dSPACE RD (East=0°)
        # Use orientation.yaw directly for clarity (equivalent to heading for most cases)
        try:
            if hasattr(obj, 'orientation') and hasattr(obj.orientation, 'yaw'):
                scenic_yaw = obj.orientation.yaw
                dspace_orientation = scenic_yaw - math.pi / 2
                seq.VehicleOrientation = dspace_orientation
            elif hasattr(obj, 'heading'):
                # Fallback to heading if orientation.yaw not available
                dspace_orientation = obj.heading - math.pi / 2
                seq.VehicleOrientation = dspace_orientation
            else:
                seq.VehicleOrientation = 0.0
        except Exception:
            seq.VehicleOrientation = 0.0

        # Fallback: set lateral via segment Activity.LateralType when sequence "Additional lateral offset" not available
        # NOTE: Prefer sequence-level AdditionalLateralOffset (or similar) when the UI shows it; segment lateral may be ignored.
        if not ego_lat_set and abs(t_val) > 0.01:
            try:
                segments = seq.Segments
                if segments.Count > 0:
                    seg0 = segments.Item(0)
                    lat0 = seg0.Activity.LateralType
                    dutils.activate_type(lat0, "Deviation")
                    dep = getattr(lat0.ActiveElement, "DependencyType", None)
                    if dep is not None:
                        dutils.activate_type(dep, "Absolute")
                    success_lat = False
                    for prop_name in ['Constant', 'Value', 'Offset', 'LateralOffset']:
                        try:
                            if hasattr(lat0.ActiveElement, prop_name):
                                setattr(lat0.ActiveElement, prop_name, float(t_dspace))
                                success_lat = True
                                break
                            dutils.set_activity_constant(lat0, t_dspace)
                            success_lat = True
                            break
                        except Exception:
                            continue
            except Exception:
                pass

        sim._ego_created = True

        return ego_maneuver
    except Exception as e:
        print(f"[Ego] Error configuring: {e}")
        return None


def place_fellow(sim, obj):
    """Create a Fellow vehicle (non-ego) using the Fellows API."""
    # Get vehicle name for logging
    vehicle_name = getattr(obj, "name", f"Fellow_{sim.ts.Fellows.Count}")
    
    # 1) Position in map frame (Scenic XODR), then translate to RD frame for projection.
    # See place_ego docstring for frame conventions; identity translation if no calibration.
    if getattr(obj, "position", None) is not None:
        scenic_x, scenic_y = obj.position.x, obj.position.y
        _xodr_path = get_map_path(getattr(getattr(sim, "scene", None), "params", None) or {})
        work_x, work_y = xodr_to_rd(scenic_x, scenic_y, _xodr_path)

        # SD-24c: contradiction warning when the fellow's TTL category
        # disagrees with the polygon classification of its placed (x, y).
        # Same predicate as the ego check; see place_ego for the full
        # docstring reference.
        _maybe_warn_placement_contradiction(sim, obj, scenic_x, scenic_y, vehicle_name)

        # 2) Determine route: TTL centerlines (ttl_main_road.csv vs ttl_pitlane.csv) live in
        # RD frame, so feed them the translated work_x/work_y.
        route_pref = _route_pref_from_ttl_distances(sim, work_x, work_y)
        if not route_pref:
            try:
                position_xy = (work_x, work_y)
                track_segment = sim.detectTrackSegment(position_xy)
                if track_segment:
                    route_pref = sim.assignRoute(obj, track_segment)
            except Exception:
                pass
        if not route_pref:
            try:
                route_pref = sim._detect_route_from_road_segment(obj)
            except Exception:
                pass
        if not route_pref:
            route_pref = 'Lap'

        # Use TTL centerline index if built (by ego placement); else XODR index
        road_index_for_projection = getattr(sim, '_road_index_ttl', None) or sim._road_index

        # 3) (s,t): racing-library semantics when _racing_st_offset is set (ahead/behind → keep t, move s; left/right → keep s, move t)
        use_racing_offset = False
        racing_delta_s = 0.0
        racing_delta_t = 0.0
        st_offset = getattr(obj, '_racing_st_offset', None)
        deltas = _racing_st_offset_to_deltas(st_offset) if st_offset is not None else None
        if deltas is not None:
            ego = getattr(getattr(sim, 'scene', None), 'egoObject', None)
            if ego is not None:
                ego_st = getattr(ego, '_route_s_t', None)
                ego_route = getattr(ego, '_route', None)
                if ego_st is not None and len(ego_st) >= 2 and ego_route is not None:
                    ego_s, ego_t = float(ego_st[0]), float(ego_st[1])
                    delta_s, delta_t = deltas
                    racing_delta_s, racing_delta_t = float(delta_s), float(delta_t)
                    s_val = ego_s + delta_s
                    t_val = ego_t + delta_t  # e.g. ("right", 3) -> delta_t=-3 -> fellow t = ego_t - 3 (right of ego)
                    route_pref = ego_route
                    use_racing_offset = True
                    print(f"[Placement] {vehicle_name}: racing (s,t) from ego + {st_offset} -> s={s_val:.2f}, t={t_val:.2f} (same route as ego)")
                    # SD-21a: structured record alongside the print so monitors
                    # don't have to regex-parse the [Placement] line.
                    try:
                        _gap = st_offset[0] if isinstance(st_offset, (tuple, list)) and len(st_offset) >= 1 else None
                        _lat = st_offset[1] if isinstance(st_offset, (tuple, list)) and len(st_offset) >= 2 else None
                        sim.records['FellowPlacement'].append((sim.currentTime, {
                            'name': str(vehicle_name),
                            'gap_m': (float(_gap) if _gap is not None else None),
                            'lat_m': (float(_lat) if _lat is not None else None),
                            's': float(s_val),
                            't': float(t_val),
                        }))
                    except Exception:
                        pass
                else:
                    print(f"[Placement] {vehicle_name}: _racing_st_offset={st_offset} ignored (ego _route_s_t or _route not set yet; place ego first)")
            else:
                print(f"[Placement] {vehicle_name}: _racing_st_offset={st_offset} ignored (no ego in scene)")

        if not use_racing_offset:
            # Project RD → (s,t) using route-specific road index
            if road_index_for_projection:
                from ..geometry.route_projection import project_world_to_st_route_specific
                s_val, t_val = project_world_to_st_route_specific(
                    road_index_for_projection,
                    (work_x, work_y),
                    route_preference=route_pref
                )
            else:
                s_val, t_val = dutils.project_world_to_st(sim._road_index, (work_x, work_y))

        obj._route_s_t = (s_val, t_val)
        obj._route = route_pref
        # We never clamp (s,t); out-of-bounds positions are sent to ModelDesk as-is
        if abs(t_val) > T_OUT_OF_BOUNDS_THRESHOLD_M:
            print(f"[Placement] {vehicle_name}: t={t_val:.2f} m (out of track bounds; placing as-is, no clamping)")
        # Log ego once for context (heading vs road explains absurd t when "ahead/right of ego")
        if not getattr(sim, '_placement_ego_debug_logged', False):
            try:
                ego = getattr(sim, 'scene', None) and getattr(sim.scene, 'egoObject', None)
                if ego and getattr(ego, 'position', None) is not None:
                    ex, ey = ego.position.x, ego.position.y
                    # Use simulator-aligned heading if set (Scenic may cache orientation)
                    if getattr(ego, '_road_aligned_heading_deg', None) is not None:
                        yaw_deg = float(ego._road_aligned_heading_deg)
                    else:
                        yaw = getattr(
                            getattr(ego, 'orientation', None), 'yaw',
                            getattr(ego, 'heading', 0.0)
                        )
                        yaw_deg = math.degrees(yaw)
                    road_deg = _road_direction_deg_at(getattr(sim, '_road_index', None), ex, ey)
                    road_deg_str = f"{road_deg:.1f}" if road_deg is not None else "n/a"
                    ego_s, ego_t = getattr(ego, '_route_s_t', (None, None))
                    st_str = f" s={ego_s:.2f}, t={ego_t:.2f}" if ego_s is not None and ego_t is not None else ""
                    msg = (
                        f"[Ego debug] xy=({ex:.4f}, {ey:.4f}) ->{st_str}, heading_deg={yaw_deg:.2f}, "
                        f"road_direction_deg={road_deg_str} (0=North, 90=East, -90=West)"
                    )
                    if road_index_for_projection:
                        ego_road_id = dutils.find_road_id_for_position(road_index_for_projection, ex, ey)
                        ego_road_name = dutils.get_road_name_for_id(road_index_for_projection, ego_road_id) if ego_road_id is not None else None
                        msg += f" | projected onto road_id={ego_road_id} ({ego_road_name or 'n/a'})"
                    # Diagnostic: distance from ego to TTL main centerline (TTL = source used to generate XODR)
                    try:
                        from ..ttl.loader import get_ttl_config, load_ttl_region
                        scene_params = getattr(getattr(sim, "scene", None), "params", None) or {}
                        ttl_folder, _ = get_ttl_config(scene_params)
                        if ttl_folder:
                            _, main_pts = load_ttl_region(str(ttl_folder), TTL_MAIN_ROAD_FILE)
                            if main_pts:
                                dist_to_ttl = _min_dist_to_polyline(ex, ey, main_pts)
                                msg += f" | dist_to_ttl_main_centerline_m={dist_to_ttl:.2f}"
                                if ego_s is not None and ego_t is not None and abs(ego_t) > 0.5:
                                    msg += " (if dist_to_ttl<<|t| then projection centerline is offset from TTL)"
                    except Exception:
                        pass
                    if road_deg is not None:
                        diff = abs((yaw_deg - road_deg + 180) % 360 - 180)
                        if diff > 20:
                            msg += f" | MISMATCH {diff:.0f}deg -> 'ahead of ego' will be off road (ego heading aligned in simulator)"
                    print(msg)
                    # SD-21a: structured record alongside the [Ego debug] print.
                    try:
                        sim.records['EgoStart'].append((sim.currentTime, {
                            'x': float(ex),
                            'y': float(ey),
                            'heading_deg': float(yaw_deg),
                        }))
                    except Exception:
                        pass
            except Exception as e:
                print(f"[Ego debug] Could not log ego: {e}")
            sim._placement_ego_debug_logged = True
        # Log fellow (s, t) and placement context for debugging absurd t values.
        # IMPORTANT: for ego-anchored placement (_racing_st_offset), do not use the
        # fellow Scenic object's raw sampled xy for diagnostics; that sampled position
        # is not the source of the final placement and can make repeat runs appear random.
        modeldesk_route = 'R2' if route_pref == 'Lap' else 'R1'
        distance_from_ego = float('nan')
        angle_from_ego_deg = float('nan')
        road_info = ""
        try:
            ego = getattr(sim, 'scene', None) and getattr(sim.scene, 'egoObject', None)
            if ego and getattr(ego, 'position', None) is not None:
                ex, ey = ego.position.x, ego.position.y
                if getattr(ego, '_road_aligned_heading_deg', None) is not None:
                    yaw = math.radians(float(ego._road_aligned_heading_deg))
                else:
                    yaw = getattr(
                        getattr(ego, 'orientation', None), 'yaw',
                        getattr(ego, 'heading', 0.0)
                    )
                if use_racing_offset:
                    # For ego-anchor placement, distance/angle are deterministic from (ds, dt).
                    distance_from_ego = math.hypot(racing_delta_s, racing_delta_t)
                    # Convention in logs: 0=ahead, 90=right, -90=left.
                    angle_from_ego_deg = math.degrees(math.atan2(-racing_delta_t, racing_delta_s))
                    # For road diagnostics in ego-anchor mode, use ego road projection for both.
                    if road_index_for_projection:
                        ego_road_id = dutils.find_road_id_for_position(road_index_for_projection, ex, ey)
                        ego_road_name = (
                            dutils.get_road_name_for_id(road_index_for_projection, ego_road_id)
                            if ego_road_id is not None
                            else None
                        )
                        road_info = (
                            f" | projected onto road_id={ego_road_id} ({ego_road_name or 'n/a'}) "
                            f"[ego_anchor]"
                        )
                else:
                    dx = scenic_x - ex
                    dy = scenic_y - ey
                    distance_from_ego = math.hypot(dx, dy)
                    # Angle of (fellow - ego) vs ego heading: 0 = ahead, 90 = right (+X), -90 = left (-X)
                    angle_world = math.atan2(dx, dy)
                    angle_from_ego_deg = math.degrees(angle_world - yaw)
                    # Normalize to [-180, 180] for readability
                    while angle_from_ego_deg > 180:
                        angle_from_ego_deg -= 360
                    while angle_from_ego_deg < -180:
                        angle_from_ego_deg += 360
        except Exception:
            pass
        # Which road this fellow projects onto (for debugging large |t|) in non-anchor mode.
        if (not road_info) and road_index_for_projection:
            fellow_road_id = dutils.find_road_id_for_position(road_index_for_projection, work_x, work_y)
            fellow_road_name = (
                dutils.get_road_name_for_id(road_index_for_projection, fellow_road_id)
                if fellow_road_id is not None
                else None
            )
            road_info = f" | projected onto road_id={fellow_road_id} ({fellow_road_name or 'n/a'})"
        print(
            f"[Fellow s,t] {vehicle_name}: route={route_pref} ({modeldesk_route}), "
            f"xy=({scenic_x:.4f}, {scenic_y:.4f}) -> s={s_val:.4f}, t={t_val:.4f} | "
            f"distance_from_ego={distance_from_ego:.2f}m, angle_from_ego_deg={angle_from_ego_deg:.1f} (0=ahead, 90=right, -90=left)"
            f"{road_info}"
        )
    else:
        s_val, t_val = 0.0, 0.0
        route_pref = 'Lap'
        print(f"[Fellow s,t] {vehicle_name}: no position -> s=0, t=0, route=Lap (default)")

    # 3) Create Fellow
    F = sim.ts.Fellows.Add()

    try:
        if getattr(obj, "name", None):
            F.Name = str(obj.name)
        else:
            F.Name = f"Fellow_{sim.ts.Fellows.Count}"
    except Exception as e:
        F.Name = f"Fellow_{sim.ts.Fellows.Count}"

    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)

    # seg0 = ABSOLUTE pose: Position = s, Deviation(Absolute) = t (convert to dSPACE lateral sign)
    # NOTE: Known limitation - dSPACE ModelDesk may ignore t-coordinate (lateral deviation)
    # for fellow vehicles. Testing shows vehicles are placed on centerline regardless of t value.
    # This is a dSPACE ModelDesk configuration issue, not a bug in our code.
    # See debug_ego_cord/README.md and debug_route_code/README.md for details.
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_for_dspace_lateral(t_val)))

    # seg1 = Longitudinal Velocity (Extern), Lateral deviation (Extern)
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"    Warning: could not set seg1 Velocity(Extern)/Lateral deviation(Extern): {e}")
    try:
        dutils.make_endless_transition(segs)
    except Exception:
        pass

    # Orientation
    # Transform from Scenic ENU (North=0°) to dSPACE RD (East=0°)
    # Use orientation.yaw directly for clarity (equivalent to heading for most cases)
    try:
        if hasattr(obj, 'orientation') and hasattr(obj.orientation, 'yaw'):
            scenic_yaw = obj.orientation.yaw
            dspace_orientation = scenic_yaw - math.pi / 2
            if hasattr(S1, 'VehicleOrientation'):
                S1.VehicleOrientation = dspace_orientation
        elif hasattr(obj, 'heading'):
            # Fallback to heading if orientation.yaw not available
            dspace_orientation = obj.heading - math.pi / 2
            if hasattr(S1, 'VehicleOrientation'):
                S1.VehicleOrientation = dspace_orientation
    except Exception:
        pass

    # Route (already determined above, but set it explicitly to ensure consistency)
    # The route was determined before projection, so we know it's correct
    try:
        route_sel = S1.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0  # Direct
        
        # Map route preference to ModelDesk route names
        route_name_map = {
            'Pit': 'R1',
            'Lap': 'R2'
        }
        modeldesk_route = route_name_map.get(route_pref, 'R2')
        
        try:
            route_sel.Activate(modeldesk_route)
        except Exception as e:
            # Fallback to original method if direct activation fails
            sim._set_fellow_route_via_sequence(S1, obj)
    except Exception as e:
        # Fallback to original method
        sim._set_fellow_route_via_sequence(S1, obj)

    apply_fellow_traffic_object(F)

    # Store reference
    sim._fellow_vehicles[F.Name] = {
        'fellow_object': F,
        'sequence': S1,
        'segments': segs,
        'scenic_object': obj,
        'index': sim.ts.Fellows.Count - 1
    }

    return F