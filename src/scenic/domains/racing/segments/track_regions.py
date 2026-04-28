"""Build mainTrack and pitTrack regions from segment centerlines (OpenDRIVE or TTL).

Track regions are buffered centerlines with width:
- mainTrack: main road segments with 6m on each side of the centerline (default).
- pitTrack: pit lane segments with 1.5m on each side of the centerline (default).

Overlap rule (same as segment logic): where main and pit overlap (e.g. Corkscrew in
pitlane CSV), main wins — pitTrack is defined as (buffered pit) minus mainTrack so
the regions are mutually exclusive and overlap belongs to mainTrack.

Two build options:
- From OpenDRIVE: use RacingTrack's _mainRacingRoads and _pitRoads centerlines.
- From TTL: use ttl_main_road.csv and ttl_pitlane.csv centerline waypoints.

Used by the racing world model to expose mainTrack and pitTrack for
`new RacingCar on mainTrack` / `new RacingCar on pitTrack`.
"""

from pathlib import Path
from typing import Any, List, Optional, Tuple

from scenic.core.regions import PolygonalRegion, PolylineRegion, regionFromShapelyObject, nowhere


def ttl_category(ttl_file_name: Optional[str]) -> Optional[str]:
    """Classify a TTL filename as ``'main'``, ``'pit'``, or ``None``.

    Mirrors the same string-match used at ``tracks.py:343-350`` for pit-road
    identification: any TTL filename containing ``'pit'`` (case-insensitive)
    is the pit TTL; any other non-empty filename is a main TTL
    (``ttl_optimal_xodr.csv`` / ``ttl_left_xodr.csv`` / ``ttl_right_xodr.csv``).
    A ``None`` or empty filename returns ``None`` — there is no implicit
    track context, which the unified ``trackRegion`` helper interprets as
    "fall back to mainTrack" for the no-segment case or "use the pit + main
    union" for axis-region requests.

    Used by:
    - ``model.scenic`` ``trackRegion(ttlFileName, segment)`` to pick the
      cross-product polygon when a segment filter is requested.
    - ``simulators/dspace/modeldesk/placement.py`` to detect the four
      contradiction cases (explicit ``main*`` placement with pit TTL,
      explicit ``pit*`` placement with main TTL).
    """
    if not ttl_file_name:
        return None
    return "pit" if "pit" in str(ttl_file_name).lower() else "main"

# Default buffer: meters on each side of the segment centerline
MAIN_TRACK_BUFFER_M = 6.0
PIT_TRACK_BUFFER_M = 1.5


def _load_ttl_csv(folder: Path, filename: str) -> List[Tuple[float, float, float]]:
    """Load TTL CSV (x,y,z) as list of (x,y,z). Returns [] on failure."""
    import csv

    path = folder / filename
    if not path.exists():
        return []
    pts: List[Tuple[float, float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        first = next(r, None)
        if first and len(first) >= 2 and first[0].strip().lower() == "x":
            pass  # skip header
        elif first and len(first) >= 2:
            try:
                x, y = float(first[0]), float(first[1])
                z = float(first[2]) if len(first) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                pass
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x, y = float(row[0]), float(row[1])
                z = float(row[2]) if len(row) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                continue
    return pts


def _polyline_from_waypoints(waypoints: List[Tuple[float, ...]]) -> Optional[PolylineRegion]:
    """Build a PolylineRegion from waypoints (x,y) or (x,y,z)."""
    if not waypoints or len(waypoints) < 2:
        return None
    points = [(float(wp[0]), float(wp[1])) for wp in waypoints]
    return PolylineRegion(points=points)


def _closed_centerline_to_band(ls, buffer_m: float):
    """Build track band (outer boundary only; inner boundary = infield) so infield is not part of the region.
    For a closed centerline, buffer(d) would fill the loop interior; we subtract the infield so the
    track is only the corridor (band) between inner and outer edge. Returns Shapely polygon or None.
    """
    import shapely.geometry

    buf = ls.buffer(buffer_m)
    if buf.is_empty:
        return None
    coords = list(ls.coords)
    if len(coords) < 3:
        return buf
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    try:
        infield = shapely.geometry.Polygon(coords)
        if not infield.is_valid:
            infield = infield.buffer(0)
        if infield.is_empty or infield.area < 1e-6:
            return buf
        band = buf.difference(infield)
        if band.is_empty:
            return buf
        return band
    except Exception:
        return buf


def _buffer_polyline_region(polyline_region: PolylineRegion, buffer_m: float) -> PolygonalRegion:
    """Return a PolygonalRegion: track band (centerline ± buffer_m) with inner boundary so infield is excluded."""
    ls = polyline_region.lineString
    band = _closed_centerline_to_band(ls, buffer_m)
    if band is None:
        buf = ls.buffer(buffer_m)
        if buf.is_empty:
            buf = ls.buffer(buffer_m + 1e-6)
        return regionFromShapelyObject(buf)
    return regionFromShapelyObject(band)


def _centerlines_to_buffered_region(
    centerlines: List[Any], buffer_m: float
) -> Optional[PolygonalRegion]:
    """Union of track bands (centerlines with inner boundary so infield excluded)."""
    if not centerlines:
        return None
    from shapely.geometry import LineString
    import shapely.ops

    polys = []
    for cl in centerlines:
        if hasattr(cl, "lineString"):
            ls = cl.lineString
        elif hasattr(cl, "coords"):
            ls = LineString(cl.coords)
        else:
            continue
        band = _closed_centerline_to_band(ls, buffer_m)
        if band is None:
            b = ls.buffer(buffer_m)
            if not b.is_empty:
                polys.append(b)
        else:
            polys.append(band)
    if not polys:
        return None
    if len(polys) == 1:
        return regionFromShapelyObject(polys[0])
    union = shapely.ops.unary_union(polys)
    return regionFromShapelyObject(union)


def build_track_regions_from_opendrive(
    track: Any,
    main_buffer_m: float = MAIN_TRACK_BUFFER_M,
    pit_buffer_m: float = PIT_TRACK_BUFFER_M,
    use_road_polygons: bool = True,
) -> Tuple[Optional[Any], Optional[Any]]:
    """Build mainTrack and pitTrack from an existing RacingTrack (OpenDRIVE).

    SD-19a: by default uses road polygons directly (Road inherits from
    NetworkElement -> PolygonalRegion in driving/roads.py). The polygon
    already encodes the road's full drivable width from XODR
    (lane widths, junctions, etc.), so we just take the union per
    side and apply the main-wins-on-overlap rule. Buffer arguments
    are accepted for back-compat but ignored on the polygon path.

    When `use_road_polygons=False` (or when polygon-side fails), falls
    back to the legacy centerline-buffer approach which uses
    `main_buffer_m` / `pit_buffer_m`.

    Args:
        track: RacingTrack with _mainRacingRoads and _pitRoads.
        main_buffer_m: Buffer (legacy fallback only). Default 6.
        pit_buffer_m: Buffer (legacy fallback only). Default 1.5.
        use_road_polygons: If True (default), use Road.polygon via
            PolygonalRegion.unionAll. If False, force the legacy
            centerline-buffer path.

    Returns:
        (mainTrack, pitTrack) as Scenic Regions, or (None, None) if no geometry.
    """
    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])

    if use_road_polygons and (main_roads or pit_roads):
        try:
            main_track = PolygonalRegion.unionAll(main_roads) if main_roads else None
            pit_track = PolygonalRegion.unionAll(pit_roads) if pit_roads else None
            # PolygonalRegion.unionAll returns `nowhere` for empty input.
            if main_track is nowhere:
                main_track = None
            if pit_track is nowhere:
                pit_track = None
            # Main wins on overlap.
            if pit_track is not None and main_track is not None:
                pit_track = pit_track.difference(main_track)
            return (main_track, pit_track)
        except Exception as exc:
            print(
                f"[TrackRegions] road-polygon union failed ({exc}); "
                f"falling back to centerline-buffer (legacy path)"
            )

    # Legacy fallback: buffer each road's centerline by main/pit buffer.
    from scenic.domains.racing.segments.segment_map import _get_road_centerline

    main_centerlines = []
    for road in main_roads:
        cl = _get_road_centerline(road)
        if cl is not None:
            main_centerlines.append(cl)

    pit_centerlines = []
    for road in pit_roads:
        cl = _get_road_centerline(road)
        if cl is not None:
            pit_centerlines.append(cl)

    main_track = _centerlines_to_buffered_region(main_centerlines, main_buffer_m)
    pit_track = _centerlines_to_buffered_region(pit_centerlines, pit_buffer_m)
    if pit_track is not None and main_track is not None:
        pit_track = pit_track.difference(main_track)

    return (main_track, pit_track)


def build_track_regions_from_ttl(
    ttl_folder: Path,
    main_file: Optional[str] = None,
    pit_file: Optional[str] = None,
    main_buffer_m: float = MAIN_TRACK_BUFFER_M,
    pit_buffer_m: float = PIT_TRACK_BUFFER_M,
) -> Tuple[Optional[Any], Optional[Any]]:
    """Build mainTrack and pitTrack from TTL centerline CSVs.

    Args:
        ttl_folder: Folder containing main_file and optionally pit_file.
        main_file: CSV filename for main road centerline. Default: try
            ttl_main_road_xodr.csv (XODR-derived true centerline, NEW XODR frame --
            aligns with LGS_v1.xodr map for visualization), fall back to
            ttl_main_road.csv (empirical, RD frame -- preserves OLD-map workflow).
        pit_file: CSV filename for pit lane centerline. Default behaves the same
            way (ttl_pitlane_xodr.csv preferred, ttl_pitlane.csv fallback).
        main_buffer_m: Buffer in meters on each side of main centerline (default 6).
        pit_buffer_m: Buffer in meters on each side of pit centerline (default 1.5).

    Returns:
        (mainTrack, pitTrack) as Scenic Regions. pitTrack is None if pit CSV missing or invalid.
    """
    folder_path = Path(ttl_folder)
    # Prefer XODR-derived centerline (new XODR frame, geometric centerline of
    # race_common track boundaries). Empirical CSVs are fallback for OLD-map
    # workflow. See docs/frames.md.
    if main_file is None:
        main_file = ("ttl_main_road_xodr.csv"
                     if (folder_path / "ttl_main_road_xodr.csv").is_file()
                     else "ttl_main_road.csv")
    if pit_file is None:
        pit_file = ("ttl_pitlane_xodr.csv"
                    if (folder_path / "ttl_pitlane_xodr.csv").is_file()
                    else "ttl_pitlane.csv")

    main_pts = _load_ttl_csv(folder_path, main_file)
    if not main_pts or len(main_pts) < 2:
        return (None, None)

    main_poly = _polyline_from_waypoints(main_pts)
    if main_poly is None:
        return (None, None)

    main_track = _buffer_polyline_region(main_poly, main_buffer_m)

    pit_pts = _load_ttl_csv(Path(ttl_folder), pit_file)
    pit_track = None
    if pit_pts and len(pit_pts) >= 2:
        pit_poly = _polyline_from_waypoints(pit_pts)
        if pit_poly is not None:
            pit_track = _buffer_polyline_region(pit_poly, pit_buffer_m)

    # Overlap rule (same as segments): main wins; pitTrack = pit minus main so regions are mutually exclusive
    if pit_track is not None and main_track is not None:
        pit_track = pit_track.difference(main_track)

    return (main_track, pit_track)


def create_ttl_region_from_file(
    ttl_folder: Optional[Any],
    ttl_file_name: str,
) -> Optional[PolylineRegion]:
    """Build a PolylineRegion from one TTL centerline CSV.

    SD-19b: returns the PolylineRegion directly (no buffer). Per-vehicle
    TTL placement (`new RacingCar with ttlFileName 'X'` and the
    RacingCar default `position: new Point on ttlRegion(self.ttlFileName)`)
    now lands EXACTLY on the racing-line waypoint chain. Lateral offset
    becomes a separate concern (Frenet, see Ask 1 in the deck).

    Use for placement like:
        new RacingCar on ttlRegion('ttl_optimal_xodr.csv')

    Args:
        ttl_folder: Folder containing the TTL CSV (Path or str). If None, returns None.
        ttl_file_name: CSV filename (e.g. 'ttl_main_road.csv', 'ttl_optimal_xodr.csv').

    Returns:
        PolylineRegion (the TTL centerline polyline) or None if folder/file invalid.
    """
    if ttl_folder is None or not str(ttl_folder).strip():
        return None
    folder = Path(ttl_folder)
    if not folder.is_absolute():
        folder = folder.resolve()
    pts = _load_ttl_csv(folder, ttl_file_name)
    if not pts or len(pts) < 2:
        return None
    return _polyline_from_waypoints(pts)


# --------------------------------------------------------------------------
# SD-24: curve/straight track regions
# --------------------------------------------------------------------------

# Slivers smaller than this (m^2) are dropped after polygon slicing — they
# are usually polygon-arithmetic artifacts at cut-line intersections, not
# meaningful track sections.
_MIN_SLICE_AREA_M2 = 1.0

# Length (m) of the perpendicular cut line at each segment boundary.
# Needs to comfortably exceed any road's drivable width. Laguna Seca's
# widest section is ~12 m; 200 m is overkill-safe for any race track.
_CUT_LINE_LENGTH_M = 200.0


def _make_perpendicular_cut(line_string, s_value: float, length: float = _CUT_LINE_LENGTH_M):
    """Build a perpendicular cut line through the centerline at arc-length ``s_value``.

    The line passes through ``line_string.interpolate(s_value)`` and is
    perpendicular to the local tangent direction. Returns a Shapely
    ``LineString`` of length ``length``, centered on the cut point, or
    ``None`` if the cut would be at the very start or end of the centerline
    (no interior tangent there).
    """
    from shapely.geometry import LineString

    if s_value <= 0 or s_value >= line_string.length:
        return None
    pt = line_string.interpolate(s_value)
    # Local tangent: small offset before and after the cut point.
    eps = min(0.5, line_string.length / 1000.0)
    s_a = max(0.0, s_value - eps)
    s_b = min(line_string.length, s_value + eps)
    pt_a = line_string.interpolate(s_a)
    pt_b = line_string.interpolate(s_b)
    dx = pt_b.x - pt_a.x
    dy = pt_b.y - pt_a.y
    norm = (dx * dx + dy * dy) ** 0.5
    if norm < 1e-9:
        return None
    # Rotate tangent (dx, dy) by 90deg to get perpendicular direction.
    perp_x = -dy / norm
    perp_y = dx / norm
    half = length / 2.0
    p1 = (pt.x - perp_x * half, pt.y - perp_y * half)
    p2 = (pt.x + perp_x * half, pt.y + perp_y * half)
    return LineString([p1, p2])


def _label_at_arc_length(segments: List[Tuple[float, float, str]], s_value: float) -> str:
    """Return the segment label whose ``[s_start, s_end]`` range contains ``s_value``.

    Falls back to the last segment's label if ``s_value`` is past the end
    (within Shapely's projection tolerance).
    """
    for s_start, s_end, label in segments:
        if s_start <= s_value <= s_end:
            return label
    return segments[-1][2] if segments else "straight"


def slice_road_polygon_at_segments(
    road: Any,
    segments: List[Tuple[float, float, str]],
    road_category: str,
) -> List[dict]:
    """Subdivide a road's drivable polygon into per-segment slices.

    A single XODR road can contain multiple curve/straight segments (e.g.
    Laguna Seca's main loop is one road with curves AND straights all
    over it). This helper takes the road's drivable polygon and partitions
    it at perpendicular cut lines drawn through each interior segment
    boundary, then labels each resulting piece by projecting its centroid
    onto the centerline and looking up which segment's arc-length range
    contains the projected ``s``.

    Args:
        road: A Scenic ``Road`` object with ``.polygon`` (Shapely
            ``Polygon`` / ``MultiPolygon``) and a centerline obtainable
            via ``segment_map._get_road_centerline(road)``.
        segments: Output of ``_build_curve_straight_segments`` for this
            road's centerline. Each tuple is ``(s_start, s_end, label)``
            where ``label in {'curve', 'straight'}``.
        road_category: ``'main'`` or ``'pit'`` — the parent track side.

    Returns:
        A list of dicts ``{'label': str, 'polygon': shapely.Polygon,
        'category': str}``. May be empty if the road has no usable
        centerline or polygon. Slivers smaller than ``_MIN_SLICE_AREA_M2``
        are dropped (polygon-arithmetic artifacts).
    """
    from scenic.domains.racing.segments.segment_map import _get_road_centerline
    from shapely.ops import split

    polygon = getattr(road, "polygon", None)
    if polygon is None or polygon.is_empty:
        return []
    centerline = _get_road_centerline(road)
    if centerline is None or not segments:
        return []
    line_string = getattr(centerline, "lineString", None)
    if line_string is None or line_string.length < 1e-6:
        return []

    # Single-segment road: no slicing needed.
    if len(segments) == 1:
        return [{
            "label": segments[0][2],
            "polygon": polygon,
            "category": road_category,
        }]

    # Build cut lines at every interior boundary.
    cut_lines = []
    for i in range(len(segments) - 1):
        s_b = float(segments[i][1])  # equals segments[i+1][0]
        cut = _make_perpendicular_cut(line_string, s_b)
        if cut is not None:
            cut_lines.append(cut)

    # Iteratively split. Each cut may fail to cross a piece (e.g. it lies
    # outside that piece's bounds) — Shapely returns the piece intact in
    # that case, which is fine.
    pieces = [polygon]
    for cl in cut_lines:
        new_pieces = []
        for piece in pieces:
            try:
                result = split(piece, cl)
                geoms = list(getattr(result, "geoms", [result]))
                new_pieces.extend(geoms)
            except Exception:
                new_pieces.append(piece)
        pieces = new_pieces

    # Label each piece by centroid arc-length.
    out: List[dict] = []
    for piece in pieces:
        if piece.is_empty or piece.area < _MIN_SLICE_AREA_M2:
            continue
        try:
            s_val = float(line_string.project(piece.centroid))
        except Exception:
            # Centroid projection failed; assign to whichever segment is
            # closer to the piece's representative point.
            try:
                rp = piece.representative_point()
                s_val = float(line_string.project(rp))
            except Exception:
                s_val = 0.0
        label = _label_at_arc_length(segments, s_val)
        out.append({
            "label": label,
            "polygon": piece,
            "category": road_category,
        })
    return out


def build_curve_straight_regions_from_opendrive(track: Any) -> dict:
    """Build the six curve/straight Scenic regions from a RacingTrack.

    Runs the per-station classifier from ``segments/segment_map.py`` over
    every main + pit road, slices each road's polygon at the segment
    boundaries (via :func:`slice_road_polygon_at_segments`), and unions
    the slices into:

    - ``curve``: all curve slices, both pit + main
    - ``straight``: all straight slices, both pit + main
    - ``mainCurve``: curve slices on main racing roads only
    - ``mainStraight``: straight slices on main racing roads only
    - ``pitCurve``: curve slices on pit roads only (often empty at LGS)
    - ``pitStraight``: straight slices on pit roads only

    Pit regions are subtracted by the union of main slices to enforce the
    same "main wins on overlap" rule used by ``mainTrack`` / ``pitTrack``.

    Returns:
        dict mapping each region name above to a Scenic ``Region``
        (``PolygonalRegion`` or ``nowhere`` if the corresponding bucket
        was empty). Always returns all six keys.
    """
    from scenic.domains.racing.segments.segment_map import (
        _build_curve_straight_segments,
        _get_road_centerline,
    )
    import shapely.ops

    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])

    # Bucket per (category, label).
    buckets: dict = {
        ("main", "curve"): [],
        ("main", "straight"): [],
        ("pit", "curve"): [],
        ("pit", "straight"): [],
    }
    for road in main_roads:
        cl = _get_road_centerline(road)
        if cl is None:
            continue
        segs = _build_curve_straight_segments(cl)
        for slc in slice_road_polygon_at_segments(road, segs, "main"):
            buckets[("main", slc["label"])].append(slc["polygon"])
    for road in pit_roads:
        cl = _get_road_centerline(road)
        if cl is None:
            continue
        segs = _build_curve_straight_segments(cl)
        for slc in slice_road_polygon_at_segments(road, segs, "pit"):
            buckets[("pit", slc["label"])].append(slc["polygon"])

    def _union_to_region(polys):
        if not polys:
            return None
        if len(polys) == 1:
            return regionFromShapelyObject(polys[0])
        return regionFromShapelyObject(shapely.ops.unary_union(polys))

    main_curve = _union_to_region(buckets[("main", "curve")])
    main_straight = _union_to_region(buckets[("main", "straight")])
    pit_curve = _union_to_region(buckets[("pit", "curve")])
    pit_straight = _union_to_region(buckets[("pit", "straight")])

    # Main-wins-on-overlap: subtract the union of main slices from each pit
    # region. Mirrors the same rule applied to mainTrack / pitTrack at
    # build_track_regions_from_opendrive.
    main_union_polys = (
        buckets[("main", "curve")] + buckets[("main", "straight")]
    )
    if main_union_polys:
        main_union_geom = shapely.ops.unary_union(main_union_polys)
        main_union_region = regionFromShapelyObject(main_union_geom)
        if pit_curve is not None:
            try:
                pit_curve = pit_curve.difference(main_union_region)
            except Exception:
                pass
        if pit_straight is not None:
            try:
                pit_straight = pit_straight.difference(main_union_region)
            except Exception:
                pass

    # Axis regions: union of main + pit slices per label.
    curve_polys = buckets[("main", "curve")] + buckets[("pit", "curve")]
    straight_polys = buckets[("main", "straight")] + buckets[("pit", "straight")]
    curve = _union_to_region(curve_polys)
    straight = _union_to_region(straight_polys)

    return {
        "curve": curve if curve is not None else nowhere,
        "straight": straight if straight is not None else nowhere,
        "mainCurve": main_curve if main_curve is not None else nowhere,
        "mainStraight": main_straight if main_straight is not None else nowhere,
        "pitCurve": pit_curve if pit_curve is not None else nowhere,
        "pitStraight": pit_straight if pit_straight is not None else nowhere,
    }


def create_track_regions(
    map_file: Optional[str] = None,
    ttl_folder: Optional[str] = None,
    track: Optional[Any] = None,
    main_buffer_m: float = MAIN_TRACK_BUFFER_M,
    pit_buffer_m: float = PIT_TRACK_BUFFER_M,
    prefer_ttl_track_regions: bool = False,
    **create_track_kw,
) -> Tuple[Any, Any, Optional[Any]]:
    """Build mainTrack and pitTrack from XODR-native road geometry by default.

    Phase B.5 (2026-04-26): switched default to OpenDRIVE-native. The OLD path
    used the empirical centerline CSVs (``ttl_main_road.csv`` / ``ttl_pitlane.csv``)
    which were measured by driving sessions in dSPACE -- slow to produce, brittle
    when the map changed, and not actually a geometric centerline (varied 1.9-55m
    from race_common's inner edge). Verified on ``LGS_v1.xodr``: XODR-native
    mainTrack bbox matches race_common's ``outside.csv`` within 0.83m mean,
    area 21764 m^2 (~ 3600m lap x ~6m half-width). See ``tools/frames/verify_xodr_native_regions.py``.

    Args:
        map_file: Path to OpenDRIVE ``.xodr``. Used to ``createRacingTrack`` if ``track`` is None.
        ttl_folder: Path to TTL folder. ONLY used as a fallback when ``track`` is None and
            ``map_file`` is also None, OR when ``prefer_ttl_track_regions=True`` (legacy).
        track: Existing ``RacingTrack`` (optional). Preferred -- avoids re-parsing XODR.
        main_buffer_m: Buffer each side of main road centerline (default 6).
        pit_buffer_m: Buffer each side of pit road centerline (default 1.5).
        prefer_ttl_track_regions: Legacy flag to force TTL-CSV-buffered regions even when
            XODR is available. Default False (use XODR). Set to True only for back-compat
            with old empirical-centerline workflow.
        **create_track_kw: Passed to ``createRacingTrack`` when creating track from map_file.

    Returns:
        ``(mainTrack, pitTrack, track)``. ``mainTrack`` / ``pitTrack`` are Regions;
        ``track`` is the ``RacingTrack`` when XODR-derived, else None.
    """
    from scenic.domains.racing.segments.tracks import createRacingTrack

    # XODR-native path (default): use ``track`` (or build it from map_file).
    if not prefer_ttl_track_regions:
        if track is None and map_file:
            track = createRacingTrack(map_file, **create_track_kw)
        if track is not None:
            main_track, pit_track = build_track_regions_from_opendrive(
                track, main_buffer_m=main_buffer_m, pit_buffer_m=pit_buffer_m
            )
            if main_track is not None:
                # Loud startup log so the active path is unambiguous in any run log.
                print(f"[TrackRegions] mainTrack/pitTrack source = XODR road polygons "
                      f"(width-aware drivable area; centerline-buffer fallback unused)")
                return (main_track or nowhere, pit_track or nowhere, track)
            print("[TrackRegions] XODR path returned empty regions; falling back to TTL CSV path")
            # XODR path returned empty -- fall through to TTL fallback below.

    # TTL fallback (legacy or XODR-empty case).
    if ttl_folder is not None and str(ttl_folder).strip():
        folder = Path(ttl_folder)
        if not folder.is_absolute():
            folder = folder.resolve()
        main_track, pit_track = build_track_regions_from_ttl(
            folder,
            main_buffer_m=main_buffer_m,
            pit_buffer_m=pit_buffer_m,
        )
        why = "preferTtlTrackRegions=True (legacy)" if prefer_ttl_track_regions else "XODR path empty (fallback)"
        print(f"[TrackRegions] mainTrack/pitTrack source = TTL CSV ({why}, "
              f"+/-{main_buffer_m}m main, +/-{pit_buffer_m}m pit)")
        return (main_track or nowhere, pit_track or nowhere, track)

    # No XODR, no TTL -- empty regions.
    if track is None and map_file:
        track = createRacingTrack(map_file, **create_track_kw)
    if track is None:
        return (nowhere, nowhere, None)

    main_track, pit_track = build_track_regions_from_opendrive(
        track, main_buffer_m=main_buffer_m, pit_buffer_m=pit_buffer_m
    )
    return (main_track or nowhere, pit_track or nowhere, track)
