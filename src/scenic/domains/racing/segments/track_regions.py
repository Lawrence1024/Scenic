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
) -> Tuple[Optional[Any], Optional[Any]]:
    """Build mainTrack and pitTrack from an existing RacingTrack (OpenDRIVE).

    Args:
        track: RacingTrack with _mainRacingRoads and _pitRoads (each road has lanes with centerlines).
        main_buffer_m: Buffer in meters on each side of main segment centerlines (default 6).
        pit_buffer_m: Buffer in meters on each side of pit segment centerlines (default 1.5).

    Returns:
        (mainTrack, pitTrack) as Scenic Regions (PolygonalRegion), or (None, None) if no geometry.
    """
    from scenic.domains.racing.segments.segment_map import _get_road_centerline

    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])

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
    # Overlap rule (same as segments): main wins; pitTrack = pit minus main
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
    buffer_m: float = MAIN_TRACK_BUFFER_M,
) -> Optional[Any]:
    """Build a single Region from one TTL centerline CSV (random point on that TTL).

    Use for placement like: new RacingCar on ttl  (uses scene ttlFileName)
    or with a specific file: new RacingCar on ttlRegion('ttl_optimal_xodr.csv')

    Args:
        ttl_folder: Folder containing the TTL CSV (Path or str). If None, returns None.
        ttl_file_name: CSV filename (e.g. 'ttl_main_road.csv', 'ttl_optimal_xodr.csv').
        buffer_m: Meters on each side of centerline (default 6.0).

    Returns:
        PolygonalRegion (buffered centerline) or None if folder/file invalid.
    """
    if ttl_folder is None or not str(ttl_folder).strip():
        return None
    folder = Path(ttl_folder)
    if not folder.is_absolute():
        folder = folder.resolve()
    pts = _load_ttl_csv(folder, ttl_file_name)
    if not pts or len(pts) < 2:
        return None
    poly = _polyline_from_waypoints(pts)
    if poly is None:
        return None
    return _buffer_polyline_region(poly, buffer_m)


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
                print(f"[TrackRegions] mainTrack/pitTrack source = XODR (Scenic Network "
                      f"road centerlines, +/-{main_buffer_m}m main, +/-{pit_buffer_m}m pit)")
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
