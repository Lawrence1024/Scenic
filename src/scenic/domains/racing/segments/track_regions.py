"""Build mainTrack and pitTrack regions from segment centerlines (OpenDRIVE or TTL).

Track regions are buffered centerlines with width:
- mainTrack: main road segments with 6m on each side of the centerline (default).
- pitTrack: pit lane segments with 3.25m on each side of the centerline (default).

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
PIT_TRACK_BUFFER_M = 3.25


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
        pit_buffer_m: Buffer in meters on each side of pit segment centerlines (default 3.25).

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
    main_file: str = "ttl_main_road.csv",
    pit_file: str = "ttl_pitlane.csv",
    main_buffer_m: float = MAIN_TRACK_BUFFER_M,
    pit_buffer_m: float = PIT_TRACK_BUFFER_M,
) -> Tuple[Optional[Any], Optional[Any]]:
    """Build mainTrack and pitTrack from TTL centerline CSVs.

    Args:
        ttl_folder: Folder containing main_file and optionally pit_file.
        main_file: CSV filename for main road centerline (default ttl_main_road.csv).
        pit_file: CSV filename for pit lane centerline (default ttl_pitlane.csv).
        main_buffer_m: Buffer in meters on each side of main centerline (default 6).
        pit_buffer_m: Buffer in meters on each side of pit centerline (default 3.25).

    Returns:
        (mainTrack, pitTrack) as Scenic Regions. pitTrack is None if pit CSV missing or invalid.
    """
    main_pts = _load_ttl_csv(Path(ttl_folder), main_file)
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


def create_track_regions(
    map_file: Optional[str] = None,
    ttl_folder: Optional[str] = None,
    track: Optional[Any] = None,
    main_buffer_m: float = MAIN_TRACK_BUFFER_M,
    pit_buffer_m: float = PIT_TRACK_BUFFER_M,
    **create_track_kw,
) -> Tuple[Any, Any, Optional[Any]]:
    """Build mainTrack and pitTrack from either OpenDRIVE or TTL.

    If ttl_folder is set, mainTrack and pitTrack are built from TTL centerline CSVs.
    Otherwise they are built from the OpenDRIVE track (either provided or created from map_file).

    Args:
        map_file: Path to OpenDRIVE .xodr (required if track is None and ttl_folder is None).
        ttl_folder: Path to folder with ttl_main_road.csv and ttl_pitlane.csv. If set, TTL is used for track regions.
        track: Existing RacingTrack (optional). If None and not using TTL, one is created from map_file.
        main_buffer_m: Buffer each side of main centerline in meters (default 6).
        pit_buffer_m: Buffer each side of pit centerline in meters (default 3.25).
        **create_track_kw: Passed to createRacingTrack when creating track from map_file.

    Returns:
        (mainTrack, pitTrack, track). mainTrack/pitTrack are Regions; track is the RacingTrack when from OpenDRIVE else None.
    """
    from scenic.domains.racing.segments.tracks import createRacingTrack

    if ttl_folder is not None and str(ttl_folder).strip():
        folder = Path(ttl_folder)
        if not folder.is_absolute():
            folder = folder.resolve()
        main_track, pit_track = build_track_regions_from_ttl(
            folder,
            main_buffer_m=main_buffer_m,
            pit_buffer_m=pit_buffer_m,
        )
        return (main_track or nowhere, pit_track or nowhere, None)

    # OpenDRIVE path: need track
    if track is None:
        if not map_file:
            return (nowhere, nowhere, None)
        track = createRacingTrack(map_file, **create_track_kw)

    main_track, pit_track = build_track_regions_from_opendrive(
        track, main_buffer_m=main_buffer_m, pit_buffer_m=pit_buffer_m
    )
    return (main_track or nowhere, pit_track or nowhere, track)
