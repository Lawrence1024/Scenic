"""Per-step ground-truth bounds check against race_common track boundaries.

Diagnostic-only. Loads the race_common geofences once
(``assets/ttls/LS_ENU_TTL_CSV/track_inside.csv`` and ``track_outside.csv``,
both in canonical LS_ENU frame matching ``LGS_v1.xodr``'s ``<geoReference>``)
and computes per-tick distance from a position (in NEW XODR / canonical ENU
frame) to the inner and outer track polylines.

Used by the simulator's ``getProperties`` to emit ``[BoundsCheck]`` lines so
out-of-bounds excursions can be spotted at finer cadence than the existing
``[Phase0Event] type=off_track`` (which only fires when CTE>=10m).

Also exposes ``gps_to_xodr`` for converting dSPACE GPS readback (lon, lat)
into the new-XODR-frame position via pyproj using the XODR's ``<geoReference>``
proj string. Per-step comparison of this GPS-derived position against the
translation-derived position tells us whether the ``frame_calibration``
single-translation fit holds across the whole lap, or whether there's a
location-dependent error (rotation/scale we ignored).

Identity-friendly: if the boundary CSVs are missing (e.g. OLD-map workflow),
``compute_bounds_distance`` returns None and nothing is logged.
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional, Tuple

# Module-level cache: lazy load on first use.
_INSIDE = None      # numpy array or "missing" sentinel
_OUTSIDE = None
_LOADED = False


def _ttl_dir() -> Path:
    # __file__ = src/scenic/simulators/dspace/geometry/bounds_check.py -> parents[5] = repo root.
    return (Path(__file__).resolve().parents[5]
            / "assets" / "ttls" / "LS_ENU_TTL_CSV")


def _load_xy(p: Path):
    """Load Easting/Northing or x/y from CSV; returns Nx2 numpy array or None."""
    try:
        import numpy as np
    except ImportError:
        return None
    if not p.is_file():
        return None
    pts = []
    with open(p, "r", encoding="utf-8") as f:
        rd = csv.reader(f)
        header = next(rd, [])
        cols = [c.strip().lower() for c in header]
        if "easting" in cols and "northing" in cols:
            ix, iy = cols.index("easting"), cols.index("northing")
        elif "x" in cols and "y" in cols:
            ix, iy = cols.index("x"), cols.index("y")
        else:
            ix, iy = 0, 1
        for row in rd:
            if len(row) <= max(ix, iy):
                continue
            try:
                pts.append((float(row[ix]), float(row[iy])))
            except ValueError:
                continue
    if len(pts) < 2:
        return None
    return np.asarray(pts, dtype=float)


def _ensure_loaded() -> None:
    global _INSIDE, _OUTSIDE, _LOADED
    if _LOADED:
        return
    _LOADED = True
    d = _ttl_dir()
    _INSIDE = _load_xy(d / "track_inside.csv")
    _OUTSIDE = _load_xy(d / "track_outside.csv")


def _min_dist_to_polyline(p, poly) -> float:
    """Min distance from point p (shape (2,)) to polyline (shape (N, 2)). Vectorized."""
    import numpy as np
    seg_a = poly[:-1]
    seg_b = poly[1:]
    v = seg_b - seg_a
    L2 = (v * v).sum(axis=1)
    L2_safe = np.where(L2 < 1e-12, 1.0, L2)
    u = ((p - seg_a) * v).sum(axis=1) / L2_safe
    uc = np.clip(u, 0.0, 1.0)
    foot = seg_a + uc[:, None] * v
    d = np.linalg.norm(p - foot, axis=1)
    d = np.where(L2 < 1e-12, np.linalg.norm(p - seg_a, axis=1), d)
    return float(np.min(d))


def _point_in_polygon(p, poly) -> bool:
    """Even-odd ray cast for polygon containment. p shape (2,), poly shape (N, 2)."""
    x, y = float(p[0]), float(p[1])
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi):
            inside = not inside
        j = i
    return inside


_GPS_TRANSFORMER_CACHE: dict = {}


def gps_to_xodr(lon: float, lat: float, xodr_path) -> Optional[Tuple[float, float]]:
    """Convert GPS (lon, lat) to XODR-xy via pyproj using the XODR's geoReference.

    Returns None if pyproj is unavailable, the XODR has no ``<geoReference>``,
    or the proj string can't be parsed. Result is cached per xodr_path.
    """
    if xodr_path is None:
        return None
    try:
        import pyproj
        import xml.etree.ElementTree as ET
    except ImportError:
        return None
    p = Path(xodr_path)
    key = str(p.resolve())
    transformer = _GPS_TRANSFORMER_CACHE.get(key)
    if transformer is None:
        if key in _GPS_TRANSFORMER_CACHE:
            return None  # cached "no transformer" result
        try:
            tree = ET.parse(p)
            geo = tree.getroot().find("header/geoReference")
            proj_str = (geo.text or "").strip() if geo is not None else ""
            if "+proj=" not in proj_str:
                _GPS_TRANSFORMER_CACHE[key] = None
                return None
            src = pyproj.CRS.from_proj4("+proj=longlat +datum=WGS84 +no_defs")
            dst = pyproj.CRS.from_proj4(proj_str)
            transformer = pyproj.Transformer.from_crs(src, dst, always_xy=True)
            _GPS_TRANSFORMER_CACHE[key] = transformer
        except Exception:
            _GPS_TRANSFORMER_CACHE[key] = None
            return None
    try:
        x, y = transformer.transform(float(lon), float(lat))
        return float(x), float(y)
    except Exception:
        return None


def compute_bounds_distance(x: float, y: float) -> Optional[Tuple[float, float, bool]]:
    """Return (d_inside, d_outside, in_track) for a point in canonical ENU frame.

    ``in_track`` is True iff the point is inside the outer polygon AND outside
    the inner polygon. Returns None if the boundary CSVs were not found
    (e.g. OLD-map workflow with no race_common assets mirrored locally).
    """
    _ensure_loaded()
    if _INSIDE is None or _OUTSIDE is None:
        return None
    try:
        import numpy as np
    except ImportError:
        return None
    p = np.asarray([float(x), float(y)], dtype=float)
    d_in = _min_dist_to_polyline(p, _INSIDE)
    d_out = _min_dist_to_polyline(p, _OUTSIDE)
    in_track = _point_in_polygon(p, _OUTSIDE) and not _point_in_polygon(p, _INSIDE)
    return d_in, d_out, in_track
