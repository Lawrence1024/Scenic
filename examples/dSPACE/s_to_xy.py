from typing import Tuple, Optional
from pathlib import Path
from scenic.simulators.dspace import utils as dutils
from scenic.formats.opendrive import xodr_parser

try:
    from shapely.geometry import Point
    from shapely.ops import unary_union, nearest_points
    SHAPELY_OK = True
except Exception:
    SHAPELY_OK = False

_refline_cache = {}
_drivable_cache = {}

def _get_refline_and_length(xodr_path: Path, step: float):
    key = (str(xodr_path.resolve()), float(step))
    if key not in _refline_cache:
        refline, total_length = dutils.build_circuit_refline(xodr_path, step=step)
        _refline_cache[key] = (refline, float(total_length))
    return _refline_cache[key]

def get_xy_from_s(
    xodr_path: Path,
    s_meters: float,
    step: float = 2.0,
    t_offset: float = 0.0,
) -> Tuple[float, float]:
    refline, L = _get_refline_and_length(xodr_path, step)
    s_norm = float(s_meters) % L
    x, y, _z = dutils.st_to_world(refline, s_norm, t=t_offset)
    return float(x), float(y)

def get_xy_heading_from_s(
    xodr_path: Path,
    s_meters: float,
    step: float = 2.0,
    t_offset: float = 0.0,
    h_eps: float = 0.5,
) -> Tuple[float, float, float]:
    refline, L = _get_refline_and_length(xodr_path, step)
    s0 = float(s_meters) % L
    x0, y0, _ = dutils.st_to_world(refline, s0, t=t_offset)
    sp = (s0 + h_eps) % L
    sm = (s0 - h_eps) % L
    xp, yp, _ = dutils.st_to_world(refline, sp, t=t_offset)
    xm, ym, _ = dutils.st_to_world(refline, sm, t=t_offset)
    import math
    heading = math.atan2(yp - ym, xp - xm)
    return float(x0), float(y0), float(heading)

def scenic_car_at_from_s(
    xodr_path: Path,
    s_meters: float,
    *,
    step: float = 2.0,
    t_offset: float = 0.0,
    z: float = 0.0,
    name: Optional[str] = None,
) -> str:
    x, y = get_xy_from_s(xodr_path, s_meters, step=step, t_offset=t_offset)
    prefix = f"{name} = " if name else ""
    return f"{prefix}new Car at ({x:.6f}, {y:.6f}, {z:.6f})"


def _get_drivable_geom(xodr_path: Path):
    if not SHAPELY_OK:
        return None
    key = str(xodr_path.resolve())
    if key in _drivable_cache:
        return _drivable_cache[key]
    # 解析 XODR
    road_map = xodr_parser.RoadMap()
    road_map.parse(str(xodr_path))
    road_map.calculate_geometry(num=100, calc_gap=False, calc_intersect=False)
    pieces = []
    for attr in ("drivable_region", "drivableRegion"):
        geom = getattr(road_map, attr, None)
        if geom is not None:
            pieces.append(geom)
    roads = getattr(road_map, "roads", None)
    if roads:
        iterable = roads.values() if isinstance(roads, dict) else roads
        for r in iterable:
            for attr in ("drivable_region", "drivableRegion"):
                g = getattr(r, attr, None)
                if g is not None:
                    pieces.append(g)
    if not pieces:
        _drivable_cache[key] = None
        return None
    unioned = unary_union(pieces).buffer(0)  # 清拓扑
    _drivable_cache[key] = unioned
    return unioned

import math
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points

def _pick_inward_normal(drivable, x: float, y: float, heading: float) -> tuple[float, float]:
    """Choose the normal direction that points into the drivable area near (x,y)."""
    # Unit normals (left/right) from heading
    n_left  = (-math.sin(heading),  math.cos(heading))
    n_right = ( math.sin(heading), -math.cos(heading))
    pt = Point(x, y)
    # Small probe step to test which side is interior
    probe = 0.25
    pt_L = Point(x + n_left[0]*probe,  y + n_left[1]*probe)
    pt_R = Point(x + n_right[0]*probe, y + n_right[1]*probe)
    in_L = drivable.contains(pt_L)
    in_R = drivable.contains(pt_R)
    if in_L and not in_R:
        return n_left
    if in_R and not in_L:
        return n_right
    # If both or neither are True, prefer the side with larger distance *into* drivable
    # (fallback: just pick left)
    try:
        _, nL = nearest_points(pt_L, drivable)
        _, nR = nearest_points(pt_R, drivable)
        dL = pt_L.distance(nL)
        dR = pt_R.distance(nR)
        return n_left if dL >= dR else n_right
    except Exception:
        return n_left

import math
from typing import Tuple, Optional
from shapely.geometry import Point

import math
from typing import Tuple, Optional
from shapely.geometry import Point

def safe_centerline_xy_from_s(
    xodr_path: Path,
    s_meters: float,
    *,
    step: float = 2.0,
    car_width: float = 1.9,
    extra_margin: float = 0.1,
    search_window: float = 10.0,   # max ± search along s (meters)
    search_step: float = 0.25,     # s increment for the search (meters)
) -> Tuple[float, float, float]:
    """
    Return (x, y, s_adj) on the *centerline* (t=0) such that the entire car fits.

    Strategy:
      1) Build inner drivable polygon = drivable.buffer(-(car_width/2 + extra_margin)).
      2) Try the requested s. If centerline point is inside the inner polygon, accept.
      3) Otherwise, search along s in ±search_step increments up to search_window,
         always keeping t=0 (strict midline). Return the first admissible point.
      4) If nothing admissible is found, fall back to the original s (no fit guaranteed).
    """
    # 1) refline / track length
    refline, L = _get_refline_and_length(xodr_path, step)
    s0 = float(s_meters) % L

    # 2) build inner drivable polygon
    drivable = _get_drivable_geom(xodr_path)
    if (drivable is None) or (not SHAPELY_OK):
        # Shapely not available or no drivable geometry: return raw centerline point
        x0, y0, _ = dutils.st_to_world(refline, s0, t=0.0)
        return float(x0), float(y0), s0

    shrink = max(0.0, (car_width * 0.5) + float(extra_margin))
    inner = drivable.buffer(-shrink)
    # If shrink makes it empty (very narrow lanes), accept the unshrunk polygon for feasibility,
    # but we still try to honor the centerline strictly.
    target = inner if not inner.is_empty else drivable

    def on_centerline_xy(s: float) -> Tuple[float, float]:
        sN = s % L
        x, y, _ = dutils.st_to_world(refline, sN, t=0.0)
        return float(x), float(y)

    # helper: test containment with a tiny inward nudge tolerance
    def fits(x: float, y: float) -> bool:
        return target.contains(Point(x, y))

    # 3) try s0 first
    x0, y0 = on_centerline_xy(s0)
    if fits(x0, y0):
        return x0, y0, s0

    # 4) search along s in ± steps
    max_k = max(1, int(search_window / search_step))
    for k in range(1, max_k + 1):
        ds = k * search_step
        # try forward
        sf = (s0 + ds) % L
        xf, yf = on_centerline_xy(sf)
        if fits(xf, yf):
            return xf, yf, sf
        # try backward
        sb = (s0 - ds) % L
        xb, yb = on_centerline_xy(sb)
        if fits(xb, yb):
            return xb, yb, sb

    # 5) fallback: no admissible midline point found in the search window
    #    return original centerline location (may not fit)
    return x0, y0, s0


def scenic_car_at_from_s_fit(
    xodr_path: Path,
    s_meters: float,
    *,
    step: float = 2.0,
    t_offset: float = 0.0,
    car_width: float = 1.9,
    extra_margin: float = 0.1,
    z: float = 0.0,
    name: Optional[str] = None,
) -> str:
    x, y = safe_centerline_xy_from_s(
        xodr_path, s_meters,
        step=step, t_offset=t_offset,

    )
    prefix = f"{name} = " if name else ""
    return f"{prefix}new Car at ({x:.6f}, {y:.6f}, {z:.6f})"

if __name__ == "__main__":
    xodr_file = (Path(__file__).parent / "../../assets/maps/dSPACE/LagunaSeca.xodr").resolve()

    # Request s=200 m, keep the car strictly on your centerline.
    # This will adjust s a little (s_adj) if needed to guarantee full-car fit.
    x, y, s_adj = safe_centerline_xy_from_s(
        xodr_file, 0.0,
        car_width=1.9, extra_margin=0.1,
        step=0.5,            # tighter sampling improves geometric fidelity
        search_window=10.0,  # ±10 m is usually enough
        search_step=0.25
    )
    print(f"(x,y)=({x:.3f},{y:.3f}) at s_adj={s_adj:.2f}")

    # Scenic line from the strict-midline point:
    print(f"ego = new Car at ({x:.6f}, {y:.6f}, 0.0)")
