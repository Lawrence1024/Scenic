import math
from typing import Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple, Union

PointLike = Union[Sequence[float], Mapping[str, float], MutableMapping[str, float]]


def _get_xy(p: PointLike) -> Tuple[float, float]:
    """Extract (x, y) from a waypoint-like object."""
    if isinstance(p, (list, tuple)) and len(p) >= 2:
        return float(p[0]), float(p[1])
    # Support dicts like {"x": ..., "y": ...}
    if isinstance(p, dict):
        if "x" in p and "y" in p:
            return float(p["x"]), float(p["y"])
    raise TypeError(f"Unsupported waypoint type: {type(p)!r}")


def _angle_diff(a: float, b: float) -> float:
    """Smallest signed difference a-b in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def find_best_racing_waypoint(
    *,
    car_position: Tuple[float, float],
    car_heading: float,
    waypoints: Iterable[PointLike],
    last_known_index: int,
    max_search_distance: float = 100.0,
    forward_bias: float = 0.9,
    min_forward_distance: float = 0.0,
    forward_only: bool = True,
) -> Optional[dict]:
    """Heuristic waypoint selector for racing.

    This is a robust forward-progress finder used by racing behaviors:
    it searches along the waypoint list starting from ``last_known_index``,
    preferring points which are (1) reasonably close in Euclidean distance
    and (2) in front of the vehicle according to ``car_heading``.

    Returns a small dict on success::

        {
            \"index\": <int>,        # chosen waypoint index
            \"distance\": <float>,   # distance car→waypoint (m)
            \"forward_score\": <float>,  # cos(heading_diff) in [-1, 1]
        }

    or ``None`` if no suitable waypoint is found.
    """

    wps: List[Tuple[float, float]] = []
    for wp in waypoints:
        try:
            wps.append(_get_xy(wp))
        except Exception:
            # Ignore malformed waypoints; behavior will fall back as needed.
            continue

    n = len(wps)
    if n == 0:
        return None

    last_idx = int(last_known_index) % n
    cx, cy = float(car_position[0]), float(car_position[1])
    heading = float(car_heading)

    def score_candidate(dist: float, cos_fwd: float) -> float:
        # Smaller score is better. We reduce distance when the point is strongly
        # in front of the car (cos_fwd close to 1), and penalize points behind.
        # The exact form is not critical; behavior only needs a consistent order.
        return dist * (1.0 - forward_bias * cos_fwd)

    best_idx: Optional[int] = None
    best_dist: float = float("inf")
    best_cos: float = -1.0
    best_score: float = float("inf")

    # Walk forward around the closed loop until we exceed max_search_distance
    # or have seen all waypoints.
    total_along = 0.0
    i = last_idx
    steps = 0
    prev_wp = wps[last_idx]
    max_steps = n

    while steps < max_steps and total_along <= max_search_distance:
        wx, wy = wps[i]

        if steps > 0:
            seg_dx = wx - prev_wp[0]
            seg_dy = wy - prev_wp[1]
            total_along += math.hypot(seg_dx, seg_dy)
            prev_wp = (wx, wy)

        dx = wx - cx
        dy = wy - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            # Waypoint exactly at car position; treat as valid but don't early-out
            dist = 0.0

        # Compute forward alignment
        tgt_heading = math.atan2(dy, dx)
        diff = _angle_diff(tgt_heading, heading)
        cos_fwd = math.cos(diff)  # 1 = straight ahead, -1 = directly behind

        if forward_only and cos_fwd <= 0.0:
            # Behind or sideways; skip when forward-only is requested.
            steps += 1
            i = (i + 1) % n
            continue

        # Optionally enforce a minimum forward distance to avoid snapping to
        # very close points immediately around the car.
        if dist < min_forward_distance:
            steps += 1
            i = (i + 1) % n
            continue

        sc = score_candidate(dist, cos_fwd)
        if sc < best_score:
            best_score = sc
            best_idx = i
            best_dist = dist
            best_cos = cos_fwd

        steps += 1
        i = (i + 1) % n

    if best_idx is None:
        return None

    return {
        "index": int(best_idx),
        "distance": float(best_dist),
        "forward_score": float(best_cos),
    }

