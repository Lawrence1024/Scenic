"""Fellow lateral command aligned with MPC racing line vs track centerline.

MPC minimizes error to the optimal racing line (e.g. ttl_optimal_xodr.csv).
dSPACE Const_d is lateral offset from the **track centerline** (ttl_main_road).
Those curves differ by δ(s): the racing line's signed lateral offset from the
centerline at arclength s.

**Structural model**: command d_cmd = δ(s) − Kp·(t_meas − δ(s)) so the fellow
plant (which tracks Const_d in centerline coordinates) is steered toward the
same geometric path MPC uses, instead of open-loop bicycle-from-steering (which
does not match ASM fellow lateral dynamics).

See apply_fellow_control: racing-line servo vs legacy bicycle mode.
"""

from __future__ import annotations

import logging
import os
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Main-track TTL filename (centerline reference for Const_d)
TTL_MAIN_ROAD_FILE = "ttl_main_road.csv"


def _road_index_main_track_only(full_ttl_index: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    roads = (full_ttl_index or {}).get("roads") or {}
    main = roads.get("MainTrack_TTL")
    if not main:
        return None
    return {"roads": {"MainTrack_TTL": main}}


def build_racing_line_delta_table(
    ttl_folder: str,
    optimal_csv_name: str,
    road_index_ttl: Dict[str, Any],
) -> Optional[Tuple[Any, Any, float]]:
    """Build (s_samples, delta_samples, track_length_m) for main track.

    For each point on the optimal line, project onto MainTrack_TTL centerline;
    the lateral coordinate t is δ(s): where the racing line sits relative to
    centerline at that s.

    Returns None if build fails.
    """
    try:
        import numpy as np
    except ImportError:
        logger.warning("numpy required for racing-line delta table; skipping")
        return None

    from ..ttl.loader import load_ttl_region
    from ..geometry.projection import project_world_to_st

    folder = str(ttl_folder)
    opt_path = os.path.join(folder, optimal_csv_name)
    if not os.path.isfile(opt_path):
        logger.warning("[Fellow racing-line lateral] optimal CSV missing: %s", opt_path)
        return None

    _, main_pts = load_ttl_region(folder, TTL_MAIN_ROAD_FILE)
    _, optimal_pts = load_ttl_region(folder, optimal_csv_name)
    if not main_pts or not optimal_pts or len(optimal_pts) < 10:
        return None

    idx_main = _road_index_main_track_only(road_index_ttl)
    if idx_main is None:
        return None

    main_sec = road_index_ttl["roads"]["MainTrack_TTL"]["sec_points"][0]
    track_len = float(main_sec[-1][2])
    if track_len < 100.0:
        return None

    samples: List[Tuple[float, float]] = []
    for p in optimal_pts:
        ox, oy = float(p[0]), float(p[1])
        try:
            s_i, t_i = project_world_to_st(idx_main, (ox, oy))
        except Exception:
            continue
        s_i = float(s_i) % track_len
        samples.append((s_i, float(t_i)))

    if len(samples) < 50:
        return None

    nb = min(4000, max(800, len(samples) // 2))
    edges = np.linspace(0.0, track_len, nb + 1)
    acc = np.zeros(nb, dtype=np.float64)
    cnt = np.zeros(nb, dtype=np.int32)
    for s_i, t_i in samples:
        j = int(np.clip(s_i / track_len * nb, 0, nb - 1))
        acc[j] += t_i
        cnt[j] += 1
    d_bin = np.full(nb, np.nan, dtype=np.float64)
    mask = cnt > 0
    d_bin[mask] = acc[mask] / cnt[mask].astype(np.float64)
    idx = np.arange(nb, dtype=np.float64)
    valid = np.flatnonzero(~np.isnan(d_bin))
    if valid.size < nb // 4:
        return None
    d_filled = np.interp(idx, valid.astype(np.float64), d_bin[valid])
    s_centers = 0.5 * (edges[:-1] + edges[1:])
    # Periodic wrap for stable interp at seam
    s_arr = np.concatenate([[s_centers[-1] - track_len], s_centers, [s_centers[0] + track_len]])
    d_arr = np.concatenate([[d_filled[-1]], d_filled, [d_filled[0]]])

    logger.debug(
        "[Fellow racing-line lateral] delta table: %s optimal pts -> %s bins on %s, L=%.0fm",
        len(samples),
        nb,
        optimal_csv_name,
        track_len,
    )
    return s_arr, d_arr, track_len


def lookup_delta(
    s_m: float,
    t_meas: float,
    s_arr,
    d_arr,
    track_len: float,
    kp: float = 0.95,
    d_max: float = 8.0,
) -> Tuple[float, float, float]:
    """Return (d_cmd, delta_ref, lateral_error) for racing-line servo.

    delta_ref = δ(s); e = t_meas − δ. Uses clipped error and gain scheduling
    when |e| is large to avoid slamming Const_d (reduces plant oscillation /
    projection ambiguity at ~10 m off).
    """
    import numpy as np

    L = max(track_len, 1.0)
    s = float(s_m) % L
    delta_ref = float(np.interp(s, s_arr, d_arr))
    e = float(t_meas) - delta_ref
    e_cmd = float(np.clip(e, -3.2, 3.2))
    if abs(e) > 4.5:
        kp_eff = kp * min(1.0, 5.0 / abs(e))
    else:
        kp_eff = kp
    d_cmd = delta_ref - kp_eff * e_cmd
    d_cmd = max(-d_max, min(d_max, d_cmd))
    return d_cmd, delta_ref, e


def get_or_build_delta_table(
    simulation,
    ttl_folder: str,
    optimal_csv_name: str,
    road_index_ttl: Dict[str, Any],
):
    """Cached (s_arr, d_arr, track_len) per (folder, optimal file)."""
    cache = getattr(simulation, "_fellow_racing_delta_cache", None)
    if cache is None:
        cache = {}
        simulation._fellow_racing_delta_cache = cache
    key = (os.path.abspath(str(ttl_folder)), str(optimal_csv_name))
    if key not in cache:
        cache[key] = build_racing_line_delta_table(
            ttl_folder, optimal_csv_name, road_index_ttl
        )
    return cache[key]
