"""Compute fellow (v, d) plant commands for dSPACE External_Signals.

Behaviors call these updaters each simulation step; the dSPACE
:class:`~scenic.simulators.dspace.vehicle.controller.VehicleController`
reads ``_fellow_plant_v_kmh`` and ``_fellow_plant_d_m`` and writes ControlDesk
only (no geometry in the controller).

Ego vehicles are unaffected.
"""

from __future__ import annotations

import logging
from typing import Any, Optional

from scenic.domains.racing.fellow.plant import (
    FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS,
    FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS,
)

logger = logging.getLogger(__name__)

_MPH_TO_KMH = 1.609344


def get_fellow_placed_lateral_deviation(obj: Any) -> float:
    """Lateral deviation (d) from Scenic placement (``_route_s_t``)."""
    st = getattr(obj, "_route_s_t", None)
    if st is not None and len(st) == 2:
        return float(st[1])
    return 0.0


def update_fellow_constant_speed_track_offset_plant(obj: Any, simulation: Any) -> None:
    """Set ``_fellow_plant_v_kmh`` and ``_fellow_plant_d_m`` for constant-offset plant."""
    b = getattr(obj, "behavior", None)
    if b is None or b.__class__.__name__ != FELLOW_CONSTANT_SPEED_TRACK_OFFSET_CLASS:
        return
    try:
        mph = float(getattr(b, "speed_mph"))
    except (TypeError, ValueError):
        mph = 31.0
    obj._fellow_plant_v_kmh = float(mph) * _MPH_TO_KMH
    obj._fellow_plant_d_m = get_fellow_placed_lateral_deviation(obj)
    obj._fellow_plant_log_mode = "placement_t"


def update_fellow_follow_ttl_geometric_plant(
    obj: Any,
    simulation: Any,
    *,
    fellow_index: Optional[int] = None,
) -> None:
    """Set plant v and d from TTL δ(s) on control-interval steps; v refreshed every step."""
    b = getattr(obj, "behavior", None)
    if b is None or b.__class__.__name__ != FELLOW_FOLLOW_TTL_GEOMETRIC_CLASS:
        return
    try:
        mph = float(getattr(b, "speed_mph"))
    except (TypeError, ValueError):
        mph = 31.0
    v_kmh = float(mph) * _MPH_TO_KMH
    obj._fellow_plant_v_kmh = v_kmh

    control_interval = max(1, int(getattr(simulation, "_control_interval", 1) or 1))
    current_time = int(getattr(simulation, "currentTime", 0) or 0)
    if current_time % control_interval != 0:
        if getattr(obj, "_fellow_plant_d_m", None) is None:
            obj._fellow_plant_d_m = get_fellow_placed_lateral_deviation(obj)
            obj._fellow_plant_log_mode = "placement_d"
        return

    from scenic.simulators.dspace.utils import legacy as dutils
    from scenic.simulators.dspace.vehicle.fellow_racing_line_lateral import (
        _road_index_main_track_only,
        get_or_build_delta_table,
        lookup_delta,
    )
    from scenic.domains.racing.waypoints import (
        initialize_racing_waypoint_start_index,
        select_forward_racing_waypoint,
    )

    idx_label = fellow_index if fellow_index is not None else "?"

    route_pref = getattr(obj, "_route", None)
    scene = getattr(simulation, "scene", None)
    scene_params = getattr(scene, "params", None) or {}
    ttl_folder = getattr(obj, "ttlFolder", None) or scene_params.get("ttlFolder")
    optimal_csv = getattr(obj, "ttlFileName", None) or scene_params.get("ttlFileName")
    road_index = getattr(simulation, "_road_index_ttl", None) or getattr(
        simulation, "_road_index", None
    )

    dt = float(getattr(simulation, "timestep", 0.01) or 0.01) * control_interval

    d_cmd = get_fellow_placed_lateral_deviation(obj)
    used_delta = False

    x_rd = y_rd = None
    da = getattr(obj, "dspaceActor", None)
    if da is not None and getattr(da, "position", None) is not None:
        pos = da.position
        try:
            x_rd = float(pos.x)
            y_rd = float(pos.y)
        except (TypeError, ValueError):
            x_rd = y_rd = None

    heading = 0.0
    if da is not None and getattr(da, "heading", None) is not None:
        try:
            heading = float(da.heading)
        except (TypeError, ValueError):
            heading = 0.0

    wps = getattr(obj, "waypoints", None)
    if x_rd is not None and y_rd is not None and wps and len(wps) >= 2:
        if not getattr(obj, "_fellow_geo_wp_inited", False):
            idx, _res, _nd = initialize_racing_waypoint_start_index(
                (x_rd, y_rd), heading, wps
            )
            obj._fellow_geo_wp_last_idx = int(idx)
            obj._fellow_geo_wp_inited = True
        else:
            last_i = int(getattr(obj, "_fellow_geo_wp_last_idx", 0))
            res = select_forward_racing_waypoint(
                car_position=(x_rd, y_rd),
                car_heading=heading,
                waypoints=wps,
                last_known_index=last_i,
                max_search_distance=100.0,
                forward_bias=0.9,
                min_forward_distance=5.0,
                forward_only=True,
            )
            if res is not None:
                obj._fellow_geo_wp_last_idx = int(res["index"])

    if (
        x_rd is not None
        and y_rd is not None
        and route_pref == "Lap"
        and ttl_folder
        and str(ttl_folder).strip()
        and optimal_csv
        and road_index
    ):
        idx_main = _road_index_main_track_only(road_index)
        if idx_main is not None:
            tbl = get_or_build_delta_table(
                simulation,
                str(ttl_folder),
                str(optimal_csv),
                road_index,
            )
            if tbl is not None:
                pos_xy = (x_rd, y_rd)
                try:
                    s_meas, t_meas = dutils.project_world_to_st(idx_main, pos_xy)
                    s_filt = getattr(obj, "_fellow_geo_s_meas_filtered", None)
                    if s_filt is None:
                        s_use = float(s_meas)
                    else:
                        s_use = 0.42 * float(s_meas) + 0.58 * float(s_filt)
                    obj._fellow_geo_s_meas_filtered = s_use
                    s_arr, d_arr_tbl, track_len = tbl
                    d_raw, _delta_ref, _e = lookup_delta(
                        s_use, t_meas, s_arr, d_arr_tbl, track_len, kp=0.0
                    )
                    prev_d = getattr(obj, "_fellow_geo_d_cmd_prev", None)
                    max_slew = 0.32 * max(1.0, dt / 0.05)
                    if prev_d is not None:
                        d_raw = max(
                            prev_d - max_slew,
                            min(prev_d + max_slew, d_raw),
                        )
                    obj._fellow_geo_d_cmd_prev = float(d_raw)
                    d_cmd = float(d_raw)
                    used_delta = True
                except Exception as ex:
                    if not getattr(obj, "_fellow_geo_warned_projection", False):
                        obj._fellow_geo_warned_projection = True
                        logger.warning(
                            "[Fellow %s] geometric TTL: δ(s) projection failed (%s); placement d",
                            idx_label,
                            ex,
                        )
            elif not getattr(obj, "_fellow_geo_warned_no_table", False):
                obj._fellow_geo_warned_no_table = True
                logger.warning(
                    "[Fellow %s] geometric TTL: delta table missing (ttlFolder=%r ttlFileName=%r); placement d",
                    idx_label,
                    ttl_folder,
                    optimal_csv,
                )
        elif not getattr(obj, "_fellow_geo_warned_no_main_idx", False):
            obj._fellow_geo_warned_no_main_idx = True
            logger.warning(
                "[Fellow %s] geometric TTL: no MainTrack_TTL road index; placement d",
                idx_label,
            )
    else:
        if not getattr(obj, "_fellow_geo_warned_requirements", False):
            obj._fellow_geo_warned_requirements = True
            reasons = []
            if x_rd is None or y_rd is None:
                reasons.append("no dspaceActor position")
            if route_pref != "Lap":
                reasons.append(f"route={route_pref!r} (need Lap)")
            if not ttl_folder or not str(ttl_folder).strip():
                reasons.append("no ttlFolder (param or per-object)")
            if not optimal_csv:
                reasons.append(
                    "no ttlFileName (set param ttlFileName or fellow with ttlFileName, e.g. ttl_optimal_xodr.csv)"
                )
            if not road_index:
                reasons.append("no road_index")
            logger.warning(
                "[Fellow %s] geometric TTL inactive: %s — using placement d",
                idx_label,
                "; ".join(reasons) if reasons else "preconditions not met",
            )

    obj._fellow_plant_d_m = d_cmd
    obj._fellow_plant_log_mode = "delta(s)" if used_delta else "placement_d"
