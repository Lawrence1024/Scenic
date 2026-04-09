"""Compute fellow (v, d) plant commands for dSPACE External_Signals.

Racing fellow behaviors should **take**
:class:`~scenic.domains.racing.actions.SetFellowPlantAction` each control tick; ``applyTo``
stages ``_fellow_plant_state`` (keys ``v_kmh``, ``d_m``). Use the **compute_*** helpers
below for numeric primitives (TTL **d**, cruise/stop **v**, swerve slew).

The dSPACE :class:`~scenic.simulators.dspace.vehicle.controller.VehicleController` reads
``_fellow_plant_state`` and writes ControlDesk only (no geometry in the controller).

**Behaviors** (``behaviors.scenic``): ``FellowSuddenStopIntervalBehavior``
(``examples/combined/fellow_sudden_stop.scenic``),
``FellowSwerveOutOfControlBehavior`` (``examples/combined/fellow_swerve_out_of_control.scenic``),
constant-offset and TTL-geometric fellows.

Ego vehicles are unaffected.
"""

from __future__ import annotations

import logging
import math
from typing import Any, Optional

logger = logging.getLogger(__name__)

_MPH_TO_KMH = 1.609344


def mph_to_kmh(mph: float) -> float:
    """Convert statute mph to km/h (exact factor 1.609344)."""
    return float(mph) * _MPH_TO_KMH


def _behavior_or_obj(obj: Any) -> Any:
    """Prefer Scenic behavior instance when present; else return the object itself."""
    b = getattr(obj, "behavior", None)
    return b if b is not None else obj


def _parse_speed_mph_from_agent(obj: Any, *, default: float = 31.0) -> float:
    src = _behavior_or_obj(obj)
    try:
        return float(getattr(src, "speed_mph"))
    except (TypeError, ValueError, AttributeError):
        return float(default)


# Sim-time spacing for swerve_oc progress lines during slew/stop (phase edges always log).
_SWERVE_OC_LOG_INTERVAL_S = 0.2


def _ensure_fellow_plant_state(obj: Any) -> dict:
    if not hasattr(obj, "_fellow_plant_state") or obj._fellow_plant_state is None:
        obj._fellow_plant_state = {}
    return obj._fellow_plant_state


def set_fellow_plant_v_kmh(obj: Any, v_kmh: float) -> None:
    """Set commanded fellow speed (km/h) in ``_fellow_plant_state``."""
    st = _ensure_fellow_plant_state(obj)
    st["v_kmh"] = float(v_kmh)
    if st.get("d_m") is None:
        st["d_m"] = float(get_fellow_placed_lateral_deviation(obj))


def set_fellow_plant_d_m(obj: Any, d_m: float) -> None:
    """Set commanded lateral **d** in meters (Frenet **t**) in ``_fellow_plant_state``."""
    st = _ensure_fellow_plant_state(obj)
    st["d_m"] = float(d_m)
    if st.get("v_kmh") is None:
        st["v_kmh"] = 0.0


def get_fellow_plant_d_m(obj: Any, *, if_missing: Optional[float] = None) -> float:
    """Read commanded **d** (m). If unset and *if_missing* is ``None``, use placement."""
    st = getattr(obj, "_fellow_plant_state", None)
    if isinstance(st, dict) and st.get("d_m") is not None:
        return float(st["d_m"])
    if if_missing is not None:
        return float(if_missing)
    return float(get_fellow_placed_lateral_deviation(obj))


def get_fellow_plant_v_kmh(obj: Any) -> Optional[float]:
    st = getattr(obj, "_fellow_plant_state", None)
    if isinstance(st, dict) and st.get("v_kmh") is not None:
        return float(st["v_kmh"])
    return None


def get_fellow_placed_lateral_deviation(obj: Any) -> float:
    """Lateral deviation (d) from Scenic placement (``_route_s_t``)."""
    st = getattr(obj, "_route_s_t", None)
    if st is not None and len(st) == 2:
        return float(st[1])
    return 0.0


def compute_fellow_ttl_geometric_d_m(
    obj: Any,
    simulation: Any,
    *,
    fellow_index: Optional[int] = None,
) -> tuple[float, str]:
    """Compute lateral **d** (m) from TTL δ(s) or placement; returns ``(d_m, log_suffix)``.

    ``log_suffix`` is ``delta(s)`` or ``placement_d``. On non-control-interval steps, returns
    the last commanded **d** from ``_fellow_plant_state`` if present, else placement.
    """
    control_interval = max(1, int(getattr(simulation, "_control_interval", 1) or 1))
    current_time = int(getattr(simulation, "currentTime", 0) or 0)
    if current_time % control_interval != 0:
        st = getattr(obj, "_fellow_plant_state", None)
        if isinstance(st, dict) and st.get("d_m") is not None:
            return float(st["d_m"]), getattr(
                obj, "_fellow_plant_log_mode", "placement_d"
            )
        d0 = get_fellow_placed_lateral_deviation(obj)
        return d0, "placement_d"

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
                            "[Fellow %s] geometric TTL: delta(s) projection failed (%s); placement d",
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

    sub = "delta(s)" if used_delta else "placement_d"
    return float(d_cmd), sub


def sudden_stop_v_kmh(
    simulation: Any,
    mph: float,
    interval: float,
    duration: float,
) -> float:
    """Longitudinal **v** (km/h) for repeating cruise / stop schedule using ``currentRealTime``."""
    interval = max(0.0, float(interval))
    duration = max(0.0, float(duration))
    period = interval + duration
    t_sim = float(getattr(simulation, "currentRealTime", 0.0) or 0.0)
    mph = float(mph)

    if duration <= 0.0 or period <= 0.0:
        return mph_to_kmh(mph)
    pos_in_cycle = t_sim % period
    if pos_in_cycle < interval:
        return mph_to_kmh(mph)
    return 0.0


def _swerve_oc_slew_d_toward(
    obj: Any,
    simulation: Any,
    target_d: float,
    rate_m_s: float,
) -> float:
    """Integrate lateral command toward *target_d* with |dd/dt| <= *rate_m_s* (simulation time).

    Uses ``currentRealTime`` so duplicate calls in the same sim instant do not double-move.
    Stores ``_fellow_swerve_oc_d_smooth`` and ``_fellow_swerve_oc_slew_last_t`` on *obj*.
    """
    t_sim = float(getattr(simulation, "currentRealTime", 0.0) or 0.0)
    ts = float(getattr(simulation, "timestep", 0.01) or 0.01)
    last_t = getattr(obj, "_fellow_swerve_oc_slew_last_t", None)
    if last_t is None:
        dt = ts
    else:
        dt = max(0.0, min(float(t_sim) - float(last_t), 0.25))
    obj._fellow_swerve_oc_slew_last_t = t_sim

    d_s = getattr(obj, "_fellow_swerve_oc_d_smooth", None)
    if d_s is None:
        d_s = get_fellow_plant_d_m(obj, if_missing=0.0)

    d_start = float(d_s)
    rate = max(0.0, float(rate_m_s))
    max_dd = rate * dt
    diff = float(target_d) - float(d_s)
    if abs(diff) <= max_dd:
        d_s = float(target_d)
    else:
        d_s += math.copysign(max_dd, diff)

    dd_step = float(d_s) - d_start
    obj._fellow_swerve_oc_d_smooth = d_s
    obj._fellow_swerve_oc_slew_dbg = {
        "dt": dt,
        "dd_step": dd_step,
        "target_d": float(target_d),
        "max_dd_step": max_dd,
        "err_to_target": abs(float(target_d) - float(d_s)),
    }
    if max_dd > 0.0 and abs(dd_step) > max_dd + 1e-5:
        logger.warning(
            "[Fellow swerve_oc] slew step exceeded rate cap: |dd|=%.6f max_dd=%.6f dt=%.6f",
            abs(dd_step),
            max_dd,
            dt,
        )
    return d_s


def _parse_swerve_oc_stop_hold_d(b: Any) -> bool:
    """True: after v=0, keep commanded **d** fixed (no TTL tracking). False: slew **d** toward TTL."""
    v = getattr(b, "stop_hold_d", True)
    if v in (False, 0):
        return False
    if isinstance(v, str) and v.strip().lower() in ("0", "false", "no", "off"):
        return False
    return True


def _swerve_oc_progress_log(
    obj: Any,
    *,
    phase: str,
    t_sim: float,
    v_kmh: float,
    swerve_amp_m: float,
    t_swerve_r_end: float,
    t_swerve_done: float,
    d_cmd: Optional[float] = None,
) -> None:
    """Print throttled diagnostics: target **d**, slew step size, and cap (verifies gradual slew)."""
    prev_phase = getattr(obj, "_fellow_swerve_oc_prog_log_phase", None)
    last_t = getattr(obj, "_fellow_swerve_oc_prog_log_t", None)
    phase_changed = phase != prev_phase
    slew_phases = ("swerve_right", "swerve_left", "stopped")
    due_interval = (
        phase in slew_phases
        and last_t is not None
        and (t_sim - last_t) >= _SWERVE_OC_LOG_INTERVAL_S
    )
    if not (phase_changed or due_interval):
        return

    if d_cmd is None:
        d_cmd = get_fellow_plant_d_m(obj, if_missing=float("nan"))
    parts = [
        f"[Fellow swerve_oc] t={t_sim:.6f}s phase={phase!r}",
        f"v_cmd={v_kmh:.2f}km/h d_cmd={d_cmd:.4f}m",
    ]
    if phase == "cruise":
        parts.append("(d from TTL delta(s))")
    else:
        dbg = getattr(obj, "_fellow_swerve_oc_slew_dbg", None)
        if isinstance(dbg, dict):
            tgt = float(dbg.get("target_d", float("nan")))
            err = float(dbg.get("err_to_target", float("nan")))
            dd = float(dbg.get("dd_step", float("nan")))
            dt_u = float(dbg.get("dt", float("nan")))
            mx = float(dbg.get("max_dd_step", float("nan")))
            parts.append(
                f"target_d={tgt:.4f}m err={err:.4f}m "
                f"dd_step={dd:+.6f}m dt={dt_u:.6f}s max|dd|={mx:.6f}m"
            )
        else:
            parts.append("slew_dbg=missing")
        d0 = getattr(obj, "_fellow_swerve_oc_d_cruise_ref", None)
        if d0 is not None and phase in ("swerve_right", "swerve_left"):
            parts.append(
                f"delta_vs_cruise_start={(d_cmd - float(d0)):+.4f}m (cruise_d_ref={float(d0):.4f}m)"
            )
        extras = getattr(obj, "_fellow_swerve_oc_prog_extras", None)
        if isinstance(extras, dict) and phase == "stopped":
            if extras.get("stop_hold_d"):
                parts.append(
                    f"stop_mode=hold_d d_frozen={extras.get('d_frozen', float('nan')):.4f}m"
                )
            elif "d_ttl_geo" in extras:
                parts.append(
                    f"stop_mode=track_ttl d_ttl_geo={float(extras['d_ttl_geo']):.4f}m"
                )

    if phase_changed:
        if phase == "swerve_right":
            parts.append(f"-> toward -amp (-{swerve_amp_m:.4f}m) until t<{t_swerve_r_end:.6f}s")
        elif phase == "swerve_left":
            parts.append(f"-> toward +amp (+{swerve_amp_m:.4f}m) until t<{t_swerve_done:.6f}s")
        elif phase == "stopped":
            parts.append(
                "-> v=0 (see stop_mode: hold_d=fixed lateral cmd, track_ttl=slew d to delta(s))"
            )

    print(" ".join(parts))
    obj._fellow_swerve_oc_prog_log_phase = phase
    obj._fellow_swerve_oc_prog_log_t = t_sim


def compute_fellow_swerve_out_of_control_command(
    obj: Any,
    simulation: Any,
    *,
    fellow_index: Optional[int] = None,
    speed_mph: Optional[float] = None,
    interval_s: Optional[float] = None,
    swerve_right_s: Optional[float] = None,
    swerve_left_s: Optional[float] = None,
    swerve_amp_m: Optional[float] = None,
    swerve_d_rate_m_s: Optional[float] = None,
    stop_hold_d: Optional[bool] = None,
) -> tuple[float, float, str]:
    """Cruise on TTL, then gradual swerve toward full right, then full left, then v=0.

    Returns ``(v_kmh, d_m, log_mode)`` without writing ``_fellow_plant_state``; callers
    should commit via :class:`~scenic.domains.racing.actions.SetFellowPlantAction`.

    Lateral convention (centerline / Const_d): positive = left, negative = right.
    During swerve legs, **d** slews toward +/- **swerve_amp_m** at at most **swerve_d_rate_m_s**
    (m/s change in commanded d) to avoid step jumps. **interval** is seconds of TTL cruise
    at **speed_mph** before the maneuver.

    When **stop_hold_d** is true (default), commanded **d** stays fixed after **v=0** so the
    fellow does not drift laterally toward a moving TTL target while stationary.

    Fallback defaults for missing behavior fields match
    ``examples/combined/fellow_swerve_out_of_control.scenic`` and
    ``FellowSwerveOutOfControlBehavior`` in ``behaviors.scenic``.
    """
    src = _behavior_or_obj(obj)
    if speed_mph is None:
        try:
            mph = float(getattr(src, "speed_mph", 150.0))
        except (TypeError, ValueError):
            mph = 150.0
    else:
        mph = float(speed_mph)
    if interval_s is None:
        try:
            interval = float(getattr(src, "interval", 10.0))
        except (TypeError, ValueError):
            interval = 10.0
    else:
        interval = float(interval_s)
    if swerve_right_s is None:
        try:
            swerve_right_s = float(getattr(src, "swerve_right_s", 1.8))
        except (TypeError, ValueError):
            swerve_right_s = 1.8
    else:
        swerve_right_s = float(swerve_right_s)
    if swerve_left_s is None:
        try:
            swerve_left_s = float(getattr(src, "swerve_left_s", 2.0))
        except (TypeError, ValueError):
            swerve_left_s = 2.0
    else:
        swerve_left_s = float(swerve_left_s)
    if swerve_amp_m is None:
        try:
            swerve_amp_m = float(getattr(src, "swerve_amp_m", 6.0))
        except (TypeError, ValueError):
            swerve_amp_m = 6.0
    else:
        swerve_amp_m = float(swerve_amp_m)
    if swerve_d_rate_m_s is None:
        try:
            swerve_d_rate_m_s = float(getattr(src, "swerve_d_rate_m_s", 6.5))
        except (TypeError, ValueError):
            swerve_d_rate_m_s = 6.5
    else:
        swerve_d_rate_m_s = float(swerve_d_rate_m_s)
    if stop_hold_d is None:
        stop_hold_d = _parse_swerve_oc_stop_hold_d(src)
    else:
        stop_hold_d = bool(stop_hold_d)

    interval = max(0.0, interval)
    swerve_right_s = max(0.0, swerve_right_s)
    swerve_left_s = max(0.0, swerve_left_s)
    swerve_amp_m = max(0.1, abs(swerve_amp_m))
    swerve_d_rate_m_s = max(0.05, swerve_d_rate_m_s)

    t_sim = float(getattr(simulation, "currentRealTime", 0.0) or 0.0)
    t_swerve_r_end = interval + swerve_right_s
    t_swerve_done = t_swerve_r_end + swerve_left_s

    if not getattr(obj, "_fellow_swerve_oc_config_printed", False):
        obj._fellow_swerve_oc_config_printed = True
        t_need_r = swerve_amp_m / swerve_d_rate_m_s if swerve_d_rate_m_s > 0 else float("nan")
        t_need_l = (2.0 * swerve_amp_m) / swerve_d_rate_m_s if swerve_d_rate_m_s > 0 else float("nan")
        print(
            f"[Fellow swerve_oc] config: speed={mph:.2f} mph interval={interval:.4f}s "
            f"swerve_right_s={swerve_right_s:.4f}s swerve_left_s={swerve_left_s:.4f}s "
            f"swerve_amp_m={swerve_amp_m:.4f} swerve_d_rate_m_s={swerve_d_rate_m_s:.4f} "
            f"stop_hold_d={stop_hold_d} "
            f"(d: +left / -right vs centerline); time base=simulation.currentRealTime; "
            f"approx min leg times: right>={t_need_r:.2f}s to hit -amp, "
            f"left>={t_need_l:.2f}s to go -amp -> +amp at this rate"
        )

    if t_sim < interval:
        phase = "cruise"
    elif t_sim < t_swerve_r_end:
        phase = "swerve_right"
    elif t_sim < t_swerve_done:
        phase = "swerve_left"
    else:
        phase = "stopped"

    prev_motion_phase = getattr(obj, "_fellow_swerve_oc_motion_phase", None)
    if prev_motion_phase is None:
        obj._fellow_swerve_oc_motion_phase = phase
    elif phase != prev_motion_phase:
        obj._fellow_swerve_oc_motion_phase = phase
        obj._fellow_swerve_oc_slew_last_t = None
        if phase == "swerve_right" and prev_motion_phase == "cruise":
            obj._fellow_swerve_oc_d_smooth = float(
                get_fellow_plant_d_m(obj, if_missing=0.0)
            )
            obj._fellow_swerve_oc_d_cruise_ref = float(obj._fellow_swerve_oc_d_smooth)
        elif phase == "cruise":
            obj._fellow_swerve_oc_d_smooth = None
        if phase == "stopped":
            ds = getattr(obj, "_fellow_swerve_oc_d_smooth", None)
            obj._fellow_swerve_oc_stop_d_frozen = float(
                ds if ds is not None else get_fellow_plant_d_m(obj, if_missing=0.0)
            )
        if phase == "swerve_left" and prev_motion_phase == "swerve_right":
            mn = float(getattr(obj, "_fellow_swerve_oc_leg_min_d", float("nan")))
            tgt = -swerve_amp_m
            print(
                f"[Fellow swerve_oc] leg_right done: min_d_cmd={mn:.4f}m "
                f"target={tgt:.4f}m miss={abs(mn - tgt):.4f}m"
            )
            obj._fellow_swerve_oc_leg_max_d = float("nan")
        if phase == "stopped" and prev_motion_phase == "swerve_left":
            mx = float(getattr(obj, "_fellow_swerve_oc_leg_max_d", float("nan")))
            tgt = swerve_amp_m
            print(
                f"[Fellow swerve_oc] leg_left done: max_d_cmd={mx:.4f}m "
                f"target={tgt:.4f}m miss={abs(mx - tgt):.4f}m "
                f"frozen_stop_d={obj._fellow_swerve_oc_stop_d_frozen:.4f}m"
            )

    if phase == "cruise":
        v_kmh = mph_to_kmh(mph)
        d_m, _sub = compute_fellow_ttl_geometric_d_m(
            obj, simulation, fellow_index=fellow_index
        )
        obj._fellow_swerve_oc_d_smooth = d_m
        obj._fellow_swerve_oc_slew_dbg = None
        log_mode = f"swerve_oc/cruise/{_sub}"
    elif phase == "swerve_right":
        v_kmh = mph_to_kmh(mph)
        d_m = _swerve_oc_slew_d_toward(
            obj, simulation, -swerve_amp_m, swerve_d_rate_m_s
        )
        cur = d_m
        pmin = getattr(obj, "_fellow_swerve_oc_leg_min_d", None)
        obj._fellow_swerve_oc_leg_min_d = cur if pmin is None else min(float(pmin), cur)
        log_mode = "swerve_oc/right"
    elif phase == "swerve_left":
        v_kmh = mph_to_kmh(mph)
        d_m = _swerve_oc_slew_d_toward(
            obj, simulation, swerve_amp_m, swerve_d_rate_m_s
        )
        cur = d_m
        pmax = getattr(obj, "_fellow_swerve_oc_leg_max_d", None)
        if pmax is None or (isinstance(pmax, float) and math.isnan(pmax)):
            obj._fellow_swerve_oc_leg_max_d = cur
        else:
            obj._fellow_swerve_oc_leg_max_d = max(float(pmax), cur)
        log_mode = "swerve_oc/left"
    else:
        v_kmh = 0.0
        if stop_hold_d:
            d_f = getattr(obj, "_fellow_swerve_oc_stop_d_frozen", None)
            if d_f is None:
                d_f = get_fellow_plant_d_m(obj, if_missing=0.0)
                obj._fellow_swerve_oc_stop_d_frozen = d_f
            d_f = float(d_f)
            d_m = d_f
            obj._fellow_swerve_oc_d_smooth = d_f
            obj._fellow_swerve_oc_slew_dbg = {
                "dt": 0.0,
                "dd_step": 0.0,
                "target_d": d_f,
                "max_dd_step": 0.0,
                "err_to_target": 0.0,
            }
            log_mode = "swerve_oc/stopped/hold_d"
        else:
            d_ttl, _sub = compute_fellow_ttl_geometric_d_m(
                obj, simulation, fellow_index=fellow_index
            )
            obj._fellow_swerve_oc_last_d_ttl_geo = d_ttl
            d_m = _swerve_oc_slew_d_toward(
                obj, simulation, d_ttl, swerve_d_rate_m_s
            )
            log_mode = f"swerve_oc/stopped/{_sub}"

    obj._fellow_swerve_oc_prog_extras = None
    if phase == "stopped":
        if stop_hold_d:
            obj._fellow_swerve_oc_prog_extras = {
                "stop_hold_d": True,
                "d_frozen": float(
                    getattr(obj, "_fellow_swerve_oc_stop_d_frozen", float("nan"))
                ),
            }
        else:
            obj._fellow_swerve_oc_prog_extras = {
                "stop_hold_d": False,
                "d_ttl_geo": float(
                    getattr(obj, "_fellow_swerve_oc_last_d_ttl_geo", float("nan"))
                ),
            }

    _swerve_oc_progress_log(
        obj,
        phase=phase,
        t_sim=t_sim,
        v_kmh=v_kmh,
        swerve_amp_m=swerve_amp_m,
        t_swerve_r_end=t_swerve_r_end,
        t_swerve_done=t_swerve_done,
        d_cmd=d_m,
    )
    return v_kmh, d_m, log_mode


def compute_constant_offset_plant_command(obj: Any) -> tuple[float, float, str]:
    """Constant **v** from ``speed_mph`` on agent or behavior and **d** from Scenic placement."""
    mph = _parse_speed_mph_from_agent(obj)
    return mph_to_kmh(mph), get_fellow_placed_lateral_deviation(obj), "placement_t"


def compute_follow_ttl_geometric_plant_command(
    obj: Any,
    simulation: Any,
    mph: float,
    *,
    fellow_index: Optional[int] = None,
) -> tuple[float, float, str]:
    """Constant **v** and TTL geometric **d** (or placement fallback)."""
    v_kmh = mph_to_kmh(mph)
    d_m, sub = compute_fellow_ttl_geometric_d_m(
        obj, simulation, fellow_index=fellow_index
    )
    return v_kmh, d_m, sub


def _parse_sudden_stop_behavior_fields(obj: Any) -> tuple[float, float, float]:
    src = _behavior_or_obj(obj)
    try:
        mph = float(getattr(src, "speed_mph", 150.0))
    except (TypeError, ValueError):
        mph = 150.0
    try:
        interval = float(getattr(src, "interval", 20.0))
    except (TypeError, ValueError):
        interval = 20.0
    try:
        duration = float(getattr(src, "duration", 3.0))
    except (TypeError, ValueError):
        duration = 3.0
    return mph, interval, duration


def compute_sudden_stop_plant_command(
    obj: Any,
    simulation: Any,
    *,
    fellow_index: Optional[int] = None,
    speed_mph: Optional[float] = None,
    interval_s: Optional[float] = None,
    duration_s: Optional[float] = None,
) -> tuple[float, float, str]:
    """Periodic cruise / stop **v** and TTL geometric **d**."""
    if speed_mph is None or interval_s is None or duration_s is None:
        mph, interval, duration = _parse_sudden_stop_behavior_fields(obj)
        if speed_mph is not None:
            mph = float(speed_mph)
        if interval_s is not None:
            interval = float(interval_s)
        if duration_s is not None:
            duration = float(duration_s)
    else:
        mph = float(speed_mph)
        interval = float(interval_s)
        duration = float(duration_s)
    v_kmh = sudden_stop_v_kmh(simulation, mph, interval, duration)
    d_m, sub = compute_fellow_ttl_geometric_d_m(
        obj, simulation, fellow_index=fellow_index
    )
    return v_kmh, d_m, f"sudden_stop/{sub}"


