"""Virtual steering and heading for fellow vehicles using MPC.

Fellows often have no steer_actual readback and GPS heading may be track-aligned
(not body heading). We maintain internal states that evolve with commanded steer
so lateral MPC sees a consistent bicycle-style state (yaw, delta_actual).
"""

from __future__ import annotations

import math
from typing import Any, List, Optional, Tuple, Union

from scenic.domains.racing.constants import DELTA_MAX_RAD


def path_tangent_at_waypoint(
    wp_list: List[Union[Tuple[float, ...], list]], wp_idx: int
) -> float:
    """Heading (rad) of segment wp_idx -> wp_idx+1 along closed polyline."""
    n = len(wp_list)
    if n < 2:
        return 0.0
    i = int(wp_idx) % n
    j = (i + 1) % n
    x0, y0 = float(wp_list[i][0]), float(wp_list[i][1])
    x1, y1 = float(wp_list[j][0]), float(wp_list[j][1])
    return math.atan2(y1 - y0, x1 - x0)


def init_fellow_virtual_mpc_state(
    agent: Any,
    wp_list: Optional[List],
    wp_last_idx: int,
    fallback_heading_rad: float,
) -> None:
    """One-time init: virtual heading along path tangent; virtual steer 0."""
    if getattr(agent, "_fellow_virt_mpc_inited", False):
        return
    if wp_list is not None and len(wp_list) >= 2:
        psi0 = path_tangent_at_waypoint(wp_list, wp_last_idx)
    else:
        psi0 = float(fallback_heading_rad)
    agent._fellow_virt_psi = math.atan2(math.sin(psi0), math.cos(psi0))
    agent._fellow_virt_delta = 0.0
    agent._fellow_virt_mpc_inited = True


def apply_virtual_state_to_vehicle_state(agent: Any, vehicle_state: dict) -> None:
    """Overwrite yaw and steer_actual with virtual states (call before lateral MPC)."""
    vehicle_state["yaw"] = float(getattr(agent, "_fellow_virt_psi", 0.0))
    vehicle_state["steer_actual"] = float(getattr(agent, "_fellow_virt_delta", 0.0))


def step_fellow_virtual_after_mpc(
    agent: Any,
    delta_cmd_rad: float,
    speed_mps: float,
    kappa_ref: Optional[float],
    dt: float,
    wheel_base: float,
    steer_tau: float,
) -> None:
    """After MPC: lag delta toward command; integrate heading (bicycle + path curvature)."""
    d = float(getattr(agent, "_fellow_virt_delta", 0.0))
    psi = float(getattr(agent, "_fellow_virt_psi", 0.0))
    tau = max(float(steer_tau), 1e-3)
    alpha = min(1.0, float(dt) / tau)
    d_cmd = max(-DELTA_MAX_RAD, min(DELTA_MAX_RAD, float(delta_cmd_rad)))
    d_new = d + alpha * (d_cmd - d)
    d_new = max(-DELTA_MAX_RAD, min(DELTA_MAX_RAD, d_new))

    L = max(0.1, float(wheel_base))
    v = float(speed_mps)
    kap = float(kappa_ref) if kappa_ref is not None else 0.0
    if abs(v) < 0.01:
        psi_new = psi
    else:
        psi_dot = v * math.tan(d_new) / L - v * kap
        psi_new = psi + psi_dot * float(dt)
        psi_new = math.atan2(math.sin(psi_new), math.cos(psi_new))

    agent._fellow_virt_delta = d_new
    agent._fellow_virt_psi = psi_new


def reset_fellow_virtual_mpc_state(agent: Any) -> None:
    """Clear virtual state (e.g. new scenario)."""
    for attr in ("_fellow_virt_mpc_inited", "_fellow_virt_psi", "_fellow_virt_delta"):
        if hasattr(agent, attr):
            delattr(agent, attr)


def fellow_virt_prepare_for_scenic(
    agent: Any,
    vehicle_state: dict,
    wp_list: Any,
    use_waypoints: bool,
    wp_last_idx: int,
    car_heading: Any,
) -> None:
    """Single entry for Scenic (call via __import__(..., fromlist=['x']).fellow_virt_prepare_for_scenic(...))."""
    wpl = wp_list if (use_waypoints and wp_list and len(wp_list) >= 2) else None
    init_fellow_virtual_mpc_state(
        agent,
        wpl,
        int(wp_last_idx),
        float(car_heading) if car_heading is not None else 0.0,
    )
    apply_virtual_state_to_vehicle_state(agent, vehicle_state)


def fellow_virt_step_for_scenic(
    agent: Any,
    steer_mpc: float,
    current_speed: Any,
    kappa_ref: Optional[float],
    ctrl_dt: float,
    wheel_base: float,
    steer_tau: float,
) -> None:
    """Single entry for Scenic after lateral MPC."""
    step_fellow_virtual_after_mpc(
        agent,
        float(steer_mpc),
        float(current_speed) if current_speed is not None else 0.0,
        kappa_ref,
        float(ctrl_dt),
        float(wheel_base),
        float(steer_tau),
    )
