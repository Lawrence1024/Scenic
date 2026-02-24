"""dSPACE steering IO adapter: single place for road wheel angle (rad) → steering wheel deg.

Per steer_restructure_plan: only here do we use 240 (theta_sw_max_deg) and the conversion.
"""

import math
import numpy as np

# Single source of truth (plan)
DELTA_MAX_RAD = 0.2816
THETA_SW_MAX_DEG = 240.0
# R = theta_sw_max_deg / (delta_max_rad * 180/pi) ≈ 14.9
R = THETA_SW_MAX_DEG / (DELTA_MAX_RAD * 180.0 / math.pi)
# Recommendation A: actuator sign at write. +1 = positive delta_rad -> positive deg -> turn left (match kappa).
# Log showed positive delta_cmd_rad with -1 produced right turn (kappa_meas<0); use +1 so left command = left turn.
STEER_CMD_SIGN = 1.0

_startup_logged = False


def road_rad_to_dspace_value(delta_road_rad: float) -> float:
    """Convert road wheel angle (rad) to dSPACE steering command (steering wheel deg, ±240).

    Only place in the codebase that uses 240 and the conversion.
    STEER_CMD_SIGN applied so delta_cmd_rad sign matches actual curvature (fix actuation sign inversion).
    """
    theta_sw_deg = STEER_CMD_SIGN * delta_road_rad * R * 180.0 / math.pi
    return float(np.clip(theta_sw_deg, -THETA_SW_MAX_DEG, THETA_SW_MAX_DEG))


def log_startup_once():
    """One-time startup log so we know which mode we're in (plan D)."""
    global _startup_logged
    if _startup_logged:
        return
    _startup_logged = True
    print(f"[dSpace steer] dspace_steer_units=\"steering_wheel_deg\" delta_max_rad={DELTA_MAX_RAD} theta_sw_max_deg={THETA_SW_MAX_DEG} R={R:.2f} STEER_CMD_SIGN={STEER_CMD_SIGN}")
