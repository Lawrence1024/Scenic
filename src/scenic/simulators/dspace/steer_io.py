"""dSPACE steering IO adapter: single place for road wheel angle (rad) → steering wheel deg.

Per steer_restructure_plan: only here do we use the conversion (R, 240).
Constants DELTA_MAX_RAD, THETA_SW_MAX_DEG, R come from racing.constants.
"""

import math
import numpy as np

from scenic.domains.racing.constants import DELTA_MAX_RAD, THETA_SW_MAX_DEG, R

# Recommendation A: actuator sign at write. +1 = positive delta_rad -> positive deg -> turn left (match kappa).
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
