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

_startup_logged = False


def road_rad_to_dspace_value(delta_road_rad: float) -> float:
    """Convert road wheel angle (rad) to dSPACE steering command (steering wheel deg, ±240).

    Only place in the codebase that uses 240 and the conversion.
    """
    theta_sw_deg = delta_road_rad * R * 180.0 / math.pi
    return float(np.clip(theta_sw_deg, -THETA_SW_MAX_DEG, THETA_SW_MAX_DEG))


def log_startup_once():
    """One-time startup log so we know which mode we're in (plan D)."""
    global _startup_logged
    if _startup_logged:
        return
    _startup_logged = True
    print(f"[dSpace steer] dspace_steer_units=\"steering_wheel_deg\" delta_max_rad={DELTA_MAX_RAD} theta_sw_max_deg={THETA_SW_MAX_DEG} R={R:.2f}")
