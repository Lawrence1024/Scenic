"""Racing domain constants — single source of truth for vehicle/steering limits.

Used by: behaviors, MPC config, dSPACE steer_io. Do not hardcode DELTA_MAX_RAD
or THETA_SW_MAX_DEG elsewhere.
"""

import math

# Road wheel angle (front wheel) limit in radians. From dspace_iac_car.param.yaml.
DELTA_MAX_RAD = 0.2816

# dSPACE steering wheel full lock in degrees (±240).
THETA_SW_MAX_DEG = 240.0

# Ratio: steering_wheel_deg = delta_road_rad * R * 180/pi  =>  R = THETA_SW_MAX_DEG / (DELTA_MAX_RAD * 180/pi)
R = THETA_SW_MAX_DEG / (DELTA_MAX_RAD * 180.0 / math.pi)  # ~14.9
