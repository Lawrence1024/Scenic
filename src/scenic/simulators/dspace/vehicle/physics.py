"""Vehicle physics model for fellow (v, d) control.

Converts racing library control inputs into the v and d values written to
dSPACE External_Signals:
- Throttle and brake -> longitudinal acceleration -> velocity v (m/s).
- Steering angle (rad) -> yaw rate (kinematic bicycle) -> heading error evolution
  -> lateral deviation rate d_dot = v*sin(psi_e) -> d (m from centerline).

Uses a heading-error + kinematic bicycle formulation so that steering intent
affects turning (yaw rate) and heading error drives lateral deviation rate,
fixing the "not turning enough at low v / overshoot-throttle loop" with the
previous direct steering->lateral_velocity model.
"""

import logging
import math

logger = logging.getLogger(__name__)


class VehiclePhysicsState:
    """Physics model for fellow (v, d) with heading error and kinematic bicycle.

    State: velocity (m/s), deviation (m), heading_error (rad).
    - Longitudinal: throttle/brake -> acceleration -> v (unchanged).
    - Lateral: steering angle delta (rad) -> yaw rate r = v*tan(delta)/L
      -> heading_error_dot = r (or r - v*kappa_ref); deviation_dot = v*sin(psi_e).

    This matches the fact that dSPACE does not allow direct heading control:
    steering and throttle affect how the car turns; we predict d from heading
    error and yaw rate from steering.
    """

    # Default road wheel angle limit (rad) for clamping; match racing constants when used
    DEFAULT_DELTA_MAX_RAD = 0.2816
    DEFAULT_WHEELBASE_M = 2.7

    def __init__(self, initial_velocity=0.0, initial_deviation=0.0, initial_heading_error=0.0):
        """Initialize physics state.

        Args:
            initial_velocity: Starting velocity in m/s (default: 0.0)
            initial_deviation: Starting lateral deviation from centerline in meters (default: 0.0)
            initial_heading_error: Starting heading error vs path in rad (default: 0.0)
        """
        # State variables
        self.velocity = initial_velocity  # m/s
        self.deviation = initial_deviation  # m (lateral offset from centerline)
        self.heading_error = initial_heading_error  # rad (vehicle heading - path tangent)

        # Longitudinal dynamics parameters (tunable)
        self.max_acceleration = 20.0  # m/s²
        self.max_deceleration = 15.0  # m/s²
        self.max_velocity = 100.0  # m/s
        self.min_velocity = 0.0  # m/s

        # Lateral: kinematic bicycle
        self.wheelbase = self.DEFAULT_WHEELBASE_M  # m
        self.delta_max_rad = self.DEFAULT_DELTA_MAX_RAD  # rad, for normalizing when steering is [-1,1]
        # Path curvature (rad/m) at current s; 0 = straight (can be set by caller later)
        self.path_curvature = 0.0
        # Clamp heading error to avoid sin explosion (e.g. ±pi)
        self.heading_error_max_rad = math.pi
        # Last-step diagnostics (set in update() for controller logging)
        self._last_delta_rad = 0.0
        self._last_yaw_rate = 0.0
        self._last_psi_e_dot = 0.0
        self._last_d_dot = 0.0
        self._last_acceleration = 0.0

    def update(self, throttle, brake, steering, dt, steering_rad=None):
        """Update physics state: throttle/brake -> v; steering (angle) -> psi_e, d.

        Args:
            throttle: Throttle input in range [0.0, 1.0]
            brake: Brake input in range [0.0, 1.0]
            steering: Steering input in range [-1.0, 1.0] (used if steering_rad is None)
            dt: Time step in seconds
            steering_rad: Optional steering angle in radians (e.g. from MPC). If given, used for lateral dynamics; else delta = steering * delta_max_rad.

        Returns:
            Tuple[float, float]: (new_velocity, new_deviation) in m/s and meters
        """
        # 1. Longitudinal dynamics (velocity)
        acceleration = throttle * self.max_acceleration - brake * self.max_deceleration
        self.velocity += acceleration * dt
        self.velocity = max(self.min_velocity, min(self.max_velocity, self.velocity))
        v = self.velocity

        # 2. Steering angle in rad (for kinematic bicycle)
        if steering_rad is not None:
            delta_rad = float(steering_rad)
            delta_rad = max(-self.delta_max_rad, min(self.delta_max_rad, delta_rad))
        else:
            delta_rad = steering * self.delta_max_rad
            delta_rad = max(-self.delta_max_rad, min(self.delta_max_rad, delta_rad))

        # 3. Yaw rate from kinematic bicycle: r = v * tan(delta) / L
        # Avoid tan blow-up and division by zero
        if abs(v) < 0.01:
            yaw_rate = 0.0
        else:
            L = max(0.1, self.wheelbase)
            yaw_rate = v * math.tan(delta_rad) / L

        # 4. Heading error evolution: psi_e_dot = r - v * kappa_ref (path curvature)
        kappa = getattr(self, "path_curvature", 0.0)
        psi_e_dot = yaw_rate - v * kappa
        self.heading_error += psi_e_dot * dt
        self.heading_error = max(
            -self.heading_error_max_rad,
            min(self.heading_error_max_rad, self.heading_error),
        )

        # 5. Lateral deviation rate: d_dot = v * sin(psi_e) (path-following relation)
        d_dot = v * math.sin(self.heading_error)
        self.deviation += d_dot * dt

        # Store last-step diagnostics for controller logging
        self._last_delta_rad = delta_rad
        self._last_yaw_rate = yaw_rate
        self._last_psi_e_dot = psi_e_dot
        self._last_d_dot = d_dot
        self._last_acceleration = acceleration

        logger.debug(
            "fellow_physics step: v=%.2f d=%.3f psi_e=%.3f rad | delta=%.3f r=%.3f psi_e_dot=%.3f d_dot=%.3f | a=%.2f dt=%.3f",
            self.velocity, self.deviation, self.heading_error,
            delta_rad, yaw_rate, psi_e_dot, d_dot, acceleration, dt,
        )

        return self.velocity, self.deviation

    def update_longitudinal_only(self, throttle, brake, dt):
        """Throttle/brake -> velocity only; lateral state unchanged.

        Used when fellow lateral position is commanded via racing-line servo
        (Const_d aligned to optimal line) instead of bicycle-from-steering.
        """
        acceleration = throttle * self.max_acceleration - brake * self.max_deceleration
        self.velocity += acceleration * dt
        self.velocity = max(self.min_velocity, min(self.max_velocity, self.velocity))
        self._last_acceleration = acceleration
        self._last_delta_rad = 0.0
        self._last_yaw_rate = 0.0
        self._last_psi_e_dot = 0.0
        self._last_d_dot = 0.0
        return self.velocity

    def reset(self, velocity=0.0, deviation=0.0, heading_error=0.0):
        """Reset physics state."""
        self.velocity = velocity
        self.deviation = deviation
        self.heading_error = heading_error

    def set_parameters(self, **kwargs):
        """Set physics parameters for tuning.

        Valid parameters: max_acceleration, max_deceleration, max_velocity,
        min_velocity, wheelbase, delta_max_rad, path_curvature, heading_error_max_rad.
        """
        for param, value in kwargs.items():
            if hasattr(self, param):
                setattr(self, param, value)
            else:
                raise ValueError(f"Unknown physics parameter: {param}")
