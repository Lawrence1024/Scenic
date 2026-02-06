"""MPC longitudinal controller for racing vehicles.

Implements Model Predictive Control for longitudinal (throttle/brake) control,
replacing PID controllers with predictive control for better speed management.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
import osqp
from scipy.sparse import csc_matrix

from .config import MPCConfig
from .utils import LowPassFilter
import math


class MPCLongitudinalController:
    """MPC-based longitudinal controller for racing vehicles.
    
    Uses Model Predictive Control to compute optimal throttle/brake commands
    for tracking a reference speed profile.
    """
    
    def __init__(self, config: MPCConfig, timestep: float = 0.05):
        """Initialize MPC longitudinal controller.
        
        Args:
            config: MPC configuration object
            timestep: Control period (seconds), will adapt config if needed
        """
        self.config = config
        self.timestep = timestep
        
        # Adapt config to timestep
        if abs(config.ctrl_period - timestep) > 0.01:
            config.adapt_to_timestep(timestep)
        
        # Vehicle parameters (from config or defaults)
        # Access config as dict-like or use getattr for attributes
        self.mass = getattr(config, 'vehicle_mass', 753.87)  # kg
        self.max_accel = getattr(config, 'max_acceleration', 20.0)  # m/s^2
        self.max_decel = getattr(config, 'max_deceleration', 15.0)  # m/s^2
        self.drag_coeff = getattr(config, 'drag_coefficient', 0.881)
        self.cross_area = getattr(config, 'cross_sectional_area', 1.0)  # m^2
        self.air_density = getattr(config, 'air_density', 1.2)  # kg/m^3
        self.rolling_resistance = getattr(config, 'rolling_resistance', 0.013)  # dimensionless
        
        # Initialize low-pass filters for throttle/brake output
        throttle_lpf = getattr(config, 'throttle_lpf_cutoff_hz', 5.0)
        brake_lpf = getattr(config, 'brake_lpf_cutoff_hz', 5.0)
        self.throttle_filter = LowPassFilter(
            cutoff_hz=throttle_lpf,
            dt=timestep
        )
        self.brake_filter = LowPassFilter(
            cutoff_hz=brake_lpf,
            dt=timestep
        )
        
        # State: [v, a] - velocity and acceleration
        self.state = np.zeros(2)
        self.prev_throttle = 0.0
        self.prev_brake = 0.0
        self.prev_accel_cmd = 0.0
        
        # Safety state
        self.invalid_count = 0
        self.last_valid_throttle = 0.0
        self.last_valid_brake = 0.0
        
        # OSQP solver (will be initialized on first solve)
        self.solver = None
        self._solver_initialized = False
    
    def _initialize_solver(self, horizon: int):
        """Initialize OSQP solver for QP problem.
        
        Args:
            horizon: Prediction horizon steps
        """
        # Problem size
        n_x = 2  # state dimension [v, a]
        n_u = 1  # control dimension (acceleration command)
        n_vars = horizon * n_u + (horizon + 1) * n_x
        
        # Create OSQP problem
        self.solver = osqp.OSQP()
        
        # Placeholder matrices (will be updated)
        P = csc_matrix((n_vars, n_vars))
        q = np.zeros(n_vars)
        A = csc_matrix((n_vars, n_vars))
        l = np.zeros(n_vars)
        u = np.zeros(n_vars)
        
        self.solver.setup(P, q, A, l, u, verbose=False, warm_start=True)
        self._solver_initialized = True
    
    def _build_qp_matrices(self,
                          state: np.ndarray,
                          v_ref: np.ndarray,
                          horizon: int) -> Tuple[csc_matrix, np.ndarray, csc_matrix, np.ndarray, np.ndarray]:
        """Build QP matrices for longitudinal MPC problem.
        
        Args:
            state: Current state [v, a]
            v_ref: Reference speed array (m/s)
            horizon: Prediction horizon steps
            
        Returns:
            Tuple of (P, q, A, l, u) for QP: minimize 0.5*x'*P*x + q'*x
            subject to l <= A*x <= u
        """
        horizon = int(horizon)
        if horizon <= 0:
            raise ValueError(f"horizon must be > 0, got {horizon}")
        
        # Ensure v_ref is 1D numpy array with correct length
        if not isinstance(v_ref, np.ndarray):
            v_ref = np.array(v_ref, dtype=np.float64)
        if v_ref.ndim != 1:
            raise ValueError(f"v_ref must be 1D, got {v_ref.ndim}D array")
        if len(v_ref) != horizon:
            raise ValueError(f"v_ref length mismatch: expected {horizon}, got {len(v_ref)}")
        
        n_x = 2  # [v, a]
        n_u = 1  # acceleration command
        n_vars = horizon * n_u + (horizon + 1) * n_x
        
        # Cost matrix P (quadratic)
        P = np.zeros((n_vars, n_vars))
        q = np.zeros(n_vars)
        
        # Constraint matrix A (dynamics + constraints)
        n_constraints = (horizon + 1) * n_x + horizon * n_u  # dynamics + control limits
        A = np.zeros((n_constraints, n_vars))
        l = np.zeros(n_constraints)
        u = np.zeros(n_constraints)
        
        dt = self.config.mpc_prediction_dt
        constraint_idx = 0
        
        # Initial state constraint: x_0 = state
        x_0_idx = 0
        A[constraint_idx, x_0_idx] = 1.0  # v_0
        l[constraint_idx] = state[0]
        u[constraint_idx] = state[0]
        constraint_idx += 1
        
        A[constraint_idx, x_0_idx + 1] = 1.0  # a_0
        l[constraint_idx] = state[1]
        u[constraint_idx] = state[1]
        constraint_idx += 1
        
        # Build cost and dynamics for each step
        for k in range(horizon):
            x_k_idx = k * (n_x + n_u)
            u_k_idx = x_k_idx + n_x
            x_kp1_idx = (k + 1) * (n_x + n_u)
            
            v_k_ref = float(v_ref[k])
            v_k = x_k_idx  # Index for v_k in state vector
            
            # Cost: w_v * (v_k - v_ref)^2 + w_a * a_k^2 + w_u * u_k^2 + w_du * (u_k - u_{k-1})^2
            w_v = getattr(self.config, 'w_v', 10.0)  # Speed tracking weight
            w_a = getattr(self.config, 'w_a', 0.1)  # Acceleration smoothness weight
            w_u = getattr(self.config, 'w_u_lon', 0.05)  # Control input weight
            w_du = getattr(self.config, 'w_du_lon', 0.5)  # Control rate weight
            
            # Speed tracking cost
            P[x_k_idx, x_k_idx] += w_v
            q[x_k_idx] -= 2.0 * w_v * v_k_ref
            
            # Acceleration smoothness cost
            P[x_k_idx + 1, x_k_idx + 1] += w_a
            
            # Control input cost
            P[u_k_idx, u_k_idx] += w_u
            
            # Control rate cost
            if k == 0:
                # First step: compare to previous control
                P[u_k_idx, u_k_idx] += w_du
                q[u_k_idx] -= 2.0 * w_du * self.prev_accel_cmd
            else:
                # Compare to previous step
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                P[u_k_idx, u_k_idx] += w_du
                P[u_km1_idx, u_km1_idx] += w_du
                P[u_k_idx, u_km1_idx] -= w_du
                P[u_km1_idx, u_k_idx] -= w_du
            
            # Dynamics: v_{k+1} = v_k + a_k * dt
            A[constraint_idx, x_k_idx] = 1.0  # v_k
            A[constraint_idx, x_k_idx + 1] = dt  # a_k
            A[constraint_idx, x_kp1_idx] = -1.0  # -v_{k+1}
            l[constraint_idx] = 0.0
            u[constraint_idx] = 0.0
            constraint_idx += 1
            
            # Dynamics: a_{k+1} = a_k + (u_k - a_k) * (dt / tau)
            # Simplified: a_{k+1} = a_k * (1 - dt/tau) + u_k * (dt/tau)
            # For longitudinal, use simple first-order: a_{k+1} = u_k (instantaneous)
            # Or with time constant: a_{k+1} = a_k + (u_k - a_k) * (dt / tau_accel)
            tau_accel = getattr(self.config, 'accel_tau', 0.2)  # Acceleration time constant
            A[constraint_idx, x_k_idx + 1] = 1.0 - dt / tau_accel  # a_k
            A[constraint_idx, u_k_idx] = dt / tau_accel  # u_k
            A[constraint_idx, x_kp1_idx + 1] = -1.0  # -a_{k+1}
            l[constraint_idx] = 0.0
            u[constraint_idx] = 0.0
            constraint_idx += 1
            
            # Control limits: -max_decel <= u_k <= max_accel
            A[constraint_idx, u_k_idx] = 1.0
            l[constraint_idx] = -self.max_decel
            u[constraint_idx] = self.max_accel
            constraint_idx += 1
        
        # Terminal cost
        x_N_idx = horizon * (n_x + n_u)
        wT_v = getattr(self.config, 'wT_v', 20.0)  # Terminal speed weight
        P[x_N_idx, x_N_idx] += wT_v
        if len(v_ref) > 0:
            q[x_N_idx] -= 2.0 * wT_v * float(v_ref[-1])
        
        # Convert to sparse
        P_sparse = csc_matrix(P)
        A_sparse = csc_matrix(A)
        
        return (P_sparse, q, A_sparse, l, u)
    
    def run_step(self,
                 vehicle_state: Dict[str, float],
                 v_ref: np.ndarray,
                 curvature_profile: Optional[np.ndarray] = None,
                 grade_profile: Optional[np.ndarray] = None) -> Tuple[float, float]:
        """Compute throttle/brake commands for one control step.
        
        Args:
            vehicle_state: Dictionary with keys:
                - 'speed': current speed (m/s)
                - 'acceleration': current acceleration (m/s^2, optional)
            v_ref: Reference speed array for horizon (m/s)
            curvature_profile: Optional curvature profile for speed adaptation (not used in simplified model)
            grade_profile: Optional road grade profile (radians, positive = uphill) for gravity compensation
            
        Returns:
            Tuple of (throttle, brake) in normalized range [0.0, 1.0]
        """
        # Extract state
        v = vehicle_state.get('speed', 0.0)
        a = vehicle_state.get('acceleration', 0.0)
        
        # Estimate acceleration from speed if not provided
        if a == 0.0 and hasattr(self, '_prev_speed'):
            a = (v - self._prev_speed) / self.timestep
            # Limit acceleration estimate to reasonable range
            a = max(-self.max_decel, min(self.max_accel, a))
        self._prev_speed = v
        
        # Safety check: disable MPC if speed is very low and gear is unknown/neutral
        gear = vehicle_state.get('gear', None)
        if gear is not None and gear < 1:
            # In neutral - return zero throttle/brake
            return 0.0, 0.0
        elif abs(v) < 0.1 and gear is None:
            # Very slow or stopped AND gear unknown - return zero
            return 0.0, 0.0
        
        # Build reference speed array if single value provided
        # Accept list or numpy array, convert to numpy
        if isinstance(v_ref, list):
            v_ref = np.array(v_ref, dtype=np.float64)
        elif not isinstance(v_ref, np.ndarray):
            v_ref = np.array([v_ref] * self.config.mpc_prediction_horizon, dtype=np.float64)
        
        # Ensure correct length
        if len(v_ref) != self.config.mpc_prediction_horizon:
            # Pad or truncate to match horizon
            if len(v_ref) < self.config.mpc_prediction_horizon:
                v_ref = np.pad(v_ref, (0, self.config.mpc_prediction_horizon - len(v_ref)), 
                              mode='edge')
            else:
                v_ref = v_ref[:self.config.mpc_prediction_horizon]
        
        # Build QP matrices
        try:
            P, q, A, l, u = self._build_qp_matrices(
                np.array([v, a]), v_ref, self.config.mpc_prediction_horizon
            )
        except Exception as e:
            print(f"[MPC Longitudinal] Error building QP matrices: {e}")
            return self._fallback_control(v, v_ref[0] if len(v_ref) > 0 else v)
        
        # Initialize solver on first solve
        if not self._solver_initialized:
            self._initialize_solver(self.config.mpc_prediction_horizon)
            self._solver_initialized = True
        
        # Solve QP
        try:
            self.solver.setup(P, q, A, l, u, verbose=False, warm_start=True)
            result = self.solver.solve()
            
            if result.info.status != 'solved':
                print(f"[MPC Longitudinal] Solver failed: {result.info.status}")
                return self._fallback_control(v, v_ref[0] if len(v_ref) > 0 else v)
            
            # Extract first control input (acceleration command)
            n_x = 2
            u_0_idx = n_x  # First control is after initial state
            accel_cmd = float(result.x[u_0_idx])
            
            # Update previous control
            self.prev_accel_cmd = accel_cmd
            
            # Get current road grade (for gravity compensation)
            current_grade = 0.0
            if grade_profile is not None and len(grade_profile) > 0:
                current_grade = float(grade_profile[0])
            
            # Convert acceleration command to throttle/brake (with gravity compensation)
            throttle, brake = self._accel_to_throttle_brake(accel_cmd, v, current_grade)
            
            # Apply low-pass filters
            throttle_filtered = self.throttle_filter.update(throttle)
            brake_filtered = self.brake_filter.update(brake)
            
            # Reset invalid count on success
            self.invalid_count = 0
            self.last_valid_throttle = throttle_filtered
            self.last_valid_brake = brake_filtered
            
            # Update state estimate
            self.state = np.array([v, a])
            
            return float(throttle_filtered), float(brake_filtered)
            
        except Exception as e:
            print(f"[MPC Longitudinal] Solver error: {e}")
            self.invalid_count += 1
            max_invalid = getattr(self.config, 'max_invalid_count', 10)
            if self.invalid_count > max_invalid:
                # Too many failures, return zero
                return 0.0, 0.0
            return self._fallback_control(v, v_ref[0] if len(v_ref) > 0 else v)
    
    def _accel_to_throttle_brake(self, accel_cmd: float, current_speed: float, road_grade: float = 0.0) -> Tuple[float, float]:
        """Convert acceleration command to throttle/brake with gravity compensation.
        
        Args:
            accel_cmd: Desired acceleration (m/s^2)
            current_speed: Current vehicle speed (m/s)
            road_grade: Road grade angle (radians, positive = uphill, negative = downhill)
            
        Returns:
            Tuple of (throttle, brake) in [0.0, 1.0]
        """
        g = 9.81  # Gravitational acceleration (m/s^2)
        
        # Compute gravity force component along the road
        # Positive grade (uphill): gravity resists motion (positive force needed)
        # Negative grade (downhill): gravity assists motion (negative force, reduces needed throttle/brake)
        gravity_force = self.mass * g * math.sin(road_grade)
        gravity_accel = gravity_force / self.mass  # Acceleration due to gravity
        
        if accel_cmd > 0:
            # Accelerating: use throttle
            # Account for drag, rolling resistance, and gravity
            drag_force = 0.5 * self.air_density * self.drag_coeff * self.cross_area * current_speed * current_speed
            rolling_force = self.rolling_resistance * self.mass * g
            total_resistance = drag_force + rolling_force + gravity_force  # Add gravity force
            resistance_accel = total_resistance / self.mass
            
            # Required acceleration = accel_cmd + resistance_accel
            # Note: gravity_accel is already included in resistance_accel via gravity_force
            required_accel = accel_cmd + resistance_accel
            throttle = min(1.0, max(0.0, required_accel / self.max_accel))
            brake = 0.0
        else:
            # Decelerating: use brake
            throttle = 0.0
            
            # For braking, gravity affects the required brake force:
            # Uphill: gravity helps braking (less brake needed)
            # Downhill: gravity resists braking (more brake needed)
            # Effective deceleration needed = |accel_cmd| - gravity_accel
            # (gravity_accel is negative downhill, so we subtract it)
            effective_decel = abs(accel_cmd) - gravity_accel
            effective_decel = max(0.0, effective_decel)  # Don't allow negative brake
            
            # Brake force needed to achieve deceleration
            brake = min(1.0, max(0.0, effective_decel / self.max_decel))
        
        return throttle, brake
    
    def _fallback_control(self, v: float, v_ref: float) -> Tuple[float, float]:
        """Simple proportional fallback control.
        
        Args:
            v: Current speed (m/s)
            v_ref: Reference speed (m/s)
            
        Returns:
            Tuple of (throttle, brake) in [0.0, 1.0]
        """
        error = v_ref - v
        K_p = 0.1  # Proportional gain
        
        if error > 0:
            # Need to accelerate
            throttle = min(1.0, error * K_p)
            brake = 0.0
        else:
            # Need to decelerate
            throttle = 0.0
            brake = min(1.0, abs(error) * K_p)
        
        return throttle, brake
