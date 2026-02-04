"""MPC lateral controller for racing vehicles.

Implements Model Predictive Control for lateral (steering) control,
replacing PID controllers with predictive control for better racing performance.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
import osqp
from scipy.sparse import csc_matrix

from .config import MPCConfig
from .reference_builder import ReferenceBuilder
from .utils import LowPassFilter
import math


class MPCLateralController:
    """MPC-based lateral controller for racing vehicles.
    
    Uses Model Predictive Control to compute optimal steering commands
    for tracking a reference trajectory defined by waypoints.
    """
    
    def __init__(self, config: MPCConfig, timestep: float = 0.05):
        """Initialize MPC controller.
        
        Args:
            config: MPC configuration object
            timestep: Control period (seconds), will adapt config if needed
        """
        self.config = config
        self.timestep = timestep
        
        # Adapt config to timestep
        if abs(config.ctrl_period - timestep) > 0.01:
            config.adapt_to_timestep(timestep)
        
        # Initialize reference builder
        self.ref_builder = ReferenceBuilder(
            resample_dist=config.traj_resample_dist,
            curvature_smoothing_num=config.curvature_smoothing_num
        )
        
        # Initialize low-pass filter for steering output
        self.steering_filter = LowPassFilter(
            cutoff_hz=config.steering_lpf_cutoff_hz,
            dt=timestep
        )
        
        # State: [e_y, e_psi, delta]
        self.state = np.zeros(3)
        self.prev_control = 0.0  # Previous steering command (for rate penalty)
        self.prev_prev_control = 0.0  # Previous-previous steering command (for acceleration penalty)
        
        # Safety state
        self.invalid_count = 0
        self.last_valid_steering = 0.0
        
        # OSQP solver (will be initialized on first solve)
        self.solver = None
        self._solver_initialized = False
    
    def _initialize_solver(self, horizon: int):
        """Initialize OSQP solver for QP problem.
        
        Args:
            horizon: Prediction horizon steps
        """
        # Problem size
        n_x = 3  # state dimension
        n_u = 1  # control dimension
        n_vars = horizon * n_u + (horizon + 1) * n_x
        
        # Create OSQP problem (will be updated each step)
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
                          psi_ref: np.ndarray,
                          kappa_ref: np.ndarray,
                          v_ref: np.ndarray,
                          horizon: int) -> Tuple[csc_matrix, np.ndarray, csc_matrix, np.ndarray, np.ndarray]:
        """Build QP matrices for MPC problem.
        
        Args:
            state: Current state [e_y, e_psi, delta]
            psi_ref: Reference heading array (radians)
            kappa_ref: Reference curvature array (1/meters)
            v_ref: Reference speed array (m/s)
            horizon: Prediction horizon steps
            
        Returns:
            Tuple of (P, q, A, l, u) for QP: minimize 0.5*x'*P*x + q'*x
            subject to l <= A*x <= u
        """
        # Ensure horizon is a positive integer
        horizon = int(horizon)
        if horizon <= 0:
            raise ValueError(f"horizon must be > 0, got {horizon}")
        
        # Ensure all reference arrays are 1D numpy arrays with correct shape
        # Arrays should already be correct from build_reference, but validate them
        if not isinstance(psi_ref, np.ndarray):
            raise TypeError(f"psi_ref must be a numpy array, got {type(psi_ref)}")
        if not isinstance(kappa_ref, np.ndarray):
            raise TypeError(f"kappa_ref must be a numpy array, got {type(kappa_ref)}")
        if not isinstance(v_ref, np.ndarray):
            raise TypeError(f"v_ref must be a numpy array, got {type(v_ref)}")
        
        # Ensure arrays are 1D
        if psi_ref.ndim != 1:
            raise ValueError(f"psi_ref must be 1D, got {psi_ref.ndim}D array with shape {psi_ref.shape}")
        if kappa_ref.ndim != 1:
            raise ValueError(f"kappa_ref must be 1D, got {kappa_ref.ndim}D array with shape {kappa_ref.shape}")
        if v_ref.ndim != 1:
            raise ValueError(f"v_ref must be 1D, got {v_ref.ndim}D array with shape {v_ref.shape}")
        
        # Verify arrays have correct length
        if len(psi_ref) != horizon:
            raise ValueError(f"psi_ref length mismatch: expected {horizon}, got {len(psi_ref)}. Shape: {psi_ref.shape}, dtype: {psi_ref.dtype}")
        if len(kappa_ref) != horizon:
            raise ValueError(f"kappa_ref length mismatch: expected {horizon}, got {len(kappa_ref)}. Shape: {kappa_ref.shape}, dtype: {kappa_ref.dtype}")
        if len(v_ref) != horizon:
            raise ValueError(f"v_ref length mismatch: expected {horizon}, got {len(v_ref)}. Shape: {v_ref.shape}, dtype: {v_ref.dtype}")
        
        n_x = 3
        n_u = 1
        n_vars = horizon * n_u + (horizon + 1) * n_x
        
        # Cost matrix P (quadratic)
        P = np.zeros((n_vars, n_vars))
        q = np.zeros(n_vars)
        
        # Constraint matrix A (dynamics + constraints)
        n_constraints = (horizon + 1) * n_x + horizon * n_u  # dynamics + control limits
        A = np.zeros((n_constraints, n_vars))
        l = np.zeros(n_constraints)
        u = np.zeros(n_constraints)
        
        # Variable ordering: [x_0, u_0, x_1, u_1, ..., x_N]
        # x_k indices: k * (n_x + n_u)
        # u_k indices: k * (n_x + n_u) + n_x
        
        dt = self.config.mpc_prediction_dt
        L = self.config.wheel_base
        tau = self.config.steer_tau
        delta_max = self.config.max_steer_angle
        
        # Build cost and constraints
        constraint_idx = 0
        
        # Initial state constraint: x_0 = state
        for i in range(n_x):
            A[constraint_idx, i] = 1.0
            l[constraint_idx] = state[i]
            u[constraint_idx] = state[i]
            constraint_idx += 1
        
        # Dynamics and cost for each step
        for k in range(horizon):
            x_k_idx = k * (n_x + n_u)
            u_k_idx = x_k_idx + n_x
            x_kp1_idx = (k + 1) * (n_x + n_u)
            
            v_k = v_ref[k]
            kappa_k = kappa_ref[k]
            psi_ref_k = psi_ref[k]
            
            # Select weights based on curvature (three regions: low, moderate, high)
            if self.config.use_adaptive_weights:
                abs_kappa = abs(kappa_k)
                if abs_kappa < self.config.low_curvature_threshold:
                    # Low curvature (straight sections) - relaxed tracking
                    w_ey_k = self.config.w_ey_low_curv
                    w_epsi_k = self.config.w_epsi_low_curv
                    w_epsi_vel_k = self.config.w_epsi_vel_low_curv
                    w_u_k = self.config.w_u_low_curv
                    w_u_vel_k = self.config.w_u_vel_low_curv
                    w_ddu_k = self.config.w_ddu_low_curv
                elif abs_kappa >= self.config.high_curvature_threshold:
                    # High curvature (sharp turns) - aggressive tracking, fast steering
                    w_ey_k = self.config.w_ey_high_curv
                    w_epsi_k = self.config.w_epsi_high_curv
                    w_epsi_vel_k = self.config.w_epsi_vel_high_curv
                    w_u_k = self.config.w_u_high_curv
                    w_u_vel_k = self.config.w_u_vel_high_curv
                    w_ddu_k = self.config.w_ddu_high_curv
                else:
                    # Moderate curvature (normal curves) - base weights
                    w_ey_k = self.config.w_ey
                    w_epsi_k = self.config.w_epsi
                    w_epsi_vel_k = self.config.w_epsi_vel
                    w_u_k = self.config.w_u
                    w_u_vel_k = self.config.w_u_vel
                    w_ddu_k = self.config.w_ddu
            else:
                # Adaptive weights disabled - use base weights for all
                w_ey_k = self.config.w_ey
                w_epsi_k = self.config.w_epsi
                w_epsi_vel_k = self.config.w_epsi_vel
                w_u_k = self.config.w_u
                w_u_vel_k = self.config.w_u_vel
                w_ddu_k = self.config.w_ddu
            
            # State cost: w_ey * e_y^2 + w_epsi * e_psi^2 + w_epsi_vel * e_psi^2 * v^2
            P[x_k_idx, x_k_idx] += w_ey_k  # e_y
            P[x_k_idx + 1, x_k_idx + 1] += w_epsi_k + w_epsi_vel_k * v_k * v_k  # e_psi (with velocity weighting)
            
            # Control cost: w_u * u^2 + w_u_vel * u^2 * v^2
            P[u_k_idx, u_k_idx] += w_u_k + w_u_vel_k * v_k * v_k
            
            # Control rate cost: w_du * (u_k - u_{k-1})^2
            if k == 0:
                # First step: compare to previous control
                P[u_k_idx, u_k_idx] += self.config.w_du
                q[u_k_idx] -= 2.0 * self.config.w_du * self.prev_control
            else:
                # Compare to previous step
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                P[u_k_idx, u_k_idx] += self.config.w_du
                P[u_km1_idx, u_km1_idx] += self.config.w_du
                P[u_k_idx, u_km1_idx] -= self.config.w_du
                P[u_km1_idx, u_k_idx] -= self.config.w_du
            
            # Steering acceleration cost: w_ddu * (u_k - 2*u_{k-1} + u_{k-2})^2
            # This penalizes rapid changes in steering rate (jerk)
            if k == 0:
                # First step: compare to previous two controls
                # (u_0 - 2*u_{-1} + u_{-2})^2 = u_0^2 - 4*u_0*u_{-1} + 4*u_{-1}^2 + 2*u_0*u_{-2} - 4*u_{-1}*u_{-2} + u_{-2}^2
                P[u_k_idx, u_k_idx] += w_ddu_k
                q[u_k_idx] -= 4.0 * w_ddu_k * self.prev_control
                q[u_k_idx] += 2.0 * w_ddu_k * self.prev_prev_control
            elif k == 1:
                # Second step: compare to previous step and previous-previous control
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                # (u_1 - 2*u_0 + u_{-1})^2
                P[u_k_idx, u_k_idx] += w_ddu_k
                P[u_km1_idx, u_km1_idx] += 4.0 * w_ddu_k
                P[u_k_idx, u_km1_idx] -= 4.0 * w_ddu_k
                P[u_km1_idx, u_k_idx] -= 4.0 * w_ddu_k
                q[u_k_idx] += 2.0 * w_ddu_k * self.prev_prev_control
                q[u_km1_idx] -= 4.0 * w_ddu_k * self.prev_prev_control
            else:
                # Compare to previous two steps
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                u_km2_idx = (k - 2) * (n_x + n_u) + n_x
                # (u_k - 2*u_{k-1} + u_{k-2})^2
                P[u_k_idx, u_k_idx] += w_ddu_k
                P[u_km1_idx, u_km1_idx] += 4.0 * w_ddu_k
                P[u_km2_idx, u_km2_idx] += w_ddu_k
                P[u_k_idx, u_km1_idx] -= 4.0 * w_ddu_k
                P[u_km1_idx, u_k_idx] -= 4.0 * w_ddu_k
                P[u_k_idx, u_km2_idx] += 2.0 * w_ddu_k
                P[u_km2_idx, u_k_idx] += 2.0 * w_ddu_k
                P[u_km1_idx, u_km2_idx] -= 4.0 * w_ddu_k
                P[u_km2_idx, u_km1_idx] -= 4.0 * w_ddu_k
            
            # Dynamics: x_{k+1} = A_k * x_k + B_k * u_k + g_k
            # e_y_{k+1} = e_y_k + v_k * e_psi_k * dt
            A[constraint_idx, x_k_idx] = 1.0  # e_y_k
            A[constraint_idx, x_k_idx + 1] = v_k * dt  # e_psi_k
            A[constraint_idx, x_kp1_idx] = -1.0  # -e_y_{k+1}
            l[constraint_idx] = 0.0
            u[constraint_idx] = 0.0
            constraint_idx += 1
            
            # e_psi_{k+1} = e_psi_k + (v_k/L) * delta_k * dt - v_k * kappa_ref_k * dt
            A[constraint_idx, x_k_idx + 1] = 1.0  # e_psi_k
            A[constraint_idx, x_k_idx + 2] = (v_k / L) * dt  # delta_k
            A[constraint_idx, x_kp1_idx + 1] = -1.0  # -e_psi_{k+1}
            l[constraint_idx] = -v_k * kappa_k * dt  # curvature feedforward
            u[constraint_idx] = -v_k * kappa_k * dt
            constraint_idx += 1
            
            # delta_{k+1} = delta_k + (dt/tau) * (u_k - delta_k)
            A[constraint_idx, x_k_idx + 2] = 1.0 - dt/tau  # delta_k
            A[constraint_idx, u_k_idx] = dt/tau  # u_k
            A[constraint_idx, x_kp1_idx + 2] = -1.0  # -delta_{k+1}
            l[constraint_idx] = 0.0
            u[constraint_idx] = 0.0
            constraint_idx += 1
            
            # Control limits: |u_k| <= delta_max
            A[constraint_idx, u_k_idx] = 1.0
            l[constraint_idx] = -delta_max
            u[constraint_idx] = delta_max
            constraint_idx += 1
        
        # Terminal cost
        x_N_idx = horizon * (n_x + n_u)
        P[x_N_idx, x_N_idx] += self.config.wT_ey
        P[x_N_idx + 1, x_N_idx + 1] += self.config.wT_epsi
        
        # Convert to sparse
        P_sparse = csc_matrix(P)
        A_sparse = csc_matrix(A)
        
        return (P_sparse, q, A_sparse, l, u)
    
    def run_step(self,
                 vehicle_state: Dict[str, float],
                 waypoints: List[Tuple[float, float]],
                 current_waypoint_idx: Optional[int] = None,
                 cte_magnitude: Optional[float] = None) -> float:
        """Compute steering command for one control step.
        
        Args:
            vehicle_state: Dictionary with keys:
                - 'x', 'y': position (meters)
                - 'yaw': heading (radians)
                - 'speed': speed (m/s)
                - 'yaw_rate': yaw rate (rad/s, optional)
                - 'gear': current gear (optional, for checking if in neutral)
            waypoints: List of waypoint (x, y) tuples
            current_waypoint_idx: Current waypoint index (for efficiency)
            
        Returns:
            Steering command in normalized range [-1.0, 1.0]
        """
        # Extract state
        x = vehicle_state.get('x', 0.0)
        y = vehicle_state.get('y', 0.0)
        yaw = vehicle_state.get('yaw', 0.0)
        speed = vehicle_state.get('speed', 0.0)
        gear = vehicle_state.get('gear', None)  # Get gear if available
        
        # Safety check: disable MPC if in neutral (gear 0) or if speed is very low AND gear unknown
        # Allow MPC to work from stopped state if gear is set (gear >= 1)
        if gear is not None:
            if gear < 1:
                # In neutral - return zero steering
                return 0.0
        elif abs(speed) < 0.1:
            # Very slow or stopped AND gear unknown - return zero steering
            return 0.0
        
        # Build reference trajectory
        try:
            psi_ref, kappa_ref, v_ref, new_waypoint_idx = self.ref_builder.build_reference(
                waypoints=waypoints,
                current_position=(x, y),
                current_heading=yaw,
                horizon_steps=self.config.mpc_prediction_horizon,
                dt=self.config.mpc_prediction_dt,
                speed=speed,
                last_waypoint_idx=current_waypoint_idx,
                cte_magnitude=cte_magnitude
            )
            # Debug: verify arrays right after build_reference returns
            if len(psi_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned psi_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(psi_ref)}. Shape: {psi_ref.shape}, dtype: {psi_ref.dtype}")
            if len(kappa_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned kappa_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(kappa_ref)}. Shape: {kappa_ref.shape}, dtype: {kappa_ref.dtype}")
            if len(v_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned v_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(v_ref)}. Shape: {v_ref.shape}, dtype: {v_ref.dtype}")
        except Exception as e:
            print(f"[MPC] Error building reference: {e}")
            # Try to compute errors for proportional fallback, but don't fail if it doesn't work
            try:
                e_y, e_psi, _ = self._compute_errors(
                    position=(x, y),
                    heading=yaw,
                    waypoints=waypoints
                )
                return self._fallback_steering(e_y=e_y, e_psi=e_psi)
            except:
                return self._fallback_steering(e_y=None, e_psi=None)
        
        # Compute current state [e_y, e_psi, delta]
        # MPC now dynamically selects the best segment each step
        e_y, e_psi, mpc_segment_idx = self._compute_errors(
            position=(x, y),
            heading=yaw,
            waypoints=waypoints
        )

        # Log the actual reference segment being used for MPC control
        print(f"[MPC Reference] Using segment {mpc_segment_idx} (waypoints {mpc_segment_idx} -> {mpc_segment_idx+1}) for control")
        
        # Compute reference heading for logging (extract from _compute_errors logic)
        # NOTE: Use different variable name to avoid shadowing the psi_ref array from build_reference
        psi_ref_logging = None
        if waypoints and len(waypoints) > new_waypoint_idx and new_waypoint_idx >= 0:
            if new_waypoint_idx < len(waypoints) - 1:
                wp0 = waypoints[new_waypoint_idx]
                wp1 = waypoints[new_waypoint_idx + 1]
                seg_dx = wp1[0] - wp0[0]
                seg_dy = wp1[1] - wp0[1]
                seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
                if seg_len > 1e-6:
                    psi_ref_logging = np.arctan2(seg_dy, seg_dx)
                    # Apply same 180° flip logic as in _compute_errors
                    heading_diff = psi_ref_logging - yaw
                    heading_diff = np.arctan2(np.sin(heading_diff), np.cos(heading_diff))
                    if abs(heading_diff) > np.pi / 2:  # > 90 degrees
                        psi_ref_logging = np.arctan2(np.sin(psi_ref_logging + np.pi), np.cos(psi_ref_logging + np.pi))
        
        # Safety check: disable MPC if errors too large
        # Use proportional fallback to prevent catch-22 (large error → no steering → larger error)
        if abs(e_y) > self.config.admissible_position_error:
            print(f"[MPC] Position error too large: {e_y:.2f}m > {self.config.admissible_position_error}m")
            return self._fallback_steering(e_y=e_y, e_psi=e_psi)
        
        if abs(e_psi) > self.config.admissible_yaw_error_rad:
            # Enhanced logging: show orientation information to diagnose large yaw errors
            vehicle_heading_deg = yaw * 180.0 / np.pi
            yaw_error_deg = e_psi * 180.0 / np.pi
            if psi_ref_logging is not None:
                ref_heading_deg = psi_ref_logging * 180.0 / np.pi
                print(f"[MPC] Yaw error too large: {e_psi:.2f}rad ({yaw_error_deg:.1f}deg) > {self.config.admissible_yaw_error_rad:.2f}rad")
                print(f"[MPC] Orientation details: vehicle_heading={vehicle_heading_deg:.1f}deg, reference_heading={ref_heading_deg:.1f}deg, error={yaw_error_deg:.1f}deg")
            else:
                print(f"[MPC] Yaw error too large: {e_psi:.2f}rad ({yaw_error_deg:.1f}deg) > {self.config.admissible_yaw_error_rad:.2f}rad")
                print(f"[MPC] Orientation details: vehicle_heading={vehicle_heading_deg:.1f}deg, reference_heading=N/A, error={yaw_error_deg:.1f}deg")
            return self._fallback_steering(e_y=e_y, e_psi=e_psi)
        
        # Get current steering angle (delta)
        # Priority: 1) From vehicle_state (read from ControlDesk), 2) Previous state, 3) Zero
        delta = vehicle_state.get('steer_actual', None)
        if delta is None:
            # Fallback: use previous state estimate
            delta = self.state[2] if hasattr(self, 'state') and len(self.state) > 2 else 0.0
        
        self.state = np.array([e_y, e_psi, delta])
        
        # Build QP matrices
        # Don't catch exceptions - let them propagate to identify bugs
        P, q, A, l, u = self._build_qp_matrices(
            self.state, psi_ref, kappa_ref, v_ref,
            self.config.mpc_prediction_horizon
        )
        
        # Initialize solver on first solve
        if not self._solver_initialized:
            self._initialize_solver(self.config.mpc_prediction_horizon)
            self._solver_initialized = True
        
        # Solve QP
        try:
            # OSQP requires matrix structure to remain constant for updates
            # Since our matrices may have different sparsity patterns each step,
            # we need to setup with actual matrices each time
            # (This is acceptable for MPC as setup is fast compared to solve)
            self.solver.setup(P, q, A, l, u, verbose=False, warm_start=True)
            result = self.solver.solve()
            
            if result.info.status != 'solved':
                print(f"[MPC] Solver failed: {result.info.status}")
                return self._fallback_steering()
            
            # Extract first control input
            n_x = 3  # State dimension
            u_0_idx = n_x  # First control is after initial state
            delta_cmd_rad_raw = float(result.x[u_0_idx])
            delta_cmd_rad = delta_cmd_rad_raw
            
            # Update previous controls (for next iteration's acceleration penalty)
            self.prev_prev_control = self.prev_control
            self.prev_control = delta_cmd_rad
            
            # Convert to normalized steering [-1, 1]
            steer_normalized = np.clip(delta_cmd_rad / self.config.max_steer_angle, -1.0, 1.0)
            
            # Apply low-pass filter
            steer_filtered = self.steering_filter.update(steer_normalized)

            # Debug: solver -> command pipeline
            print(
                f"[MPC Actuation DBG] u0_raw_rad={delta_cmd_rad_raw:+.6f} "
                f"u0_used_rad={delta_cmd_rad:+.6f} max_steer_angle_rad={self.config.max_steer_angle:.6f} "
                f"norm={steer_normalized:+.6f} lpf={float(steer_filtered):+.6f}"
            )
            
            # Reset invalid count on success
            self.invalid_count = 0
            self.last_valid_steering = steer_filtered
            
            return float(steer_filtered)
            
        except Exception as e:
            print(f"[MPC] Error solving QP: {e}")
            return self._fallback_steering()
    
    def _compute_errors(self,
                       position: Tuple[float, float],
                       heading: float,
                       waypoints: List[Tuple[float, float]]) -> Tuple[float, float, int]:
        """Compute lateral error (e_y) and heading error (e_psi) from waypoints.

        Dynamically selects the best waypoint segment for control based on proximity to vehicle.

        Args:
            position: Current vehicle position (x, y)
            heading: Current vehicle heading (radians)
            waypoints: List of waypoint (x, y) tuples

        Returns:
            Tuple of (e_y, e_psi, segment_idx):
            - e_y: Lateral error (meters), positive = left of path
            - e_psi: Heading error (radians), positive = heading left of path direction
            - segment_idx: Index of the waypoint segment being used for control
        """
        if not waypoints or len(waypoints) < 2:
            return (0.0, 0.0, 0)

        px, py = position

        # Find the best waypoint segment dynamically
        # Choose segment with closest perpendicular distance to vehicle
        best_segment_idx = 0
        best_distance = float('inf')

        for i in range(len(waypoints) - 1):
            x0, y0 = waypoints[i]
            x1, y1 = waypoints[i + 1]

            seg_dx = x1 - x0
            seg_dy = y1 - y0
            seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)

            if seg_len < 1e-6:
                continue

            # Project vehicle position onto segment
            wx = px - x0
            wy = py - y0
            u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
            u_proj = np.clip(u_proj, 0.0, 1.0)

            proj_x = x0 + u_proj * seg_dx
            proj_y = y0 + u_proj * seg_dy

            # Distance from vehicle to projection point
            dx = px - proj_x
            dy = py - proj_y
            distance = np.sqrt(dx*dx + dy*dy)

            if distance < best_distance:
                best_distance = distance
                best_segment_idx = i

        # Use the best segment found
        waypoint_idx = best_segment_idx
        x0, y0 = waypoints[waypoint_idx]
        x1, y1 = waypoints[waypoint_idx + 1]
        
        seg_dx = x1 - x0
        seg_dy = y1 - y0
        seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
        
        if seg_len < 1e-6:
            return (0.0, 0.0)
        
        # Project vehicle position onto segment
        wx = px - x0
        wy = py - y0
        u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
        u_proj = np.clip(u_proj, 0.0, 1.0)
        
        proj_x = x0 + u_proj * seg_dx
        proj_y = y0 + u_proj * seg_dy
        
        # Compute heading error (e_psi)
        # Reference heading is the segment direction
        psi_ref_original = np.arctan2(seg_dy, seg_dx)
        psi_ref = psi_ref_original
        
        # Check if reference heading is opposite to vehicle heading (>90° difference)
        # If so, flip it by 180° to align with vehicle's forward direction
        heading_diff = psi_ref - heading
        heading_diff = np.arctan2(np.sin(heading_diff), np.cos(heading_diff))  # Normalize to [-pi, pi]
        heading_flipped = False
        if abs(heading_diff) > np.pi / 2:  # > 90 degrees
            # Flip reference heading by 180°
            psi_ref = np.arctan2(np.sin(psi_ref + np.pi), np.cos(psi_ref + np.pi))
            heading_flipped = True
        
        # Compute lateral error (e_y)
        # Normal vector: (-dy, dx) points LEFT of forward direction along segment
        # Positive e_y = LEFT of path, Negative e_y = RIGHT of path
        nx = -seg_dy / seg_len
        ny = seg_dx / seg_len
        
        # NOTE: We do NOT flip the normal vector when heading is flipped
        # The normal vector is based on geometric position, not travel direction
        
        e_y_raw = (px - proj_x)*nx + (py - proj_y)*ny
        e_y = e_y_raw
        
        # CRITICAL FIX: When heading is flipped by 180°, CTE sign must also be flipped!
        # Why: The normal vector (-dy, dx) is defined relative to the original segment direction.
        # When we flip the reference heading by 180°, we're saying "travel in the opposite direction".
        # The CTE "left/right" is relative to travel direction, so it must also flip.
        # Example: If vehicle is physically right of the geometric line (e_y < 0), but we're
        # traveling opposite to the line direction, then vehicle is LEFT of the travel path (e_y > 0).
        if heading_flipped:
            e_y = -e_y
        
        # Diagnostic logging
        vehicle_heading_deg = heading * 180.0 / np.pi
        seg_heading_deg = psi_ref_original * 180.0 / np.pi
        heading_diff_deg = heading_diff * 180.0 / np.pi
        flipped_heading_deg = psi_ref * 180.0 / np.pi if heading_flipped else None
        print(
            f"[MPC Errors DBG] seg_idx={waypoint_idx} "
            f"wp0=({x0:.2f},{y0:.2f}) wp1=({x1:.2f},{y1:.2f}) "
            f"seg_d=({seg_dx:.2f},{seg_dy:.2f}) seg_len={seg_len:.3f} "
            f"u_proj={u_proj:.3f} proj=({proj_x:.2f},{proj_y:.2f}) "
            f"n=({nx:.3f},{ny:.3f})"
        )
        print(
            f"[MPC Error Computation] Vehicle heading={vehicle_heading_deg:.1f}deg, "
            f"Segment heading={seg_heading_deg:.1f}deg, diff={heading_diff_deg:.1f}deg, flip={heading_flipped}"
        )
        if heading_flipped:
            print(f"[MPC Error Computation] HEADING FLIPPED: {seg_heading_deg:.1f}deg -> {flipped_heading_deg:.1f}deg (for heading alignment only, normal vector unchanged)")
        print(
            f"[MPC Error Computation] CTE_raw={e_y_raw:.3f}m ({'LEFT' if e_y_raw > 0 else 'RIGHT'}), "
            f"CTE_used={e_y:.3f}m ({'LEFT' if e_y > 0 else 'RIGHT'})"
        )
        
        # Heading error: difference between reference and actual
        # Normalize to [-pi, pi]
        # Use e_psi = vehicle_heading - reference_heading (matches the discrete model sign)
        e_psi = heading - psi_ref
        # normalize to [-pi, pi]
        e_psi = math.atan2(math.sin(e_psi), math.cos(e_psi))
        e_psi = np.arctan2(np.sin(e_psi), np.cos(e_psi))  # Normalize to [-pi, pi]
        e_psi_deg = e_psi * 180.0 / np.pi
        # Expose last errors for outer behavior logic (conditioning/safety/debug)
        print(f"[MPC Error Computation] Final errors: e_y={e_y:.3f}m, e_psi={e_psi:.3f}rad ({e_psi_deg:.1f}deg)")
        self.last_e_y = float(e_y)
        self.last_e_psi = float(e_psi)
        self.last_seg_idx = int(waypoint_idx)
        
        return (float(e_y), float(e_psi), waypoint_idx)
    
    def _fallback_steering(self, e_y: Optional[float] = None, e_psi: Optional[float] = None) -> float:
        """Fallback steering when MPC fails.
        
        Uses proportional control based on lateral error to prevent catch-22 situations
        where large errors disable MPC, causing zero steering, which makes errors worse.
        
        Args:
            e_y: Lateral error (meters). Positive = LEFT of path, Negative = RIGHT of path.
            e_psi: Heading error (radians). Optional, used for additional correction.
            
        Returns:
            Steering command in normalized range [-1.0, 1.0]
        """
        self.invalid_count += 1
        
        # Proportional fallback: use error-based steering to correct large deviations
        # This prevents the catch-22: large error → MPC disabled → zero steering → larger error
        if e_y is not None:
            # FIX 4: Increase steering authority for large CTE errors
            error_magnitude = abs(e_y)
            
            # Adaptive steering authority based on error magnitude
            if error_magnitude > 10.0:
                # Very large error (>10m): maximum steering authority
                max_error_for_full_steer = 10.0
                proportional_gain = 0.6  # Maximum steering
            elif error_magnitude > 5.0:
                # Large error (5-10m): increased steering authority
                max_error_for_full_steer = 10.0
                proportional_gain = 0.4  # Strong steering
            elif error_magnitude > 2.0:
                # Moderate error (2-5m): increased steering authority to prevent overshooting
                max_error_for_full_steer = 5.0  # Full steering at 5m (more responsive)
                proportional_gain = 0.5  # Increased from 0.3 - stronger correction for moderate errors
            else:
                # Small error (<2m): increased steering authority to prevent overshooting when close to track
                max_error_for_full_steer = 2.0  # Full steering at 2m (more responsive for small errors)
                proportional_gain = 0.4  # Increased from 0.3 - stronger correction to prevent overshooting
            
            # Proportional gain: steer toward path based on lateral error
            # Negative e_y (RIGHT of path) → positive steering (LEFT) to correct
            # Positive e_y (LEFT of path) → negative steering (RIGHT) to correct
            
            # Compute proportional steering: steer opposite to error direction
            steer_proportional = -np.sign(e_y) * proportional_gain * min(error_magnitude / max_error_for_full_steer, 1.0)
            
            # Add heading error correction if available (smaller contribution, but only when lateral error is small)
            # When lateral error is large, lateral correction should dominate
            if e_psi is not None:
                # Reduce heading error contribution when lateral error is large
                if error_magnitude > 5.0:
                    heading_gain = 0.05  # Very small gain when lateral error is large
                elif error_magnitude > 2.0:
                    heading_gain = 0.08  # Small gain for moderate lateral error
                else:
                    heading_gain = 0.1  # Standard gain for small lateral error
                
                # Only apply heading correction if it doesn't counteract lateral correction
                steer_heading = -np.sign(e_psi) * heading_gain * min(abs(e_psi) / (np.pi / 2), 1.0)
                
                # Check if heading correction would counteract lateral correction
                if np.sign(steer_proportional) != 0 and np.sign(steer_heading) != 0:
                    if np.sign(steer_proportional) != np.sign(steer_heading):
                        # Heading correction opposes lateral correction - reduce it
                        steer_heading = steer_heading * 0.5
                
                steer_proportional += steer_heading
            
            # Combine with last valid steering (weighted average)
            if self.invalid_count <= self.config.max_invalid_count and abs(self.last_valid_steering) > 0.01:
                # Blend: more proportional as invalid count increases
                blend_factor = min(self.invalid_count / self.config.max_invalid_count, 1.0)
                steer = (1.0 - blend_factor) * self.last_valid_steering + blend_factor * steer_proportional
            else:
                # Use pure proportional after max invalid count or if no valid steering
                steer = steer_proportional
            
            # Clamp to valid range
            steer = max(-1.0, min(1.0, steer))
            
            # CRITICAL: Negate steering for dSPACE sign convention (same as MPC output above)
            steer = -steer
            
            print(f"[MPC Fallback] Using proportional steering: {steer:.3f} (e_y={e_y:.2f}m, e_psi={e_psi:.2f}rad if available)")
            return float(steer)
        
        # Fallback to old behavior if no error information available
        if self.invalid_count <= self.config.max_invalid_count:
            # Hold last valid steering
            return self.last_valid_steering
        else:
            # Zero steering after max invalid count (should rarely happen now)
            return 0.0

