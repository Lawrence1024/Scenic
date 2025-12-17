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
        self.ref_builder = ReferenceBuilder(config.traj_resample_dist)
        
        # Initialize low-pass filter for steering output
        self.steering_filter = LowPassFilter(
            cutoff_hz=config.steering_lpf_cutoff_hz,
            dt=timestep
        )
        
        # State: [e_y, e_psi, delta]
        self.state = np.zeros(3)
        self.prev_control = 0.0  # Previous steering command (for rate penalty)
        
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
            
            # State cost: w_ey * e_y^2 + w_epsi * e_psi^2
            P[x_k_idx, x_k_idx] += self.config.w_ey  # e_y
            P[x_k_idx + 1, x_k_idx + 1] += self.config.w_epsi  # e_psi
            
            # Control cost: w_u * u^2
            P[u_k_idx, u_k_idx] += self.config.w_u
            
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
                 current_waypoint_idx: Optional[int] = None) -> float:
        """Compute steering command for one control step.
        
        Args:
            vehicle_state: Dictionary with keys:
                - 'x', 'y': position (meters)
                - 'yaw': heading (radians)
                - 'speed': speed (m/s)
                - 'yaw_rate': yaw rate (rad/s, optional)
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
        
        # Safety check: disable MPC if errors too large
        # TODO: Compute actual errors from waypoints
        if abs(speed) < 0.1:
            # Very slow or stopped - return zero steering
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
                last_waypoint_idx=current_waypoint_idx
            )
        except Exception as e:
            print(f"[MPC] Error building reference: {e}")
            return self._fallback_steering()
        
        # Compute current state [e_y, e_psi, delta]
        e_y, e_psi = self._compute_errors(
            position=(x, y),
            heading=yaw,
            waypoints=waypoints,
            waypoint_idx=new_waypoint_idx
        )
        
        # Safety check: disable MPC if errors too large
        if abs(e_y) > self.config.admissible_position_error:
            print(f"[MPC] Position error too large: {e_y:.2f}m > {self.config.admissible_position_error}m")
            return self._fallback_steering()
        
        if abs(e_psi) > self.config.admissible_yaw_error_rad:
            print(f"[MPC] Yaw error too large: {e_psi:.2f}rad > {self.config.admissible_yaw_error_rad}rad")
            return self._fallback_steering()
        
        # Get current steering angle (delta)
        # TODO: Read from ControlDesk if available, otherwise use previous control
        delta = self.state[2] if hasattr(self, 'state') and len(self.state) > 2 else 0.0
        
        self.state = np.array([e_y, e_psi, delta])
        
        # Build QP matrices
        try:
            P, q, A, l, u = self._build_qp_matrices(
                self.state, psi_ref, kappa_ref, v_ref,
                self.config.mpc_prediction_horizon
            )
        except Exception as e:
            print(f"[MPC] Error building QP: {e}")
            return self._fallback_steering()
        
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
            delta_cmd_rad = result.x[u_0_idx]
            
            # Update previous control
            self.prev_control = delta_cmd_rad
            
            # Convert to normalized steering [-1, 1]
            steer_normalized = np.clip(delta_cmd_rad / self.config.max_steer_angle, -1.0, 1.0)
            
            # Apply low-pass filter
            steer_filtered = self.steering_filter.update(steer_normalized)
            
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
                       waypoints: List[Tuple[float, float]],
                       waypoint_idx: int) -> Tuple[float, float]:
        """Compute lateral error (e_y) and heading error (e_psi) from waypoints.
        
        Args:
            position: Current vehicle position (x, y)
            heading: Current vehicle heading (radians)
            waypoints: List of waypoint (x, y) tuples
            waypoint_idx: Current waypoint index
            
        Returns:
            Tuple of (e_y, e_psi):
            - e_y: Lateral error (meters), positive = left of path
            - e_psi: Heading error (radians), positive = heading left of path direction
        """
        if not waypoints or len(waypoints) < 2:
            return (0.0, 0.0)
        
        px, py = position
        
        # Find the waypoint segment to use (use nearest segment)
        if waypoint_idx >= len(waypoints) - 1:
            waypoint_idx = len(waypoints) - 2
        
        # Get segment endpoints
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
        
        # Compute lateral error (e_y)
        # Normal vector: (-dy, dx) points LEFT of forward direction
        # Positive e_y = LEFT of path, Negative e_y = RIGHT of path
        nx = -seg_dy / seg_len
        ny = seg_dx / seg_len
        e_y = (px - proj_x)*nx + (py - proj_y)*ny
        
        # Compute heading error (e_psi)
        # Reference heading is the segment direction
        psi_ref = np.arctan2(seg_dy, seg_dx)
        
        # Heading error: difference between reference and actual
        # Normalize to [-pi, pi]
        e_psi = psi_ref - heading
        e_psi = np.arctan2(np.sin(e_psi), np.cos(e_psi))  # Normalize to [-pi, pi]
        
        return (float(e_y), float(e_psi))
    
    def _fallback_steering(self) -> float:
        """Fallback steering when MPC fails.
        
        Returns:
            Steering command (holds last valid or zeros)
        """
        self.invalid_count += 1
        
        if self.invalid_count <= self.config.max_invalid_count:
            # Hold last valid steering
            return self.last_valid_steering
        else:
            # Zero steering after max invalid count
            return 0.0

