"""MPC lateral controller for racing vehicles (MPCC-style, Phase 3).

Implements Model Predictive Contouring Control (MPCC): state [e_y, e_psi, delta, s]
with contouring cost (e_y, e_psi), lag cost Q_lag*(s_ref - s)^2, and progress
reward -Q_progress*(s_N - s_0). Progress dynamics: s_{k+1} = s_k + v_ref_k*dt
(linearized; full MPCC would use s_dot = v*cos(e_psi)). Set Q_lag=0, Q_progress=0
for pure trajectory-tracking mode.
"""

import math
import time
import numpy as np
from typing import Dict, List, Tuple, Optional, Union
import osqp
from scipy.sparse import csc_matrix

from .config import MPCConfig
from .reference_builder import ReferenceBuilder
from .utils import LowPassFilter
from . import timing as _mpc_timing


class MPCLateralController:
    """MPCC-style lateral controller: contouring + lag + progress cost.
    
    State: [e_y, e_psi, delta, s]. Cost: contouring (e_y, e_psi), lag (s_ref - s)^2,
    progress reward at terminal. Progress dynamics linearized as s_{k+1} = s_k + v_ref*dt.
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
            curvature_smoothing_num=config.curvature_smoothing_num,
            use_splines=config.use_splines
        )
        
        # Initialize low-pass filter for steering output
        self.steering_filter = LowPassFilter(
            cutoff_hz=config.steering_lpf_cutoff_hz,
            dt=timestep
        )
        
        # State: [e_y, e_psi, delta, s] (Phase 2: s = progress for lag/progress cost)
        self.state = np.zeros(4)
        self.prev_control = 0.0  # Previous steering command (for rate penalty)
        self.prev_prev_control = 0.0  # Previous-previous steering command (for acceleration penalty)
        
        # Safety state
        self.invalid_count = 0
        self.last_valid_steering = 0.0
        
        # Segment selection smoothing: hysteresis only (no advance cap, to avoid lag at high speed)
        # Current-index contract: Behavior owns progress (wp_last_idx, arc-length advancement). It passes
        # that as current_waypoint_idx so we search locally. We choose the segment and store it here;
        # behavior does not overwrite its index from us — single source of progress is behavior.
        self.last_seg_idx = None  # set after first _compute_errors
        self._segment_hysteresis_m = getattr(
            config, 'segment_hysteresis_m', 0.4
        )  # only switch segment when new score is better by this margin (m)
        # When |CTE| > this (m), stick to current segment to avoid reference flip (generic, any TTL)
        self._segment_stick_cte_m = getattr(config, 'segment_stick_cte_m', 1.5)
        # Reference blend at boundaries: blend heading toward next segment when u_proj >= this
        self._segment_blend_u_start = getattr(config, 'segment_blend_u_start', 0.7)
        # Reference continuity gate: don't swap to a segment when association is weak (curve-approach stability)
        self._max_wp_match_dist_m = getattr(config, 'max_wp_match_dist_m', 3.0)
        self._max_s_jump_m = getattr(config, 'max_s_jump_m', 4.0)
        # Recommendation B: when gate rejects and match_dist > this, force re-association (m)
        self._gate_hard_fail_dist_m = getattr(config, 'gate_hard_fail_dist_m', 6.0)
        # Reacquire on weak association: trigger full scan when best_match_dist > this (m)
        self._reacquire_dist_m = getattr(config, 'reacquire_dist_m', 3.0)
        # Recommendation C: only stick to segment when match_dist < this (m)
        self._stick_association_ok_m = getattr(config, 'stick_association_ok_m', 2.0)
        
        # OSQP solver (will be initialized on first solve)
        self.solver = None
        self._solver_initialized = False
        # To-Do 4.1: saturation counter (increment when |steer_mpc_raw| > 0.98)
        self._saturation_count = 0
    
    def _initialize_solver(self, horizon: int):
        """Initialize OSQP solver for QP problem.
        
        Args:
            horizon: Prediction horizon steps
        """
        # Problem size (Phase 2: state includes progress s for lag/progress cost)
        n_x = 4  # state dimension [e_y, e_psi, delta, s]
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
        
        # Higher max_iter and relaxed tolerances for Phase 2 QP (larger problem) to avoid "maximum iterations reached" / "solved inaccurate"
        self.solver.setup(P, q, A, l, u, verbose=False, warm_start=True,
                         max_iter=20000, eps_abs=5e-4, eps_rel=5e-4)
        self._solver_initialized = True
    
    def _build_qp_matrices(self,
                          state: np.ndarray,
                          psi_ref: np.ndarray,
                          kappa_ref: np.ndarray,
                          v_ref: np.ndarray,
                          horizon: int,
                          cte_magnitude: Optional[float] = None,
                          s_0: Optional[float] = None,
                          s_horizon: Optional[np.ndarray] = None) -> Tuple[csc_matrix, np.ndarray, csc_matrix, np.ndarray, np.ndarray]:
        """Build QP matrices for MPC problem.
        
        Args:
            state: Current state [e_y, e_psi, delta] or [e_y, e_psi, delta, s_0]
            psi_ref: Reference heading array (radians)
            kappa_ref: Reference curvature array (1/meters)
            v_ref: Reference speed array (m/s)
            horizon: Prediction horizon steps
            cte_magnitude: Optional CTE magnitude for adaptive weights
            s_0: Current progress along path (arc length, m). If None, 0 is used and s_horizon built from v_ref.
            s_horizon: Reference progress at each horizon step (length horizon). If None, built from v_ref.
            
        Returns:
            Tuple of (P, q, A, l, u) for QP: minimize 0.5*x'*P*x + q'*x
            subject to l <= A*x <= u
        """
        # Ensure horizon is a positive integer
        horizon = int(horizon)
        if horizon <= 0:
            raise ValueError(f"horizon must be > 0, got {horizon}")
        
        # Ensure all reference arrays are 1D numpy arrays with correct shape
        if not isinstance(psi_ref, np.ndarray):
            raise TypeError(f"psi_ref must be a numpy array, got {type(psi_ref)}")
        if not isinstance(kappa_ref, np.ndarray):
            raise TypeError(f"kappa_ref must be a numpy array, got {type(kappa_ref)}")
        if not isinstance(v_ref, np.ndarray):
            raise TypeError(f"v_ref must be a numpy array, got {type(v_ref)}")
        if psi_ref.ndim != 1 or kappa_ref.ndim != 1 or v_ref.ndim != 1:
            raise ValueError("psi_ref, kappa_ref, v_ref must be 1D")
        if len(psi_ref) != horizon or len(kappa_ref) != horizon or len(v_ref) != horizon:
            raise ValueError("reference arrays length must equal horizon")
        
        # Phase 2: state includes progress s [e_y, e_psi, delta, s]
        n_x = 4
        n_u = 1
        n_vars = horizon * n_u + (horizon + 1) * n_x
        
        # State: pad to length 4 if needed
        state = np.asarray(state, dtype=np.float64)
        if state.size == 3:
            s_0_val = float(s_0) if s_0 is not None else 0.0
            state = np.append(state, s_0_val)
        if state.size != 4:
            raise ValueError(f"state must have 3 or 4 elements, got {state.size}")
        
        # Reference progress for lag cost: s_ref at step k = s_horizon[k-1] for k>=1, s_ref_0 = s_0
        dt = self.config.mpc_prediction_dt
        if s_horizon is None or len(s_horizon) != horizon:
            s_0_val = float(state[3])
            s_horizon = np.array([s_0_val + np.sum(v_ref[:i + 1]) * dt for i in range(horizon)], dtype=np.float64)
        else:
            s_horizon = np.asarray(s_horizon, dtype=np.float64)
        s_0_val = float(state[3])
        
        # Cost matrix P (quadratic)
        P = np.zeros((n_vars, n_vars))
        q = np.zeros(n_vars)
        
        # Constraint matrix: (horizon+1)*n_x initial + horizon*(n_x + n_u + 2) per step (n_x dynamics + control + rate)
        n_constraints = (horizon + 1) * n_x + horizon * (n_x + n_u + 2)
        A = np.zeros((n_constraints, n_vars))
        l = np.zeros(n_constraints)
        u = np.zeros(n_constraints)
        
        # Variable ordering: [x_0, u_0, x_1, u_1, ..., x_N]; x_k = [e_y, e_psi, delta, s]
        dt = self.config.mpc_prediction_dt
        L = self.config.wheel_base
        tau = self.config.steer_tau
        delta_max_base = self.config.max_steer_angle
        high_curv_threshold = getattr(self.config, 'high_curvature_threshold', 0.1)
        max_steer_high_curv = getattr(self.config, 'max_steer_angle_high_curv', None)
        Q_lag = getattr(self.config, 'Q_lag', 0.0)
        Q_progress = getattr(self.config, 'Q_progress', 0.0)
        
        constraint_idx = 0
        
        # Initial state constraint: x_0 = state (4 components)
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
            # Todo 4: curvature feedforward — u_k is delta_fb; steering = delta_ff_k + u_k
            delta_ff_k = math.atan(L * kappa_k)
            
            # Select weights based on curvature; smooth blend between low and high (oscillation fix)
            if self.config.use_adaptive_weights:
                abs_kappa = abs(kappa_k)
                low_t = self.config.low_curvature_threshold
                high_t = self.config.high_curvature_threshold
                t = (abs_kappa - low_t) / (high_t - low_t) if high_t > low_t else 0.0
                t = max(0.0, min(1.0, t))
                w_ey_k = (1.0 - t) * self.config.w_ey_low_curv + t * self.config.w_ey_high_curv
                w_epsi_k = (1.0 - t) * self.config.w_epsi_low_curv + t * self.config.w_epsi_high_curv
                w_epsi_vel_k = (1.0 - t) * self.config.w_epsi_vel_low_curv + t * self.config.w_epsi_vel_high_curv
                w_u_k = (1.0 - t) * self.config.w_u_low_curv + t * self.config.w_u_high_curv
                w_u_vel_k = (1.0 - t) * self.config.w_u_vel_low_curv + t * self.config.w_u_vel_high_curv
                w_ddu_k = (1.0 - t) * self.config.w_ddu_low_curv + t * self.config.w_ddu_high_curv
            else:
                # Adaptive weights disabled - use base weights for all
                w_ey_k = self.config.w_ey
                w_epsi_k = self.config.w_epsi
                w_epsi_vel_k = self.config.w_epsi_vel
                w_u_k = self.config.w_u
                w_u_vel_k = self.config.w_u_vel
                w_ddu_k = self.config.w_ddu
            
            # CTE-adaptive weight scaling: increase tracking when off-track; cap to avoid overcorrection (oscillation fix)
            if cte_magnitude is not None and cte_magnitude > 0.0:
                if cte_magnitude >= 3.0:
                    cte_multiplier = 3.0
                elif cte_magnitude >= 1.5:
                    cte_multiplier = 2.0
                elif cte_magnitude >= 0.5:
                    cte_multiplier = 1.5
                else:
                    cte_multiplier = 1.0
                cap = getattr(self.config, 'cte_multiplier_max', 2.0)
                cte_multiplier = min(cte_multiplier, cap)
                w_ey_k *= cte_multiplier
                w_epsi_k *= cte_multiplier
                w_epsi_vel_k *= cte_multiplier
            
            # State cost: w_ey * e_y^2 + w_epsi * e_psi^2 + w_epsi_vel * e_psi^2 * v^2
            P[x_k_idx, x_k_idx] += w_ey_k  # e_y
            P[x_k_idx + 1, x_k_idx + 1] += w_epsi_k + w_epsi_vel_k * v_k * v_k  # e_psi (with velocity weighting)
            
            # Phase 2: lag error cost Q_lag * (s_ref_k - s_k)^2
            if Q_lag != 0:
                s_ref_k = s_horizon[k - 1] if k >= 1 else s_0_val
                P[x_k_idx + 3, x_k_idx + 3] += Q_lag
                q[x_k_idx + 3] -= 2.0 * Q_lag * s_ref_k
            
            # Control cost: w_u * u^2 + w_u_vel * u^2 * v^2
            P[u_k_idx, u_k_idx] += w_u_k + w_u_vel_k * v_k * v_k
            # w_ff_track * (delta - delta_ff)^2 = w_ff_track * u_k^2 (u = delta_fb) — shrink feedback relative to feedforward
            w_ff_track = getattr(self.config, 'w_ff_track', 0.2)
            P[u_k_idx, u_k_idx] += 2.0 * w_ff_track
            
            # Control rate cost: w_du * (u_k - u_{k-1})^2 (u = delta_fb; compare to previous delta_fb)
            if k == 0:
                # First step: compare to previous delta_fb (prev_control was total = delta_ff + delta_fb)
                prev_delta_fb = self.prev_control - getattr(self, '_last_delta_ff_rad', 0.0)
                P[u_k_idx, u_k_idx] += self.config.w_du
                q[u_k_idx] -= 2.0 * self.config.w_du * prev_delta_fb
            else:
                # Compare to previous step
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                P[u_k_idx, u_k_idx] += self.config.w_du
                P[u_km1_idx, u_km1_idx] += self.config.w_du
                P[u_k_idx, u_km1_idx] -= self.config.w_du
                P[u_km1_idx, u_k_idx] -= self.config.w_du
            
            # Steering acceleration cost: w_ddu * (u_k - 2*u_{k-1} + u_{k-2})^2 (u = delta_fb)
            if k == 0:
                prev_delta_fb = self.prev_control - getattr(self, '_last_delta_ff_rad', 0.0)
                prev_prev_delta_fb = self.prev_prev_control - getattr(self, '_last_delta_ff_prev_rad', 0.0)
                P[u_k_idx, u_k_idx] += w_ddu_k
                q[u_k_idx] -= 2.0 * w_ddu_k * (2.0 * prev_delta_fb - prev_prev_delta_fb)
            elif k == 1:
                u_km1_idx = (k - 1) * (n_x + n_u) + n_x
                prev_prev_delta_fb = self.prev_prev_control - getattr(self, '_last_delta_ff_prev_rad', 0.0)
                P[u_k_idx, u_k_idx] += w_ddu_k
                P[u_km1_idx, u_km1_idx] += 4.0 * w_ddu_k
                P[u_k_idx, u_km1_idx] -= 4.0 * w_ddu_k
                P[u_km1_idx, u_k_idx] -= 4.0 * w_ddu_k
                q[u_k_idx] += 2.0 * w_ddu_k * prev_prev_delta_fb
                q[u_km1_idx] -= 4.0 * w_ddu_k * prev_prev_delta_fb
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
            
            # Task 3: Yaw-rate damping — penalize (e_psi_{k+1} - e_psi_k)^2 to prevent snap oscillations
            w_epsi_rate = getattr(self.config, 'w_epsi_rate', 0.1)
            if w_epsi_rate > 0:
                P[x_k_idx + 1, x_k_idx + 1] += w_epsi_rate
                P[x_kp1_idx + 1, x_kp1_idx + 1] += w_epsi_rate
                P[x_k_idx + 1, x_kp1_idx + 1] -= 2.0 * w_epsi_rate
                P[x_kp1_idx + 1, x_k_idx + 1] -= 2.0 * w_epsi_rate
            
            # Dynamics: x_{k+1} = A_k * x_k + B_k * u_k + g_k
            # e_y_{k+1} = e_y_k + v_k * e_psi_k * dt
            A[constraint_idx, x_k_idx] = 1.0  # e_y_k
            A[constraint_idx, x_k_idx + 1] = v_k * dt  # e_psi_k
            A[constraint_idx, x_kp1_idx] = -1.0  # -e_y_{k+1}
            l[constraint_idx] = 0.0
            u[constraint_idx] = 0.0
            constraint_idx += 1
            
            # Todo 4: e_psi with steering = delta_ff_k + u_k (u_k = delta_fb)
            # e_psi_{k+1} = e_psi_k + (v_k/L)*(delta_ff_k + u_k)*dt - v_k*kappa_k*dt
            A[constraint_idx, x_k_idx + 1] = 1.0  # e_psi_k
            A[constraint_idx, u_k_idx] = (v_k / L) * dt  # u_k (delta_fb)
            A[constraint_idx, x_kp1_idx + 1] = -1.0  # -e_psi_{k+1}
            rhs = (v_k / L) * delta_ff_k * dt - v_k * kappa_k * dt
            l[constraint_idx] = rhs
            u[constraint_idx] = rhs
            constraint_idx += 1
            
            # delta_{k+1} = delta_k + (dt/tau)*((delta_ff_k + u_k) - delta_k)
            A[constraint_idx, x_k_idx + 2] = 1.0 - dt/tau  # delta_k
            A[constraint_idx, u_k_idx] = dt/tau  # u_k
            A[constraint_idx, x_kp1_idx + 2] = -1.0  # -delta_{k+1}
            l[constraint_idx] = (dt / tau) * delta_ff_k
            u[constraint_idx] = (dt / tau) * delta_ff_k
            constraint_idx += 1
            
            # Phase 3 MPCC: progress dynamics s_{k+1} = s_k + v_ref_k*dt (linearized; full: s_dot = v*cos(e_psi))
            A[constraint_idx, x_k_idx + 3] = -1.0  # -s_k
            A[constraint_idx, x_kp1_idx + 3] = 1.0  # s_{k+1}
            l[constraint_idx] = v_k * dt
            u[constraint_idx] = v_k * dt
            constraint_idx += 1
            
            # Control limits: |delta_ff_k + u_k| <= delta_max_k (To-Do 4.2: higher cap in high curvature if max_steer_angle_high_curv set)
            delta_max_k = delta_max_base
            if max_steer_high_curv is not None and abs(kappa_k) >= high_curv_threshold:
                delta_max_k = max_steer_high_curv
            A[constraint_idx, u_k_idx] = 1.0
            l[constraint_idx] = -delta_max_k - delta_ff_k
            u[constraint_idx] = delta_max_k - delta_ff_k
            constraint_idx += 1
            
            # Steering rate: |(delta_ff_k + u_k) - delta_k| <= steer_rate_lim * tau
            rate_bound = self.config.steer_rate_lim * tau
            A[constraint_idx, u_k_idx] = 1.0
            A[constraint_idx, x_k_idx + 2] = -1.0
            l[constraint_idx] = -rate_bound - delta_ff_k
            u[constraint_idx] = rate_bound - delta_ff_k
            constraint_idx += 1
        
        # Terminal cost
        x_N_idx = horizon * (n_x + n_u)
        P[x_N_idx, x_N_idx] += self.config.wT_ey
        P[x_N_idx + 1, x_N_idx + 1] += self.config.wT_epsi
        # Phase 2: progress reward -Q_progress * (s_N - s_0) => linear term in q for s_N
        if Q_progress != 0:
            q[x_N_idx + 3] -= Q_progress  # minimize -Q_progress*s_N => reward progress
        
        # Convert to sparse
        P_sparse = csc_matrix(P)
        A_sparse = csc_matrix(A)
        
        return (P_sparse, q, A_sparse, l, u)
    
    def run_step(self,
                 vehicle_state: Dict[str, float],
                 waypoints: List[Tuple[float, float]],
                 current_waypoint_idx: Optional[int] = None,
                 cte_magnitude: Optional[float] = None,
                 v_ref_profile: Optional[Union[List[float], np.ndarray]] = None,
                 curvature_ahead_max: Optional[float] = None) -> float:
        """Compute steering command for one control step.
        
        When v_ref_profile is provided (same profile used by longitudinal MPC), the
        lateral reference (s_horizon) matches the planned speed so the controller
        sees the whole trajectory and avoids over-steer then correct.
        
        Args:
            vehicle_state: Dictionary with keys:
                - 'x', 'y': position (meters)
                - 'yaw': heading (radians)
                - 'speed': speed (m/s)
                - 'yaw_rate': yaw rate (rad/s, optional)
                - 'gear': current gear (optional, for checking if in neutral)
            waypoints: List of waypoint (x, y) tuples
            current_waypoint_idx: Hint from behavior (its wp_last_idx) for local segment search; we own
                chosen segment (last_seg_idx), behavior does not sync back from us.
            cte_magnitude: Optional CTE magnitude for adaptive search
            v_ref_profile: Optional speed profile over horizon (m/s). Pass the same
                profile used by longitudinal MPC for trajectory-consistent control.
            curvature_ahead_max: Optional max curvature (1/m) over lookahead; used for deadzone eligibility
                (require curv_ahead_max < curv_deadzone_max so we never deadzone in moderate curvature).
            
        Returns:
            Steering command in normalized range [-1.0, 1.0]
        """
        t0 = time.perf_counter()
        # Extract state
        x = vehicle_state.get('x', 0.0)
        y = vehicle_state.get('y', 0.0)
        yaw = vehicle_state.get('yaw', 0.0)
        speed = vehicle_state.get('speed', 0.0)
        yaw_rate = vehicle_state.get('yaw_rate', None)  # rad/s, for Recommendation D (STEER_SIGN_SANITY)
        gear = vehicle_state.get('gear', None)  # Get gear if available
        
        # Safety check: disable MPC if in neutral (gear 0) or if speed is very low AND gear unknown
        # Allow MPC to work from stopped state if gear is set (gear >= 1)
        if gear is not None:
            if gear < 1:
                # In neutral - return zero steering
                _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
                return 0.0
        elif abs(speed) < 0.1:
            # Very slow or stopped AND gear unknown - return zero steering
            _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
            return 0.0
        
        # Compute lateral errors and segment first so reference uses same segment (single-segment
        # consistency: avoids ff/fb mismatch and steering wobble at curve exits)
        prev_e_y = float(self.state[0]) if hasattr(self, 'state') and self.state is not None and len(self.state) > 0 else None
        e_y, e_psi, mpc_segment_idx = self._compute_errors(
            position=(x, y),
            heading=yaw,
            waypoints=waypoints,
            current_waypoint_idx=current_waypoint_idx,
            prev_e_y=prev_e_y
        )
        self._log_mpc_e_y = float(e_y)
        
        # Build reference trajectory (Phase 1: returns s_0 and s_horizon for MPCC)
        # Pass reference_segment_idx so kappa_ref/s_0 and e_y/e_psi refer to the same path segment
        try:
            (psi_ref, kappa_ref, v_ref, grade_ref, s_0, s_horizon) = self.ref_builder.build_reference(
                waypoints=waypoints,
                current_position=(x, y),
                current_heading=yaw,
                horizon_steps=self.config.mpc_prediction_horizon,
                dt=self.config.mpc_prediction_dt,
                speed=speed,
                last_waypoint_idx=current_waypoint_idx,
                cte_magnitude=cte_magnitude,
                v_ref_profile=v_ref_profile,
                reference_segment_idx=mpc_segment_idx
            )
            # Phase 1: store progress s_0 and s along horizon for logging / Phase 2 (no cost change yet)
            self._last_s_0 = s_0
            self._last_s_horizon = s_horizon
            # Task 1: signed curvature sanity — kappa at +10 m along arc for log
            s_ahead_m = 10.0
            kappa_ref_ahead_signed = None
            if s_horizon is not None and kappa_ref is not None and len(s_horizon) == len(kappa_ref):
                for k in range(len(s_horizon)):
                    if s_horizon[k] >= s_0 + s_ahead_m:
                        kappa_ref_ahead_signed = float(kappa_ref[k])
                        break
                if kappa_ref_ahead_signed is None and len(kappa_ref) > 0:
                    kappa_ref_ahead_signed = float(kappa_ref[-1])
            self._log_kappa_ref_ahead_signed = kappa_ref_ahead_signed
            # grade_ref is computed but not used by lateral MPC (used by longitudinal MPC)
            # Debug: verify arrays right after build_reference returns
            if len(psi_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned psi_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(psi_ref)}. Shape: {psi_ref.shape}, dtype: {psi_ref.dtype}")
            if len(kappa_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned kappa_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(kappa_ref)}. Shape: {kappa_ref.shape}, dtype: {kappa_ref.dtype}")
            if len(v_ref) != self.config.mpc_prediction_horizon:
                raise ValueError(f"[MPC] CRITICAL: build_reference returned v_ref with wrong length: expected {self.config.mpc_prediction_horizon}, got {len(v_ref)}. Shape: {v_ref.shape}, dtype: {v_ref.dtype}")
            # Cap reference curvature to kinematic limit: |kappa| <= kappa_max = tan(delta_max)/L
            # So the reference is feasible for the kinematic bicycle (no unreachable curvature)
            kappa_max = math.tan(self.config.max_steer_angle) / self.config.wheel_base
            kappa_ref = np.clip(kappa_ref, -kappa_max, kappa_max)
        except Exception as e:
            print(f"[MPC] Error building reference: {e}")
            # Try to compute errors for proportional fallback, but don't fail if it doesn't work
            try:
                e_y, e_psi, _ = self._compute_errors(
                    position=(x, y),
                    heading=yaw,
                    waypoints=waypoints
                )
                _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
                return self._fallback_steering(e_y=e_y, e_psi=e_psi)
            except:
                _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
                return self._fallback_steering(e_y=None, e_psi=None)
        
        # e_y, e_psi, mpc_segment_idx already computed above (single-segment consistency)
        # Compute reference heading for logging (use segment index from reference = mpc_segment_idx)
        psi_ref_logging = None
        n_wp = len(waypoints) if waypoints else 0
        if waypoints and n_wp >= 2 and 0 <= mpc_segment_idx < n_wp:
            wp0 = waypoints[mpc_segment_idx]
            wp1 = waypoints[(mpc_segment_idx + 1) % n_wp]
            seg_dx = wp1[0] - wp0[0]
            seg_dy = wp1[1] - wp0[1]
            seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
            if seg_len > 1e-6:
                psi_ref_logging = np.arctan2(seg_dy, seg_dx)
        
        # To-Do 2: Conditional CTE deadzone — only apply when |CTE| small AND association good AND curv_ahead_max < curv_deadzone_max
        dz = getattr(self.config, 'cte_deadzone', 0.2)
        dist_ok = getattr(self.config, 'deadzone_dist_ok_m', 1.0)
        curv_dz_max = getattr(self.config, 'curv_deadzone_max', 0.04)  # never deadzone in moderate curvature (e.g. 0.03-0.05)
        low_t = getattr(self.config, 'low_curvature_threshold', 0.02)
        high_t = getattr(self.config, 'high_curvature_threshold', 0.1)
        cte_raw = float(e_y)
        match_dist_m = getattr(self, '_log_match_dist_m', None)
        kappa_at_proj = float(kappa_ref[0]) if kappa_ref is not None and len(kappa_ref) > 0 else 0.0
        abs_kappa = abs(kappa_at_proj)
        # Explicitly require curv_ahead_max < curv_deadzone_max (not just regime == LOW) so we never deadzone in moderate curvature
        curv_ok = (curvature_ahead_max is not None and curvature_ahead_max < curv_dz_max)
        ct_small = (dz > 0 and abs(cte_raw) < dz)
        match_good = (match_dist_m is not None and match_dist_m < dist_ok)
        deadzone_applied = bool(ct_small and match_good and curv_ok)
        if deadzone_applied:
            e_y = 0.0
            deadzone_reason = "CTE_SMALL,MATCH_GOOD,CURV_LOW"
        else:
            if not ct_small:
                deadzone_reason = "BLOCKED_CTE_BIG"
            elif not match_good:
                deadzone_reason = "BLOCKED_MATCH_BAD"
            else:
                deadzone_reason = "BLOCKED_CURV_HIGH"
        cte_used_for_control = float(e_y)
        curv_regime = "LOW" if abs_kappa < low_t else ("HIGH" if abs_kappa >= high_t else "MID")
        self._log_deadzone_applied = deadzone_applied
        self._log_dz_cte_m = dz
        self._log_cte_used_for_control = cte_used_for_control
        self._log_cte_raw = cte_raw
        self._log_deadzone_reason = deadzone_reason
        self._log_kappa_ref_at_proj = kappa_at_proj
        self._log_curv_regime = curv_regime
        self._log_gate_accept = (getattr(self, '_log_gate_status', '') == 'ACCEPT')
        self._log_segment_id = getattr(self, 'last_seg_idx', None)
        self._log_s_ref_for_dz = getattr(self, '_log_s_ref', None)
        # Tripwire: deadzone must never apply when |cte_raw|>0.5 or match_dist_m>1.5
        if deadzone_applied and (abs(cte_raw) > 0.5 or (match_dist_m is not None and match_dist_m > 1.5)):
            print(f"[MPC] *** DEADZONE TRIPWIRE: deadzone_applied=True but |cte_raw|={abs(cte_raw):.3f}m or match_dist_m={match_dist_m} — Todo2 miswired / unsafe")
        
        # Safety check: disable MPC if errors too large
        # Use proportional fallback to prevent catch-22 (large error → no steering → larger error)
        if abs(e_y) > self.config.admissible_position_error:
            print(f"[MPC] Position error too large: {e_y:.2f}m > {self.config.admissible_position_error}m")
            _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
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
            _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
            return self._fallback_steering(e_y=e_y, e_psi=e_psi)
        
        # Get current steering angle (delta)
        # Priority: 1) From vehicle_state (read from ControlDesk), 2) Previous state, 3) Zero
        delta = vehicle_state.get('steer_actual', None)
        if delta is None:
            # Fallback: use previous state estimate
            delta = self.state[2] if hasattr(self, 'state') and len(self.state) > 2 else 0.0
        
        # State [e_y, e_psi, delta, s_0] for Phase 2 (progress s used in lag/progress cost)
        self.state = np.array([e_y, e_psi, delta, s_0], dtype=np.float64)
        
        # Build QP matrices (Phase 2: pass s_0 and s_horizon for lag/progress cost)
        cte_mag_for_mpc = abs(e_y) if e_y is not None else (cte_magnitude if cte_magnitude is not None else 0.0)
        P, q, A, l, u = self._build_qp_matrices(
            self.state, psi_ref, kappa_ref, v_ref,
            self.config.mpc_prediction_horizon,
            cte_magnitude=cte_mag_for_mpc,
            s_0=s_0,
            s_horizon=s_horizon
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
            self.solver.setup(P, q, A, l, u, verbose=False, warm_start=True,
                             max_iter=20000, eps_abs=5e-4, eps_rel=5e-4)
            result = self.solver.solve()
            
            # Accept "solved" and "solved inaccurate" (solution meets relaxed tolerances, usable for control)
            if result.info.status not in ('solved', 'solved inaccurate'):
                print(f"[MPC] Solver failed: {result.info.status}")
                _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
                return self._fallback_steering()
            if result.info.status == 'solved inaccurate':
                # Log occasionally (first time, then every 500 occurrences)
                step = getattr(self, '_inaccurate_log_count', 0)
                if step % 500 == 0:
                    print(f"[MPC] Solver returned solved inaccurate (using solution); residual norm may be elevated")
                self._inaccurate_log_count = step + 1
            
            # Extract first control input (MPC outputs feedback part; we add curvature feedforward)
            n_x = 4  # State dimension [e_y, e_psi, delta, s] (Phase 2)
            u_0_idx = n_x  # First control is after initial state
            delta_fb_rad = float(result.x[u_0_idx])
            # Task 2: Feedforward uses preview curvature (signed) for chicanes — switch to higher preview blend when chicane detected
            L = self.config.wheel_base
            kappa_at_proj_ff = float(kappa_ref[0]) if kappa_ref is not None and len(kappa_ref) > 0 else 0.0
            kappa_ahead = getattr(self, '_log_kappa_ref_ahead_signed', None)
            ff_preview_blend = getattr(self.config, 'ff_preview_blend', 0.4)
            ff_chicane_blend = getattr(self.config, 'ff_chicane_preview_blend', 0.85)  # Task 2: use preview in chicanes
            chicane_thresh = getattr(self.config, 'ff_chicane_curvature_threshold', 0.02)  # 1/m
            is_chicane = False
            if kappa_ahead is not None:
                ka = float(kappa_ahead)
                if abs(kappa_at_proj_ff) > chicane_thresh and abs(ka) > chicane_thresh and (kappa_at_proj_ff * ka < 0):
                    is_chicane = True  # sign flip = chicane (S-bend)
                blend = ff_chicane_blend if is_chicane else ff_preview_blend
                kappa_ff = (1.0 - blend) * kappa_at_proj_ff + blend * ka
            else:
                kappa_ff = kappa_at_proj_ff
            delta_ff_rad = math.atan(L * kappa_ff)
            delta_cmd_rad_raw = delta_ff_rad + delta_fb_rad
            # To-Do 4.2: use higher steer cap in high curvature if max_steer_angle_high_curv set
            kappa_at_proj_0 = float(kappa_ref[0]) if kappa_ref is not None and len(kappa_ref) > 0 else 0.0
            # Same-sign clamp in curvature: never steer opposite to path (avoids spin from ff/fb opposition)
            _curv_sign_clamp_min = getattr(self.config, 'curvature_same_sign_clamp_min', 0.02)  # 1/m
            if abs(kappa_at_proj_0) >= _curv_sign_clamp_min:
                _same_sign = (delta_cmd_rad_raw > 0) == (delta_ff_rad > 0)
                if not _same_sign:
                    min_ff_frac = getattr(self.config, "curvature_same_sign_min_ff_frac", 0.25)  # 25% of FF
                    min_cmd = min_ff_frac * delta_ff_rad
                    delta_cmd_rad_raw = min_cmd

            high_t = getattr(self.config, 'high_curvature_threshold', 0.1)
            current_delta_max = self.config.max_steer_angle
            if getattr(self.config, 'max_steer_angle_high_curv', None) is not None and abs(kappa_at_proj_0) >= high_t:
                current_delta_max = self.config.max_steer_angle_high_curv
            delta_clamped_rad = np.clip(delta_cmd_rad_raw, -current_delta_max, current_delta_max)
            # Step 2 (plan): rate limit output in rad/s inside controller
            rate_max_radps = getattr(self.config, 'steer_rate_limit_output_radps', 1.0)
            dt_step = self.timestep
            delta_prev_rad = getattr(self, '_delta_prev_output_rad', 0.0)
            delta_cmd_rad = float(np.clip(
                delta_clamped_rad,
                delta_prev_rad - rate_max_radps * dt_step,
                delta_prev_rad + rate_max_radps * dt_step
            ))
            self._delta_prev_output_rad = delta_cmd_rad
            
            # Update previous controls (for next iteration's acceleration penalty)
            self.prev_prev_control = self.prev_control
            self.prev_control = delta_cmd_rad
            
            # Todo 4: store feedforward/feedback for next step's w_du/w_ddu and for logging
            self._last_delta_ff_prev_rad = getattr(self, '_last_delta_ff_rad', 0.0)
            self._last_delta_ff_rad = delta_ff_rad
            self._last_delta_fb_rad = delta_fb_rad
            
            # Feedforward validation logs (delta_ff, delta_fb, delta_total, kappa_ref, v)
            kappa_at_proj = kappa_at_proj_0
            self._log_delta_ff = float(delta_ff_rad)
            self._log_delta_fb = float(delta_fb_rad)
            self._log_delta_total = float(delta_cmd_rad_raw)
            self._log_delta_cmd_rad = float(delta_cmd_rad)  # post-MPC clamp (command side)
            self._log_current_delta_max = float(current_delta_max)
            self._log_kappa_ref = kappa_at_proj if kappa_ref is not None and len(kappa_ref) > 0 else None
            self._log_v = float(speed)
            
            # Sanity tripwires for feedforward validation
            _eps = 0.01  # rad
            _k_min = 0.005  # 1/m, avoid noise when curvature near zero
            if abs(delta_ff_rad) > abs(delta_cmd_rad_raw) + _eps:
                print(f"[MPC] *** FF TRIPWIRE 1: abs(delta_ff)={abs(delta_ff_rad):.4f} > abs(delta_total)+eps={abs(delta_cmd_rad_raw) + _eps:.4f} — suspicious (ff dominating unexpectedly)")
            if abs(kappa_at_proj) > _k_min and ((delta_ff_rad > 0) != (kappa_at_proj > 0)):
                print(f"[MPC] *** FF TRIPWIRE 2: sign(delta_ff) != sign(kappa_ref_at_proj) with |kappa_ref|={abs(kappa_at_proj):.4f} > {_k_min} — likely sign/axis mismatch")
            
            # Recommendation D: STEER_SIGN_SANITY — compare sign(delta_cmd) vs sign(yaw_rate/v) (curvature from motion)
            kappa_pred = kappa_at_proj_0
            _v = float(speed) if speed is not None else 0.0
            if _v > 0.15 and yaw_rate is not None:
                kappa_meas = float(yaw_rate) / _v  # curvature from motion (same as STEER_CAL kappa)
            else:
                kappa_meas = kappa_at_proj  # fallback to path curvature when yaw_rate/v unavailable
            sign_delta = 1 if delta_cmd_rad > 0 else (-1 if delta_cmd_rad < 0 else 0)
            sign_kappa = 1 if kappa_meas > 0 else (-1 if kappa_meas < 0 else 0)
            print(f"[MPC] STEER_SIGN_SANITY delta_cmd_rad={delta_cmd_rad:.4f} kappa_pred={kappa_pred:.4f} kappa_meas={kappa_meas:.4f} (yaw_rate/v) sign_delta_cmd={sign_delta} sign_kappa_meas={sign_kappa}")
            _steer_sanity_count = getattr(self, '_steer_sanity_mismatch_count', 0)
            if abs(delta_cmd_rad) > 0.05 and abs(kappa_meas) > 0.02 and delta_cmd_rad * kappa_meas < 0:
                _steer_sanity_count += 1
                self._steer_sanity_mismatch_count = _steer_sanity_count
                print(f"[MPC] STEER_SIGN_SANITY MISMATCH #{_steer_sanity_count} (delta_cmd and yaw_rate/v opposite signs — actuation sign inversion)")
            
            # Plan: output is road wheel angle (rad). Normalized only for debug logs.
            steer_norm_debug = float(np.clip(delta_cmd_rad / current_delta_max, -1.0, 1.0))
            self._log_steer_mpc_raw = steer_norm_debug
            self._log_steer_after_caps = steer_norm_debug
            self._log_steer_after_lpf = steer_norm_debug  # no LPF on output; IO adapter converts rad→dSPACE
            delta_rate_radps = (delta_cmd_rad - delta_prev_rad) / dt_step if dt_step > 0 else 0.0
            self._log_steer_rate = delta_rate_radps
            
            # Plan A) [CTRL] log: delta_raw_rad, delta_clamped_rad, delta_cmd_rad, delta_max_rad, rate_max_radps, delta_rate_radps, sat_mag, sat_rate
            sat_mag = abs(delta_cmd_rad_raw) > current_delta_max
            sat_rate = abs(delta_clamped_rad - delta_prev_rad) > rate_max_radps * dt_step
            self._log_ctrl_delta_raw_rad = float(delta_cmd_rad_raw)
            self._log_ctrl_delta_clamped_rad = float(delta_clamped_rad)
            self._log_ctrl_delta_cmd_rad = float(delta_cmd_rad)
            self._log_ctrl_delta_max_rad = float(current_delta_max)
            self._log_ctrl_rate_max_radps = float(rate_max_radps)
            self._log_ctrl_delta_rate_radps = float(delta_rate_radps)
            self._log_ctrl_sat_mag = bool(sat_mag)
            self._log_ctrl_sat_rate = bool(sat_rate)
            # Log every 10 ticks to verify constraints without flooding (plan A)
            _ctrl_log_count = getattr(self, '_ctrl_log_count', 0)
            if _ctrl_log_count % 10 == 0:
                print(f"[CTRL] delta_raw_rad={delta_cmd_rad_raw:.4f} delta_clamped_rad={delta_clamped_rad:.4f} delta_cmd_rad={delta_cmd_rad:.4f} delta_max_rad={current_delta_max:.4f} rate_max_radps={rate_max_radps} delta_rate_radps={delta_rate_radps:.4f} sat_mag={sat_mag} sat_rate={sat_rate}")
            self._ctrl_log_count = _ctrl_log_count + 1

            # To-Do 4.1: saturation counter (use rad: sat when |delta_cmd_rad| near delta_max)
            if abs(steer_norm_debug) > 0.98:
                self._saturation_count = getattr(self, '_saturation_count', 0) + 1
                _seg = getattr(self, '_log_segment_id', None)
                _cte = getattr(self, '_log_cte_raw', None)
                _cte_s = f"{_cte:.3f}" if _cte is not None else "?"
                print(f"[MPC] SATURATION #{self._saturation_count} delta_max_used={current_delta_max:.4f} segment_id={_seg} v={self._log_v:.3f} kappa_ref_at_proj={kappa_at_proj:.4f} cte_raw={_cte_s} delta_total={delta_cmd_rad_raw:.4f} delta_ff={delta_ff_rad:.4f} delta_fb={delta_fb_rad:.4f}")

            # Reset invalid count on success
            self.invalid_count = 0
            self.last_valid_steering = delta_cmd_rad  # now in rad
            
            # Plan Step 1: controller returns steer_road_rad (rad), not normalized
            _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
            return float(delta_cmd_rad)
            
        except Exception as e:
            print(f"[MPC] Error solving QP: {e}")
            _mpc_timing.record_lateral_mpc_ms((time.perf_counter() - t0) * 1000)
            return self._fallback_steering()
    
    def _is_3d_waypoint(self, waypoint) -> bool:
        """Check if waypoint is 3D (has z coordinate)."""
        return len(waypoint) >= 3
    
    def _is_3d_waypoints(self, waypoints) -> bool:
        """Check if waypoints list contains 3D points."""
        if not waypoints or len(waypoints) == 0:
            return False
        return self._is_3d_waypoint(waypoints[0])

    def _best_segment_in_window(self,
                                waypoints: List[Tuple[float, float]],
                                px: float, py: float,
                                start_idx: int, length: int, n_wp: int,
                                is_3d: bool,
                                last_seg: Optional[int] = None
                                ) -> Tuple[int, float, float, Optional[float]]:
        """Scan a window of segments and return the best segment for (px, py).

        Used for initial scan (local or full), reacquire full scan, and gate-triggered full scan
        so the same rule lives in one place. Closed loop: segment i = waypoints[i] -> waypoints[(i+1)%n_wp].

        Returns:
            (best_segment_idx, best_score, best_match_dist, prev_seg_score).
            prev_seg_score is the score of segment last_seg if visited, else None.
        """
        best_segment_idx = 0
        best_score = float('inf')
        best_match_dist = float('inf')
        prev_seg_score = None
        for off in range(length):
            i = (start_idx + off) % n_wp
            wp0 = waypoints[i]
            wp1 = waypoints[(i + 1) % n_wp]
            x0, y0 = wp0[0], wp0[1]
            x1, y1 = wp1[0], wp1[1]
            seg_dx = x1 - x0
            seg_dy = y1 - y0
            if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                seg_dz = wp1[2] - wp0[2]
                seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)  # XY for CTE
            else:
                seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)
            if seg_len < 1e-6:
                continue
            wx = px - x0
            wy = py - y0
            u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
            u_proj = np.clip(u_proj, 0.0, 1.0)
            proj_x = x0 + u_proj * seg_dx
            proj_y = y0 + u_proj * seg_dy
            dx = px - proj_x
            dy = py - proj_y
            distance = np.sqrt(dx*dx + dy*dy)
            if u_proj < 0.3:
                behind_penalty = 10.0 * (0.3 - u_proj) / 0.3
            elif u_proj < 0.5:
                behind_penalty = 2.0 * (0.5 - u_proj) / 0.5
            else:
                behind_penalty = 0.0
            score = distance + behind_penalty
            if i == last_seg:
                prev_seg_score = score
            if score < best_score:
                best_score = score
                best_segment_idx = i
                best_match_dist = distance
        return (best_segment_idx, best_score, best_match_dist, prev_seg_score)

    def _compute_errors(self,
                       position: Tuple[float, float],
                       heading: float,
                       waypoints: List[Tuple[float, float]],
                       current_waypoint_idx: Optional[int] = None,
                       prev_e_y: Optional[float] = None) -> Tuple[float, float, int]:
        """Compute lateral error (e_y) and heading error (e_psi) from waypoints.

        Dynamically selects the best waypoint segment for control based on proximity to vehicle.
        When |prev_e_y| > segment_stick_cte_m, keeps current segment to avoid reference flip and
        steer oscillation (generic for any TTL). Supports both 2D (x, y) and 3D (x, y, z) waypoints.

        Args:
            position: Current vehicle position (x, y) or (x, y, z)
            heading: Current vehicle heading (radians) - yaw angle in XY plane
            waypoints: List of waypoint (x, y) or (x, y, z) tuples
            current_waypoint_idx: Optional hint from behavior (wp_last_idx) for local scan start; we choose
                segment and set last_seg_idx (see class comment on current-index contract).
            prev_e_y: Previous lateral error (m); when |prev_e_y| > threshold, segment is not switched

        Returns:
            Tuple of (e_y, e_psi, segment_idx):
            - e_y: Lateral error (meters) in XY plane, positive = left of path
            - e_psi: Heading error (radians), positive = heading left of path direction
            - segment_idx: Index of the waypoint segment being used for control
        """
        if not waypoints or len(waypoints) < 2:
            # No waypoints: set log defaults so ref_log line does not use stale values
            self._log_match_dist_m = None
            self._log_gate_status = 'ACCEPT'
            self._log_gate_reason = None
            self._log_s_ref = None
            self._log_delta_s_ref = 0.0
            self._log_s_jump_flag = False
            self._log_segment_prev = None
            self._log_segment_new = 0
            self._log_stick_blocked = False
            self._log_ref_point = None
            self._log_ego = None
            self._log_s_ref_continuous = True
            self._log_proj_hop = False
            self._log_projection_continuity_ok = True
            return (0.0, 0.0, 0)

        # Handle both 2D and 3D positions (always use XY for CTE computation)
        px, py = position[0], position[1]
        is_3d = self._is_3d_waypoints(waypoints)

        # Find the best waypoint segment dynamically
        # CRITICAL: Prefer segments AHEAD of the vehicle, not behind
        # Choose segment with closest perpendicular distance, but bias toward segments ahead
        best_segment_idx = 0
        best_score = float('inf')
        best_match_dist = float('inf')  # perpendicular distance for continuity gate
        prev_seg_score = None  # score of last step's segment (for hysteresis)
        last_seg = getattr(self, 'last_seg_idx', None)

        # Start search near current index, but wrap around end->start
        n_wp = len(waypoints)

        lookback = 5
        forward_window = 300  # how far ahead we search (in segments); tune if needed
        if current_waypoint_idx is not None and current_waypoint_idx >= 0:
            start_idx = (int(current_waypoint_idx) - lookback) % n_wp
            search_len = min(n_wp, lookback + forward_window)  # local scan
        else:
            start_idx = 0
            search_len = n_wp                              # FULL scan to acquire lock
        best_segment_idx, best_score, best_match_dist, prev_seg_score = self._best_segment_in_window(
            waypoints, px, py, start_idx, search_len, n_wp, is_3d, last_seg
        )

        # Reference continuity gate: reject candidate if too far or non-monotonic progress (data association)
        # Recommendation B: when gate rejects AND match_dist > hard_fail, force re-association (do not keep stale segment)
        gate_reason = None  # for logging: None = accept; 'too_far' | 'backward' | 's_jump' = reject reason
        max_wp_dist = getattr(self, '_max_wp_match_dist_m', 3.0)
        max_s_jump = getattr(self, '_max_s_jump_m', 4.0)
        hard_fail_dist_m = getattr(self, '_gate_hard_fail_dist_m', 6.0)  # Recommendation B: force re-assoc when off track
        if last_seg is not None and 0 <= last_seg < n_wp and best_segment_idx != last_seg:
            reject_candidate = False
            if best_match_dist > max_wp_dist:
                gate_reason = 'too_far'
                reject_candidate = True  # association weak: match point too far
            elif best_segment_idx < last_seg and (last_seg - best_segment_idx) < n_wp // 2:
                gate_reason = 'backward'
                reject_candidate = True  # progress went backwards (not wrap: wrap has last_seg - best > n_wp/2)
            else:
                # Along-path distance from last_seg to best_segment_idx (closed loop: wrap)
                s_jump = 0.0
                k = last_seg
                while k != best_segment_idx:
                    next_k = (k + 1) % n_wp
                    w0, w1 = waypoints[k], waypoints[next_k]
                    dx = w1[0] - w0[0]
                    dy = w1[1] - w0[1]
                    s_jump += np.sqrt(dx*dx + dy*dy)
                    k = next_k
                if s_jump > max_s_jump:
                    gate_reason = 's_jump'
                    reject_candidate = True  # progress jumped forward too much
            if reject_candidate:
                # Recommendation B: when far off track (best_match_dist > hard_fail), keep new segment.
                # For s_jump: never revert — full scan already found best segment; reverting would keep stale segment.
                # For backward: revert to avoid going backward. For too_far: revert only if new match is still bad.
                if gate_reason == 'backward':
                    best_segment_idx = last_seg
                elif gate_reason == 'too_far' and best_match_dist <= hard_fail_dist_m:
                    best_segment_idx = last_seg
                # else: keep best_segment_idx (s_jump or too_far with good new match)

        # One reacquire path: weak association (match-dist spike) or gate reject (too_far/s_jump) -> single full scan
        reacquire_dist_m = getattr(self, '_reacquire_dist_m', 3.0)
        need_reacquire = (
            (best_match_dist > reacquire_dist_m and search_len != n_wp)
            or (gate_reason in ('too_far', 's_jump'))
        )
        did_reacquire = False
        if need_reacquire:
            best_segment_idx, best_score, best_match_dist, prev_seg_score = self._best_segment_in_window(
                waypoints, px, py, 0, n_wp, n_wp, is_3d, last_seg
            )
            did_reacquire = True

        # Smooth segment selection: hysteresis only (no advance cap — cap caused lag at high speed)
        # Skip hysteresis when we just reacquired so the new segment is not reverted to last_seg.
        # In bends (high curvature), use stronger hysteresis so we don't switch segment and cause steer flip (generic: curvature in 1/m)
        waypoint_idx = best_segment_idx
        effective_hysteresis_m = self._segment_hysteresis_m
        if last_seg is not None and 0 <= last_seg < n_wp and n_wp >= 3:
            p0 = (waypoints[(last_seg - 1) % n_wp][0], waypoints[(last_seg - 1) % n_wp][1])
            p1 = (waypoints[last_seg][0], waypoints[last_seg][1])
            p2 = (waypoints[(last_seg + 1) % n_wp][0], waypoints[(last_seg + 1) % n_wp][1])
            v1x = p1[0] - p0[0]; v1y = p1[1] - p0[1]
            v2x = p2[0] - p1[0]; v2y = p2[1] - p1[1]
            cross = v1x * v2y - v1y * v2x
            len1 = np.sqrt(v1x*v1x + v1y*v1y)
            len2 = np.sqrt(v2x*v2x + v2y*v2y)
            if len1 > 1e-6 and len2 > 1e-6:
                avg_len = (len1 + len2) / 2.0
                abs_kappa = abs(2.0 * cross / (len1 * len2 * avg_len))
                effective_hysteresis_m = self._segment_hysteresis_m * (1.0 + min(abs_kappa * 5.0, 1.5))
        if last_seg is not None and 0 <= last_seg < n_wp and not did_reacquire:
            if prev_seg_score is not None and best_segment_idx != last_seg:
                if best_score >= prev_seg_score - effective_hysteresis_m:
                    waypoint_idx = last_seg
        # Recommendation C: stick only when association is good (gate_accept and match_dist < threshold)
        # When far off, prefer reacquisition over locking to a bad segment
        stick_m = getattr(self, '_segment_stick_cte_m', 1.5)
        stick_dist_ok_m = getattr(self, '_stick_association_ok_m', 2.0)  # only stick when match_dist < this
        gate_accept = (gate_reason is None)
        waypoint_idx_before_stick = waypoint_idx
        if (prev_e_y is not None and abs(prev_e_y) >= stick_m and last_seg is not None and 0 <= last_seg < n_wp
                and gate_accept and best_match_dist < stick_dist_ok_m):
            waypoint_idx = last_seg
        stick_blocked = (waypoint_idx_before_stick != waypoint_idx)
        wp0 = waypoints[waypoint_idx]
        wp1 = waypoints[(waypoint_idx + 1) % n_wp]
        x0, y0 = wp0[0], wp0[1]
        x1, y1 = wp1[0], wp1[1]
        
        seg_dx = x1 - x0
        seg_dy = y1 - y0
        seg_len = np.sqrt(seg_dx*seg_dx + seg_dy*seg_dy)  # XY projection for CTE
        
        if seg_len < 1e-6:
            self._log_match_dist_m = None
            self._log_gate_status = getattr(self, '_log_gate_status', 'ACCEPT')
            self._log_gate_reason = getattr(self, '_log_gate_reason', None)
            self._log_s_ref = getattr(self, '_log_s_ref', None)
            self._log_delta_s_ref = 0.0
            self._log_s_jump_flag = False
            self._log_segment_prev = last_seg
            self._log_segment_new = waypoint_idx
            self._log_stick_blocked = stick_blocked
            self._log_ref_point = None
            self._log_ego = (float(px), float(py))
            self._log_s_ref_continuous = True
            self._log_proj_hop = False
            self._log_projection_continuity_ok = True
            return (0.0, 0.0, waypoint_idx)
        
        # Project vehicle position onto segment (in XY plane)
        wx = px - x0
        wy = py - y0
        u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
        u_proj = np.clip(u_proj, 0.0, 1.0)
        
        proj_x = x0 + u_proj * seg_dx
        proj_y = y0 + u_proj * seg_dy
        
        # Reference heading: segment direction (no flip — avoids spin-induced discontinuity)
        psi_ref = np.arctan2(seg_dy, seg_dx)
        
        # Option A: blend reference heading toward next segment near boundary (closed loop: use modulo)
        u_start = self._segment_blend_u_start
        if u_proj >= u_start:
            wp0_next = waypoints[(waypoint_idx + 1) % n_wp]
            wp1_next = waypoints[(waypoint_idx + 2) % n_wp]
            x0n, y0n = wp0_next[0], wp0_next[1]
            x1n, y1n = wp1_next[0], wp1_next[1]
            seg_dx_next = x1n - x0n
            seg_dy_next = y1n - y0n
            seg_len_next = np.sqrt(seg_dx_next*seg_dx_next + seg_dy_next*seg_dy_next)
            if seg_len_next >= 1e-6 and n_wp >= 2:
                psi_ref_next = np.arctan2(seg_dy_next, seg_dx_next)
                # Ramp alpha from 0 at u_start to 1 at u_proj=1
                alpha = min(1.0, (u_proj - u_start) / (1.0 - u_start))
                # Blend angles via unit vector to avoid wrap
                psi_ref = np.arctan2(
                    (1.0 - alpha) * np.sin(psi_ref) + alpha * np.sin(psi_ref_next),
                    (1.0 - alpha) * np.cos(psi_ref) + alpha * np.cos(psi_ref_next)
                )
        
        # Compute lateral error (e_y)
        # Normal vector: (-dy, dx) points LEFT of forward direction along segment
        # Positive e_y = LEFT of path, Negative e_y = RIGHT of path
        nx = -seg_dy / seg_len
        ny = seg_dx / seg_len
        
        # Normal vector (-dy, dx) points LEFT of segment direction; e_y positive = left of path
        e_y = (px - proj_x)*nx + (py - proj_y)*ny
        
        # Heading error: difference between reference and actual
        # Normalize to [-pi, pi]
        # Use e_psi = vehicle_heading - reference_heading (matches the discrete model sign)
        e_psi = heading - psi_ref
        # normalize to [-pi, pi]
        e_psi = math.atan2(math.sin(e_psi), math.cos(e_psi))
        e_psi = np.arctan2(np.sin(e_psi), np.cos(e_psi))  # Normalize to [-pi, pi]
        # --- Reference/segment logging for diagnostics (todo1 gate, continuity, MPCC vs behavior CTE) ---
        match_dist_m = float(np.sqrt((px - proj_x)**2 + (py - proj_y)**2))
        s_ref_cum = 0.0
        for k in range(0, waypoint_idx):
            w0, w1 = waypoints[k], waypoints[(k + 1) % n_wp]
            d = np.sqrt((w1[0] - w0[0])**2 + (w1[1] - w0[1])**2)
            s_ref_cum += d
        s_ref = s_ref_cum + float(u_proj * seg_len)
        last_s_ref = getattr(self, '_last_s_ref', None)
        delta_s_ref = float(s_ref - last_s_ref) if last_s_ref is not None else 0.0
        self._last_s_ref = s_ref
        self._log_match_dist_m = match_dist_m
        self._log_gate_status = 'REJECT' if gate_reason else 'ACCEPT'
        self._log_gate_reason = gate_reason  # None or 'too_far' | 'backward' | 's_jump'
        self._log_s_ref = s_ref
        self._log_delta_s_ref = delta_s_ref
        self._log_s_jump_flag = (gate_reason == 's_jump')
        self._log_segment_prev = last_seg  # may be None on first step
        self._log_segment_new = waypoint_idx
        self._log_stick_blocked = stick_blocked
        self._log_ref_point = (float(proj_x), float(proj_y))  # CTE cross-check: projection used for match_dist
        self._log_ego = (float(px), float(py))
        # Task 1: projection continuity — s doesn't jump, projection doesn't hop across track
        s_jump_ok = (gate_reason != 's_jump')
        proj_hop = (last_seg is not None and waypoint_idx != last_seg and match_dist_m > 2.0)
        self._log_s_ref_continuous = s_jump_ok
        self._log_proj_hop = proj_hop
        self._log_projection_continuity_ok = s_jump_ok and (not proj_hop)
        # Expose last errors for outer behavior logic (conditioning/safety/debug)
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
            Steering command in road wheel angle (rad), same contract as run_step (plan).
        """
        self.invalid_count += 1
        delta_max = self.config.max_steer_angle
        
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
            
            # Proportional term in rad (steer_proportional was in [-1,1])
            steer_proportional_rad = max(-delta_max, min(delta_max, steer_proportional * delta_max))
            # Combine with last valid steering (weighted average), both in rad
            if self.invalid_count <= self.config.max_invalid_count and abs(self.last_valid_steering) > 0.01:
                blend_factor = min(self.invalid_count / self.config.max_invalid_count, 1.0)
                steer_rad = (1.0 - blend_factor) * self.last_valid_steering + blend_factor * steer_proportional_rad
            else:
                steer_rad = steer_proportional_rad
            steer_rad = max(-delta_max, min(delta_max, steer_rad))
            self._delta_prev_output_rad = steer_rad
            print(f"[MPC Fallback] Using proportional steering: {steer_rad:.4f} rad (e_y={e_y:.2f}m, e_psi={e_psi:.2f}rad if available)")
            return float(steer_rad)
        
        # Fallback to old behavior if no error information available
        if self.invalid_count <= self.config.max_invalid_count:
            out_rad = self.last_valid_steering
            self._delta_prev_output_rad = out_rad
            return out_rad
        else:
            self._delta_prev_output_rad = 0.0
            return 0.0

