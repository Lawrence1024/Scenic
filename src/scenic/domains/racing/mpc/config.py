"""Configuration management for MPC controller.

Loads and validates MPC parameters from YAML configuration files.

**Default values policy (RC-4b, 2026-04-26):** the ``config_dict.get(key, default)``
defaults below are kept in sync with the hot values in ``vehicle_mpc.yaml``. When YAML
loads correctly (the normal case), the defaults are inert. They only take effect when
YAML is missing a key (e.g. test stubs, partial configs). Previously the defaults were
significantly different from YAML — e.g. ``w_ey_high_curv`` default was ``8.0`` while
YAML had ``22.0`` — which meant any code path that fell back to defaults silently used
under-tuned weights. RC-4b syncs them so default-fallback behavior matches YAML behavior.
If you change a YAML hot value, change the corresponding default below too.
"""

import yaml
import os
from typing import Dict, Any, Optional
from pathlib import Path

from scenic.domains.racing.constants import DELTA_MAX_RAD


class MPCConfig:
    """MPC configuration parameters.
    
    Stores all parameters needed for MPC controller operation,
    including vehicle parameters, MPC weights, safety thresholds,
    and ControlDesk variable paths.
    """
    
    def __init__(self, config_dict: Dict[str, Any]):
        """Initialize config from dictionary.
        
        Args:
            config_dict: Dictionary containing configuration parameters
        """
        # Timing
        self.ctrl_period = config_dict.get('ctrl_period', 0.05)
        self.mpc_prediction_horizon = config_dict.get('mpc_prediction_horizon', 35)  # Balanced for preview and speed
        self.mpc_prediction_dt = config_dict.get('mpc_prediction_dt', 0.05)
        
        # Vehicle geometry
        self.wheel_base = config_dict.get('wheel_base', 2.9718)
        self.max_steer_angle = config_dict.get('max_steer_angle', DELTA_MAX_RAD)
        self.steer_tau = config_dict.get('steer_tau', 0.3)
        self.steer_rate_lim = config_dict.get('steer_rate_lim', 6.98)  # QP internal rate limit (rad/s)
        self.steer_rate_limit_output_radps = config_dict.get('steer_rate_limit_output_radps', 1.0)  # output rate limit (rad/s), plan: 1.0
        self.steer_cmd_max = config_dict.get('steer_cmd_max', 240)  # ControlDesk units (matches YAML)
        # To-Do 4.2: optional higher steer cap in high curvature only (if vehicle/VEOS accepts more angle; else reduce speed or horizon)
        self.max_steer_angle_high_curv = config_dict.get('max_steer_angle_high_curv', None)  # rad; if set, use when |kappa| >= high_curvature_threshold
        
        # Steering mapping (calibrated)
        self.steer_scale = config_dict.get('steer_scale', None)  # Will be calibrated
        
        # MPC weights (base weights). Defaults synced to vehicle_mpc.yaml (RC-4b).
        self.w_ey = config_dict.get('w_ey', 10.0)
        self.w_epsi = config_dict.get('w_epsi', 5.0)
        self.w_u = config_dict.get('w_u', 0.05)
        self.w_du = config_dict.get('w_du', 2.2)  # Steering rate weight (higher = smoother, less overcorrect)
        self.wT_ey = config_dict.get('wT_ey', 15.0)
        self.wT_epsi = config_dict.get('wT_epsi', 2.0)

        # Velocity-weighted costs (improved performance at different speeds)
        self.w_epsi_vel = config_dict.get('w_epsi_vel', 1.5)  # Heading error * velocity^2 weight
        self.w_u_vel = config_dict.get('w_u_vel', 0.1)  # Control input * velocity^2 weight

        # Steering acceleration penalty (smoother steering)
        self.w_ddu = config_dict.get('w_ddu', 0.00003)  # Steering acceleration weight (smoother steering)
        # Feedforward tracking: penalize (delta - delta_ff)^2 = delta_fb^2 to shrink feedback relative to ff (smoother curves, less CTE oscillation)
        self.w_ff_track = config_dict.get('w_ff_track', 1.0)  # ~0.1x w_ey scale; start small, increase if delta_fb still large
        
        # Phase 2/3 MPCC: lag error and progress incentive (set both to 0 for trajectory-tracking only)
        self.Q_lag = config_dict.get('Q_lag', 0.02)   # Lag error: Q_lag * (s_ref - s)^2 per step (default active for Phase 3)
        self.Q_progress = config_dict.get('Q_progress', 0.005)  # Progress reward: -Q_progress * (s_N - s_0) at terminal
        
        # Adaptive weights based on curvature (low curvature = straights, high curvature = sharp turns)
        self.use_adaptive_weights = config_dict.get('use_adaptive_weights', True)
        self.low_curvature_threshold = config_dict.get('low_curvature_threshold', 0.02)  # 1/m
        self.high_curvature_threshold = config_dict.get('high_curvature_threshold', 0.07)  # 1/m (matches YAML)
        # Low curvature weights (for straights)
        self.w_ey_low_curv = config_dict.get('w_ey_low_curv', 0.05)  # raised from 0.01 to reduce straight-line drift (oscillation fix)
        self.w_epsi_low_curv = config_dict.get('w_epsi_low_curv', 0.0)
        self.w_epsi_vel_low_curv = config_dict.get('w_epsi_vel_low_curv', 0.3)
        self.w_u_low_curv = config_dict.get('w_u_low_curv', 1.0)
        self.w_u_vel_low_curv = config_dict.get('w_u_vel_low_curv', 0.25)
        self.w_ddu_low_curv = config_dict.get('w_ddu_low_curv', 0.000001)
        # High curvature weights (for sharp turns). Defaults synced to vehicle_mpc.yaml (RC-4b).
        self.w_ey_high_curv = config_dict.get('w_ey_high_curv', 22.0)
        self.w_epsi_high_curv = config_dict.get('w_epsi_high_curv', 12.0)
        self.w_epsi_vel_high_curv = config_dict.get('w_epsi_vel_high_curv', 2.0)
        self.w_u_high_curv = config_dict.get('w_u_high_curv', 0.02)
        self.w_u_vel_high_curv = config_dict.get('w_u_vel_high_curv', 0.05)
        self.w_ddu_high_curv = config_dict.get('w_ddu_high_curv', 0.000003)
        
        # Oscillation fixes (Phase 3+): CTE deadzone and cap on off-track weight scaling
        self.cte_deadzone = config_dict.get('cte_deadzone', 0.2)  # (m) treat |e_y| < this as 0 so MPC does not overcorrect small errors
        # To-Do 2: conditional deadzone — only apply when association is good (match_dist < this) and curvature ahead is low
        self.deadzone_dist_ok_m = config_dict.get('deadzone_dist_ok_m', 1.0)  # (m) max match_dist to allow deadzone; above = BLOCKED_MATCH_BAD
        self.curv_deadzone_max = config_dict.get('curv_deadzone_max', 0.04)  # (1/m) max curv_ahead_max to allow deadzone; never deadzone in moderate curvature (e.g. 0.03-0.05)
        self.cte_multiplier_max = config_dict.get('cte_multiplier_max', 1.5)  # cap on tracking-weight multiplier when off-track (avoid over-aggressive recovery)
        
        # QP numerics: floor speed in lateral dynamics so rows stay well-conditioned at v~0
        self.mpc_min_speed_for_qp_mps = config_dict.get('mpc_min_speed_for_qp_mps', 2.5)
        self.qp_hessian_reg = config_dict.get('qp_hessian_reg', 1e-5)

        # Safety thresholds
        self.admissible_position_error = config_dict.get('admissible_position_error', 30.0)  # Default 30.0m for sparse waypoints
        # Reduced from 1.57 rad (90 deg) to 2.36 rad (135 deg) to allow MPC to run more often
        # Large yaw errors are common when off-track, and MPC can handle them better than fallback
        self.admissible_yaw_error_rad = config_dict.get('admissible_yaw_error_rad', 2.36)
        self.max_invalid_count = config_dict.get('max_invalid_count', 10)
        
        # Task 3: Yaw-rate damping to prevent snap oscillations (cost on (e_psi_{k+1} - e_psi_k)^2)
        self.w_epsi_rate = config_dict.get('w_epsi_rate', 0.1)

        # Corridor barrier (mirrors race_common Frenet planner pattern). Per-step quadratic
        # cost ``w_b_k * (e_y_k - mid_offset_k)**2`` that pulls the planned trajectory
        # toward the corridor midpoint when the racing line would put the car body off
        # the curb. **Hard activation deadzone**: when the car body's clearance to the
        # nearest bound exceeds ``corridor_activation_clearance_m``, ``w_b_k = 0`` (no
        # cost) so the racing line wins exactly on safe sections. Cubic ramp only when in
        # genuine danger. Identity if the TTL lacks LEFT_BOUND/RIGHT_BOUND columns.
        # See ``docs/frames.md`` for math.
        self.corridor_barrier_enabled = config_dict.get('corridor_barrier_enabled', True)
        # Half the IAC AV-21 vehicle width (1.93 m / 2 = 0.965 m).
        self.vehicle_half_width_m = config_dict.get('vehicle_half_width_m', 0.965)
        # Body clearance (= min(d_left, d_right) - vehicle_half_width) above which the
        # barrier is FULLY DISABLED. Default 1.5 m -- chosen so that:
        #   * Typical straights with min_d ~5-6 m have clearance ~4-5 m >> threshold,
        #     hard deadzone, racing line wins exactly (no fight, no speed regression).
        #   * The OOB-suspect Andretti exit section had min_d ~2 m, clearance ~1 m
        #     (deep inside the activation zone), so the cost fires there meaningfully.
        # Larger value = pulls inward earlier (safer but more conservative); smaller
        # value = lets the line cut closer (faster but body may go off-curb).
        self.corridor_activation_clearance_m = config_dict.get('corridor_activation_clearance_m', 1.5)
        # Cap on the per-step barrier weight (when body clearance reaches 0). Cubic ramp
        # from 0 (at activation threshold) to this max (at zero clearance / boundary
        # contact). Keep large enough to dominate w_ey so the planner pulls inward.
        self.corridor_barrier_weight_max = config_dict.get('corridor_barrier_weight_max', 5000.0)
        
        # Filter
        self.steering_lpf_cutoff_hz = config_dict.get('steering_lpf_cutoff_hz', 2.0)  # Smoother steering output
        
        # Waypoint/Reference
        self.traj_resample_dist = config_dict.get('traj_resample_dist', 0.1)  # Finer resolution (0.1m vs 0.2m)
        # Segment selection smoothing: only switch segment when new score is better by this margin (m)
        self.segment_hysteresis_m = config_dict.get('segment_hysteresis_m', 0.4)
        # When |CTE| > this (m), do not switch segment — stick to current segment to avoid reference flip and steer oscillation (generic, any TTL)
        self.segment_stick_cte_m = config_dict.get('segment_stick_cte_m', 1.5)
        # Reference blend at boundaries: blend toward next segment when u_proj >= this (0 = start, 1 = end)
        self.segment_blend_u_start = config_dict.get('segment_blend_u_start', 0.7)
        # Reference continuity gate: reject candidate segment if match is weak (avoid reference snap at curve entry)
        self.max_wp_match_dist_m = config_dict.get('max_wp_match_dist_m', 3.0)  # (m) max perpendicular distance to accept segment switch
        self.max_s_jump_m = config_dict.get('max_s_jump_m', 4.0)  # (m) max along-path progress jump per tick; reject if exceeded
        self.gate_hard_fail_dist_m = config_dict.get('gate_hard_fail_dist_m', 6.0)  # (m) when gate rejects and match_dist > this, force re-association (Rec B)
        self.reacquire_dist_m = config_dict.get('reacquire_dist_m', 3.0)  # (m) trigger full scan when best_match_dist > this (reacquire on weak association)
        self.stick_association_ok_m = config_dict.get('stick_association_ok_m', 2.0)  # (m) only stick to segment when match_dist < this (Rec C)
        
        # Feedforward preview (Task B: chicane) — blend of at-proj and +10 m ahead curvature for delta_ff
        self.ff_preview_blend = config_dict.get('ff_preview_blend', 0.4)  # 0 = at-proj only, 0.4 = 40% ahead
        self.ff_chicane_preview_blend = config_dict.get('ff_chicane_preview_blend', 0.85)  # Task 2: use preview in chicanes (sign flip)
        self.ff_chicane_curvature_threshold = config_dict.get('ff_chicane_curvature_threshold', 0.02)  # 1/m, min |kappa| to detect chicane
        
        # Curvature smoothing
        self.curvature_smoothing_num = config_dict.get('curvature_smoothing_num', 15)  # Points for curvature calculation
        
        # Spline-based resampling
        self.use_splines = config_dict.get('use_splines', True)  # Use spline fitting with arc-length parameterization
        
        # Longitudinal MPC parameters
        self.vehicle_mass = config_dict.get('vehicle_mass', 753.87)  # kg
        self.max_acceleration = config_dict.get('max_acceleration', 20.0)  # m/s^2
        self.max_deceleration = config_dict.get('max_deceleration', 15.0)  # m/s^2
        self.drag_coefficient = config_dict.get('drag_coefficient', 0.881)
        self.cross_sectional_area = config_dict.get('cross_sectional_area', 1.0)  # m^2
        self.air_density = config_dict.get('air_density', 1.2)  # kg/m^3
        self.rolling_resistance = config_dict.get('rolling_resistance', 0.013)
        self.accel_tau = config_dict.get('accel_tau', 0.2)  # Acceleration time constant (s)
        
        # Longitudinal MPC weights
        self.w_v = config_dict.get('w_v', 10.0)  # Speed tracking weight
        self.w_a = config_dict.get('w_a', 0.25)  # Acceleration smoothness weight (smoother throttle/brake)
        self.w_u_lon = config_dict.get('w_u_lon', 0.05)  # Control input weight
        self.w_du_lon = config_dict.get('w_du_lon', 2.0)  # Control rate weight (smoother throttle/brake)
        self.wT_v = config_dict.get('wT_v', 20.0)  # Terminal speed weight
        
        # Longitudinal MPC filters
        self.throttle_lpf_cutoff_hz = config_dict.get('throttle_lpf_cutoff_hz', 3.5)
        self.brake_lpf_cutoff_hz = config_dict.get('brake_lpf_cutoff_hz', 3.5)
        
        # Gear 1 creep (race car idle torque: car moves without throttle in gear 1)
        self.creep_accel_gear1 = config_dict.get('creep_accel_gear1', 0.3)  # m/s^2 equivalent at zero throttle
        self.creep_speed_threshold = config_dict.get('creep_speed_threshold', 3.0)  # m/s, apply creep below this
        
        # Deadbands to avoid brake/throttle oscillation (especially in turns)
        self.speed_deadband = config_dict.get('speed_deadband', 0.3)  # m/s, hold command if |v - v_ref| < this
        self.accel_deadband = config_dict.get('accel_deadband', 0.25)  # m/s^2, zero accel_cmd if |accel_cmd| < this
        
        # Curvature-based speed limiting
        self.max_lateral_acceleration = config_dict.get('max_lateral_acceleration', 8.0)  # m/s² (conservative for indoor sim)
        self.curvature_slew_threshold = config_dict.get('curvature_slew_threshold', 0.05)  # 1/m (curvature threshold for increased slew rate)
        
        # ControlDesk variable paths (optional, can be overridden)
        self.controldesk_paths = config_dict.get('controldesk_paths', {})
    
    def adapt_to_timestep(self, timestep: float):
        """Adapt control period to match Scenic simulation timestep.
        
        Args:
            timestep: Scenic simulation timestep in seconds
        """
        self.ctrl_period = timestep
        # Optionally adjust prediction dt to match
        if abs(self.mpc_prediction_dt - timestep) > 0.01:
            self.mpc_prediction_dt = timestep


def load_mpc_config(config_path: Optional[str] = None) -> MPCConfig:
    """Load MPC configuration from YAML file.
    
    Args:
        config_path: Path to YAML config file. If None, uses default.
        
    Returns:
        MPCConfig object with loaded parameters
        
    Raises:
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If YAML parsing fails
    """
    if config_path is None:
        # Use default config file in the same directory as this module
        default_path = Path(__file__).parent / 'vehicle_mpc.yaml'
        config_path = str(default_path)
    
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"MPC config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        config_dict = yaml.safe_load(f)
    
    # Handle ROS-style parameter nesting (/**/ros__parameters)
    # YAML parser may read '/**:' as '/**' (colon is special in YAML)
    if '/**:' in config_dict:
        config_dict = config_dict['/**:'].get('ros__parameters', config_dict)
    elif '/**' in config_dict:
        config_dict = config_dict['/**'].get('ros__parameters', config_dict)
    
    return MPCConfig(config_dict)

