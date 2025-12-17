"""Configuration management for MPC controller.

Loads and validates MPC parameters from YAML configuration files.
"""

import yaml
import os
from typing import Dict, Any, Optional
from pathlib import Path


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
        self.mpc_prediction_horizon = config_dict.get('mpc_prediction_horizon', 30)
        self.mpc_prediction_dt = config_dict.get('mpc_prediction_dt', 0.05)
        
        # Vehicle geometry
        self.wheel_base = config_dict.get('wheel_base', 2.9718)
        self.max_steer_angle = config_dict.get('max_steer_angle', 0.2816)
        self.steer_tau = config_dict.get('steer_tau', 0.3)
        self.steer_rate_lim = config_dict.get('steer_rate_lim', 1.0)  # rad/s
        self.steer_cmd_max = config_dict.get('steer_cmd_max', 70)  # ControlDesk units
        
        # Steering mapping (calibrated)
        self.steer_scale = config_dict.get('steer_scale', None)  # Will be calibrated
        
        # MPC weights
        self.w_ey = config_dict.get('w_ey', 2.0)
        self.w_epsi = config_dict.get('w_epsi', 0.5)
        self.w_u = config_dict.get('w_u', 0.2)
        self.w_du = config_dict.get('w_du', 5.0)
        self.wT_ey = config_dict.get('wT_ey', 5.0)
        self.wT_epsi = config_dict.get('wT_epsi', 1.0)
        
        # Safety thresholds
        self.admissible_position_error = config_dict.get('admissible_position_error', 5.0)
        # Reduced from 1.57 rad (90 deg) to 2.36 rad (135 deg) to allow MPC to run more often
        # Large yaw errors are common when off-track, and MPC can handle them better than fallback
        self.admissible_yaw_error_rad = config_dict.get('admissible_yaw_error_rad', 2.36)
        self.max_invalid_count = config_dict.get('max_invalid_count', 10)
        
        # Filter
        self.steering_lpf_cutoff_hz = config_dict.get('steering_lpf_cutoff_hz', 3.0)
        
        # Waypoint/Reference
        self.traj_resample_dist = config_dict.get('traj_resample_dist', 0.2)
        
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
        # Find Scenic root by looking for debug_mpc directory
        current = Path(__file__).resolve()
        default_path = None
        while current.parent != current:  # Stop at filesystem root
            debug_mpc_dir = current / 'debug_mpc'
            if debug_mpc_dir.exists() and debug_mpc_dir.is_dir():
                default_path = debug_mpc_dir / 'vehicle_mpc.yaml'
                break
            current = current.parent
        
        # Fallback to relative path calculation if search failed
        if default_path is None:
            default_path = Path(__file__).parent.parent.parent.parent.parent.parent / 'debug_mpc' / 'vehicle_mpc.yaml'
        
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

