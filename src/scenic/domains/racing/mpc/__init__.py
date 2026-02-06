"""MPC (Model Predictive Control) module for racing domain.

This module provides MPC-based lateral and longitudinal control for racing vehicles,
replacing PID controllers with predictive control for better performance
on racing tracks.
"""

from .mpc_lateral import MPCLateralController
from .mpc_longitudinal import MPCLongitudinalController
from .config import MPCConfig, load_mpc_config
from .reference_builder import ReferenceBuilder

__all__ = [
    'MPCLateralController',
    'MPCLongitudinalController',
    'MPCConfig',
    'load_mpc_config',
    'ReferenceBuilder',
]

