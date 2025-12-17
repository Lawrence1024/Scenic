"""MPC (Model Predictive Control) module for racing domain.

This module provides MPC-based lateral control for racing vehicles,
replacing PID controllers with predictive control for better performance
on racing tracks.
"""

from .mpc_lateral import MPCLateralController
from .config import MPCConfig, load_mpc_config
from .reference_builder import ReferenceBuilder

__all__ = [
    'MPCLateralController',
    'MPCConfig',
    'load_mpc_config',
    'ReferenceBuilder',
]

