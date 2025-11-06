"""Vehicle control and physics module for dSPACE simulator.

This module contains classes for:
- Vehicle physics simulation (VehiclePhysicsState)
- Vehicle control logic (VehicleController)
"""

from .physics import VehiclePhysicsState
from .controller import VehicleController

__all__ = ['VehiclePhysicsState', 'VehicleController']

