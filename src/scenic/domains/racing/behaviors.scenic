"""Racing-specific behaviors for dynamic agents.

These behaviors extend the driving domain behaviors with racing-specific
strategies and maneuvers, including waypoint-based autonomous racing.
"""

from scenic.domains.driving.behaviors import *
from scenic.domains.driving.controllers import PIDLateralController, PIDLongitudinalController
from scenic.domains.driving.actions import SetThrottleAction, SetBrakeAction, SetSteerAction
import scenic.domains.racing.model as _racing

# Import dSPACE-specific actions if available
try:
    from scenic.simulators.dspace.actions import SetVehicleControl as DSPACESetVehicleControl
    from scenic.simulators.dspace.actions import SetThrottleAction as DSPACESetThrottleAction
    from scenic.simulators.dspace.actions import SetBrakeAction as DSPACESetBrakeAction
    from scenic.simulators.dspace.actions import SetSteerAction as DSPACESetSteerAction
    from scenic.simulators.dspace.actions import SetVelocityAction as DSPACESetVelocityAction
    DSPACE_AVAILABLE = True
except ImportError:
    DSPACE_AVAILABLE = False

behavior SimpleRacingBehavior():
    """Simple racing behavior with basic throttle and steering control."""
    while True:
        if DSPACE_AVAILABLE:
            take DSPACESetThrottleAction(0.5)
            take DSPACESetSteerAction(0.1)
            wait
            take DSPACESetSteerAction(-0.1)
            wait
            take DSPACESetSteerAction(0.0)
            wait
        else:
            take SetThrottleAction(0.5)
            take SetSteerAction(0.1)
            wait
            take SetSteerAction(-0.1)
            wait
            take SetSteerAction(0.0)
            wait

behavior SimplePitBehavior():
    """Simple pit lane behavior with reduced speed."""
    while True:
        if DSPACE_AVAILABLE:
            take DSPACESetThrottleAction(0.2)
            take DSPACESetSteerAction(0.05)
            wait
            take DSPACESetSteerAction(-0.05)
            wait
            take DSPACESetSteerAction(0.0)
            wait
        else:
            take SetThrottleAction(0.2)
            take SetSteerAction(0.05)
            wait
            take SetSteerAction(-0.05)
            wait
            take SetSteerAction(0.0)
            wait