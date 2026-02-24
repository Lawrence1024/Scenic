"""dSPACE-specific actions.

This module provides ONLY dSPACE-specific actions. Standard driving and racing
actions (SetThrottleAction, SetBrakeAction, SetSteerAction, etc.) are inherited
from the driving domain.

Following the CARLA pattern: import domain actions, only define simulator-specific ones.
"""

from scenic.core.simulators import Action


# Marker mixin to identify dSPACE-backed vehicle agents (mirrors CARLA pattern)
class _DSpaceVehicle:
    """Mixin identifying dSPACE vehicles.
    
    Used to avoid importing Scenic classes from model.scenic in Python modules.
    Action gating can check isinstance(agent, _DSpaceVehicle).
    """
    pass


class SetVehicleControl(Action):
    """Set multiple control inputs simultaneously (dSPACE-specific convenience action).
    
    This is a dSPACE-specific action for setting throttle, brake, and steering
    in a single action. For individual controls, use the standard driving domain
    actions (SetThrottleAction, SetBrakeAction, SetSteerAction).
    
    Args:
        throttle: Throttle input (0.0 to 1.0)
        brake: Brake input (0.0 to 1.0)
        steer: Steering. For PID / normalized use: -1.0 to 1.0. For ego with MPC
            (agent._racing_steer_units == 'rad'), pass road wheel angle in radians.
            See RACING_CONTROL_CONTRACT.md.
        velocity: Target velocity in m/s (optional)
    """
    
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0, velocity=None):
        self.throttle = max(0.0, min(1.0, throttle))
        self.brake = max(0.0, min(1.0, brake))
        self.steer = max(-1.0, min(1.0, steer))
        self.velocity = velocity
    
    def canBeTakenBy(self, agent):
        """Check if agent can take this action."""
        return isinstance(agent, _DSpaceVehicle)
    
    def applyTo(self, obj, sim):
        """Apply control inputs to the vehicle in dSPACE."""
        # Store all controls at once in _control_state
        # The simulator's step() method will apply these via ControlDesk
        if not hasattr(obj, '_control_state'):
            obj._control_state = {}
        obj._control_state.update({
            'throttle': self.throttle,
            'braking': self.brake,
            'steering': self.steer
        })
        if self.velocity is not None:
            obj._control_state['velocity'] = self.velocity


# NOTE: SetThrottleAction, SetBrakeAction, SetSteerAction are NOT defined here.
# They come from scenic.domains.driving.actions and work through the Steers protocol.
# The DSPACERacingCar class implements the Steers protocol methods (setThrottle, 
# setSteering, setBraking) which store values in _control_state, and the simulator's
# step() method applies them via ControlDesk.
#
# Only dSPACE-specific actions should be defined here (like SetVehicleControl above).
