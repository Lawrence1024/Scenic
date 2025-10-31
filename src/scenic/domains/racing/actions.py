"""Racing actions.

Racing domain actions extending the driving domain:
- SetMaxSpeedAction: set an agent's maximum allowed speed
- SetTTLAction: set the agent's TTL (target line to drive on)
- SetGearAction: set the current gear (0-6)
- PressClutchAction: press the clutch pedal
- ReleaseClutchAction: release the clutch pedal

The gear and clutch actions follow the same protocol pattern as Steers from
the driving domain - simulators implement the protocol methods, actions call them.
"""

from scenic.core.simulators import Action


## Mixin protocol for racing-specific vehicle control

class HasManualTransmission:
    """Mixin protocol for agents with manual transmission control.
    
    Racing cars may support manual gear changes and clutch control.
    Simulators should implement these methods to enable gear/clutch actions.
    """
    
    def setGear(self, gear):
        """Set gear to specific value (0-6). 0=Neutral, 1-6=Gears."""
        raise NotImplementedError
    
    def setClutch(self, clutch):
        """Set clutch pedal position (0.0=released, 1.0=fully pressed)."""
        raise NotImplementedError


## Racing-specific actions

class SetMaxSpeedAction(Action):
    """Set the maximum allowed speed for a racing car (in m/s)."""

    def __init__(self, max_speed: float):
        self.max_speed = float(max_speed)

    def applyTo(self, obj, sim):
        # Prefer simulator hook if provided, otherwise set property directly
        if hasattr(obj, 'setMaxSpeed'):
            obj.setMaxSpeed(self.max_speed)
        else:
            obj.maxSpeed = self.max_speed


class SetTTLAction(Action):
    """Set the car's TTL (target line to drive on).

    The TTL can be any Region-like object supporting signedDistanceTo, such as
    the centerline of a lane or a custom region approximating a racing line.
    """

    def __init__(self, ttl):
        self.ttl = ttl

    def applyTo(self, obj, sim):
        # Prefer simulator hook if provided, otherwise set property directly
        if hasattr(obj, 'setTTL'):
            obj.setTTL(self.ttl)
        else:
            obj.ttl = self.ttl


class SetGearAction(Action):
    """Set gear to a specific value (racing domain action).
    
    This action changes gears directly. The simulator handles the transmission logic.
    
    **Note**: To start from neutral (gear 0 → gear 1), you may need to use
    PressClutchAction/ReleaseClutchAction. For gear changes while moving (1→2→3, etc.),
    just use SetGearAction directly.
    
    Args:
        gear: Gear number (0-6)
            0 = Neutral
            1-6 = Gears 1-6
    """
    
    def __init__(self, gear: int):
        self.gear = int(max(0, min(6, gear)))
    
    def applyTo(self, obj, sim):
        """Apply gear change via HasManualTransmission protocol."""
        if hasattr(obj, 'setGear'):
            obj.setGear(self.gear)
        else:
            # Fallback: set property directly
            obj.gear = self.gear


class PressClutchAction(Action):
    """Press clutch pedal (racing domain action).
    
    This is a one-shot action that presses the clutch pedal once.
    
    **Primary use case**: Starting from neutral (gear 0 → gear 1)
    - Press clutch when in neutral
    - Use SetGearAction(1) to engage 1st gear  
    - Release clutch to start moving
    
    **Note**: Clutch is typically NOT needed for gear changes while moving (1→2→3, etc.).
    Use SetGearAction directly for those.
    """
    
    def applyTo(self, obj, sim):
        """Press clutch via HasManualTransmission protocol."""
        if hasattr(obj, 'setClutch'):
            obj.setClutch(1.0)  # Fully pressed
        else:
            # Fallback: set property directly
            obj.clutch = 1.0


class ReleaseClutchAction(Action):
    """Release clutch pedal (racing domain action).
    
    This is a one-shot action that releases the clutch pedal once.
    
    **Primary use case**: Completing the start from neutral (gear 0 → gear 1)
    - After pressing clutch and engaging 1st gear
    - Release clutch to begin moving
    
    Pairs with PressClutchAction for starting the vehicle from neutral.
    """
    
    def applyTo(self, obj, sim):
        """Release clutch via HasManualTransmission protocol."""
        if hasattr(obj, 'setClutch'):
            obj.setClutch(0.0)  # Fully released
        else:
            # Fallback: set property directly
            obj.clutch = 0.0

