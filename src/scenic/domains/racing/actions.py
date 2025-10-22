"""Racing actions (minimal set).

This module intentionally keeps the racing action surface small:
- SetMaxSpeedAction: set an agent's maximum allowed speed.
- SetTTLAction: set the agent's TTL (target line to drive on).
"""

from scenic.core.simulators import Action


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

