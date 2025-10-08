"""Racing-specific actions for dynamic agents.

These actions extend the driving domain actions with racing-specific
maneuvers and controls.
"""

from scenic.core.simulators import Action
from scenic.domains.driving.actions import *


class DRSAction(Action):
    """Activate or deactivate DRS (Drag Reduction System).
    
    DRS is a system used in Formula 1 and other racing series that reduces
    drag to increase straight-line speed. It can only be used in designated
    DRS zones and when within range of the car ahead.
    
    Args:
        activate: True to activate DRS, False to deactivate
    """
    
    def __init__(self, activate: bool = True):
        self.activate = activate
    
    def applyTo(self, obj, sim):
        # This would be implemented by the simulator
        # For now, it could increase top speed temporarily
        if hasattr(obj, 'setDRS'):
            obj.setDRS(self.activate)
        else:
            # Fallback: slightly increase throttle when DRS is active
            if self.activate and hasattr(obj, 'setThrottle'):
                obj.setThrottle(1.1)  # 10% bonus (capped by simulator)


class ERSDeployAction(Action):
    """Deploy ERS (Energy Recovery System) power boost.
    
    ERS is used in modern racing to provide temporary power boosts.
    The system has limited energy per lap, so strategic deployment is key.
    
    Args:
        mode: Deployment mode ('hotlap', 'overtake', 'defend', 'conserve')
        amount: Power deployment amount (0.0 to 1.0)
    """
    
    def __init__(self, mode: str = 'hotlap', amount: float = 1.0):
        self.mode = mode
        self.amount = max(0.0, min(1.0, amount))
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'deployERS'):
            obj.deployERS(self.mode, self.amount)
        else:
            # Fallback: temporary throttle boost
            if hasattr(obj, 'setThrottle'):
                boost = 1.0 + (0.2 * self.amount)  # Up to 20% boost
                obj.setThrottle(boost)


class TractionControlAction(Action):
    """Adjust traction control setting.
    
    Traction control helps prevent wheel spin during acceleration,
    especially important in wet conditions or when exiting corners.
    
    Args:
        level: TC level (0 = off, 1-12 = varying intervention levels)
    """
    
    def __init__(self, level: int = 5):
        self.level = max(0, min(12, level))
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setTractionControl'):
            obj.setTractionControl(self.level)


class BrakeBiasAction(Action):
    """Adjust brake bias (front/rear brake balance).
    
    Brake bias affects handling during braking. More front bias prevents
    rear lockup, more rear bias helps rotate the car into corners.
    
    Args:
        bias: Brake bias (0.0 = all rear, 1.0 = all front, 0.5 = balanced)
    """
    
    def __init__(self, bias: float = 0.55):
        self.bias = max(0.0, min(1.0, bias))
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setBrakeBias'):
            obj.setBrakeBias(self.bias)


class DifferentialAction(Action):
    """Adjust differential setting.
    
    The differential controls how power is distributed between wheels,
    affecting corner entry and exit behavior.
    
    Args:
        entry: Diff setting on corner entry (0-100%)
        mid: Diff setting mid-corner (0-100%)
        exit: Diff setting on corner exit (0-100%)
    """
    
    def __init__(self, entry: float = 30, mid: float = 50, exit: float = 70):
        self.entry = max(0.0, min(100.0, entry))
        self.mid = max(0.0, min(100.0, mid))
        self.exit = max(0.0, min(100.0, exit))
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setDifferential'):
            obj.setDifferential(self.entry, self.mid, self.exit)


class PitLimiterAction(Action):
    """Activate or deactivate pit speed limiter.
    
    The pit limiter ensures the car doesn't exceed the pit lane speed limit,
    which is strictly enforced in racing.
    
    Args:
        activate: True to activate limiter, False to deactivate
    """
    
    def __init__(self, activate: bool = True):
        self.activate = activate
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setPitLimiter'):
            obj.setPitLimiter(self.activate)
        else:
            # Fallback: limit speed if activated
            if self.activate and hasattr(obj, 'setSpeed'):
                # Typical pit lane limit is ~60-80 km/h = ~17-22 m/s
                obj.setSpeed(min(obj.speed, 20.0))


class FormationHoldAction(Action):
    """Hold position during formation lap.
    
    Used to maintain grid spacing during formation laps before the race start.
    
    Args:
        target_distance: Distance to maintain from car ahead (meters)
        target_car: The car to follow (typically the one ahead on grid)
    """
    
    def __init__(self, target_distance: float = 8.0, target_car=None):
        self.target_distance = target_distance
        self.target_car = target_car
    
    def applyTo(self, obj, sim):
        if self.target_car is None:
            # Just maintain formation speed
            if hasattr(obj, 'setSpeed'):
                obj.setSpeed(15.0)  # ~54 km/h formation speed
        else:
            # Adjust speed to maintain distance
            from scenic.core.vectors import Vector
            current_distance = (obj.position - self.target_car.position).norm()
            
            error = current_distance - self.target_distance
            
            # Simple proportional control
            speed_adjust = -0.5 * error  # Adjust speed based on distance error
            target_speed = 15.0 + speed_adjust
            target_speed = max(10.0, min(20.0, target_speed))  # Clamp
            
            if hasattr(obj, 'setSpeed'):
                obj.setSpeed(target_speed)


class OvertakeAction(Action):
    """Attempt an overtaking maneuver.
    
    This action combines multiple controls to execute an overtake:
    - Lateral movement to the side
    - Increased throttle
    - Possible DRS/ERS deployment
    
    Args:
        target_car: The car to overtake
        side: 'left' or 'right' - which side to pass on
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    def __init__(self, target_car, side: str = 'left', aggressive: bool = False):
        self.target_car = target_car
        self.side = side
        self.aggressive = aggressive
    
    def applyTo(self, obj, sim):
        # This is a complex action that would need proper implementation
        # For now, just increase throttle and steer to the side
        if hasattr(obj, 'setThrottle'):
            obj.setThrottle(1.0)
        
        if hasattr(obj, 'setSteering'):
            # Steer left or right
            steer_amount = 0.3 if self.side == 'left' else -0.3
            obj.setSteering(steer_amount)
        
        # If aggressive, deploy boost systems
        if self.aggressive:
            if hasattr(obj, 'deployERS'):
                obj.deployERS('overtake', 1.0)
            if hasattr(obj, 'setDRS'):
                obj.setDRS(True)


class DefendPositionAction(Action):
    """Defend racing position from overtaking attempts.
    
    Adjusts the racing line to defend from cars behind while staying legal
    (one defensive move allowed in racing).
    
    Args:
        defending_side: 'left' or 'right' - which side to defend
        firmness: How firmly to defend (0.0-1.0)
    """
    
    def __init__(self, defending_side: str = 'left', firmness: float = 0.7):
        self.defending_side = defending_side
        self.firmness = max(0.0, min(1.0, firmness))
    
    def applyTo(self, obj, sim):
        # Move to defensive line
        if hasattr(obj, 'setSteering'):
            steer_amount = 0.2 * self.firmness
            if self.defending_side == 'right':
                steer_amount = -steer_amount
            obj.setSteering(steer_amount)
        
        # Maintain steady speed to hold position
        if hasattr(obj, 'setThrottle'):
            obj.setThrottle(0.7)


class SlipstreamAction(Action):
    """Position car in slipstream of car ahead for reduced drag.
    
    Slipstreaming (drafting) reduces drag and increases top speed by following
    closely behind another car.
    
    Args:
        target_car: The car to slipstream
        distance: Following distance in meters (typically 1-3m)
    """
    
    def __init__(self, target_car, distance: float = 2.0):
        self.target_car = target_car
        self.distance = distance
    
    def applyTo(self, obj, sim):
        # Position behind target car at specified distance
        # This would need proper implementation with the simulator
        if hasattr(obj, 'setThrottle'):
            # Adjust throttle to maintain distance
            current_distance = (obj.position - self.target_car.position).norm()
            error = current_distance - self.distance
            
            # Simple control - speed up if too far, slow down if too close
            if error > 1.0:
                obj.setThrottle(0.9)
            elif error < -0.5:
                obj.setThrottle(0.3)
                if hasattr(obj, 'setBraking'):
                    obj.setBraking(0.5)
            else:
                obj.setThrottle(0.7)

