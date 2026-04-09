"""Racing actions.

Racing domain actions extending the driving domain:
- SetMaxSpeedAction: set an agent's maximum allowed speed
- SetTTLAction: set the agent's TTL (target line to drive on)
- SetGearAction: set the current gear (0-6)
- PressClutchAction: press the clutch pedal
- ReleaseClutchAction: release the clutch pedal
- SetSpeedLimitAction: set speed limit based on speed type
- SetTTLSelectionAction: select TTL (left/right/race/optimal/pit)
- SetTargetGapAction: set target following gap
- SetStrategyAction: set driving strategy (cruise_control/follow_mode)
- SetPowertrainModeAction: set powertrain mode
- SetScaleFactorAction: apply speed scale factor
- SetPush2PassAction: activate/deactivate push2pass
- StopCarAction: emergency/immediate/safe stop
- SetFellowPlantAction: stage fellow traffic (v, d) plant commands for dSPACE External_Signals

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


class RacingSteers:
    """Extended protocol for racing-specific vehicle control.
    
    This protocol extends the standard Steers protocol with racing-specific
    controls like speed limits, TTL selection, gap management, and strategy modes.
    Simulators should implement these methods to enable racing decision tree actions.
    """
    
    def setSpeedLimit(self, speed_limit):
        """Set maximum speed limit (m/s)."""
        raise NotImplementedError
    
    def setTTLSelection(self, selection):
        """Select TTL: 'left', 'right', 'race', 'optimal', 'pit'."""
        raise NotImplementedError
    
    def setTargetGap(self, gap):
        """Set target following gap (meters)."""
        raise NotImplementedError
    
    def setStrategy(self, strategy_type):
        """Set strategy: 'cruise_control' or 'follow_mode'."""
        raise NotImplementedError
    
    def setPowertrainMode(self, mode):
        """Set powertrain mode: 'pit_lane', 'quiet', 'nominal', 'race', 'overboost'."""
        raise NotImplementedError
    
    def setScaleFactor(self, scale_factor):
        """Apply speed scale factor (0.0-1.0, multiplier for speed)."""
        raise NotImplementedError
    
    def setPush2Pass(self, active):
        """Activate/deactivate push2pass."""
        raise NotImplementedError


class HasFellowPlant:
    """Mixin protocol for traffic agents driven by route-relative **v** and **d** (Frenet **t**).

    Same structural idea as :class:`~scenic.domains.driving.actions.Steers` for ego: simulators
    implement a setter that stages per-step commands. Fellow plant output is mirrored in
    ``agent._fellow_plant_state`` with keys ``v_kmh`` and ``d_m`` (see
    :mod:`scenic.domains.racing.fellow.commands`).
    """

    def setFellowPlant(self, v_kmh: float, d_m: float):
        """Command longitudinal speed (km/h) and lateral offset **d** in meters (same as **t**)."""
        raise NotImplementedError


class FellowPlantAction(Action):
    """Abstract base for actions that command the fellow (v, d) plant."""

    def canBeTakenBy(self, agent):
        return hasattr(agent, "setFellowPlant")


class SetFellowPlantAction(FellowPlantAction):
    """Stage fellow plant commands (km/h lateral speed, **d** in m = Frenet **t**).

    Same role for traffic as :class:`~scenic.domains.driving.actions.SetThrottleAction` for ego:
    ``applyTo`` calls :meth:`HasFellowPlant.setFellowPlant`, which updates ``_fellow_plant_state``.
    """

    def __init__(self, v_kmh: float, d_m: float):
        self.v_kmh = float(v_kmh)
        self.d_m = float(d_m)

    def applyTo(self, obj, sim):
        obj.setFellowPlant(self.v_kmh, self.d_m)


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


## Decision tree actions (for race decision engine integration)

class RacingAction(Action):
    """Abstract class for actions usable by agents which implement RacingSteers protocol.
    
    Note: This uses hasattr checks since protocols are mixins, not base classes.
    """
    
    def canBeTakenBy(self, agent):
        # Check if agent has the required methods (protocol implementation)
        return (hasattr(agent, 'setSpeedLimit') and
                hasattr(agent, 'setTTLSelection') and
                hasattr(agent, 'setTargetGap') and
                hasattr(agent, 'setStrategy'))

class SetSpeedLimitAction(RacingAction):
    """Set speed limit based on speed type (decision tree action).
    
    This action sets the maximum speed limit for the vehicle based on the
    speed type (e.g., "pit_crawl", "pit_lane", "yellow", "green", etc.).
    
    Args:
        speed_limit: Speed limit in m/s
        speed_type: Speed type string (optional, for tracking)
    """
    
    def __init__(self, speed_limit: float, speed_type: str = None):
        self.speed_limit = float(speed_limit)
        self.speed_type = speed_type
    
    def applyTo(self, obj, sim):
        """Set speed limit via RacingSteers protocol."""
        if hasattr(obj, 'setSpeedLimit'):
            obj.setSpeedLimit(self.speed_limit)
        else:
            # Fallback: set maxSpeed property
            obj.maxSpeed = self.speed_limit


class SetTTLSelectionAction(RacingAction):
    """Select TTL (target trajectory line) based on decision tree logic.
    
    This action selects which TTL to follow: left (defender), right (attacker),
    race (optimal), optimal, or pit.
    
    Args:
        selection: TTL selection string - "left", "right", "race", "optimal", or "pit"
    """
    
    def __init__(self, selection: str):
        if selection not in ["left", "right", "race", "optimal", "pit"]:
            raise ValueError(f"Invalid TTL selection: {selection}. Must be one of: left, right, race, optimal, pit")
        self.selection = selection
    
    def applyTo(self, obj, sim):
        """Set TTL selection via RacingSteers protocol."""
        if hasattr(obj, 'setTTLSelection'):
            obj.setTTLSelection(self.selection)
        else:
            # Fallback: Try to map to TTL region if available
            # This would require track context, so we just store the selection
            if not hasattr(obj, 'ttl_selection'):
                obj.ttl_selection = self.selection
            else:
                obj.ttl_selection = self.selection


class SetTargetGapAction(RacingAction):
    """Set target following gap (decision tree action).
    
    This action sets the target gap distance for following behavior.
    
    Args:
        gap: Target gap in meters
        gap_type: Gap type string (optional, for tracking) - "no_gap", "attacker_preparing", etc.
    """
    
    def __init__(self, gap: float, gap_type: str = None):
        self.gap = float(gap)
        self.gap_type = gap_type
    
    def applyTo(self, obj, sim):
        """Set target gap via RacingSteers protocol."""
        if hasattr(obj, 'setTargetGap'):
            obj.setTargetGap(self.gap)
        else:
            # Fallback: store as property
            obj.target_gap = self.gap


class SetStrategyAction(RacingAction):
    """Set driving strategy (decision tree action).
    
    This action sets the driving strategy mode: cruise_control or follow_mode.
    
    Args:
        strategy_type: Strategy string - "cruise_control" or "follow_mode"
    """
    
    def __init__(self, strategy_type: str):
        if strategy_type not in ["cruise_control", "follow_mode"]:
            raise ValueError(f"Invalid strategy type: {strategy_type}. Must be 'cruise_control' or 'follow_mode'")
        self.strategy_type = strategy_type
    
    def applyTo(self, obj, sim):
        """Set strategy via RacingSteers protocol."""
        if hasattr(obj, 'setStrategy'):
            obj.setStrategy(self.strategy_type)
        else:
            # Fallback: store as property
            obj.strategy_type = self.strategy_type


class SetPowertrainModeAction(RacingAction):
    """Set powertrain mode (decision tree action).
    
    This action sets the powertrain operating mode.
    
    Args:
        mode: Powertrain mode string - "pit_lane", "quiet", "nominal", "race", "overboost"
    """
    
    def __init__(self, mode: str):
        valid_modes = ["pit_lane", "quiet", "nominal", "race", "overboost"]
        if mode not in valid_modes:
            raise ValueError(f"Invalid powertrain mode: {mode}. Must be one of: {valid_modes}")
        self.mode = mode
    
    def applyTo(self, obj, sim):
        """Set powertrain mode via RacingSteers protocol."""
        if hasattr(obj, 'setPowertrainMode'):
            obj.setPowertrainMode(self.mode)
        else:
            # Fallback: store as property
            obj.powertrain_mode = self.mode


class SetScaleFactorAction(RacingAction):
    """Apply speed scale factor (decision tree action).
    
    This action applies a scale factor to the speed limit, typically used
    for region-based speed modifiers (e.g., slower in turns).
    
    Args:
        scale_factor: Scale factor (0.0-1.0, multiplier for speed)
    """
    
    def __init__(self, scale_factor: float):
        self.scale_factor = float(max(0.0, min(1.0, scale_factor)))
    
    def applyTo(self, obj, sim):
        """Set scale factor via RacingSteers protocol."""
        if hasattr(obj, 'setScaleFactor'):
            obj.setScaleFactor(self.scale_factor)
        else:
            # Fallback: store as property
            obj.scale_factor = self.scale_factor


class SetPush2PassAction(RacingAction):
    """Activate/deactivate push2pass (decision tree action).
    
    This action activates or deactivates the push2pass system.
    
    Args:
        active: Boolean indicating if push2pass should be active
    """
    
    def __init__(self, active: bool):
        self.active = bool(active)
    
    def applyTo(self, obj, sim):
        """Set push2pass via RacingSteers protocol."""
        if hasattr(obj, 'setPush2Pass'):
            obj.setPush2Pass(self.active)
        else:
            # Fallback: store as property
            obj.push2pass_active = self.active


class StopCarAction(RacingAction):
    """Stop car with specified stop type (decision tree action).
    
    This action stops the car with emergency, immediate, or safe stop behavior.
    Composes brake and throttle actions.
    
    Args:
        stop_type: Stop type string - "emergency", "immediate", or "safe"
    """
    
    def __init__(self, stop_type: str = "safe"):
        if stop_type not in ["emergency", "immediate", "safe"]:
            raise ValueError(f"Invalid stop type: {stop_type}. Must be 'emergency', 'immediate', or 'safe'")
        self.stop_type = stop_type
    
    def applyTo(self, obj, sim):
        """Apply stop via RacingSteers protocol and driving actions."""
        # Apply maximum brake and zero throttle
        from scenic.domains.driving.actions import SetBrakeAction, SetThrottleAction
        SetBrakeAction(1.0).applyTo(obj, sim)
        SetThrottleAction(0.0).applyTo(obj, sim)
        
        # Set speed limit to zero
        if hasattr(obj, 'setSpeedLimit'):
            obj.setSpeedLimit(0.0)
        
        # Store stop type for tracking
        obj.stop_type = self.stop_type

