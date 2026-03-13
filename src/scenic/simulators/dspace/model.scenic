"""dSPACE-specific racing model.

This model extends the racing domain with dSPACE/ModelDesk simulator support.
It implements the abstract racing protocols defined in the racing domain.

Usage::

    param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
    param use2DMap = True
    param trackDirection = 'counterclockwise'
    model scenic.simulators.dspace.racing_model
"""

# Import racing domain model (which imports driving domain)
from scenic.domains.racing.model import *
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction, HasManualTransmission, RacingSteers

# Import dSPACE-specific components
import scenic.simulators.dspace as dspace
from scenic.simulators.dspace.actions import _DSpaceVehicle
from scenic.domains.driving.actions import Steers

# dSPACE ModelDesk parameters
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
# Support both time_step (from examples) and timestep (from model) for compatibility
param timestep = (globalParameters.time_step if 'time_step' in globalParameters else (globalParameters.timestep if 'timestep' in globalParameters else 1.0))
# Period between control/readback updates in seconds. Must be a multiple of timestep.
# None or omit = every step. Example: timestep=0.01, control_period=0.05 → 20 Hz control and readback
param control_period = (globalParameters.control_period if 'control_period' in globalParameters else None)
# scenic_control: default True = racing library sends control signals (Scenic controls ego).
# Set to False for external control (e.g. baseline from external_control_baseline.json).
param scenic_control = (globalParameters.scenic_control if 'scenic_control' in globalParameters else True)

# Configure the dSPACE simulator
simulator dspace.DSpaceSimulator(
    scenario_src=globalParameters.scenario_src,
    scenario_name=globalParameters.scenario_name,
    timestep=globalParameters.timestep,
    control_period=globalParameters.control_period,
    scenic_control=globalParameters.scenic_control,
)

# dSPACE-specific racing car implementation
class DSPACERacingCar(RacingCar, _DSpaceVehicle, Steers, HasManualTransmission, RacingSteers):
    """dSPACE implementation of racing car with racing-specific systems.
    
    This class implements:
    - RacingCar: Racing domain car with racing-specific behaviors
    - _DSpaceVehicle: Marker for dSPACE-specific actions
    - Steers: Protocol for standard driving domain steering actions
    - HasManualTransmission: Protocol for gear and clutch control
    - RacingSteers: Protocol for racing decision tree actions
    
    Based on the IAC AV-24 (Dallara AV chassis) specifications:
    - Length: 4.80 m (189 inches)
    - Width: 1.93 m (76 inches)
    - Height: 0.97 m (38 inches)
    """
    
    # IAC AV-24 physical dimensions
    length: 4.80  # meters (189 inches)
    width: 1.93   # meters (76 inches)
    height: 0.97  # meters (38 inches)
    
    # dSPACE-specific properties
    dspaceActor: None  # Link to dSPACE internal representation
    routeId: None      # dSPACE route identifier
    # Optional: (s,t) placement relative to ego (racing-library semantics). When set, fellow (s,t) = ego (s,t) + offset
    # instead of projecting world position. Use for "ahead/behind" (keep t, move s) or "left/right" (keep s, move t).
    # - (delta_s, delta_t) in meters: e.g. (5, 0) = 5 m ahead; (0, -2) = 2 m right (t<0 = right in road direction).
    # - Or ('ahead'|'behind'|'left'|'right', distance): e.g. ('ahead', 5), ('right', 2).
    _racing_st_offset: None
    
    # Racing-specific methods
    def setMaxSpeed(self, max_speed):
        # Persist on object for behaviors and simulator control loop
        self.maxSpeed = max_speed
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.set_control({'max_speed': float(max_speed)})
    
    def setTTL(self, ttl):
        # Persist on object for behaviors and control computation
        self.ttl = ttl
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            # TTL is a Scenic-side concept; we forward a named handle if needed
            self.dspaceActor.set_control({'ttl_set': True})
    
    # Steers protocol implementation (for driving domain actions)
    def setThrottle(self, throttle):
        """Set throttle using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['throttle'] = float(throttle)
    
    def setSteering(self, steering):
        """Set steering using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['steering'] = float(steering)
    
    def setBraking(self, braking):
        """Set braking using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['braking'] = float(braking)
    
    def setHandbrake(self, handbrake):
        """Set handbrake (not implemented in dSPACE yet)."""
        pass
    
    def setReverse(self, reverse):
        """Set reverse gear (not implemented in dSPACE yet)."""
        pass
    
    # HasManualTransmission protocol implementation (for racing domain actions)
    def setGear(self, gear):
        if getattr(self, '_debug_transmission', False):
            print(f"[DSPACERacingCar.setGear] Called with gear={gear}")
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        # remove previous pending gear action, keep only latest
        self._oneshot_actions = [a for a in self._oneshot_actions if a[0] != 'gear']
        self._oneshot_actions.append(('gear', int(gear)))

    def setClutch(self, clutch):
        if getattr(self, '_debug_transmission', False):
            print(f"[DSPACERacingCar.setClutch] Called with clutch={clutch}")
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        # remove previous pending clutch action, keep only latest
        self._oneshot_actions = [a for a in self._oneshot_actions if a[0] != 'clutch']
        self._oneshot_actions.append(('clutch', float(clutch)))
    
    # RacingSteers protocol implementation (for decision tree actions)
    def setSpeedLimit(self, speed_limit):
        """Set speed limit using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setSpeedLimit] Called with speed_limit={speed_limit}")
        # Store speed limit and update maxSpeed
        self.maxSpeed = float(speed_limit)
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.speed_limit = float(speed_limit)
            self.dspaceActor.set_control({'speed_limit': float(speed_limit)})
    
    def setTTLSelection(self, selection):
        """Set TTL selection using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setTTLSelection] Called with selection={selection}")
        # Store TTL selection in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.ttl_selection = selection
            self.dspaceActor.set_control({'ttl_selection': selection})
        
        # Map selection to actual TTL region (requires track context)
        # This is a simplified version - full implementation would need track TTL indices
        # For now, we store the selection and let behaviors handle the mapping
        self.ttl_selection = selection
    
    def setTargetGap(self, gap):
        """Set target gap using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setTargetGap] Called with gap={gap}")
        # Store target gap in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.target_gap = float(gap)
            self.dspaceActor.set_control({'target_gap': float(gap)})
    
    def setStrategy(self, strategy_type):
        """Set strategy using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setStrategy] Called with strategy_type={strategy_type}")
        # Store strategy type in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.strategy_type = strategy_type
            self.dspaceActor.set_control({'strategy_type': strategy_type})
    
    def setPowertrainMode(self, mode):
        """Set powertrain mode using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setPowertrainMode] Called with mode={mode}")
        # Store powertrain mode in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.powertrain_mode = mode
            self.dspaceActor.set_control({'powertrain_mode': mode})
    
    def setScaleFactor(self, scale_factor):
        """Set scale factor using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setScaleFactor] Called with scale_factor={scale_factor}")
        # Store scale factor in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.scale_factor = float(scale_factor)
            self.dspaceActor.set_control({'scale_factor': float(scale_factor)})
    
    def setPush2Pass(self, active):
        """Set push2pass using RacingSteers protocol."""
        print(f"[DSPACERacingCar.setPush2Pass] Called with active={active}")
        # Store push2pass state in dspaceActor
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.push2pass_active = bool(active)
            self.dspaceActor.set_control({'push2pass_active': bool(active)})

# Replace the abstract RacingCar with dSPACE implementation
RacingCar = DSPACERacingCar
