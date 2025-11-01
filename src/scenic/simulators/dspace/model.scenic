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
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction, HasManualTransmission

# Import dSPACE-specific components
import scenic.simulators.dspace as dspace
from scenic.simulators.dspace.actions import _DSpaceVehicle
from scenic.domains.driving.actions import Steers

# dSPACE ModelDesk parameters
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
param timestep = 0.1

# Configure the dSPACE simulator
simulator dspace.DSpaceSimulator(
    scenario_src=globalParameters.scenario_src,
    scenario_name=globalParameters.scenario_name,
    timestep=globalParameters.timestep,
)

# dSPACE-specific racing car implementation
class DSPACERacingCar(RacingCar, _DSpaceVehicle, Steers, HasManualTransmission):
    """dSPACE implementation of racing car with racing-specific systems.
    
    This class implements:
    - RacingCar: Racing domain car with racing-specific behaviors
    - _DSpaceVehicle: Marker for dSPACE-specific actions
    - Steers: Protocol for standard driving domain steering actions
    - HasManualTransmission: Protocol for gear and clutch control
    
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
        print(f"[DSPACERacingCar.setThrottle] Called with throttle={throttle}")
        # Store for later application via simulator
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['throttle'] = float(throttle)
        print(f"[DSPACERacingCar.setThrottle] Stored in _control_state: {self._control_state}")
    
    def setSteering(self, steering):
        """Set steering using driving domain protocol."""
        print(f"[DSPACERacingCar.setSteering] Called with steering={steering}")
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['steering'] = float(steering)
        print(f"[DSPACERacingCar.setSteering] Stored in _control_state: {self._control_state}")
    
    def setBraking(self, braking):
        """Set braking using driving domain protocol."""
        print(f"[DSPACERacingCar.setBraking] Called with braking={braking}")
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['braking'] = float(braking)
        print(f"[DSPACERacingCar.setBraking] Stored in _control_state: {self._control_state}")
    
    def setHandbrake(self, handbrake):
        """Set handbrake (not implemented in dSPACE yet)."""
        pass
    
    def setReverse(self, reverse):
        """Set reverse gear (not implemented in dSPACE yet)."""
        pass
    
    # HasManualTransmission protocol implementation (for racing domain actions)
    def setGear(self, gear):
        """Set gear using racing domain protocol."""
        print(f"[DSPACERacingCar.setGear] Called with gear={gear}")
        # Store as one-shot action for immediate application
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        self._oneshot_actions.append(('gear', int(gear)))
    
    def setClutch(self, clutch):
        """Set clutch using racing domain protocol."""
        print(f"[DSPACERacingCar.setClutch] Called with clutch={clutch}")
        # Store as one-shot action for immediate application
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        self._oneshot_actions.append(('clutch', float(clutch)))

# Replace the abstract RacingCar with dSPACE implementation
RacingCar = DSPACERacingCar
