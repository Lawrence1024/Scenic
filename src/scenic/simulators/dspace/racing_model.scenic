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
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction

import scenic.simulators.dspace as dspace
from scenic.simulators.dspace.actions import *

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
class DSPACERacingCar(RacingCar):
    """dSPACE implementation of racing car with racing-specific systems.
    
    This class implements the abstract RacingSteers protocol with dSPACE-specific
    control mechanisms.
    """
    
    # dSPACE-specific properties
    dspaceActor: None  # Link to dSPACE internal representation
    routeId: None      # dSPACE route identifier
    
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

# Replace the abstract RacingCar with dSPACE implementation
RacingCar = DSPACERacingCar
