"""dSPACE-specific racing model.

This model combines the racing domain with dSPACE/ModelDesk simulator support.

Usage::

    param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
    param use2DMap = True
    param trackDirection = 'counterclockwise'
    model scenic.simulators.dspace.racing_model
"""

# Import racing domain model (which imports driving domain)
from scenic.domains.racing.model import *

import scenic.simulators.dspace as dspace

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
