from scenic.domains.driving.model import *
from scenic.domains.driving.actions import *
from scenic.domains.driving.behaviors import *

import scenic.simulators.dspace as dspace

# dSPACE ModelDesk parameters
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
param timestep = 0.1

# Configure the dSPACE simulator
simulator dspace.DSpaceSimulator(
    scenario_src="LagunaSeca_ExternalControl",
    scenario_name=None,
    timestep=0.1,
)