# scenic/simulators/dspace/model.scenic

# Pull in the Driving world model's classes/behaviors (Car, road, Drive…)
# import scenic.domains.driving.model as driving
from scenic.domains.driving.model import *
from scenic.domains.driving.actions import *
from scenic.domains.driving.behaviors import *

# Select the dSPACE backend (Option C: AURELION time barrier)
import scenic.simulators.dspace as dspace
simulator dspace.DSpaceSimulator(
    scenario_src="LagunaSeca_ExternalControl",
    timestep=0.05,
    aurl_base="http://localhost:8585",
    clone_scenario=True
)



# (Nothing else here; world-model files must not use the `model` statement.)
