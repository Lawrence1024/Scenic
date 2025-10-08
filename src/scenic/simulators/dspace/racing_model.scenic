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

