from .log import log, DEBUG_ENABLED

# Temporary compatibility layer: re-export legacy utils surface so that
# 'from scenic.simulators.dspace import utils as dutils' continues to work.
try:
    from .legacy import *  # noqa: F401,F403
except Exception:
    pass


