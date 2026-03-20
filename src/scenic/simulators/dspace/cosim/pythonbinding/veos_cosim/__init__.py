"""
VeosCoSim client — Python layer for dSPACE VEOS co-simulation.

The native extension ``veos_cosim._veos_cosim`` links ``VeosCoSimAppl`` and exposes
:class:`CoSimClient` for connect / run (time trigger) workflows.
"""

from __future__ import annotations

from ._veos_cosim import CoSimClient
from ._veos_cosim import __version__

__all__ = ["CoSimClient", "__version__"]
