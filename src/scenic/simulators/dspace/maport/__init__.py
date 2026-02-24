# -*- coding: utf-8 -*-
"""MAPort (XIL API) variable access for dSPACE simulator.

Use connection.MAPortApp for get_var/set_var; use session.connect_and_prepare_maport
to create a configured instance. Session control (go online, start maneuver, step)
remains in controldesk.
"""

from .connection import MAPortApp
from . import session

__all__ = ["MAPortApp", "session"]
