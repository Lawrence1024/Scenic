# -*- coding: utf-8 -*-
"""MAPort session: create and configure MAPort for variable read/write.

Session control (go online, start maneuver, stepping) remains in ControlDesk;
this module only prepares the MAPort instance for get_var/set_var.
"""

import os

from .connection import MAPortApp


def connect_and_prepare_maport(sim=None, config_path=None, start_if_needed=True):
    """Create and configure MAPort for variable access.

    Args:
        sim: Optional DSpaceSimulation (used only to resolve timestep/config if needed).
        config_path: Path to MAPortConfig VEOS XML. If None, uses default next to this module.
        start_if_needed: If True, call StartSimulation() when state is not already running.

    Returns:
        MAPortApp instance with connect() already called, ready for get_var/set_var.
        None if creation or configure fails.
    """
    if config_path is None:
        this_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(this_dir, "MAPortConfigVEOS.xml")
    if not os.path.isfile(config_path):
        print("[MAPort] Config file not found: %s" % config_path)
        return None
    try:
        app = MAPortApp(config_path)
        app.connect(start_if_needed=start_if_needed)
        return app
    except Exception as e:
        print("[MAPort] Connection failed: %s" % e)
        return None
