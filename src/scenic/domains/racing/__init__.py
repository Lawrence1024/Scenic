"""Domain for racing scenarios on closed-circuit race tracks.

The racing domain extends the :doc:`driving domain <scenic.domains.driving>` with 
racing-specific features such as:

* **Racing tracks** - Closed-loop circuits with defined direction
* **Pit lanes** - Special lanes for pit stops, separate from racing lanes
* **Racing lines** - Optimal paths through corners
* **Starting grid** - Formation positions for race starts
* **Track limits** - Boundaries that must not be exceeded

Traffic fellow helpers for dSPACE-style (v, d) plants live under
:mod:`scenic.domains.racing.fellow` (submodules ``plant`` and ``commands``).

The :doc:`world model <scenic.domains.racing.model>` defines Scenic classes for racing
cars, pit crews, track marshals, etc., as well as racing-specific behaviors like
following the racing line, executing pit stops, and overtaking maneuvers.

Scenarios for the racing domain should import the model as follows::

    model scenic.domains.racing.model

Racing scenarios inherit all features from the driving domain but add racing-specific
constraints:

* Tracks are typically one-way (no opposing traffic)
* Pit lanes have speed limits and entry/exit rules
* Track direction is enforced
* Racing lines optimize for speed rather than safety

Example racing scenario::

    param map = localPath('../../assets/maps/dSPACE/LGS_v1.xodr')
    param use2DMap = True
    model scenic.domains.racing.model
    
    # Create cars on track (use mainTrack or pitTrack for placement)
    ego = new RacingCar on mainTrack
    opponent1 = new RacingCar on mainTrack
    opponent2 = new RacingCar on mainTrack
    
    # Follow racing line behavior
    ego with behavior FollowRacingLineMPCBehavior()

The racing domain is designed to work with simulators that inherit from
:class:`RacingSimulator`:

* **dSPACE ModelDesk** - Laguna Seca and other racing circuits
  (see :doc:`scenic.simulators.dspace.racing_model`)
* **CARLA** - Racing tracks and circuits
* **Any simulator** supporting the driving domain with racing-specific maps

Scenarios written for the racing domain should work without changes in any
simulator that implements the racing domain interface. For example, the
:file:`examples/racing/three_segments.scenic` scenario can be run in:

* dSPACE ModelDesk (with proper route setup):

    .. code-block:: console

        $ scenic --2d --model scenic.simulators.dspace.racing_model \\
            examples/racing/three_segments.scenic --simulate

* CARLA (with racing track maps):

    .. code-block:: console

        $ scenic --model scenic.simulators.carla.racing_model \\
            examples/racing/three_segments.scenic --simulate

.. note::

    The racing domain requires maps in OpenDRIVE format (.xodr) that include
    proper lane types for distinguishing pit lanes from racing lanes.
"""

from .gnss_transform import (
    GNSSLocalTransform,
    fit_transform_from_csv,
    fit_transform_from_table,
    load_calibration,
    load_calibration_table_csv,
    load_gps_table_rows,
    save_calibration,
)
from .mode import (
    RACING_MODE_MAIN,
    RACING_MODE_PIT,
    get_racing_mode,
    is_pit_mode,
)

__all__ = [
    "GNSSLocalTransform",
    "fit_transform_from_csv",
    "fit_transform_from_table",
    "load_calibration",
    "load_calibration_table_csv",
    "load_gps_table_rows",
    "save_calibration",
    "RACING_MODE_MAIN",
    "RACING_MODE_PIT",
    "get_racing_mode",
    "is_pit_mode",
]

