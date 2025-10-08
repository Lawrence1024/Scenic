"""Domain for racing scenarios on closed-circuit race tracks.

The racing domain extends the :doc:`driving domain <scenic.domains.driving>` with 
racing-specific features such as:

* **Racing tracks** - Closed-loop circuits with defined direction
* **Pit lanes** - Special lanes for pit stops, separate from racing lanes
* **Sectors** - Track divisions for timing and performance analysis
* **Racing lines** - Optimal paths through corners
* **Starting grid** - Formation positions for race starts
* **Track limits** - Boundaries that must not be exceeded

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

    param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
    param use2DMap = True
    model scenic.domains.racing.model
    
    # Create cars on racing grid
    ego = new RacingCar at startingGrid[0]
    opponent1 = new RacingCar at startingGrid[1]
    opponent2 = new RacingCar at startingGrid[2]
    
    # Follow racing line behavior
    ego with behavior FollowRacingLineBehavior()

The racing domain is designed to work with:

* dSPACE ModelDesk (Laguna Seca and other racing circuits)
* CARLA (racing tracks)
* Any simulator supporting the driving domain with racing-specific maps

.. note::

    The racing domain requires maps in OpenDRIVE format (.xodr) that include
    proper lane types for distinguishing pit lanes from racing lanes.
"""

