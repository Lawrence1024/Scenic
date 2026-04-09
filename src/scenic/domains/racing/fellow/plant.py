"""Fellow plant-mode helpers for simulators that drive traffic via (v, d) signals.

Behaviors in ``behaviors.scenic`` for **dSPACE** fellow (v, d) plants stage
``_fellow_plant_state`` (keys ``v_kmh``, ``d_m``) via
:class:`~scenic.domains.racing.actions.SetFellowPlantAction`; the dSPACE controller writes
those to External_Signals (see :mod:`scenic.domains.racing.fellow.commands`).

Detection uses behavior class names that **start with** ``Fellow`` (the four plant
behaviors in ``behaviors.scenic``). Renaming a plant behavior keeps working as long as
the name still starts with ``Fellow``.

- :obj:`FellowConstantSpeedTrackOffsetBehavior` — constant ``speed_mph`` and lateral offset
  from placement.
- :obj:`FellowFollowTTLGeometricBehavior` — constant ``speed_mph`` and lateral ``d`` from
  feedforward δ(s) on the main centerline (optimal TTL vs ``ttl_main_road``), with waypoint
  index updates via shared racing helpers.

- :obj:`FellowSuddenStopIntervalBehavior` — **repeating** cruise / full-stop schedule on
  simulation time: ``interval`` seconds at **speed** (mph), then ``duration`` seconds at
  commanded **v = 0**, then repeat. Lateral **d** always follows TTL δ(s) like
  :obj:`FellowFollowTTLGeometricBehavior` (no open-loop lateral maneuver). Defaults:
  ``speed=150``, ``interval=20``, ``duration=3``. Example:
  ``examples/combined/fellow_sudden_stop.scenic``.

- :obj:`FellowSwerveOutOfControlBehavior` — **one-shot** maneuver: ``interval`` seconds TTL
  cruise, then rate-limited slew of **d** toward full right (−amp) then full left (+amp),
  then **v = 0**. Use ``stop_hold_d`` (default true) to freeze **d** after the stop so TTL
  tracking does not move the lateral command while the car is stationary. Defaults match
  ``examples/combined/fellow_swerve_out_of_control.scenic``.

Other simulators may ignore these unless they implement the same contract.
"""

from __future__ import annotations

from typing import Any


def is_fellow_vd_plant_behavior(obj: Any) -> bool:
    """True if ``obj`` has an active Scenic behavior whose class name starts with ``Fellow``.

    Used to route dSPACE fellow control to the (v, d) plant path without branching on
    individual behavior class names.
    """
    b = getattr(obj, "behavior", None)
    return b is not None and b.__class__.__name__.startswith("Fellow")
