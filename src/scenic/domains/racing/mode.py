"""Racing mode: high-level indication of whether a vehicle is in pit or main racing.

The racing library treats vehicles as being in one of two modes:
- **Main (lap) mode**: normal racing on the main circuit (route 'Lap', R2 in dSPACE).
- **Pit mode**: driving in the pit lane (route 'Pit', R1 in dSPACE); speed limits and
  control strategy differ (e.g. coast at low speed instead of tracking high target).

Mode is determined by the vehicle's route, which is set at placement and updated on
pit exit / pit enter segment transitions. Behaviors and simulators should use
:func:`is_pit_mode` to branch logic so that pit mode drives differently than main.
"""

# Route values used by placement and pit exit/enter transitions (simulator maps these to R1/R2).
RACING_MODE_MAIN = "Lap"
RACING_MODE_PIT = "Pit"


def is_pit_mode(agent) -> bool:
    """Return True if the vehicle is in pit mode (pit lane route), False for main (lap) mode.

    Uses the agent's _route attribute, which is set by:
    - Simulator/placement at spawn (from position or TTL distance)
    - Pit exit / pit enter segment transitions during the run

    When _route is missing or None, returns False (main mode).
    """
    return getattr(agent, "_route", None) == RACING_MODE_PIT


def get_racing_mode(agent) -> str:
    """Return the current racing mode: RACING_MODE_PIT or RACING_MODE_MAIN."""
    return RACING_MODE_PIT if is_pit_mode(agent) else RACING_MODE_MAIN
