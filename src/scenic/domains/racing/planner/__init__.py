"""Racing planner pipeline (SD-11+): trajectory-based strategy selection.

SD-41 introduces the dense `PlannerReference` contract — the planner emits
a 7-column trajectory (s, x, y, psi, kappa, vx, ax) per tick that the MPC
consumes directly. See `planner_reference.py` for the full schema rationale.

SD-42 adds the `VelocityProfile` — per-TTL precomputed optimal vx(s) via
TUM's forward-backward pass. The planner consults this profile each tick
to fill in `PlannerReference.vx_mps`, replacing the runtime cap-composition
chain that accumulated in `behaviors.scenic` over SD-30..41. See
`velocity_profile.py` for the algorithm.
"""

from scenic.domains.racing.planner.planner_reference import (
    PlannerReference,
)
from scenic.domains.racing.planner.strategy_selector import (
    SelectedStrategy,
    select_strategy,
)
from scenic.domains.racing.planner.velocity_profile import (
    VelocityProfile,
    compute_velocity_profile,
)

__all__ = [
    "PlannerReference",
    "SelectedStrategy",
    "VelocityProfile",
    "compute_velocity_profile",
    "select_strategy",
]
