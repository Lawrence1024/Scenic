"""Racing planner pipeline (SD-11+): trajectory-based strategy selection.

SD-41 introduces the dense `PlannerReference` contract — the planner emits
a 7-column trajectory (s, x, y, psi, kappa, vx, ax) per tick that the MPC
consumes directly. See `planner_reference.py` for the full schema rationale.
"""

from scenic.domains.racing.planner.planner_reference import (
    PlannerReference,
)
from scenic.domains.racing.planner.strategy_selector import (
    SelectedStrategy,
    select_strategy,
)

__all__ = [
    "PlannerReference",
    "SelectedStrategy",
    "select_strategy",
]
