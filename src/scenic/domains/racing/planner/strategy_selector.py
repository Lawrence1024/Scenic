"""SD-11c: pure-function strategy selector.

Takes the four StrategyOutcomes from strategy_simulator.simulate_strategy and
picks the winner under a correctness-first ranking:

  1. Filter survivors with min_clearance_m >= min_clearance_m threshold
     (default 1.0m; this is OBB edge-to-edge gap, NOT centroid
     distance — see strategy_simulator.simulate_strategy).
  1a. SD-33: when closing_flag=True AND no pass_* survives the hard filter,
      demote stay_optimal if its predicted clearance is below
      safe_stay_clearance_m (default 5.0m). Prevents stay_optimal from
      winning on progress while ego actively closes on a slower fellow at
      marginal clearance.
  2. Among survivors, rank by reachable_progress_at_horizon_m (highest wins).
  3. Tiebreak (within ~0.1m progress): stay_optimal > pass_* > follow_fellow,
     i.e., prefer the strategy that requires the least mode change.
  4. Soft fallback if filter empties: try follow_fellow at a softer threshold
     (0.2m). If that also fails, return stay_optimal as the last resort
     (the SD-4 emergency-brake layer downstream will catch any actual collision).

Threshold history:
  - Pre-SD-27 used 2.5m / 1.5m on centroid distance. With OBB-aware clearance
    (true edge-to-edge gap between IAC Dallaras), every legitimate pass has
    min_clearance ~0.5–3 m during the alongside/merge transition, so the
    centroid-era 2.5m threshold rejected everything. New defaults reflect
    "physical gap between bumpers" rather than "centroid separation".
  - Post-SD-30 raised the hard filter from 0.5m to 1.0m (~half the Dallara's
    1.93m width) after S2 falsifier sample 1 collided despite a passing
    prediction: the 0.5m bar admitted close-call pass_left choices when a
    safer pass_right was available. The selector also now uses clearance as
    a secondary tiebreak key within rank-1, so when both pass strategies
    survive the filter, the one with greater predicted clearance wins.

No state. No side effects. Deterministic given input outcomes.

Used by SD-11d (telemetry-only) and SD-11e (authority) in the planner.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Sequence

from scenic.domains.racing.prediction.strategy_simulator import (
    Strategy,
    StrategyOutcome,
)


# Tiebreak preference: lower is preferred (stay > pass > follow).
_TIEBREAK_RANK = {
    "stay_optimal": 0,
    "pass_left": 1,
    "pass_right": 1,
    "follow_fellow": 2,
}


@dataclass
class SelectedStrategy:
    """The selector's decision plus diagnostics for logging / A/B comparison."""

    name: Strategy
    reason: str  # "primary" / "soft_fallback_follow" / "last_resort_stay"
    chosen_outcome: StrategyOutcome
    survivors: List[Strategy] = field(default_factory=list)
    all_outcomes: List[StrategyOutcome] = field(default_factory=list)


def select_strategy(
    outcomes: Sequence[StrategyOutcome],
    *,
    min_clearance_m: float = 1.0,
    soft_clearance_m: float = 0.2,
    progress_tiebreak_m: float = 0.5,
    closing_flag: bool = False,
    safe_stay_clearance_m: float = 5.0,
) -> SelectedStrategy:
    """Pick the fastest safe strategy.

    Args:
      outcomes: list of StrategyOutcome from simulate_strategy (one per Strategy).
      min_clearance_m: hard safety threshold; outcomes below this are filtered.
      soft_clearance_m: softer threshold used for the chicken-out fallback.
      progress_tiebreak_m: outcomes within this much progress are considered tied.
      closing_flag: True iff the race situation assessment marked ego as closing
                    on the fellow this tick. When set, stay_optimal is excluded
                    from the survivor set if its predicted clearance is below
                    ``safe_stay_clearance_m`` (SD-33). This prevents the selector
                    from picking stay_optimal — which always wins on progress
                    because it cruises at full target speed — over follow_fellow
                    when ego is actively catching up to fellow at marginal
                    clearance (S2 sample 8: stay_optimal=1.02m at t=3.00s won
                    over follow_fellow=5.75m, ego cruised into the danger zone).
      safe_stay_clearance_m: stay_optimal needs at least this much predicted
                    clearance (m) to be considered when ``closing_flag`` is set.

    Returns SelectedStrategy with:
      name           — the chosen strategy
      reason         — "primary" / "soft_fallback_follow" / "last_resort_stay"
      chosen_outcome — the StrategyOutcome that was selected
      survivors      — strategies that passed the hard filter (sorted by progress desc)
      all_outcomes   — the full input list (preserved for diagnostics)
    """
    if not outcomes:
        raise ValueError("select_strategy requires at least one outcome")

    by_name = {o.strategy: o for o in outcomes}

    # Step 1: hard-clearance filter.
    survivors = [o for o in outcomes if o.min_clearance_m >= float(min_clearance_m)]

    # SD-33: when fellow is closing on ego AND no pass strategy survives the
    # hard filter, demote stay_optimal so follow_fellow wins. stay_optimal
    # cruises at full target_speed and so always wins the progress sort,
    # even when its clearance just-barely passes the hard filter (~1.0m). In
    # closing scenarios with no pass alternative that single tick of cruising-
    # into-danger is enough to commit the run to a rear-end before
    # follow_fellow can take over.
    #
    # IMPORTANT: only fire when no pass_* survives. If pass_* is an option,
    # the normal selector picks correctly between pass and stay; demoting
    # stay then would force pass_* commits in scenarios where stay_optimal is
    # the prudent same-line cruise (S2 sample 1 regressed off_track at t=14.8s
    # because the un-gated demote forced extra pass_right commits, leaving
    # ego on the right TTL through a sharp curve).
    if closing_flag and survivors:
        stay = by_name.get("stay_optimal")
        pass_survivors = [
            o for o in survivors if o.strategy in ("pass_left", "pass_right")
        ]
        if (
            stay is not None
            and stay.min_clearance_m < float(safe_stay_clearance_m)
            and not pass_survivors
        ):
            survivors = [o for o in survivors if o.strategy != "stay_optimal"]

    if survivors:
        # Step 2: rank by reachable progress (highest wins).
        survivors_sorted = sorted(
            survivors,
            key=lambda o: o.reachable_progress_at_horizon_m,
            reverse=True,
        )
        # Step 3: tiebreak — collect all within progress_tiebreak_m of the top.
        top_progress = survivors_sorted[0].reachable_progress_at_horizon_m
        tied = [
            o for o in survivors_sorted
            if (top_progress - o.reachable_progress_at_horizon_m) <= float(progress_tiebreak_m)
        ]
        # Pick the one with lowest tiebreak rank (stay_optimal > pass_* > follow_fellow);
        # within the same rank (notably pass_left vs pass_right), prefer the strategy
        # with greater predicted clearance so a 2x-safer side beats canonical-order luck.
        chosen = min(
            tied,
            key=lambda o: (_TIEBREAK_RANK.get(o.strategy, 99), -o.min_clearance_m),
        )
        return SelectedStrategy(
            name=chosen.strategy,
            reason="primary",
            chosen_outcome=chosen,
            survivors=[o.strategy for o in survivors_sorted],
            all_outcomes=list(outcomes),
        )

    # Step 4: soft fallback. Prefer follow_fellow if it survives the soft threshold.
    soft_follow = by_name.get("follow_fellow")
    if soft_follow is not None and soft_follow.min_clearance_m >= float(soft_clearance_m):
        return SelectedStrategy(
            name="follow_fellow",
            reason="soft_fallback_follow",
            chosen_outcome=soft_follow,
            survivors=[],
            all_outcomes=list(outcomes),
        )

    # Last resort: stay_optimal. SD-4's 1.5s emergency brake will catch any
    # actual collision at runtime — this is the chicken-out path.
    stay = by_name.get("stay_optimal")
    if stay is None:
        # Truly nothing to pick — return the first outcome as a fallback to
        # avoid raising in production. Caller should treat this as degenerate.
        return SelectedStrategy(
            name=outcomes[0].strategy,
            reason="degenerate_no_stay_optimal",
            chosen_outcome=outcomes[0],
            survivors=[],
            all_outcomes=list(outcomes),
        )
    return SelectedStrategy(
        name="stay_optimal",
        reason="last_resort_stay",
        chosen_outcome=stay,
        survivors=[],
        all_outcomes=list(outcomes),
    )


__all__ = [
    "SelectedStrategy",
    "select_strategy",
]
