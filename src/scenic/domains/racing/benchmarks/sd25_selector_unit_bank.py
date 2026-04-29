"""SD-25d offline regression bank for the strategy selector.

Exercises ``select_strategy`` directly with hand-crafted ``StrategyOutcome``
lists and asserts that the chosen strategy and the reason string match
expectations. No simulator, no Scenic compile — pure unit-bank style.

Mirrors the SD-24 placement bank pattern: single-command runner, log
output to file + stdout, exit code 0 on full pass / 1 on any failure.

USAGE:

    python src/scenic/domains/racing/benchmarks/sd25_selector_unit_bank.py
    python src/scenic/domains/racing/benchmarks/sd25_selector_unit_bank.py --log sd25_selector.log

Cases cover:
- Tied progress with asymmetric clearance, fellow-on-left -> Stage A picks pass_right
- Tied progress with asymmetric clearance, fellow-on-right -> Stage A picks pass_left (symmetry)
- Tied progress with equal clearance -> deterministic but either pass side is acceptable
- Untied progress -> rank winner (no tiebreak triggered)
- Hard filter eliminates all but stay_optimal -> reason="primary"
- Hard filter eliminates everything; soft fallback to follow_fellow -> reason="soft_fallback_follow"
- Hard filter + soft filter both fail -> reason="last_resort_stay"
- Stay_optimal vs pass with equal progress -> stay wins via lower TIEBREAK_RANK
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


from scenic.domains.racing.planner.strategy_selector import select_strategy
from scenic.domains.racing.prediction.strategy_simulator import StrategyOutcome


# ---------------------------------------------------------------------------
# Synthetic outcome helpers
# ---------------------------------------------------------------------------

def _outcome(strategy: str, progress: float, clearance: float) -> StrategyOutcome:
    """Build a minimal StrategyOutcome for a unit-bank case."""
    return StrategyOutcome(
        strategy=strategy,
        reachable_progress_at_horizon_m=float(progress),
        reachable_speed_at_horizon_mps=0.0,
        min_clearance_m=float(clearance),
        closest_t_s=0.0,
        completed=False,
    )


# ---------------------------------------------------------------------------
# Bank cases
# ---------------------------------------------------------------------------

@dataclass
class BankCase:
    name: str
    description: str
    outcomes: List[StrategyOutcome]
    # Either expected_name (single acceptable answer) OR
    # expected_name_in (a tuple of acceptable answers for genuine-tie cases).
    expected_name: Optional[str] = None
    expected_name_in: Optional[tuple] = None
    expected_reason: Optional[str] = None


_BANK: List[BankCase] = [
    # -------- Stage A invariant: tied progress, asymmetric clearance --------
    BankCase(
        name="tiebreak_fellow_on_left",
        description=(
            "All four strategies tied on progress=3000m. "
            "Clearances: stay=0.5 (filtered), follow=27 (filtered), "
            "pass_left=4.0, pass_right=13.0. With the SD-25a fix, "
            "pass_right wins via the higher-clearance secondary key."
        ),
        outcomes=[
            _outcome("stay_optimal", 3000.0, 0.5),       # filtered
            _outcome("follow_fellow", 3000.0, 27.0),     # passes filter
            _outcome("pass_left", 3000.0, 4.0),
            _outcome("pass_right", 3000.0, 13.0),
        ],
        expected_name="pass_right",
        expected_reason="primary",
    ),
    BankCase(
        name="tiebreak_fellow_on_right",
        description=(
            "Mirror of fellow_on_left. pass_right=4.0, pass_left=13.0. "
            "Stage A symmetry: pass_left wins."
        ),
        outcomes=[
            _outcome("stay_optimal", 3000.0, 0.5),
            _outcome("follow_fellow", 3000.0, 27.0),
            _outcome("pass_left", 3000.0, 13.0),
            _outcome("pass_right", 3000.0, 4.0),
        ],
        expected_name="pass_left",
        expected_reason="primary",
    ),
    BankCase(
        name="tiebreak_equal_clearance_pass_sides",
        description=(
            "pass_left and pass_right both at progress=3000, both "
            "clearance=5.0. Genuine tie. Either pass side is acceptable; "
            "the bank just asserts the selector picks ONE of them "
            "deterministically."
        ),
        outcomes=[
            _outcome("stay_optimal", 3000.0, 0.5),
            _outcome("follow_fellow", 3000.0, 0.5),
            _outcome("pass_left", 3000.0, 5.0),
            _outcome("pass_right", 3000.0, 5.0),
        ],
        expected_name_in=("pass_left", "pass_right"),
        expected_reason="primary",
    ),

    # -------- Untied progress: tiebreak NEVER fires --------
    BankCase(
        name="untied_progress_pass_left_higher",
        description=(
            "pass_left progress=3100, pass_right=3000. Both above filter. "
            "Tiebreak never triggered (>0.5m gap). pass_left wins on "
            "primary metric."
        ),
        outcomes=[
            _outcome("stay_optimal", 2000.0, 5.0),
            _outcome("follow_fellow", 2500.0, 5.0),
            _outcome("pass_left", 3100.0, 5.0),
            _outcome("pass_right", 3000.0, 13.0),
        ],
        expected_name="pass_left",
        expected_reason="primary",
    ),

    # -------- Stay_optimal preference at tied progress --------
    BankCase(
        name="tied_with_stay_optimal_in_tie",
        description=(
            "stay_optimal, pass_left, pass_right all at progress=3000 with "
            "clearance >2.5. stay_optimal has rank 0; wins the tiebreak "
            "regardless of clearance."
        ),
        outcomes=[
            _outcome("stay_optimal", 3000.0, 3.0),       # rank 0
            _outcome("follow_fellow", 3000.0, 0.5),      # filtered
            _outcome("pass_left", 3000.0, 5.0),          # rank 1
            _outcome("pass_right", 3000.0, 13.0),        # rank 1
        ],
        expected_name="stay_optimal",
        expected_reason="primary",
    ),

    # -------- Hard filter cases --------
    BankCase(
        name="hard_filter_only_stay_passes",
        description=(
            "Only stay_optimal passes the 2.5m hard filter. Selector picks "
            "it as the lone survivor with reason='primary'."
        ),
        outcomes=[
            _outcome("stay_optimal", 1000.0, 3.0),
            _outcome("follow_fellow", 2000.0, 1.0),
            _outcome("pass_left", 1000.0, 1.0),
            _outcome("pass_right", 1000.0, 1.0),
        ],
        expected_name="stay_optimal",
        expected_reason="primary",
    ),
    BankCase(
        name="hard_filter_eliminates_all_soft_fallback",
        description=(
            "Every clearance below 2.5m hard threshold; follow_fellow at "
            "1.8m (above 1.5m soft threshold). Selector takes the soft "
            "fallback path: follow_fellow."
        ),
        outcomes=[
            _outcome("stay_optimal", 1000.0, 1.0),
            _outcome("follow_fellow", 2000.0, 1.8),
            _outcome("pass_left", 1000.0, 1.0),
            _outcome("pass_right", 1000.0, 1.0),
        ],
        expected_name="follow_fellow",
        expected_reason="soft_fallback_follow",
    ),
    BankCase(
        name="hard_and_soft_filters_both_fail",
        description=(
            "Every clearance well below 1.5m (the soft threshold). "
            "Selector falls all the way through to last-resort stay_optimal."
        ),
        outcomes=[
            _outcome("stay_optimal", 1000.0, 0.3),
            _outcome("follow_fellow", 1000.0, 0.3),
            _outcome("pass_left", 1000.0, 0.3),
            _outcome("pass_right", 1000.0, 0.3),
        ],
        expected_name="stay_optimal",
        expected_reason="last_resort_stay",
    ),
]


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

@dataclass
class CaseResult:
    name: str
    selected: str
    selected_reason: str
    expected_name: str  # for display only
    passed: bool
    error: Optional[str] = None


def _run_one_case(case: BankCase) -> CaseResult:
    expected_display = (
        case.expected_name if case.expected_name is not None
        else f"one of {case.expected_name_in}"
    )
    try:
        sel = select_strategy(case.outcomes)
    except Exception as exc:
        return CaseResult(
            name=case.name,
            selected="<error>",
            selected_reason="<error>",
            expected_name=str(expected_display),
            passed=False,
            error=f"select_strategy raised: {exc!r}",
        )
    name_ok = (
        sel.name == case.expected_name if case.expected_name is not None
        else sel.name in (case.expected_name_in or ())
    )
    reason_ok = (
        case.expected_reason is None
        or sel.reason == case.expected_reason
    )
    return CaseResult(
        name=case.name,
        selected=sel.name,
        selected_reason=sel.reason,
        expected_name=str(expected_display),
        passed=name_ok and reason_ok,
    )


def _write_log(out, line: str = ""):
    print(line)
    out.write(line + "\n")
    out.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SD-25d: offline regression bank for the strategy selector."
    )
    parser.add_argument(
        "--log", "-l", type=str, default=None,
        help="Output log path. Defaults to "
             "sd25_selector_unit_bank_<TIMESTAMP>.log in cwd."
    )
    parser.add_argument(
        "--filter", "-f", type=str, default=None,
        help="Only run cases whose name contains this substring."
    )
    args = parser.parse_args()

    log_path = (
        Path(args.log) if args.log else
        Path.cwd() / f"sd25_selector_unit_bank_{time.strftime('%Y%m%d_%H%M%S')}.log"
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)

    cases = _BANK
    if args.filter:
        cases = [c for c in _BANK if args.filter in c.name]
    if not cases:
        print(f"[Bank] no cases matched filter '{args.filter}'")
        return 2

    with log_path.open("w", encoding="utf-8") as fh:
        _write_log(fh, "=" * 78)
        _write_log(fh, "SD-25d strategy selector unit bank")
        _write_log(fh, f"  cases : {len(cases)}")
        _write_log(fh, f"  log   : {log_path}")
        _write_log(fh, "=" * 78)
        _write_log(fh)

        results: List[CaseResult] = []
        for case in cases:
            _write_log(fh, f"[CASE] {case.name}")
            _write_log(fh, f"       {case.description}")
            res = _run_one_case(case)
            results.append(res)
            if res.error:
                _write_log(fh, f"       ERROR: {res.error}")
                _write_log(fh, "       overall: FAIL")
            else:
                _write_log(
                    fh,
                    f"       expected: {res.expected_name}  "
                    f"reason={case.expected_reason or '<any>'}"
                )
                _write_log(
                    fh,
                    f"       got:      {res.selected}  "
                    f"reason={res.selected_reason}"
                )
                _write_log(
                    fh,
                    f"       overall: {'PASS' if res.passed else 'FAIL'}"
                )
            _write_log(fh)

        _write_log(fh, "=" * 78)
        _write_log(fh, "Summary")
        _write_log(fh, "=" * 78)
        n_total = len(results)
        n_pass = sum(1 for r in results if r.passed)
        n_fail = n_total - n_pass
        for r in results:
            tag = "PASS" if r.passed else "FAIL"
            _write_log(fh, f"  [{tag}] {r.name}  -> got {r.selected}")
        _write_log(fh)
        _write_log(fh, f"  TOTAL: {n_pass}/{n_total} pass  ({n_fail} fail)")

    print(f"\n[Bank] log written to {log_path}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
