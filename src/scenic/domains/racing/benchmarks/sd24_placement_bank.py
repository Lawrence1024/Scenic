"""SD-24 placement bank: exercise every cell of the placement matrix.

Compiles a small synthetic scenario for each of the placement-region /
TTL-category combinations enabled by SD-24 and asserts the resulting
ego (x, y) lands where it should. No simulator is attached -- we use
``scenic.scenarioFromString`` to drive the Scenic compile + sampler
end-to-end without ever launching a viewer or the dSPACE bridge.

Two banks of cases:

1. **Consistent cases (no warning expected).** TTL category and the
   explicit placement region agree, OR the placement goes through the
   unified ``trackRegion(...)`` pipeline which can't disagree by
   construction. For each case, we verify the sampled (x, y) is inside
   the requested polygon (and, where applicable, on the requested TTL
   polyline within tolerance).

2. **Contradiction cases (warning expected).** TTL category and the
   explicit placement region disagree -- the four mismatch combinations
   spelled out in ``docs/scenic_changes_from_presentation.md`` SD-24c.
   Without a simulator we can't observe ``placement.py``'s
   ``[Placement] [WARN]`` line, but we can run the same predicate
   (``ttl_category(name) != classify(x, y)``) locally and confirm it
   would fire.

USAGE:

    python src/scenic/domains/racing/benchmarks/sd24_placement_bank.py
    python src/scenic/domains/racing/benchmarks/sd24_placement_bank.py --samples 5 --seed 7
    python src/scenic/domains/racing/benchmarks/sd24_placement_bank.py --log my_bank.log

Output is written both to the terminal and to a log file
(``sd24_placement_bank_<TIMESTAMP>.log`` by default, or ``--log
PATH``). Exit code 0 on full pass, 1 if any case failed (consistency
violation in a "consistent" case OR predicate not firing in a
"contradiction" case).
"""

from __future__ import annotations

import argparse
import io
import sys
import time
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional


# Force UTF-8 stdout under PowerShell so non-ASCII chars don't crash
# when output is redirected to a log file.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# Repo paths used by the synthesized scenarios.
_REPO_ROOT = Path(__file__).resolve().parents[5]
_MAP = _REPO_ROOT / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"
_TTL_FOLDER = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"


@dataclass
class BankCase:
    """One row in the placement bank."""

    name: str
    description: str
    # Scenic snippet declaring the ego. Inserted verbatim into a
    # template after the param block. Must produce a `RacingCar` named
    # `ego`. Use the unified `trackRegion(...)` helper, the cross-product
    # region names (mainCurve / mainStraight / pitCurve / pitStraight),
    # or the axis names (curve / straight) -- all of these are
    # top-level Scenic names exposed by SD-24.
    ego_decl: str
    # The polygon the ego is *expected* to land in. Use one of:
    # 'mainCurve', 'mainStraight', 'pitCurve', 'pitStraight',
    # 'curve' (full union), 'straight' (full union),
    # 'mainTrack', 'pitTrack', 'raceTrack',
    # or None to skip the polygon-membership check (used for cases
    # where placement is on a polyline rather than a polygon).
    expected_region: Optional[str]
    # The car's `ttlFileName` attribute, or None. Drives the contradiction
    # predicate -- if `ttl_category(this) != classify(x, y)`, the warning
    # would fire at simulation time.
    ttl_file_name: Optional[str]
    # Whether the local contradiction predicate is expected to fire for
    # this case. Independent of `expected_region`: a case can be
    # placement-consistent with its requested polygon AND still trigger
    # the contradiction warning (when explicit cross-product placement
    # disagrees with the TTL category).
    expect_warn: bool


# ---------------------------------------------------------------------------
# Bank cases
# ---------------------------------------------------------------------------

# Note on TTL category derivation: any filename containing 'pit' (case
# insensitive) is the pit category; any other non-empty filename is main
# (see ``ttl_category`` in track_regions.py). 'ttl_optimal_xodr.csv' /
# 'ttl_left_xodr.csv' / 'ttl_right_xodr.csv' are all category=main.
# 'ttl_pit_xodr.csv' is category=pit.

_BANK: List[BankCase] = [
    # --- Consistent: explicit cross-product placement matching TTL ---
    BankCase(
        name="explicit_mainCurve_main_ttl",
        description="on mainCurve + main TTL (ttl_optimal_xodr.csv)",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with position new Point on mainCurve"
        ),
        expected_region="mainCurve",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="explicit_mainStraight_main_ttl",
        description="on mainStraight + main TTL",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with position new Point on mainStraight"
        ),
        expected_region="mainStraight",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="explicit_pitCurve_pit_ttl",
        description="on pitCurve + pit TTL (ttl_pit_xodr.csv)",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with position new Point on pitCurve"
        ),
        expected_region="pitCurve",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="explicit_pitStraight_pit_ttl",
        description="on pitStraight + pit TTL",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with position new Point on pitStraight"
        ),
        expected_region="pitStraight",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=False,
    ),

    # --- Consistent: unified trackRegion(...) pipeline ---
    BankCase(
        name="trackRegion_main_curve",
        description="trackRegion('ttl_optimal_xodr.csv', 'curve') = optimal TTL ∩ mainCurve",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), "
            "with position new Point on trackRegion('ttl_optimal_xodr.csv', 'curve')"
        ),
        expected_region="mainCurve",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="trackRegion_main_straight",
        description="trackRegion('ttl_optimal_xodr.csv', 'straight') = optimal TTL ∩ mainStraight",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), "
            "with position new Point on trackRegion('ttl_optimal_xodr.csv', 'straight')"
        ),
        expected_region="mainStraight",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="trackRegion_pit_curve",
        description="trackRegion('ttl_pit_xodr.csv', 'curve') = pit TTL ∩ pitCurve",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), "
            "with position new Point on trackRegion('ttl_pit_xodr.csv', 'curve')"
        ),
        expected_region="pitCurve",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="trackRegion_pit_straight",
        description="trackRegion('ttl_pit_xodr.csv', 'straight') = pit TTL ∩ pitStraight",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV'), "
            "with position new Point on trackRegion('ttl_pit_xodr.csv', 'straight')"
        ),
        expected_region="pitStraight",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=False,
    ),

    # --- Consistent: bare axis regions (no TTL implication) ---
    BankCase(
        name="axis_curve_no_ttl",
        description="on curve (full union, both pit + main) + no TTL",
        ego_decl="ego = new RacingCar on curve, with raceNumber 1",
        expected_region="curve",
        ttl_file_name=None,
        expect_warn=False,
    ),
    BankCase(
        name="axis_straight_no_ttl",
        description="on straight (full union) + no TTL",
        ego_decl="ego = new RacingCar on straight, with raceNumber 1",
        expected_region="straight",
        ttl_file_name=None,
        expect_warn=False,
    ),

    # --- Consistent: default position (RacingCar's `position:` default
    # routes through trackRegion(self.ttlFileName) since SD-24). The car
    # ends up on the TTL polyline; we don't enforce a polygon membership
    # check (expected_region=None) but verify the contradiction
    # predicate doesn't fire.
    BankCase(
        name="default_main_ttl",
        description="default position with main TTL = TTL polyline",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')"
        ),
        expected_region=None,
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=False,
    ),
    BankCase(
        name="default_pit_ttl",
        description="default position with pit TTL = TTL polyline",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')"
        ),
        expected_region=None,
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=False,
    ),

    # --- Contradiction: 4 mismatch combinations -------------------------------
    BankCase(
        name="contradiction_mainCurve_pit_ttl",
        description="on mainCurve but ttlFileName='ttl_pit_xodr.csv'",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with position new Point on mainCurve"
        ),
        expected_region="mainCurve",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=True,
    ),
    BankCase(
        name="contradiction_mainStraight_pit_ttl",
        description="on mainStraight but ttlFileName='ttl_pit_xodr.csv'",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_pit_xodr.csv', "
            "with position new Point on mainStraight"
        ),
        expected_region="mainStraight",
        ttl_file_name="ttl_pit_xodr.csv",
        expect_warn=True,
    ),
    BankCase(
        name="contradiction_pitCurve_main_ttl",
        description="on pitCurve but ttlFileName='ttl_optimal_xodr.csv'",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with position new Point on pitCurve"
        ),
        expected_region="pitCurve",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=True,
    ),
    BankCase(
        name="contradiction_pitStraight_main_ttl",
        description="on pitStraight but ttlFileName='ttl_optimal_xodr.csv'",
        ego_decl=(
            "ego = new RacingCar with raceNumber 1, "
            "with ttlFileName 'ttl_optimal_xodr.csv', "
            "with position new Point on pitStraight"
        ),
        expected_region="pitStraight",
        ttl_file_name="ttl_optimal_xodr.csv",
        expect_warn=True,
    ),
]


# ---------------------------------------------------------------------------
# Scenic source assembly
# ---------------------------------------------------------------------------

_PREAMBLE_TEMPLATE = """\
param map = '{map_path}'
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = '{ttl_folder}'
param launch_veos_ipc_client = False
model scenic.simulators.dspace.racing_model

"""


def _build_source(case: BankCase) -> str:
    """Render one scenic source for a bank case.

    Uses absolute paths (forward-slashed) for ``map`` and ``ttlFolder``
    so the source compiles via ``scenarioFromString`` without depending
    on ``localPath`` (which expects a real .scenic file location).
    """
    # Some bank cases pass `with ttlFolder localPath('../../../...')` in
    # their ego_decl. Replace that with an absolute path so the
    # compiled-from-string source resolves cleanly.
    ego_decl = case.ego_decl.replace(
        "localPath('../../../assets/ttls/LS_ENU_TTL_CSV')",
        f"'{_TTL_FOLDER.as_posix()}'",
    )
    preamble = _PREAMBLE_TEMPLATE.format(
        map_path=_MAP.as_posix(),
        ttl_folder=_TTL_FOLDER.as_posix(),
    )
    return preamble + ego_decl + "\n"


# ---------------------------------------------------------------------------
# Contradiction predicate (mirrors placement.py:_maybe_warn_placement_contradiction)
# ---------------------------------------------------------------------------

# Tolerance: identical to placement.py:_TTL_PROXIMITY_TOLERANCE_M (1.0 m).
_TTL_PROXIMITY_TOLERANCE_M = 1.0


def _placement_is_on_ttl_local(ttl_file_name, x, y, tol_m=_TTL_PROXIMITY_TOLERANCE_M) -> bool:
    """True if (x, y) is within ``tol_m`` of the TTL polyline.

    Mirrors placement.py:_placement_is_on_ttl. Same skip rule means the
    bank validates the same predicate the simulator will execute.
    """
    if not ttl_file_name:
        return False
    try:
        from scenic.domains.racing.segments.track_regions import (
            create_ttl_region_from_file,
        )
        from shapely.geometry import Point
    except Exception:
        return False
    try:
        polyline = create_ttl_region_from_file(_TTL_FOLDER, ttl_file_name)
        if polyline is None:
            return False
        return float(polyline.lineString.distance(Point(float(x), float(y)))) < tol_m
    except Exception:
        return False


# ---------------------------------------------------------------------------
# Per-case verification
# ---------------------------------------------------------------------------

@dataclass
class CaseResult:
    name: str
    samples: List[dict]      # each: {'x': float, 'y': float, ...}
    consistency_pass: bool
    warning_pass: bool
    error: Optional[str] = None

    @property
    def overall_pass(self) -> bool:
        return self.error is None and self.consistency_pass and self.warning_pass


def _run_one_case(case: BankCase, n_samples: int) -> CaseResult:
    """Compile + sample one bank case, run consistency + warning checks."""
    import scenic
    from scenic.domains.racing.segments.track_regions import ttl_category as _ttl_cat

    src = _build_source(case)

    # Suppress Scenic / racing-domain compile chatter (the [TrackRegions]
    # banner, the [RacingTrack] feature-id messages, etc.). We capture
    # via redirect_stdout so the bank log stays clean -- the per-case
    # decisions are what we care about.
    _silent = io.StringIO()

    try:
        with redirect_stdout(_silent), redirect_stderr(_silent):
            scenario = scenic.scenarioFromString(
                src,
                model="scenic.simulators.dspace.racing_model",
                mode2D=True,
            )
    except Exception as exc:
        return CaseResult(
            name=case.name,
            samples=[],
            consistency_pass=False,
            warning_pass=False,
            error=f"compile failed: {exc!r}",
        )

    # Pull region objects from the scenario's compiled namespace via
    # params (mainTrackRegion / pitTrackRegion are exposed for SD-24c)
    # plus the cross-product / axis region names which we need for the
    # consistency check. The scenario object also stores all top-level
    # bindings; we can read them via `scenario.namespace` (translator
    # output) -- but the simplest portable handle is via the generated
    # scene's `workspace.network` and the per-scene Region objects.
    #
    # In practice the regions are accessible directly from the
    # scenario's compiled module via attributes on the workspace's
    # network or via `scenario.namespace`. We avoid that brittle path
    # by re-importing the helper that built them.
    from scenic.domains.racing.segments.track_regions import (
        build_curve_straight_regions_from_opendrive,
    )
    track = scenario.params.get("track")
    if track is None:
        return CaseResult(
            name=case.name,
            samples=[],
            consistency_pass=False,
            warning_pass=False,
            error="no track in scenario params",
        )
    cs_regions = build_curve_straight_regions_from_opendrive(track)
    main_track_region = scenario.params.get("mainTrackRegion")
    pit_track_region = scenario.params.get("pitTrackRegion")

    region_lookup = {
        "curve": cs_regions["curve"],
        "straight": cs_regions["straight"],
        "mainCurve": cs_regions["mainCurve"],
        "mainStraight": cs_regions["mainStraight"],
        "pitCurve": cs_regions["pitCurve"],
        "pitStraight": cs_regions["pitStraight"],
        "mainTrack": main_track_region,
        "pitTrack": pit_track_region,
    }

    expected_region = (
        region_lookup.get(case.expected_region)
        if case.expected_region else None
    )

    # Sample N times.
    from scenic.core.vectors import Vector
    samples = []
    consistency_pass = True
    warning_pass = True
    cat_ttl = _ttl_cat(case.ttl_file_name)

    for i in range(n_samples):
        try:
            with redirect_stdout(_silent), redirect_stderr(_silent):
                scene, _ = scenario.generate()
        except Exception as exc:
            return CaseResult(
                name=case.name,
                samples=[],
                consistency_pass=False,
                warning_pass=False,
                error=f"generate {i+1}/{n_samples} failed: {exc!r}",
            )
        egos = [o for o in scene.objects if getattr(o, "raceNumber", None) == 1]
        if not egos:
            return CaseResult(
                name=case.name,
                samples=[],
                consistency_pass=False,
                warning_pass=False,
                error="no ego in generated scene",
            )
        ego = egos[0]
        ex, ey = float(ego.position.x), float(ego.position.y)
        pt = Vector(ex, ey)

        # Consistency check: is ego inside the expected polygon?
        in_expected = None
        if expected_region is not None:
            try:
                in_expected = bool(expected_region.containsPoint(pt))
            except Exception:
                in_expected = None
            if in_expected is False:
                consistency_pass = False

        # Contradiction predicate: does the simulator-side warning fire?
        # Same logic as placement.py:_maybe_warn_placement_contradiction —
        # including the "skip when on the TTL polyline" rule that prevents
        # false positives at pit entry/exit (where the pit TTL legitimately
        # traverses the mainTrack polygon due to main-wins-on-overlap).
        in_main = bool(main_track_region.containsPoint(pt)) if main_track_region else False
        in_pit = bool(pit_track_region.containsPoint(pt)) if pit_track_region else False
        on_ttl = _placement_is_on_ttl_local(case.ttl_file_name, ex, ey)
        warn_would_fire = False
        warn_reason = ""
        if cat_ttl is not None and not on_ttl:
            if cat_ttl == "main" and in_pit and not in_main:
                warn_would_fire = True
                warn_reason = "ttl=main but placed in pit"
            elif cat_ttl == "pit" and in_main and not in_pit:
                warn_would_fire = True
                warn_reason = "ttl=pit but placed in main"

        if case.expect_warn and not warn_would_fire:
            warning_pass = False
        if (not case.expect_warn) and warn_would_fire:
            warning_pass = False

        samples.append({
            "x": ex, "y": ey,
            "in_expected": in_expected,
            "in_main": in_main,
            "in_pit": in_pit,
            "warn_would_fire": warn_would_fire,
            "warn_reason": warn_reason,
        })

    return CaseResult(
        name=case.name,
        samples=samples,
        consistency_pass=consistency_pass,
        warning_pass=warning_pass,
        error=None,
    )


# ---------------------------------------------------------------------------
# CLI + log writer
# ---------------------------------------------------------------------------

def _write_log(out, line: str = ""):
    print(line)
    out.write(line + "\n")
    out.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SD-24 placement bank: exercise every cell of the "
                    "placement matrix and log the results."
    )
    parser.add_argument(
        "--samples", "-n", type=int, default=3,
        help="Number of samples per case (default 3)."
    )
    parser.add_argument(
        "--seed", "-s", type=int, default=None,
        help="Optional random seed for the Scenic global RNG. If "
             "omitted, runs are non-deterministic (useful to flush "
             "out edge cases)."
    )
    parser.add_argument(
        "--log", "-l", type=str, default=None,
        help="Output log path. Defaults to "
             "sd24_placement_bank_<TIMESTAMP>.log in cwd."
    )
    parser.add_argument(
        "--filter", "-f", type=str, default=None,
        help="Only run cases whose name contains this substring."
    )
    args = parser.parse_args()

    if args.seed is not None:
        import random as _r
        import numpy as _np
        _r.seed(args.seed)
        _np.random.seed(args.seed)

    log_path = (
        Path(args.log) if args.log else
        Path.cwd() / f"sd24_placement_bank_{time.strftime('%Y%m%d_%H%M%S')}.log"
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
        _write_log(fh, "SD-24 placement bank")
        _write_log(fh, f"  samples per case : {args.samples}")
        _write_log(fh, f"  seed             : {args.seed if args.seed is not None else 'random'}")
        _write_log(fh, f"  cases            : {len(cases)}")
        _write_log(fh, f"  log              : {log_path}")
        _write_log(fh, "=" * 78)
        _write_log(fh)

        results: List[CaseResult] = []
        for case in cases:
            _write_log(fh, f"[CASE] {case.name}")
            _write_log(fh, f"       {case.description}")
            t0 = time.perf_counter()
            res = _run_one_case(case, args.samples)
            dt = time.perf_counter() - t0
            results.append(res)

            if res.error:
                _write_log(fh, f"       ERROR: {res.error}")
                _write_log(fh, f"       overall: FAIL  ({dt:.1f}s)")
                _write_log(fh)
                continue

            for i, s in enumerate(res.samples):
                xy = f"({s['x']:8.2f}, {s['y']:8.2f})"
                in_exp = s["in_expected"]
                in_exp_s = (
                    "n/a" if in_exp is None else ("✓" if in_exp else "✗")
                )
                warn_s = (
                    "WARN-would-fire" if s["warn_would_fire"]
                    else "no-warn"
                )
                detail = ""
                if s["warn_would_fire"]:
                    detail = f"  [{s['warn_reason']}]"
                _write_log(
                    fh,
                    f"       sample {i+1}/{len(res.samples)}: ego={xy}  "
                    f"in_{case.expected_region or '?'}={in_exp_s}  {warn_s}{detail}"
                )

            _write_log(
                fh,
                f"       consistency: {'PASS' if res.consistency_pass else 'FAIL'}  "
                f"warning: {'PASS' if res.warning_pass else 'FAIL'} "
                f"(expected={'yes' if case.expect_warn else 'no'})  "
                f"overall: {'PASS' if res.overall_pass else 'FAIL'}  ({dt:.1f}s)"
            )
            _write_log(fh)

        # Summary
        _write_log(fh, "=" * 78)
        _write_log(fh, "Summary")
        _write_log(fh, "=" * 78)
        n_total = len(results)
        n_pass = sum(1 for r in results if r.overall_pass)
        n_fail = n_total - n_pass
        for r in results:
            tag = "PASS" if r.overall_pass else "FAIL"
            _write_log(fh, f"  [{tag}] {r.name}")
        _write_log(fh)
        _write_log(fh, f"  TOTAL: {n_pass}/{n_total} pass  ({n_fail} fail)")

    print(f"\n[Bank] log written to {log_path}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
