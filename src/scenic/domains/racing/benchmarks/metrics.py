"""Records-driven SampleMetrics extraction for the falsification pipeline.

Consumes Scenic's ``simulation.records`` defaultdict (populated by the
``_record_event(...)`` helper in ``behaviors.scenic`` and by direct
``self.records[...].append(...)`` in the simulator) and produces the
:class:`SampleMetrics` dataclass that the falsifier monitors read.

The entry point is :func:`parse_sample`. Callers must pass
``records=`` from ``simulation.result.records`` — there is no log-file
fallback. (The earlier subprocess-based ``sampled_runner.py`` flow was
removed because subprocesses can't access the simulation object;
``verifai_runner.py`` is the in-process replacement.)

Module also exports the CSV / text summary writers used by the
falsifier driver.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Per-sample metrics shape
# ---------------------------------------------------------------------------

@dataclass
class SampleMetrics:
    """Parsed per-sample summary."""

    sample_index: int
    seed: int
    return_code: int
    log_path: str
    # Outcome
    collision: bool
    off_track: bool
    lap_time_s: Optional[float]
    # Continuous robustness signal -- minimum bbox_gap_m observed in this run.
    # Smaller value = closer to a collision; <=0 means OBBs touched/overlapped
    # at least once. Optional because some runs (early crashes, scene-only
    # smoke tests) won't have any [EvalGT] records.
    bbox_gap_m_min: Optional[float]
    # Positions (from EgoStart / FellowPlacement records).
    ego_start_xy: Optional[str]
    opp_start_xy: Optional[str]
    sampled_gap_m: Optional[float]
    # Tactical activity (from Commit records, one per control tick).
    commit_pass_success_count: int
    commit_pass_left_count: int
    commit_pass_right_count: int
    commit_abort_pass_count: int
    guard_emergency_stable_count: int
    # Strategy distribution (from Strategy records).
    selected_stay_optimal: int
    selected_follow_fellow: int
    selected_pass_left: int
    selected_pass_right: int
    # Per-tick perf (from TickTime records).
    tick_count: int
    tick_ms_p50: Optional[float]
    # Continuous off-track robustness, derived from ``BoundsCheck`` records.
    # Unified signed-distance signal:
    #     in_track=1 frame  ->  margin = +min(d_in, d_out)   (distance to nearest edge; positive)
    #     in_track=0 frame  ->  margin = -min(d_in, d_out)   (depth past nearest edge; negative)
    # ``track_clearance_m`` is the MIN of ``margin`` across all frames in the
    # run, so lower = closer to / deeper-in violation. Optional because runs
    # without a BoundsCheck stream (scene-only smoke tests) have no frames.
    track_clearance_m: Optional[float] = None


# ---------------------------------------------------------------------------
# Records -> SampleMetrics
# ---------------------------------------------------------------------------

# Tags emitted by the racing pipeline. Defined here for documentation /
# discoverability; ``_records_extract`` only reads the ones it knows about.
_RECORD_TAGS = (
    # SD-37: removed 'EvalEventDiag' (folded into EvalEvent fields) and
    # 'EvalContact' (deleted; was redundant with EvalEvent + EvalGT and
    # never read by _records_extract).
    'EvalGT', 'EvalEvent',
    'Commit', 'Strategy', 'TickTime', 'BoundsCheck',
    'Guard', 'EgoStart', 'FellowPlacement',
)


def _entries(records, tag) -> List[dict]:
    """Return the list of payload dicts stored under ``records[tag]``.

    ``records`` is either a ``defaultdict(list)`` (live ``simulation.records``)
    or a plain dict (``SimulationResult.records``). Each value is a list of
    ``(currentTime, payload_dict)`` tuples appended by ``_record_event``.
    Non-dict payloads are filtered out so a stray scalar doesn't break the
    extractor.
    """
    try:
        raw = records.get(tag, []) if hasattr(records, 'get') else records[tag]
    except Exception:
        raw = []
    return [p for (_t, p) in (raw or []) if isinstance(p, dict)]


def _records_extract(records) -> dict:
    """Extract every SampleMetrics field that has a record source.

    Returns a dict keyed by SampleMetrics field name. Caller fills in the
    fields without record sources (``return_code``, ``log_path``, etc.).
    """
    # ---- Collision / bbox_gap_min from EvalEvent + EvalGT.
    collision = any(
        p.get('type') == 'eval_contact' for p in _entries(records, 'EvalEvent')
    )
    gap_min: Optional[float] = None
    for p in _entries(records, 'EvalGT'):
        v = p.get('bbox_gap_m')
        if v is None:
            continue
        try:
            v = float(v)
        except (TypeError, ValueError):
            continue
        if gap_min is None or v < gap_min:
            gap_min = v
        # Mirrors the legacy regex behavior: a sub-zero gap also flags collision
        # when EvalEvent didn't fire (e.g. single-tick numerical zeros).
        if not collision and v < 0.0:
            collision = True

    # ---- Off-track + signed track clearance from BoundsCheck.
    off_track = False
    clearance: Optional[float] = None
    for p in _entries(records, 'BoundsCheck'):
        in_track = bool(p.get('in_track', True))
        if not in_track:
            off_track = True
        try:
            d_in = float(p.get('d_in_m'))
            d_out = float(p.get('d_out_m'))
        except (TypeError, ValueError):
            continue
        edge_dist = min(d_in, d_out)
        margin = edge_dist if in_track else -edge_dist
        if clearance is None or margin < clearance:
            clearance = margin

    # ---- Commit lifecycle counts.
    pass_success_count = 0
    pass_left_count = 0
    pass_right_count = 0
    abort_count = 0
    for p in _entries(records, 'Commit'):
        if p.get('pass_success'):
            pass_success_count += 1
        reason = str(p.get('decision_reason', '') or '')
        if reason in ('commit_pass_left_hold', 'strategy_pass_left'):
            pass_left_count += 1
        elif reason in ('commit_pass_right_hold', 'strategy_pass_right'):
            pass_right_count += 1
        elif reason in ('abort_pass', 'abort_hold',
                        'abort_commit_invalidated', 'abort_recover_follow'):
            abort_count += 1

    # ---- Stability guard: count entries where emergency_stable_mode fired.
    guard_emergency = sum(
        1 for p in _entries(records, 'Guard') if p.get('emergency_stable_mode')
    )

    # ---- Strategy distribution.
    sel_counts: Dict[str, int] = {
        'stay_optimal': 0, 'follow_fellow': 0,
        'pass_left': 0, 'pass_right': 0,
    }
    for p in _entries(records, 'Strategy'):
        name = str(p.get('selected', '') or '')
        if name in sel_counts:
            sel_counts[name] += 1

    # ---- Per-tick wallclock timing.
    tick_ms_values: List[float] = []
    for p in _entries(records, 'TickTime'):
        v = p.get('tick_ms')
        if v is None:
            continue
        try:
            tick_ms_values.append(float(v))
        except (TypeError, ValueError):
            continue
    tick_ms_p50: Optional[float] = None
    if tick_ms_values:
        sorted_v = sorted(tick_ms_values)
        tick_ms_p50 = sorted_v[len(sorted_v) // 2]

    # ---- Ego start position (first / only EgoStart record).
    ego_xy: Optional[str] = None
    ego_entries = _entries(records, 'EgoStart')
    if ego_entries:
        p0 = ego_entries[0]
        try:
            ego_xy = f"({float(p0['x']):.4f}, {float(p0['y']):.4f})"
        except (KeyError, TypeError, ValueError):
            ego_xy = None

    # ---- Fellow placement (first FellowPlacement record).
    fellow_xy: Optional[str] = None
    sampled_gap: Optional[float] = None
    fellow_entries = _entries(records, 'FellowPlacement')
    if fellow_entries:
        p0 = fellow_entries[0]
        s = p0.get('s')
        t = p0.get('t')
        if s is not None and t is not None:
            try:
                fellow_xy = f"s={float(s):.2f}, t={float(t):.2f}"
            except (TypeError, ValueError):
                fellow_xy = None
        try:
            sampled_gap = float(p0.get('gap_m')) if p0.get('gap_m') is not None else None
        except (TypeError, ValueError):
            sampled_gap = None

    return dict(
        collision=collision,
        off_track=off_track,
        bbox_gap_m_min=gap_min,
        track_clearance_m=clearance,
        commit_pass_success_count=pass_success_count,
        commit_pass_left_count=pass_left_count,
        commit_pass_right_count=pass_right_count,
        commit_abort_pass_count=abort_count,
        guard_emergency_stable_count=guard_emergency,
        selected_stay_optimal=sel_counts['stay_optimal'],
        selected_follow_fellow=sel_counts['follow_fellow'],
        selected_pass_left=sel_counts['pass_left'],
        selected_pass_right=sel_counts['pass_right'],
        tick_count=len(tick_ms_values),
        tick_ms_p50=tick_ms_p50,
        ego_start_xy=ego_xy,
        opp_start_xy=fellow_xy,
        sampled_gap_m=sampled_gap,
    )


def parse_sample(
    idx: int,
    seed: int,
    log_path: Path,
    return_code: int,
    *,
    records,
) -> SampleMetrics:
    """Build a :class:`SampleMetrics` from a finished simulation's records.

    ``records`` is required; pass ``simulation.result.records`` (or the
    live ``simulation.records`` defaultdict). ``log_path`` is recorded
    on the result for debugging breadcrumbs (``error_table.csv`` writes
    it so a violating sample can be replayed against its log) but is
    never read.

    ``lap_time_s`` is left at ``None`` — no behavior emits a ``LapTime``
    record today. Add a record event in the lap-completion code path
    if a future scenario needs it.
    """
    rec = _records_extract(records)
    return SampleMetrics(
        sample_index=idx,
        seed=seed,
        return_code=return_code,
        log_path=str(log_path),
        collision=rec['collision'],
        off_track=rec['off_track'],
        lap_time_s=None,
        bbox_gap_m_min=rec['bbox_gap_m_min'],
        ego_start_xy=rec['ego_start_xy'],
        opp_start_xy=rec['opp_start_xy'],
        sampled_gap_m=rec['sampled_gap_m'],
        commit_pass_success_count=rec['commit_pass_success_count'],
        commit_pass_left_count=rec['commit_pass_left_count'],
        commit_pass_right_count=rec['commit_pass_right_count'],
        commit_abort_pass_count=rec['commit_abort_pass_count'],
        guard_emergency_stable_count=rec['guard_emergency_stable_count'],
        selected_stay_optimal=rec['selected_stay_optimal'],
        selected_follow_fellow=rec['selected_follow_fellow'],
        selected_pass_left=rec['selected_pass_left'],
        selected_pass_right=rec['selected_pass_right'],
        tick_count=rec['tick_count'],
        tick_ms_p50=rec['tick_ms_p50'],
        track_clearance_m=rec['track_clearance_m'],
    )


# ---------------------------------------------------------------------------
# Summary writers
# ---------------------------------------------------------------------------

def write_summary_csv(out_csv: Path, samples: List[SampleMetrics]) -> None:
    fields = [
        "sample_index", "seed", "return_code", "log_path",
        "collision", "off_track", "lap_time_s", "bbox_gap_m_min",
        "track_clearance_m",
        "ego_start_xy", "opp_start_xy", "sampled_gap_m",
        "commit_pass_success_count", "commit_pass_left_count",
        "commit_pass_right_count", "commit_abort_pass_count",
        "guard_emergency_stable_count",
        "selected_stay_optimal", "selected_follow_fellow",
        "selected_pass_left", "selected_pass_right",
        "tick_count", "tick_ms_p50",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for s in samples:
            w.writerow({k: getattr(s, k) for k in fields})


def write_summary_text(out_txt: Path, samples: List[SampleMetrics],
                       scenic_file: Path, base_seed: int) -> None:
    lines = []
    lines.append(f"Sampled bank summary: {scenic_file.name}")
    lines.append(f"  base_seed={base_seed}, samples={len(samples)}")
    lines.append("")
    lines.append("Per-sample:")
    for s in samples:
        outcome = "COLLISION" if s.collision else ("off_track" if s.off_track else "OK")
        gap_s = f"gap={s.sampled_gap_m:.1f}m" if s.sampled_gap_m is not None else "gap=?"
        lap_s = f"lap={s.lap_time_s:.2f}s" if s.lap_time_s is not None else "lap=?"
        tick_s = f"p50_ms={s.tick_ms_p50:.1f}" if s.tick_ms_p50 is not None else "p50_ms=?"
        lines.append(
            f"  #{s.sample_index:03d} seed={s.seed} rc={s.return_code} {outcome} "
            f"{gap_s} {lap_s} {tick_s} "
            f"commits=L{s.commit_pass_left_count}/R{s.commit_pass_right_count}/"
            f"S{s.commit_pass_success_count}/A{s.commit_abort_pass_count} "
            f"strat=opt{s.selected_stay_optimal}/foll{s.selected_follow_fellow}/"
            f"pL{s.selected_pass_left}/pR{s.selected_pass_right}"
        )
    n = len(samples)
    if n > 0:
        ok = sum(1 for s in samples if not s.collision and not s.off_track)
        coll = sum(1 for s in samples if s.collision)
        succ_total = sum(s.commit_pass_success_count for s in samples)
        lines.append("")
        lines.append("Aggregates:")
        lines.append(f"  ok: {ok}/{n} ({100*ok/n:.0f}%)")
        lines.append(f"  collision: {coll}/{n} ({100*coll/n:.0f}%)")
        lines.append(f"  total commit_pass_success: {succ_total}")
    out_txt.write_text("\n".join(lines) + "\n", encoding="utf-8")
