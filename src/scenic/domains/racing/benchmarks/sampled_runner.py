"""SD-15c: sampled batch runner for the falsifiable racing pipeline.

Runs a single Scenic scenario N times with monotonically-incrementing
seeds (base_seed + i for sample i). Each invocation samples a fresh
starting layout from the scenic file's distributions (e.g. ego placement
on mainTrack, fellow gap Range(20, 60), etc.) and runs a full simulation.

Output mirrors the F-bank's full_stack_<timestamp>/ directory layout, so
analysis tooling that reads full_stack outputs works on sampled outputs
too:

    benchmarks/results/sampled_<TIMESTAMP>/
        logs/sample_001.log
        logs/sample_002.log
        ...
        summary.csv
        summary.txt

Each log captures one full simulation run. The summary parses the
relevant per-sample metrics into one row.

USAGE:
    python src/scenic/domains/racing/benchmarks/sampled_runner.py \\
        examples/racing/sampled/S1_fellow_left_ahead.scenic \\
        --count 10 --seed 42 --time 3000

Why per-sample subprocess (instead of `scenic --count N` in one process)?
  - Per-sample log isolation: failures in sample i don't poison sample i+1.
  - Reproducible single-sample reruns: `--seed 42+i` yields the SAME layout
    as sample i in the batch, so debugging a specific failure is one
    command away.
  - Cosim warm-start (SD-10o) means subsequent invocations share the
    persistent VEOS bridge — ~10 s setup vs ~38 s cold per sample.
"""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional


# Force stdout/stderr to UTF-8 so any non-ASCII chars don't crash when the
# runner's output is redirected to a file under PowerShell on Windows
# (default cp1252 codec can't encode Unicode like '->' arrows or em-dashes).
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# ---------------------------------------------------------------------------
# Per-sample metric extraction
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
    # Positions
    ego_start_xy: Optional[str]
    opp_start_xy: Optional[str]
    sampled_gap_m: Optional[float]
    # Tactical activity
    commit_pass_success_count: int
    commit_pass_left_count: int
    commit_pass_right_count: int
    commit_abort_pass_count: int
    guard_emergency_stable_count: int
    # Strategy distribution
    selected_stay_optimal: int
    selected_follow_fellow: int
    selected_pass_left: int
    selected_pass_right: int
    # Per-tick perf
    tick_count: int
    tick_ms_p50: Optional[float]


_RE_TICKTIME = re.compile(r"\[TickTime\] t=([\d.]+)s wall_t=[\d.]+s tick_ms=([\d.]+)")
_RE_STRATEGY_SELECTED = re.compile(r"\[Strategy\] t=[\d.]+s selected=([a-z_]+)")
_RE_COMMIT_REASON = re.compile(r"decision_reason=(commit_pass_left|commit_pass_right|pass_success_free_run)")
_RE_BBOX_GAP = re.compile(r"bbox_gap_m=(-?[\d.]+)")
_RE_LAP_TIME = re.compile(r"lap_time_s=([\d.]+)")
_RE_EGO_POS = re.compile(r"\[Ego\] (?:set position|debug) xy=\(([-\d.]+),\s*([-\d.]+)\)")
_RE_FELLOW_POS = re.compile(r"\[Placement\] [Ff]ellow.*resolved.*\(([-\d.]+),\s*([-\d.]+)\)")
_RE_GAP = re.compile(r"_racing_st_offset=\(([\d.]+)")
_RE_OFF_TRACK = re.compile(r"in_track=0|out_of_track")


def _parse_log_utf16(path: Path) -> str:
    """Decode a Scenic log file. Scenic on Windows writes UTF-16-LE; fallback to UTF-8."""
    raw = path.read_bytes()
    try:
        return raw.decode("utf-16-le", "replace")
    except Exception:
        return raw.decode("utf-8", "replace")


def parse_sample(idx: int, seed: int, log_path: Path, return_code: int) -> SampleMetrics:
    """Extract per-sample metrics from one Scenic+sim log."""
    text = _parse_log_utf16(log_path) if log_path.exists() else ""

    # Collision: any tick with bbox_gap_m <= 0 (boxes touching/overlapping).
    collision = False
    for m in _RE_BBOX_GAP.finditer(text):
        try:
            if float(m.group(1)) <= 0.0:
                collision = True
                break
        except ValueError:
            continue

    off_track = bool(_RE_OFF_TRACK.search(text))

    # Lap time (last value).
    lap_time = None
    for m in _RE_LAP_TIME.finditer(text):
        try:
            lap_time = float(m.group(1))
        except ValueError:
            pass

    # First ego position observed.
    ego_xy = None
    em = _RE_EGO_POS.search(text)
    if em:
        ego_xy = f"({em.group(1)}, {em.group(2)})"
    fellow_xy = None
    fm = _RE_FELLOW_POS.search(text)
    if fm:
        fellow_xy = f"({fm.group(1)}, {fm.group(2)})"

    # Sampled gap (the Scenic Range(20, 60) resolved value, surfaced via
    # placement debug log).
    gap_m = None
    gm = _RE_GAP.search(text)
    if gm:
        try:
            gap_m = float(gm.group(1))
        except ValueError:
            pass

    # Commit / strategy counts.
    commit_pass_success = text.count("decision_reason=pass_success_free_run")
    commit_pass_left = text.count("decision_reason=commit_pass_left_hold") + text.count("decision_reason=commit_pass_left")
    commit_pass_right = text.count("decision_reason=commit_pass_right_hold") + text.count("decision_reason=commit_pass_right")
    commit_abort = text.count("decision_reason=abort_pass") + text.count("decision_reason=abort_hold")
    guard_emergency = text.count("emergency_stable_mode=1")

    # Strategy distribution.
    sel_counts = {"stay_optimal": 0, "follow_fellow": 0, "pass_left": 0, "pass_right": 0}
    for m in _RE_STRATEGY_SELECTED.finditer(text):
        name = m.group(1)
        if name in sel_counts:
            sel_counts[name] += 1

    # Per-tick wallclock timing.
    tick_ms_values: List[float] = []
    for m in _RE_TICKTIME.finditer(text):
        try:
            tick_ms_values.append(float(m.group(2)))
        except ValueError:
            pass
    tick_ms_p50 = None
    if tick_ms_values:
        sorted_v = sorted(tick_ms_values)
        tick_ms_p50 = sorted_v[len(sorted_v) // 2]

    return SampleMetrics(
        sample_index=idx,
        seed=seed,
        return_code=return_code,
        log_path=str(log_path),
        collision=collision,
        off_track=off_track,
        lap_time_s=lap_time,
        ego_start_xy=ego_xy,
        opp_start_xy=fellow_xy,
        sampled_gap_m=gap_m,
        commit_pass_success_count=commit_pass_success,
        commit_pass_left_count=commit_pass_left,
        commit_pass_right_count=commit_pass_right,
        commit_abort_pass_count=commit_abort,
        guard_emergency_stable_count=guard_emergency,
        selected_stay_optimal=sel_counts["stay_optimal"],
        selected_follow_fellow=sel_counts["follow_fellow"],
        selected_pass_left=sel_counts["pass_left"],
        selected_pass_right=sel_counts["pass_right"],
        tick_count=len(tick_ms_values),
        tick_ms_p50=tick_ms_p50,
    )


# ---------------------------------------------------------------------------
# Batch runner
# ---------------------------------------------------------------------------

def run_one_sample(
    scenic_file: Path,
    sample_idx: int,
    seed: int,
    time_steps: int,
    out_log: Path,
    extra_args: Iterable[str] = (),
) -> int:
    """Invoke `scenic --simulate --count 1 --seed S --time T` for one sample.

    Stdout+stderr captured to out_log via shell redirection (UTF-16-LE on
    Windows because Scenic emits via PowerShell which uses UTF-16).
    """
    cmd = [
        "scenic", str(scenic_file),
        "--2d",
        "--model", "scenic.simulators.dspace.racing_model",
        "--simulate",
        "--count", "1",
        "--seed", str(seed),
        "--time", str(time_steps),
    ] + list(extra_args)
    print(f"[SampledRunner] sample {sample_idx:03d} seed={seed} -> {out_log.name}")
    print(f"[SampledRunner]   cmd: {' '.join(cmd)}")
    t0 = time.perf_counter()
    with out_log.open("wb") as fh:
        proc = subprocess.run(
            cmd,
            stdout=fh,
            stderr=subprocess.STDOUT,
            cwd=Path.cwd(),
        )
    elapsed = time.perf_counter() - t0
    print(f"[SampledRunner]   rc={proc.returncode} elapsed={elapsed:.1f}s")
    return proc.returncode


def write_summary_csv(out_csv: Path, samples: List[SampleMetrics]) -> None:
    fields = [
        "sample_index", "seed", "return_code", "log_path",
        "collision", "off_track", "lap_time_s",
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
    # Aggregates.
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


def main() -> int:
    p = argparse.ArgumentParser(
        description="Run a Scenic scenario N times with monotonically-incrementing "
                    "seeds. Output mirrors the F-bank's full_stack_<timestamp>/ layout."
    )
    p.add_argument("scenic_file", type=str, help="Path to .scenic file (uses Range/on/etc. for sampling)")
    p.add_argument("--count", "-n", type=int, default=10, help="Number of samples (default 10)")
    p.add_argument("--seed", type=int, default=42, help="Base seed; sample i uses seed=base_seed+i (default 42)")
    p.add_argument("--time", type=int, default=3000, dest="time_steps",
                   help="Simulation duration in time steps (default 3000 = 30s @ 100Hz)")
    p.add_argument("--results-root", type=str,
                   default="src/scenic/domains/racing/benchmarks/results",
                   help="Root directory for the timestamped output folder")
    p.add_argument("--label", type=str, default="sampled",
                   help="Prefix for the output dir name (default 'sampled')")
    p.add_argument("--scenic-extra-arg", action="append", default=[],
                   help="Pass-through extra arg(s) to scenic (repeatable)")

    args = p.parse_args()
    scenic_file = Path(args.scenic_file)
    if not scenic_file.is_file():
        print(f"[SampledRunner] ERROR: scenic file not found: {scenic_file}", file=sys.stderr)
        return 2

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.results_root) / f"{args.label}_{timestamp}"
    log_dir = out_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"[SampledRunner] output dir: {out_dir}")
    print(f"[SampledRunner] scenic file: {scenic_file}")
    print(f"[SampledRunner] count={args.count} base_seed={args.seed} time_steps={args.time_steps}")

    # Snapshot the scenic file inside the output dir for full reproducibility.
    shutil.copy(scenic_file, out_dir / scenic_file.name)

    samples: List[SampleMetrics] = []
    for i in range(args.count):
        seed = args.seed + i
        out_log = log_dir / f"sample_{i+1:03d}.log"
        rc = run_one_sample(
            scenic_file=scenic_file,
            sample_idx=i + 1,
            seed=seed,
            time_steps=args.time_steps,
            out_log=out_log,
            extra_args=args.scenic_extra_arg,
        )
        samples.append(parse_sample(i + 1, seed, out_log, rc))

    # Outputs.
    write_summary_csv(out_dir / "summary.csv", samples)
    write_summary_text(out_dir / "summary.txt", samples, scenic_file, args.seed)

    print(f"\n[SampledRunner] DONE.")
    print(f"  summary.csv: {out_dir / 'summary.csv'}")
    print(f"  summary.txt: {out_dir / 'summary.txt'}")
    print((out_dir / "summary.txt").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
