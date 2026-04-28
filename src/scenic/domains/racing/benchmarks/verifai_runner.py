"""SD-16: in-process active-falsification driver for the racing domain.

Sits alongside `sampled_runner.py`. Where sampled_runner spawns a fresh
Scenic subprocess per sample (uniform/halton sampling, bulletproof per-sample
isolation, ~38 s cosim cold-start tax per iteration), this driver compiles
the scenario ONCE and loops in-process so the cosim bridge stays warm:

    seed = base_seed
    scenario = scenarioFromFile(...)
    simulator = scenario.getSimulator()       # spawns IPC client (~10-38 s ONCE)
    feedback = None
    for i in range(count):
        scene = scenario.generate(feedback=feedback)   # VerifAI sampler reads feedback
        run simulator.simulate(scene); capture stdout
        metrics = parse_sample(captured_log)
        feedback = monitor(metrics)                    # robustness scalar
        if feedback <= violation_threshold: append to error_table

Active samplers (cross-entropy, Bayesian opt) update their internal model
on each feedback and bias the next sample toward parameter regions where
the previous sample came closest to violation. After ~50-200 iterations
the campaign concentrates on the corners of the input space that break
the planner -- targeted edge-case discovery rather than uniform coverage.

USAGE:
    # 1) Smoke -- Halton sampler is deterministic; proves wiring.
    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/S1_falsify.scenic \\
        --sampler halton --monitor min --count 3 --seed 42 --time 1500

    # 2) Real falsification -- cross-entropy converges over ~100-500 samples.
    python src/scenic/domains/racing/benchmarks/verifai_runner.py \\
        examples/racing/falsifiable/S1_falsify.scenic \\
        --sampler ce --monitor min --count 200 --seed 42 --time 3000

PRECONDITIONS:
    - VEOS + ModelDesk + ControlDesk launched externally (user side).
    - The Scenic process spawns its own IPC client and connects to the
      pre-launched VEOS once; that connection persists for the whole
      campaign because we keep one Scenic Python process alive.
    - VerifAI is installed (`pip install verifai`; pyproject.toml lists
      it under the test-full extra).
    - The scenic file uses `VerifaiRange(...)` (or sibling) for the
      parameters you want VerifAI to control. Plain `Range(...)` is
      still available for parameters that should remain uniform.

NOTE on RNG: Scenic's `random` module is seeded once at scenario compile
time. Non-VerifAI distributions in the .scenic file (e.g. ego placement
sampled uniformly over a TTL region) advance per the global RNG and
therefore depend on iteration index. By design -- VerifAI only controls
parameters wrapped in `VerifaiRange`/`VerifaiOptions`/etc.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import shutil
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Force UTF-8 stdout so non-ASCII chars (arrows, em-dashes) don't crash
# when the runner output is redirected to a file under PowerShell.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import scenic
from scenic.core.distributions import RejectionException

from scenic.domains.racing.benchmarks import monitors
from scenic.domains.racing.benchmarks.sampled_runner import (
    SampleMetrics,
    parse_sample,
    write_summary_csv,
    write_summary_text,
)


# ---------------------------------------------------------------------------
# Tee: fan stdout writes to BOTH the original terminal and an in-memory buf
# ---------------------------------------------------------------------------

class _Tee:
    """File-like that duplicates writes to multiple streams.

    Crucial: `sys.__stdout__` is the *original* stdout (preserved across
    redirect_stdout context-manager swaps), so the user still sees live
    progress on the terminal even while we snapshot the same bytes into
    a per-iteration buffer. If we wrote to `sys.stdout` instead we'd
    recurse forever once `redirect_stdout(self)` is active.
    """

    def __init__(self, *streams):
        self._streams = streams

    def write(self, s):
        for stream in self._streams:
            try:
                stream.write(s)
            except Exception:
                pass

    def flush(self):
        for stream in self._streams:
            try:
                stream.flush()
            except Exception:
                pass

    def isatty(self):
        try:
            return self._streams[0].isatty()
        except Exception:
            return False


@contextlib.contextmanager
def _tee_stdout(buf: io.StringIO, quiet: bool = False):
    """Redirect stdout so writes go to `buf` (and optionally the terminal).

    `quiet=False` (default): writes go to BOTH `sys.__stdout__` (live terminal)
    and `buf` (per-sample log). Useful when watching a run interactively.

    `quiet=True`: writes go ONLY to `buf`. The terminal sees only the runner's
    own [VerifaiRunner] progress lines (those `print()` calls live outside
    this with-block, so they bypass the redirect entirely). Per-sample log
    files are still complete -- the simulator's stdout is captured into `buf`
    and persisted to `logs/sample_NNN.log` regardless of mode. Use this when
    redirecting the runner's output to a file under PowerShell to keep the
    captured `run.log` from ballooning to ~14 k lines per sample.
    """
    targets = (buf,) if quiet else (sys.__stdout__, buf)
    tee = _Tee(*targets)
    with contextlib.redirect_stdout(tee):
        yield


# ---------------------------------------------------------------------------
# Output rows
# ---------------------------------------------------------------------------

@dataclass
class ErrorRow:
    """One row in error_table.csv -- a parameter sample that violated a monitor.

    Sorted ascending by `rho` so the most-violating sample is at the top.
    `sampled_values` is the dict of {param_name: float} VerifAI assigned this
    iteration; copy-pasting them back into the .scenic file (or fixing the
    seed and re-running) reproduces the failure for debugging.
    """

    sample_index: int
    seed: int
    rho: float
    sampled_values: Dict[str, Any]
    metrics: SampleMetrics


# ---------------------------------------------------------------------------
# VerifAI value extraction
# ---------------------------------------------------------------------------

def _extract_verifai_values(scenario) -> Dict[str, float]:
    """Read the values VerifAI sampled this iteration.

    `scenario.externalSampler.cachedSample` is a DotMap with attributes
    `param0`, `param1`, ..., one per VerifaiParameter in source order
    (see `VerifaiSampler.nameForParam` at scenic/core/external_params.py).
    We don't have the original source identifiers (Scenic doesn't preserve
    them on the sampler), so error_table records `param0=<value>` etc.
    For single-knob scenarios like S1 the mapping is trivial.
    """
    sampler = getattr(scenario, "externalSampler", None)
    if sampler is None or getattr(sampler, "cachedSample", None) is None:
        return {}
    out: Dict[str, float] = {}
    for i, _param in enumerate(getattr(sampler, "params", ())):
        name = sampler.nameForParam(i)
        try:
            value = getattr(sampler.cachedSample, name)
        except AttributeError:
            continue
        # VerifAI's halton/CE samplers return each parameter slot as a
        # 1-tuple (one feature per slot); unwrap to a scalar so the
        # error_table JSON is `{"param0": 40.0}` not `{"param0": [40.0]}`.
        if isinstance(value, tuple) and len(value) == 1:
            value = value[0]
        try:
            out[name] = float(value)
        except (TypeError, ValueError):
            out[name] = value
    return out


# ---------------------------------------------------------------------------
# Per-iteration simulation
# ---------------------------------------------------------------------------

def _run_one_simulation(simulator, scene, time_steps: int, sample_idx: int) -> bool:
    """Invoke the simulator on one sampled scene; return True iff it ran."""
    try:
        simulation = simulator.simulate(
            scene,
            maxSteps=time_steps,
            name=str(sample_idx),
            verbosity=1,
        )
    except Exception as exc:
        traceback.print_exc()
        print(f"[VerifaiRunner] simulate() raised: {exc}")
        return False
    return simulation is not None


# ---------------------------------------------------------------------------
# error_table.csv writer
# ---------------------------------------------------------------------------

def _write_error_table(out_csv: Path, rows: List[ErrorRow]) -> None:
    """Write the falsification error table, ranked by rho ascending.

    Schema: sample_index, seed, rho, sampled_values_json, collision,
            bbox_gap_m_min, off_track, commit_pass_success_count,
            commit_abort_pass_count, log_path.
    """
    fields = [
        "sample_index", "seed", "rho", "sampled_values_json",
        "collision", "bbox_gap_m_min", "off_track",
        "commit_pass_success_count", "commit_abort_pass_count",
        "log_path",
    ]
    with out_csv.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({
                "sample_index": r.sample_index,
                "seed": r.seed,
                "rho": r.rho,
                "sampled_values_json": json.dumps(r.sampled_values),
                "collision": r.metrics.collision,
                "bbox_gap_m_min": r.metrics.bbox_gap_m_min,
                "off_track": r.metrics.off_track,
                "commit_pass_success_count": r.metrics.commit_pass_success_count,
                "commit_abort_pass_count": r.metrics.commit_abort_pass_count,
                "log_path": r.metrics.log_path,
            })


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_extra_params(items: List[str]) -> Dict[str, Any]:
    """Parse repeated `--extra-param KEY=VALUE` into a dict.

    Best-effort type inference: tries int -> float -> bool -> str.
    """
    out: Dict[str, Any] = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"--extra-param must be KEY=VALUE (got: {item!r})")
        k, v = item.split("=", 1)
        for cast in (int, float):
            try:
                out[k] = cast(v)
                break
            except ValueError:
                continue
        else:
            if v.lower() in ("true", "false"):
                out[k] = (v.lower() == "true")
            else:
                out[k] = v
    return out


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Active-falsification driver. Wraps Scenic's VerifaiSampler "
                    "in a feedback loop that biases sampling toward layouts "
                    "violating a robustness monitor."
    )
    p.add_argument("scenic_file", type=str,
                   help="Path to .scenic file. Must contain VerifaiRange/etc.")

    g_samp = p.add_argument_group("Sampler")
    g_samp.add_argument("--sampler", default="halton",
                        choices=["halton", "ce", "bo", "random"],
                        help="VerifAI sampler type (default 'halton'; use 'ce' "
                             "for active falsification once wiring is verified)")
    g_samp.add_argument("--sampler-params", default="{}",
                        help="JSON dict of sampler-specific options "
                             "(merged into a DotMap).")
    g_samp.add_argument("--monitor", default="min",
                        choices=list(monitors.RESOLVE.keys()),
                        help="Robustness function (default 'min' = composite).")
    g_samp.add_argument("--violation-threshold", type=float, default=0.0,
                        help="rho <= this -> append to error_table (default 0.0)")

    g_run = p.add_argument_group("Run control")
    g_run.add_argument("--count", "-n", type=int, default=50,
                       help="Number of samples (default 50; CE wants 100-500)")
    g_run.add_argument("--seed", type=int, default=42,
                       help="Base seed; sample i uses seed=base+i for log/csv "
                            "labelling (the actual Scenic RNG is seeded once "
                            "at scenarioFromFile by the global random module)")
    g_run.add_argument("--time", type=int, default=3000, dest="time_steps",
                       help="Simulation duration in time steps (default 3000)")
    g_run.add_argument("--max-consecutive-failures", type=int, default=5,
                       help="Abort campaign after this many consecutive simulator "
                            "failures (likely dead bridge). Default 5.")

    g_out = p.add_argument_group("Output")
    g_out.add_argument("--results-root", type=str,
                       default="src/scenic/domains/racing/benchmarks/results",
                       help="Root for the timestamped output dir.")
    g_out.add_argument("--label", type=str, default="verifai",
                       help="Prefix for the output dir name (default 'verifai').")

    g_extra = p.add_argument_group("Extra")
    g_extra.add_argument("--extra-param", action="append", default=[],
                         help="KEY=VALUE pair passed through to scenarioFromFile "
                              "params (repeatable). Useful for overriding things "
                              "like 'time_step', 'fellowHarnessLog', etc.")
    g_extra.add_argument("--model", default="scenic.simulators.dspace.racing_model",
                         help="World-model module (default dspace.racing_model).")
    g_extra.add_argument("--quiet", "-q", action="store_true",
                         help="Suppress per-sample simulator stdout from the "
                              "terminal -- per-sample logs/sample_NNN.log files "
                              "still capture everything. Recommended when "
                              "redirecting runner output to a file (run.log) so "
                              "the capture stays compact.")

    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()
    scenic_file = Path(args.scenic_file)
    if not scenic_file.is_file():
        print(f"[VerifaiRunner] ERROR: scenic file not found: {scenic_file}",
              file=sys.stderr)
        return 2

    # ---- output dirs
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.results_root) / f"{args.label}_{timestamp}"
    log_dir = out_dir / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"[VerifaiRunner] output dir:    {out_dir}")
    print(f"[VerifaiRunner] scenic file:   {scenic_file}")
    print(f"[VerifaiRunner] sampler:       {args.sampler}  monitor: {args.monitor}")
    print(f"[VerifaiRunner] count:         {args.count}    seed: {args.seed}")
    print(f"[VerifaiRunner] time steps:    {args.time_steps}")

    shutil.copy(scenic_file, out_dir / scenic_file.name)

    # ---- compile scenario once, with VerifAI sampler config injected via params
    sampler_params_raw = json.loads(args.sampler_params) if args.sampler_params else {}
    extra_params = _parse_extra_params(args.extra_param)

    # `verifaiSamplerParams` is consumed by Scenic's VerifaiSampler as a DotMap,
    # but JSON gives us a plain dict. The sampler does the DotMap conversion
    # itself in __init__ -- a plain dict here works fine.
    params: Dict[str, Any] = {
        "verifaiSamplerType": args.sampler,
        **extra_params,
    }
    if sampler_params_raw:
        from dotmap import DotMap
        params["verifaiSamplerParams"] = DotMap(sampler_params_raw)

    print(f"[VerifaiRunner] compiling scenario...")
    t0 = time.perf_counter()
    scenario = scenic.scenarioFromFile(
        str(scenic_file),
        model=args.model,
        params=params,
        mode2D=True,
    )
    print(f"[VerifaiRunner] compiled in {time.perf_counter()-t0:.1f}s")

    # Sanity-check that the scenario actually has VerifAI-controlled parameters.
    if scenario.externalSampler is None:
        print(f"[VerifaiRunner] WARN: scenario has no external (Verifai*) parameters. "
              f"Sampler will not influence anything; consider wrapping at least one "
              f"value in VerifaiRange(...) in {scenic_file.name}.")

    # ---- spawn simulator (cosim bridge cold-start happens on first .simulate())
    simulator = scenario.getSimulator()

    monitor = monitors.RESOLVE[args.monitor]

    # ---- the loop
    samples: List[SampleMetrics] = []
    error_rows: List[ErrorRow] = []
    feedback: Optional[float] = None
    consecutive_failures = 0

    for i in range(args.count):
        sample_idx = i + 1
        seed = args.seed + i
        log_path = log_dir / f"sample_{sample_idx:03d}.log"
        print(f"\n[VerifaiRunner] === sample {sample_idx:03d}/{args.count} "
              f"(seed_label={seed}, feedback={feedback}) ===")

        # ---- generate scene (active sampler reads feedback)
        try:
            scene, _gen_iters = scenario.generate(feedback=feedback)
        except RejectionException as exc:
            print(f"[VerifaiRunner] sample {sample_idx:03d} rejected by Scenic: {exc}")
            log_path.write_text(
                f"[VerifaiRunner] rejection: {exc}\n", encoding="utf-8"
            )
            # Set feedback to the sampler's rejection signal so CE doesn't
            # treat the rejected sample as a real outcome.
            feedback = (
                scenario.externalSampler.rejectionFeedback
                if scenario.externalSampler is not None else None
            )
            continue

        sampled_values = _extract_verifai_values(scenario)
        if sampled_values:
            print(f"[VerifaiRunner]   sampled: {sampled_values}")

        # ---- run simulation, capturing stdout to a per-sample buffer + terminal
        buf = io.StringIO()
        t_sim0 = time.perf_counter()
        with _tee_stdout(buf, quiet=args.quiet):
            ok = _run_one_simulation(simulator, scene, args.time_steps, sample_idx)
        elapsed = time.perf_counter() - t_sim0

        # Persist captured stdout so parse_sample reads from the same source
        # of truth as sampled_runner.py would.
        log_path.write_text(buf.getvalue(), encoding="utf-8")

        rc = 0 if ok else 1
        metrics = parse_sample(sample_idx, seed, log_path, rc)
        samples.append(metrics)

        rho = monitor(metrics)
        # The 'all' multi-objective monitor returns a tuple; CE/BO want a
        # scalar. Reduce to min so feedback stays comparable across monitors.
        rho_scalar = float(min(rho)) if isinstance(rho, tuple) else float(rho)

        print(f"[VerifaiRunner]   sim ok={ok} elapsed={elapsed:.1f}s "
              f"collision={metrics.collision} bbox_gap_min={metrics.bbox_gap_m_min} "
              f"-> rho={rho_scalar:.3f}")

        if rho_scalar <= args.violation_threshold:
            error_rows.append(ErrorRow(
                sample_index=sample_idx,
                seed=seed,
                rho=rho_scalar,
                sampled_values=sampled_values,
                metrics=metrics,
            ))
            print(f"[VerifaiRunner]   *** VIOLATION (rho={rho_scalar:.3f} "
                  f"<= {args.violation_threshold}); added to error_table ***")

        feedback = rho_scalar

        # ---- circuit breaker on consecutive sim failures (likely dead bridge)
        if not ok:
            consecutive_failures += 1
            if consecutive_failures >= args.max_consecutive_failures:
                print(f"[VerifaiRunner] {consecutive_failures} consecutive "
                      f"simulation failures -- aborting campaign. The cosim "
                      f"bridge is likely dead; restart VEOS and retry.")
                break
        else:
            consecutive_failures = 0

        # ---- incremental writes so a crash doesn't lose the run
        write_summary_csv(out_dir / "summary.csv", samples)
        _write_error_table(
            out_dir / "error_table.csv",
            sorted(error_rows, key=lambda r: r.rho),
        )

    # ---- final summary
    write_summary_text(out_dir / "summary.txt", samples, scenic_file, args.seed)

    print(f"\n[VerifaiRunner] DONE.")
    print(f"  samples:        {len(samples)}/{args.count}")
    print(f"  violations:     {len(error_rows)}")
    print(f"  summary.csv:    {out_dir / 'summary.csv'}")
    print(f"  summary.txt:    {out_dir / 'summary.txt'}")
    print(f"  error_table:    {out_dir / 'error_table.csv'}")
    if error_rows:
        worst = error_rows[0] if error_rows else None
        # Re-sort to be sure even if the loop bailed mid-write.
        error_rows.sort(key=lambda r: r.rho)
        worst = error_rows[0]
        print(f"  worst rho:      {worst.rho:.3f} at sample #{worst.sample_index} "
              f"(values: {worst.sampled_values})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
