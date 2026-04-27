"""Backward-compatible alias for the Phase 3 tactical benchmark.

The Phase 0–aligned bank now lives under ``examples/racing/tactical_tactical/``.
Prefer:

    python -m scenic.domains.racing.benchmarks.tactical_runner

This module runs the **same scenarios and CSV columns** as ``tactical_runner``,
but keeps ``run_id_prefix=tactical_on_phase0`` so existing automation that keys
off that prefix or the ``Phase3OnPhase0Runner`` label still works.
"""

from dataclasses import replace

from scenic.domains.racing.benchmarks.tactical_runner import PHASE3_RUNNER_SPEC
from scenic.domains.racing.benchmarks.phase_run_common import run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            replace(
                PHASE3_RUNNER_SPEC,
                runner_label="Phase3OnPhase0Runner",
                run_id_prefix="tactical_on_phase0",
            )
        )
    )
