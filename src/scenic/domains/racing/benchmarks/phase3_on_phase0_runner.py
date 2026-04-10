"""Backward-compatible alias for the Phase 3 tactical benchmark.

The Phase 0–aligned bank now lives under ``examples/racing/phase3_tactical/``.
Prefer:

    python -m scenic.domains.racing.benchmarks.phase3_runner

This module runs the **same scenarios and CSV columns** as ``phase3_runner``,
but keeps ``run_id_prefix=phase3_on_phase0`` so existing automation that keys
off that prefix or the ``Phase3OnPhase0Runner`` label still works.
"""

from dataclasses import replace

from scenic.domains.racing.benchmarks.phase3_runner import PHASE3_RUNNER_SPEC
from scenic.domains.racing.benchmarks.phase_run_common import run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            replace(
                PHASE3_RUNNER_SPEC,
                runner_label="Phase3OnPhase0Runner",
                run_id_prefix="phase3_on_phase0",
            )
        )
    )
