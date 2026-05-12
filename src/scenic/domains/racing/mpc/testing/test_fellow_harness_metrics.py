"""Unit tests for `collect_fellow_harness_metrics_from_log` (regex parsing only).

These tests use **synthetic** log snippets so CI does not require dSPACE. A **real**
benchmark log with ``param fellowHarnessLog = True`` should additionally contain:

- ``[Placement] ... racing (s,t) from ego`` when a fellow uses placement from ego offset
- ``[Fellow s,t]`` from projection after spawn
- Periodic ``[FellowHarness] t=...s idx=... speed_mps=... x=... y=...`` from fellow
  readback (throttled)

**Interpretation:** the parser aggregates fellow **readback**; it does not prove the
fellow's *commanded* (v,d) matches intent — compare behaviors in ``behaviors.scenic``.
**Ego** reaction shows up in benchmark ``summary.json`` (full_stack_runner output),
not in fellow-side collision detection.
"""

from pathlib import Path

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_T_OUT_OF_BAND_M,
    collect_fellow_placement_debug_metrics_from_log,
    collect_fellow_harness_metrics_from_log,
)


def test_placement_and_fellow_st_only(tmp_path: Path) -> None:
    log = tmp_path / "p.log"
    log.write_text(
        "[Placement] Fellow_0: racing (s,t) from ego + ('ahead', 40) -> s=120.50, t=0.10 (same route as ego)\n"
        "[Fellow s,t] Fellow_0: route=main xy=(1,2) -> s=120.50, t=0.10\n",
        encoding="utf-8",
    )
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_placement_from_ego_offset_observed"] is True
    assert m["fellow_st_log_present"] is True
    assert m["fellow_s0"] == 120.50
    assert m["fellow_t0"] == 0.10
    assert m["fellow_t_out_of_band"] is False
    assert m["fellow_harness_line_count"] == 0
    assert m["fellow_speed_min_mps"] is None


def test_fellow_harness_series_stats(tmp_path: Path) -> None:
    log = tmp_path / "h.log"
    log.write_text(
        "[FellowHarness] t=0.50s idx=0 speed_mps=26.800 x=100.000 y=200.000\n"
        "[FellowHarness] t=1.00s idx=0 speed_mps=27.000 x=101.000 y=201.000\n"
        "[FellowHarness] t=1.50s idx=0 speed_mps=5.000 x=102.000 y=202.000\n",
        encoding="utf-8",
    )
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_harness_line_count"] == 3
    assert m["fellow_speed_min_mps"] == 5.0
    assert m["fellow_speed_max_mps"] == 27.0
    assert m["fellow_max_speed_step_jump_mps"] == 22.0
    assert m["fellow_position_range_m"] is not None
    assert abs(m["fellow_position_range_m"] - 2.8284271247461903) < 1e-6


def test_empty_log(tmp_path: Path) -> None:
    log = tmp_path / "e.log"
    log.write_text("", encoding="utf-8")
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_placement_from_ego_offset_observed"] is False
    assert m["fellow_st_log_present"] is False
    assert m["fellow_harness_line_count"] == 0


def test_malformed_harness_lines_skipped(tmp_path: Path) -> None:
    log = tmp_path / "m.log"
    log.write_text(
        "[FellowHarness] t=broken\n"
        "[FellowHarness] t=1.00s idx=0 speed_mps=10.0 x=1.0 y=2.0\n",
        encoding="utf-8",
    )
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_harness_line_count"] == 1
    assert m["fellow_speed_min_mps"] == 10.0


def test_fellow_t_out_of_band(tmp_path: Path) -> None:
    log = tmp_path / "oob.log"
    t_bad = FELLOW_HARNESS_T_OUT_OF_BAND_M + 1.0
    log.write_text(
        f"[Fellow s,t] x -> s=0.0, t={t_bad:.1f}\n",
        encoding="utf-8",
    )
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_t_out_of_band"] is True


def test_stuck_near_zero_heuristic(tmp_path: Path) -> None:
    log = tmp_path / "stuck.log"
    lines = []
    for i in range(12):
        lines.append(
            f"[FellowHarness] t={i * 0.5:.2f}s idx=0 speed_mps=0.100 x=0.0 y=0.0\n"
        )
    log.write_text("".join(lines), encoding="utf-8")
    m = collect_fellow_harness_metrics_from_log(log)
    assert m["fellow_speed_stuck_near_zero"] is True


def test_placement_debug_metrics_ahead_and_road_mismatch(tmp_path: Path) -> None:
    log = tmp_path / "place.log"
    log.write_text(
        "[Placement] Fellow_0: racing (s,t) from ego + ('ahead', 40) -> s=479.24, t=0.98 (same route as ego)\n"
        "[Ego debug] xy=(...) -> s=439.24, t=0.98 | projected onto road_id=1 (MainTrack_TTL)\n"
        "[Fellow s,t] Fellow_0: route=Lap (R2), xy=(...) -> s=479.24, t=0.98 | "
        "distance_from_ego=360.21m, angle_from_ego_deg=-158.4 | projected onto road_id=2 (PitTrack_TTL)\n",
        encoding="utf-8",
    )
    m = collect_fellow_placement_debug_metrics_from_log(log)
    assert m["placement_command_observed"] is True
    assert m["placement_command_kind"] == "ahead"
    assert m["requested_delta_s_m"] == 40.0
    assert m["requested_delta_t_m"] == 0.0
    assert abs(m["observed_delta_s_m"] - 40.0) < 1e-6
    assert abs(m["observed_delta_t_m"] - 0.0) < 1e-6
    assert m["road_id_mismatch"] is True
    assert m["unexpected_pit_projection"] is True
    assert abs(m["spawn_distance_from_ego_m"] - 360.21) < 1e-6


def test_placement_debug_metrics_left_offset(tmp_path: Path) -> None:
    log = tmp_path / "left.log"
    log.write_text(
        "[Placement] Fellow_0: racing (s,t) from ego + ('left', 3.5) -> s=100.0, t=4.5 (same route as ego)\n"
        "[Ego debug] xy=(...) -> s=100.0, t=1.0 | projected onto road_id=1 (MainTrack_TTL)\n"
        "[Fellow s,t] Fellow_0: route=Lap (R2), xy=(...) -> s=100.0, t=4.5 | "
        "distance_from_ego=3.5m, angle_from_ego_deg=-90.0 | projected onto road_id=1 (MainTrack_TTL)\n",
        encoding="utf-8",
    )
    m = collect_fellow_placement_debug_metrics_from_log(log)
    assert m["placement_command_kind"] == "left"
    assert m["requested_delta_s_m"] == 0.0
    assert m["requested_delta_t_m"] == 3.5
    assert abs(m["placement_s_error_m"] - 0.0) < 1e-6
    assert abs(m["placement_t_error_m"] - 0.0) < 1e-6
    assert m["road_id_mismatch"] is False
    assert m["unexpected_pit_projection"] is False
