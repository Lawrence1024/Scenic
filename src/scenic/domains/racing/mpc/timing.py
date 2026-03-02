"""Per-step timing for loop_other breakdown (racing/MPC only).

Used to log lateral_mpc_ms, longitudinal_mpc_ms, speed_profile_ms and optional
behavior section times (waypoint_speed_grade, after_mpc) so we can see what
fraction of loop_other is MPC vs speed profile vs remainder.
Call record_* from the respective modules; call finish_step() at end of each
simulation step (from longitudinal run_step, which runs last).
Note: sim_steps below count simulation steps (one per tick). Control is applied
every control_interval steps (e.g. every 5), not every sim step.
"""

import time
from typing import Dict, Optional
print(f"[PatchID] timing.py loaded from {__file__}")

# Module-level state for this process
_step = 0
_last_lateral_ms = 0.0
_last_longitudinal_ms = 0.0
_last_speed_profile_ms = 0.0
_sum_lateral = 0.0
_sum_longitudinal = 0.0
_sum_speed_profile = 0.0
_count = 0
_interval = 50
_behavior_timing: Optional["BehaviorTiming"] = None


def set_behavior_timing(bt: Optional["BehaviorTiming"]) -> None:
    """Set the global behavior timing helper (called by DSpaceSimulation)."""
    global _behavior_timing
    _behavior_timing = bt


class BehaviorTiming:
    """Tracks per-step time spent in named sections of the behavior (e.g. waypoint_speed_grade, after_mpc)."""

    def __init__(self) -> None:
        self._section_ms: Dict[str, float] = {}
        self._current_section: Optional[str] = None
        self._section_start: float = 0.0

    def start_step(self) -> None:
        """Call at start of each behavior loop iteration; resets section times for this step."""
        self._section_ms = {}
        self._current_section = None

    def start_section(self, name: str) -> None:
        """Start timing a section (ends any current section first)."""
        self.end_section(self._current_section)
        self._current_section = name
        self._section_start = time.perf_counter()

    def end_section(self, name: Optional[str]) -> None:
        """End timing the given section (no-op if name doesn't match current)."""
        if name is None or self._current_section != name:
            return
        elapsed_ms = (time.perf_counter() - self._section_start) * 1000
        self._section_ms[name] = self._section_ms.get(name, 0.0) + elapsed_ms
        self._current_section = None

    def get_section_ms(self) -> Dict[str, float]:
        """Return current step's section times (ms) for logging. Does not reset."""
        return dict(self._section_ms)


def record_speed_profile_ms(ms: float) -> None:
    global _last_speed_profile_ms
    _last_speed_profile_ms = ms


def record_lateral_mpc_ms(ms: float) -> None:
    global _last_lateral_ms
    _last_lateral_ms = ms


def record_longitudinal_mpc_ms(ms: float) -> None:
    global _last_longitudinal_ms
    _last_longitudinal_ms = ms


def finish_step() -> None:
    """Call once per simulation step (after longitudinal run_step). Prints every _interval steps."""
    global _step, _count, _sum_lateral, _sum_longitudinal, _sum_speed_profile
    _step += 1
    _sum_lateral += _last_lateral_ms
    _sum_longitudinal += _last_longitudinal_ms
    _sum_speed_profile += _last_speed_profile_ms
    _count += 1
    if _step % _interval == 0 and _count > 0:
        # Report mean ms (convert to seconds for consistency with [Timing] lines)
        mean_lat = (_sum_lateral / _count) / 1000.0
        mean_lon = (_sum_longitudinal / _count) / 1000.0
        mean_sp = (_sum_speed_profile / _count) / 1000.0
        mpc_total_s = mean_lat + mean_lon
        line = (
            f"[LoopOther] sim_steps={_step} mean(s): "
            f"state_unpack="
        )
        if _behavior_timing is not None:
            section_ms = _behavior_timing.get_section_ms()
            state_unpack_s = section_ms.get("state_unpack", 0.0) / 1000.0
            path_progress_s = section_ms.get("path_progress", 0.0) / 1000.0
            waypoint_s = section_ms.get("waypoint_speed_grade", 0.0) / 1000.0
            cmd_post_s = section_ms.get("cmd_post", 0.0) / 1000.0
            line += (
                f"{state_unpack_s:.4f} "
                f"path_progress={path_progress_s:.4f} "
                f"speed_profile={mean_sp:.4f} "
                f"mpc_total={mpc_total_s:.4f} "
                f"waypoint_speed_grade={waypoint_s:.4f} "
                f"cmd_post={cmd_post_s:.4f}"
            )
        else:
            line += "N/A path_progress=N/A "
            line += f"mpc_total={mpc_total_s:.4f} waypoint_speed_grade=N/A cmd_post=N/A"
        print(line)
        _sum_lateral = 0.0
        _sum_longitudinal = 0.0
        _sum_speed_profile = 0.0
        _count = 0
