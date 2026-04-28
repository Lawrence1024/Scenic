"""SD-16: monitor (robustness) functions for the VerifAI active falsifier.

Each monitor takes a parsed `SampleMetrics` from the previous simulation and
returns a robustness scalar (or tuple, for multi-objective). The convention,
matching VerifAI's cross-entropy and Bayesian-optimization samplers, is:

    lower value  = closer to violation
    negative     = specification was actually violated
    >= threshold = "safe" (sampler should explore elsewhere)

`verifai_runner.py` reads `RESOLVE[args.monitor]` to pick the function and
threads its return value back into the next `scenario.generate(feedback=...)`
call. Active samplers update their internal model on each feedback and bias
subsequent samples toward parameter regions that minimize the score.

The monitors deliberately operate on the parsed SampleMetrics dataclass --
NOT on raw simulator state -- so the source of truth for every robustness
signal is the same `parse_sample` regex pipeline that `sampled_runner.py`
uses for its `summary.csv`. One parser, two consumers; no metric drift.
"""

from __future__ import annotations

from typing import Callable, Tuple, Union

from scenic.domains.racing.benchmarks.sampled_runner import SampleMetrics

Robustness = Union[float, Tuple[float, ...]]
MonitorFn = Callable[[SampleMetrics], Robustness]


# ---------------------------------------------------------------------------
# Single-objective monitors
# ---------------------------------------------------------------------------

def collision_robustness(m: SampleMetrics) -> float:
    """Distance from a collision, in meters.

    Returns the minimum bbox_gap_m observed in the run (smaller = closer
    to violation; 0 or negative = OBBs touched/overlapped). When the log
    has no [EvalGT]/[EvalEvent] bbox_gap_m lines at all (early crash,
    scene-only smoke test), falls back to the boolean collision flag:
    -1.0 if eval_contact fired, +1.0 otherwise.

    This is the headline monitor -- it gives CE/BO a continuous gradient
    on the most common failure mode (vehicle-vehicle contact).
    """
    if m.bbox_gap_m_min is not None:
        return float(m.bbox_gap_m_min)
    return -1.0 if m.collision else 1.0


def overtake_failure(m: SampleMetrics) -> float:
    """Falsifies "ego completes at least one overtake."

    Returns +1.0 when the ego completed an overtake (pass_success_count >= 1),
    and -(commit_abort_pass_count) otherwise. More aborts = more negative =
    sampler tries harder to reproduce "ego attempts but fails." If the ego
    never even attempted (commit_abort_pass_count == 0 and pass_success == 0),
    returns 0.0 (neutral -- not a failure of overtaking but lack of trying).
    """
    if m.commit_pass_success_count >= 1:
        return 1.0
    if m.commit_abort_pass_count == 0:
        return 0.0
    return -float(m.commit_abort_pass_count)


def emergency_brake_engaged(m: SampleMetrics) -> float:
    """Falsifies "ego never needs to engage the emergency-brake guard."

    Returns -(guard_emergency_stable_count). Each tick the SD-4 emergency
    brake is engaged is one unit of negative robustness. Use this when
    looking for layouts that force the ego into the safety net rather than
    a deliberate maneuver.
    """
    return -float(m.guard_emergency_stable_count)


def off_track(m: SampleMetrics) -> float:
    """Falsifies "ego stays within the track bounds" -- BOOLEAN flavor.

    -1.0 if the [BoundsCheck] log ever flagged in_track=0; +1.0 otherwise.
    Boolean signal -- CE degrades to coverage on this monitor (no gradient).
    Use `track_clearance` instead when you want CE/BO to converge on the
    parameter regions where the ego goes furthest off track.
    """
    return -1.0 if m.off_track else 1.0


def track_clearance(m: SampleMetrics) -> float:
    """Falsifies "ego stays within the track bounds" -- CONTINUOUS flavor.

    Signed-distance to the nearest track edge:
        +X  : ego stayed inside with X meters of margin (X = run minimum)
        -X  : ego went off-track at some point, X meters past the boundary
        0   : ego touched the boundary
    Smaller value = closer to / deeper-in violation. Smooth across the
    boundary, so CE/BO get a real gradient and can chase the depth of
    the excursion (rather than just the binary "did it leave at all").

    Falls back to +1.0 when `track_clearance_m` is None (log had no
    [BoundsCheck] frames, e.g. scene-only smoke tests).
    """
    if m.track_clearance_m is None:
        return 1.0
    return float(m.track_clearance_m)


# ---------------------------------------------------------------------------
# Aggregators
# ---------------------------------------------------------------------------

def composite_min(m: SampleMetrics) -> float:
    """min over all single-objective monitors.

    The "find ANY violation" default. Returns the smallest robustness
    across {collision, overtake_failure, emergency_brake, off_track}, so
    the sampler chases whichever spec is closest to violating. Best
    starting choice when you don't know which mode the planner is weakest
    on -- the campaign discovers it for you.

    NOTE: pulls in the BOOLEAN `off_track` (not `track_clearance`) so the
    legacy semantics are preserved. For pure-safety active falsification
    use `safety_min` instead -- it pairs the two continuous safety signals
    and gives CE a smooth gradient on both.
    """
    return min(
        collision_robustness(m),
        overtake_failure(m),
        emergency_brake_engaged(m),
        off_track(m),
    )


def safety_min(m: SampleMetrics) -> float:
    """min over the two SAFETY specs: collision and track_clearance.

    Excludes overtake/brake (those are planner-quality, not safety). Both
    components are continuous, so CE/BO see a real gradient regardless of
    which spec is closer to violating. This is the right monitor when the
    user's question is "find me a layout where the ego either crashes OR
    leaves the track":

        rho = min(bbox_gap_m_min, track_clearance_m)

    rho < 0  -> at least one safety spec violated this run
    rho == 0 -> ego touched something (opponent or boundary)
    rho > 0  -> safe; magnitude = closest the ego came to ANY violation
    """
    return min(collision_robustness(m), track_clearance(m))


def all_objectives(m: SampleMetrics) -> Tuple[float, ...]:
    """Multi-objective robustness vector for VerifAI's `mab` and similar.

    Returns (collision, track_clearance, overtake_failure, emergency_brake).
    Useful when the sampler natively handles multi-objective feedback;
    composite_min / safety_min are the right defaults for scalar samplers.
    """
    return (
        collision_robustness(m),
        track_clearance(m),
        overtake_failure(m),
        emergency_brake_engaged(m),
    )


# ---------------------------------------------------------------------------
# Resolver
# ---------------------------------------------------------------------------

RESOLVE: dict = {
    "collision": collision_robustness,
    "overtake": overtake_failure,
    "brake": emergency_brake_engaged,
    "offtrack": off_track,
    "track": track_clearance,
    "min": composite_min,
    "safety": safety_min,
    "all": all_objectives,
}
