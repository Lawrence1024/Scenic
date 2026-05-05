"""Racing-specific behaviors for dynamic agents.

These behaviors extend the driving domain behaviors with racing-specific
strategies and maneuvers, using abstract racing protocols that simulators
must implement.
"""

from scenic.domains.driving.behaviors import *
from scenic.domains.driving.actions import SetThrottleAction, SetBrakeAction, SetSteerAction
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction, SetSpeedLimitAction, SetTTLSelectionAction, SetTargetGapAction, SetStrategyAction, SetPowertrainModeAction, SetScaleFactorAction, SetPush2PassAction, StopCarAction, SetGearAction, SetFellowPlantAction
import scenic.domains.racing.model as _racing
import math
import numpy as np
import time as _wallclock_time

from scenic.domains.racing.waypoints import (
    initialize_racing_waypoint_start_index,
    select_forward_racing_waypoint,
)
from scenic.domains.racing.fellow import (
    compute_active_block_plant_command,
    compute_always_faster_plant_command,
    compute_constant_offset_plant_command,
    compute_fellow_swerve_out_of_control_command,
    compute_follow_ttl_geometric_plant_command,
    compute_sudden_stop_plant_command,
)
from scenic.domains.racing.segments import (
    build_waypoint_segment_map,
    build_waypoint_segment_map_from_ttl,
    get_segment_at_waypoint,
    get_segment_at_waypoint_ring_strict,
    get_segment_label,
)
from scenic.domains.racing.constants import DELTA_MAX_RAD
from scenic.domains.racing.mode import RACING_MODE_MAIN, RACING_MODE_PIT
from scenic.domains.racing.situation_assessment import (
    _arc_length_project_xy,
    assess_nearest_opponent,
    format_opponent_log_line,
    polyline_lap_length_m,
)
from scenic.domains.racing.tactical_planner import (
    _canonical_mode,
    TacticalPlannerConfig,
    TacticalPlannerState,
    apply_ttl_key_to_agent,
    build_reference as _planner_build_reference,
    tactical_planner_step,
    tactical_planner_step_v1,
    ABORT_PASS,
    COMMIT_PASS_LEFT,
    COMMIT_PASS_RIGHT,
    HOLD_PASS_LEFT,
    HOLD_PASS_RIGHT,
    SETUP_LEFT,
    SETUP_RIGHT,
)
from scenic.domains.racing.prediction import FellowPredictor, format_prediction_log_line
from scenic.domains.racing.assessment import (
    RaceSituationState,
    assess_race_situation,
    format_assessment_log_line,
)
from scenic.domains.racing.safety import (
    StabilityGuardConfig,
    StabilityGuardState,
    format_stability_guard_log_line,
    stability_guard_step,
    stability_guard_handle_ttl_switch,
)
from scenic.domains.racing.safety.stability_guard import (
    should_swap_for_emergency as _safety_should_swap,
    swap_reference_for_emergency as _safety_swap_reference,
)

from scenic.simulators.dspace.ttl.loader import load_ttl_region
from scenic.simulators.dspace.controldesk.readback import read_eval_gt_dist_object_1_m
from scenic.domains.racing.eval_geometry import (
    EVAL_DEFAULT_HULL_NEAR_M,
    EVAL_DEFAULT_OBB_OVERLAP_EPS_M,
    EVAL_DEFAULT_SENSOR_CLOSE_M,
    classify_eval_contact,
    eval_dspace_dist_object_1_valid,
    eval_heading_rad,
    eval_vehicle_length_width_m,
    obb_separation_distance_m,
)


_PHASE1_TTL_FILE_BY_SELECTION = {
    "optimal": "ttl_optimal_xodr.csv",
    "left": "ttl_left_xodr.csv",
    "right": "ttl_right_xodr.csv",
}


# SD-20a: parallel structured-record channel for monitors.
#
# Eval/diagnostic events are printed to stdout for human debugging, AND
# routed through `simulation().records[tag]` so VerifAI monitors can read
# the same data without regex-parsing the log. Each entry is appended as
# `(currentTime, dict_payload)` matching Scenic's `record EXPR as NAME`
# convention. Wrapped in try/except so a missing simulation context (e.g.
# scene-only smoke tests, replay) never breaks the parallel print.
def _record_event(tag, payload):
    try:
        sim = simulation()
        if sim is not None:
            sim.records[tag].append((sim.currentTime, dict(payload)))
    except Exception:
        pass


def _parse_segment_type_from_name(seg_name):
    """RC-7a: parse 'curve' or 'straight' from a segment name string.

    The OpenDRIVE-derived segment_map names follow the pattern
    "<path> <type>" where path ∈ {main, pit} and type ∈ {straight, curve}.
    Returns one of: 'straight', 'curve', or None (unknown / not classified).
    Same classification logic as situation_assessment.planner_segment_context()
    uses as its primary keyword check.
    """
    if not seg_name:
        return None
    s = str(seg_name).lower()
    if "straight" in s:
        return "straight"
    if "curve" in s or "hairpin" in s or "corkscrew" in s:
        return "curve"
    return None


def _scripted_selection_from_ttl_filename(ttl_file_name):
    """Infer planner selection key from TTL filename."""
    if ttl_file_name is None:
        return None
    try:
        name = str(ttl_file_name).lower()
    except Exception:
        return None
    if "left" in name:
        return "left"
    if "right" in name:
        return "right"
    if "optimal" in name:
        return "optimal"
    return None


def _scripted_parse_ttl_schedule(schedule):
    """Parse a scripted TTL schedule.

    Accepted forms:
    - None -> []
    - "5:left,12:right,20:optimal"
    - [("5", "left"), (12, "right"), ...]
    """
    if schedule is None:
        return []
    parsed = []
    if isinstance(schedule, str):
        for chunk in schedule.split(","):
            item = chunk.strip()
            if not item:
                continue
            if ":" not in item:
                raise ValueError(f"Invalid schedule entry '{item}' (expected '<time_s>:<ttl>').")
            t_raw, ttl_raw = item.split(":", 1)
            t_s = float(t_raw.strip())
            ttl = ttl_raw.strip().lower()
            if ttl not in _PHASE1_TTL_FILE_BY_SELECTION:
                raise ValueError(f"Invalid TTL '{ttl}' in schedule; expected one of optimal/left/right.")
            parsed.append((t_s, ttl))
    elif isinstance(schedule, (list, tuple)):
        for item in schedule:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                raise ValueError(f"Invalid schedule item {item}; expected pair (time_s, ttl).")
            t_s = float(item[0])
            ttl = str(item[1]).strip().lower()
            if ttl not in _PHASE1_TTL_FILE_BY_SELECTION:
                raise ValueError(f"Invalid TTL '{ttl}' in schedule; expected one of optimal/left/right.")
            parsed.append((t_s, ttl))
    else:
        raise ValueError("ttl_schedule must be None, a string, or a list/tuple of (time_s, ttl).")
    parsed.sort(key=lambda x: x[0])
    return parsed


behavior FellowConstantSpeedTrackOffsetBehavior(speed_mph=31):
    """Constant-speed fellow with lateral offset fixed from Scenic placement.

    Intended for **dSPACE** traffic fellows controlled via External_Signals
    (``Const_v_Fellows_External``, ``Const_d_Fellows_External``): each step this behavior
    **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with **v** from
    **speed_mph** (converted to km/h) and **d** from lateral placement (``_route_s_t``).
    The dSPACE controller reads ``_fellow_plant_state`` and writes those values to the
    platform. Not MPC; does not populate throttle/steer control state.

    Other simulators do not apply this unless they implement the same (v, d) contract.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_constant_offset_plant_command(self)
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowFollowTTLGeometricBehavior(speed_mph=31):
    """dSPACE fellow: constant speed and lateral **d** from TTL geometry (no PID/MPC).

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with **v**
    from **speed_mph** and **d** from feedforward δ(s) on control-interval steps (aligned
    with simulator readback) on the main track centerline (optimal TTL vs ``ttl_main_road``),
    matching the racing line used by MPC fellows. The dSPACE controller writes those values
    to External_Signals.

    Waypoint progress uses :func:`select_forward_racing_waypoint` (same family as
    ``FollowRacingLineMPCBehavior`` / racing line followers) for a stable polyline index;
    **d** is pure geometry.

    Requires **Lap** route, ``ttlFolder``, ``ttlFileName`` (optimal CSV), TTL waypoints on
    the agent, and a valid delta table. Uses ``dspaceActor`` pose from readback. Other
    simulators may ignore this behavior.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_follow_ttl_geometric_plant_command(
            self, simulation(), speed_mph
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowSuddenStopIntervalBehavior(speed_mph=150, interval=20.0, duration=3.0):
    """dSPACE fellow: periodic full stops (commanded v=0), then back to cruise speed.

    Uses simulation time (:obj:`Simulation.currentRealTime`). Each cycle lasts
    **interval + duration** seconds: cruise at **speed_mph** for **interval** seconds,
    then commanded **0 km/h** for **duration** seconds, then repeat forever. Unlike
    :obj:`FellowSwerveOutOfControlBehavior`, lateral **d** always tracks TTL geometry
    (no open-loop swerve legs): each step uses the same δ(s) path as
    :obj:`FellowFollowTTLGeometricBehavior` (Lap route, ``ttlFolder``, ``ttlFileName``,
    optimal CSV, waypoints). For placement-only fallback when geometry is inactive, see
    that behavior's requirements.

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with
    outputs from :func:`compute_sudden_stop_plant_command`; the dSPACE
    :class:`~scenic.simulators.dspace.vehicle.controller.VehicleController` writes them to
    ``Const_v_Fellows_External`` / ``Const_d_Fellows_External``.

    Example scene: ``examples/combined/fellow_sudden_stop.scenic``.

    Args:
        speed_mph: Cruise speed between stops in **mph**. Default **150**.
        interval: Cruise phase length in seconds (≥ 0). Default **20**.
        duration: Stop phase length in seconds (commanded longitudinal v=0; ≥ 0). Default **3**.
            If **duration** is **0**, the fellow stays in cruise only (no stop phase).
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_sudden_stop_plant_command(
            self,
            simulation(),
            speed_mph=speed_mph,
            interval_s=interval,
            duration_s=duration,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowSwerveOutOfControlBehavior(
    speed_mph=150,
    interval=10.0,
    swerve_right_s=1.8,
    swerve_left_s=2.0,
    swerve_amp_m=6.0,
    swerve_d_rate_m_s=6.5,
    stop_hold_d=True,
):
    """dSPACE fellow: TTL cruise, then gradual swerve right then left, then stop.

    Default numeric parameters match ``examples/combined/fellow_swerve_out_of_control.scenic``.

    For **interval** seconds the fellow matches :obj:`FellowFollowTTLGeometricBehavior`
    lateral **d** (delta(s)) at **speed_mph**. Then **d** slews toward **-swerve_amp_m** m
    (right of centerline) and toward **+swerve_amp_m** m (left) at up to **swerve_d_rate_m_s**
    m/s change in commanded **d**, so lateral commands ramp instead of stepping. **swerve_right_s**
    and **swerve_left_s** bound how long each leg lasts; each leg should be at least about
    **swerve_amp_m / swerve_d_rate_m_s** seconds to reach full offset in one direction, and
    crossing from **-amp** to **+amp** needs about **2 * swerve_amp_m / swerve_d_rate_m_s** in
    the second leg. Then **v = 0**. If **stop_hold_d** is true (default), commanded **d** stays
    at the end-of-maneuver value so the car does not creep laterally while stationary; if false,
    **d** slews toward TTL delta(s) like a moving target (can look like sliding in place).

    Centerline convention: positive **d** = left, negative = right (same as placement).

    Requires Lap route, ``ttlFolder``, ``ttlFileName``, waypoints, and delta table when
    using TTL phases (same as geometric fellow).

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction` with
    outputs from :func:`compute_fellow_swerve_out_of_control_command`; dSPACE writes plant
    outputs to fellow External_Signals (same as other (v, d) plant behaviors).

    Args:
        speed_mph: Cruise and swerve-leg speed in **mph**. Default **150**.
        interval: Seconds of TTL cruise before the swerve maneuver. Default **10**.
        swerve_right_s: Duration (s) of the leg slewing **d** toward **-swerve_amp_m** (right).
            Default **1.8**.
        swerve_left_s: Duration (s) of the leg slewing **d** toward **+swerve_amp_m** (left).
            Default **2.0**.
        swerve_amp_m: Lateral command magnitude (m) for each swerve target relative to
            centerline (positive = left, negative = right). Default **6.0**.
        swerve_d_rate_m_s: Maximum rate of change of commanded **d** (m/s). Default **6.5**.
        stop_hold_d: If true (default), after **v = 0** keep **d** fixed at the end of the
            maneuver; if false, slew **d** toward TTL δ(s) while stopped (can look like drift).
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_fellow_swerve_out_of_control_command(
            self,
            simulation(),
            speed_mph=speed_mph,
            interval_s=interval,
            swerve_right_s=swerve_right_s,
            swerve_left_s=swerve_left_s,
            swerve_amp_m=swerve_amp_m,
            swerve_d_rate_m_s=swerve_d_rate_m_s,
            stop_hold_d=stop_hold_d,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowAlwaysFasterThanEgoBehavior(speed_offset_mph=10, min_speed_mph=120, max_speed_mph=160):
    """dSPACE fellow: speed = ego.speed + offset (clamped); lateral **d** from TTL geometry.

    Ego-aware fellow that targets ``ego.speed + speed_offset_mph`` each control tick,
    clamped to ``[min_speed_mph, max_speed_mph]``. Designed so the fellow is structurally
    uncatchable on a straight: as long as ego cruises below
    ``max_speed_mph - speed_offset_mph``, the fellow remains the same offset faster, and
    the s-gap grows monotonically. Tests whether the planner declines COMMIT_PASS when
    no realistic overtake exists.

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction`
    with **v** from ego speed + offset (mph→km/h) and **d** from TTL δ(s). Same
    TTL/route requirements as :obj:`FellowFollowTTLGeometricBehavior`.

    Args:
        speed_offset_mph: How much faster than ego the fellow targets in **mph**.
            Default **10**. With ego at 60 m/s ≈ 134 mph, fellow targets 144 mph.
        min_speed_mph: Floor on commanded speed (mph) so the fellow doesn't drop
            to a crawl if ego stops. Default **120**.
        max_speed_mph: Ceiling on commanded speed (mph) for plant-safety / corner
            stability. Default **160**.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_always_faster_plant_command(
            self, simulation(), ego,
            speed_offset_mph=speed_offset_mph,
            min_speed_mph=min_speed_mph,
            max_speed_mph=max_speed_mph,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FellowActiveBlockBehavior(speed_offset_mph=-5.0, min_speed_mph=30.0, max_speed_mph=160.0, max_lat_speed_mps=3.0, max_lat_offset_m=5.0, deadband_m=0.4):
    """dSPACE fellow: actively blocks ego from overtaking (negative passing test).

    Adversarial blocker with full ego state visibility. Each control tick the
    blocker projects ego's xy onto its TTL, takes the resulting track-frame
    lateral as its target ``d``, clips to ``±max_lat_offset_m``, and slews the
    commanded ``d`` toward target with bounded lateral velocity. Speed tracks
    ego's speed plus ``speed_offset_mph`` (default ``-5``), clamped to
    ``[min_speed_mph, max_speed_mph]`` — the blocker stays slightly slower so
    the gap collapses toward it.

    Information is asymmetric by design: the blocker reads ego state (oracle
    access for the adversary), while ego cannot read the blocker's internal
    target. This is the negative-passing-test setup.

    Each step **takes** :class:`~scenic.domains.racing.actions.SetFellowPlantAction`
    with **v** from the speed-matching loop (mph→km/h) and **d** from the
    lateral-tracking + slew loop. Same TTL/route requirements as
    :obj:`FellowFollowTTLGeometricBehavior`; falls back to placement **d** if
    preconditions fail (logged once).

    Args:
        speed_offset_mph: Blocker speed = ego speed + this (mph). Default
            **-5** (blocker is 5 mph slower than ego).
        min_speed_mph: Floor on commanded speed (mph). Default **30**. Prevents
            both-cars-stop pathology if ego matches the blocker indefinitely.
        max_speed_mph: Ceiling on commanded speed (mph). Default **160**.
        max_lat_speed_mps: Maximum lateral velocity (m/s) for the d slew.
            Default **3.0**. Matches F5 swerve magnitude. At dt=10 ms this
            caps per-tick |Δd| at 0.03 m, ruling out teleportation.
        max_lat_offset_m: Clip target d to ``±this`` so the blocker stays on
            the drivable surface. Default **5.0** m.
        deadband_m: If the new target is within this distance of the prior
            commanded d, hold the prior value. Default **0.4** m. Suppresses
            blocker jitter when ego is roughly straight.
    """
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh, d_m, mode = compute_active_block_plant_command(
            self, simulation(), ego,
            speed_offset_mph=speed_offset_mph,
            min_speed_mph=min_speed_mph,
            max_speed_mph=max_speed_mph,
            max_lat_speed_mps=max_lat_speed_mps,
            max_lat_offset_m=max_lat_offset_m,
            deadband_m=deadband_m,
        )
        take SetFellowPlantAction(v_kmh, d_m)
        self._fellow_plant_log_mode = mode
        wait

behavior FollowRacingLineMPCBehavior(target_speed=30, manage_gears=True, use_waypoints=True, mpc_config_path=None, planner_enabled=False, ttl_schedule=None, target_speed_cap=None, tactical_planner_enabled=False, prediction_enabled=True, assessment_enabled=True, stability_guard_enabled=False, commit_abort_enabled=True, segment_aware_enabled=False):
    """Follow the car's TTL using MPC (Model Predictive Control) for lateral control.
    
    Primary Scenic behavior for line-following on the racing TTL. Lateral and longitudinal
    control use MPC for predictive tracking, especially in high-speed cornering.
    Lookahead distance for the MPC path is computed internally from speed and horizon
    (see lookahead_dist in the behavior); it is not a user parameter.
    
    Outputs NORMALIZED control signals (-1.0 to 1.0).
    The Simulator (simulator.py) automatically scales these to dSPACE VesiInterface units.
    
    Args:
        target_speed: Target speed in m/s
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based control
        mpc_config_path: Path to MPC config YAML file (optional, uses default if None)
        tactical_planner_enabled: Phase 3 — conservative FOLLOW / SETUP_LEFT / SETUP_RIGHT using
            Phase 2 situation assessment (mutually exclusive with scripted ``planner_enabled`` schedule).
        prediction_enabled: Log ``[Prediction]`` for the nearest fellow
            (ego only; constant-velocity next-step estimate and error vs realized pose).
        assessment_enabled: Log ``[Assessment]`` tactical facts
            (`fellow_relation`, `safe_gap`, `gap_ok`, corridor openness) from current/predicted fellow state.
        stability_guard_enabled: Enforce stability guard constraints
            (steer slew, brake-steer coupling, emergency-stable containment, TTL switch rate-limiting).
        commit_abort_enabled: Enable explicit pass lifecycle states
            (`COMMIT_PASS_LEFT`, `COMMIT_PASS_RIGHT`, `ABORT_PASS`) in tactical planner output.
    """
    
    # SETUP & DEFAULTS
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad)
    take SetMaxSpeedAction(target_speed)
    
    throttle_limit = 1.0

    # Ego: base throttle cap (lowered further on large CTE); raised for more throttle on straights
    if self is simulation().scene.egoObject:
        throttle_limit = 1.0   # No cap so MPC can reach 140 mph on straights (IAC vehicle)

    # Steering slew-rate and CTE safety thresholds
    max_steer_delta = 0.2          # per step (normalized units)
    cte_slowdown_threshold = 15.0  # m: start slowing down
    cte_stop_threshold = 50.0      # m: full brake to avoid runaway
    
    # Progressive throttle reduction thresholds (for better control when CTE is large)
    cte_throttle_reduction_start = 2.0   # m: start reducing throttle progressively (lowered from 5.0)
    cte_throttle_reduction_max = 10.0    # m: maximum throttle reduction zone
    min_throttle_at_large_cte = 0.03     # minimum throttle when CTE > 10m

    # Get Controllers: Longitudinal MPC + Lateral MPC
    _lon_controller, _lat_controller = simulation().getRacingControllers(self, use_mpc=True, mpc_config_path=mpc_config_path)
    _fbhv = getattr(self, '_follow_mpc_behavior_log_prefix', '[FollowRacingLineMPCBehavior]')
    _scripted_schedule_enabled = bool(planner_enabled)
    _tactical_planner_enabled = bool(tactical_planner_enabled)
    if _tactical_planner_enabled and _scripted_schedule_enabled:
        print(f"{_fbhv} tactical_planner_enabled=True: ignoring Phase 1 scripted ttl_schedule (tactical owns TTL).")
        _scripted_schedule_enabled = False
    _scripted_schedule = []
    _scripted_ttl_cache = {}
    _scripted_active_ttl = str(getattr(self, 'ttl_selection', '') or '').lower()
    if _scripted_active_ttl not in _PHASE1_TTL_FILE_BY_SELECTION:
        _scripted_active_ttl = _scripted_selection_from_ttl_filename(getattr(self, 'ttlFileName', None)) or "optimal"
    _scripted_warned_missing_ttl = set()
    _scripted_speed_cap = None
    if target_speed_cap is not None:
        try:
            _scripted_speed_cap = max(0.0, float(target_speed_cap))
        except Exception:
            _scripted_speed_cap = None
    if not _tactical_planner_enabled:
        try:
            _tactical_planner_enabled = bool((getattr(simulation().scene, 'params', None) or {}).get("tactical_planner_enabled", False))
        except Exception:
            _tactical_planner_enabled = False
    _tactical_config = TacticalPlannerConfig()
    _prediction_requested = bool(prediction_enabled)
    _assessment_enabled = bool(assessment_enabled)
    _stability_guard_enabled = bool(stability_guard_enabled)
    _commit_enabled = bool(commit_abort_enabled)
    if not _assessment_enabled:
        try:
            _assessment_enabled = bool((getattr(simulation().scene, 'params', None) or {}).get("assessment_enabled", False))
        except Exception:
            _assessment_enabled = False
    if not _stability_guard_enabled:
        try:
            _stability_guard_enabled = bool((getattr(simulation().scene, 'params', None) or {}).get("stability_guard_enabled", False))
        except Exception:
            _stability_guard_enabled = False
    # SD-41E: auto-enable the stability guard whenever the tactical planner
    # is active. The guard now runs as a pre-MPC safety supervisor (swaps
    # the planner reference for a safe-stop trajectory on predicted
    # collision) AND as a post-MPC command filter. Both are load-bearing
    # for the SD-41 contract; an unguarded tactical run would leave the
    # MPC tracking the planner's intent through a predicted collision
    # with no fallback. Override by passing `stability_guard_enabled=False`
    # explicitly OR `--param stability_guard_auto_enable False`.
    if _tactical_planner_enabled and not _stability_guard_enabled:
        try:
            _auto_enable = (getattr(simulation().scene, 'params', None) or {}).get("stability_guard_auto_enable", True)
            if isinstance(_auto_enable, bool):
                _auto_enable = bool(_auto_enable)
            else:
                _auto_enable = str(_auto_enable).strip().lower() in ("true", "1", "yes", "on")
        except Exception:
            _auto_enable = True
        if _auto_enable:
            _stability_guard_enabled = True
            print(f"{_fbhv} SD-41E auto-enabled stability_guard (tactical_planner_enabled=True). Override with stability_guard_auto_enable=False.")
    if not _commit_enabled:
        try:
            _commit_enabled = bool((getattr(simulation().scene, 'params', None) or {}).get("commit_abort_enabled", False))
        except Exception:
            _commit_enabled = False
    _tactical_config.commit_abort_enabled = bool(_commit_enabled)
    _segment_aware_enabled = bool(segment_aware_enabled)
    if not _segment_aware_enabled:
        try:
            _segment_aware_enabled = bool((getattr(simulation().scene, 'params', None) or {}).get("segment_aware_enabled", False))
        except Exception:
            _segment_aware_enabled = False
    _tactical_config.segment_aware_enabled = bool(_segment_aware_enabled)
    # SD-11e: thread the strategy-authority flag from .scenic params into the
    # tactical config so `--param use_strategy_authority True` actually flips
    # the authority branch on. Mirrors the existing assessment_enabled pattern.
    try:
        _params_for_strategy = getattr(simulation().scene, 'params', None) or {}
        _v_use_strategy = _params_for_strategy.get("use_strategy_authority", None)
        if _v_use_strategy is not None:
            if isinstance(_v_use_strategy, bool):
                _tactical_config.use_strategy_authority = bool(_v_use_strategy)
            else:
                _tactical_config.use_strategy_authority = str(_v_use_strategy).strip().lower() in ("true", "1", "yes", "on")
    except Exception:
        pass
    if _tactical_config.use_strategy_authority:
        print(f"{_fbhv} SD-11e strategy authority ENABLED (use_strategy_authority=True)")
    if _segment_aware_enabled:
        # Segment-aware gating via fine-grained modifiers;
        # disable the legacy straight-only blanket gate so corner passes are possible.
        _tactical_config.pass_requires_straight = False
    _guard_config = StabilityGuardConfig()
    if _scripted_schedule_enabled or _tactical_planner_enabled:
        try:
            _params = getattr(simulation().scene, 'params', None) or {}
            if _scripted_schedule_enabled:
                _raw_schedule = ttl_schedule if ttl_schedule is not None else _params.get("ttlSwitchSchedule")
                _scripted_schedule = _scripted_parse_ttl_schedule(_raw_schedule)
            _ttl_folder = getattr(self, 'ttlFolder', None) or _params.get("ttlFolder")
            if _ttl_folder:
                _preloaded_keys = []
                for _ttl_key, _ttl_file in _PHASE1_TTL_FILE_BY_SELECTION.items():
                    try:
                        _region, _pts = load_ttl_region(str(_ttl_folder), _ttl_file)
                        if _region is not None and _pts and len(_pts) >= 2:
                            _scripted_ttl_cache[_ttl_key] = (_region, list(_pts))
                            _preloaded_keys.append(_ttl_key)
                    except Exception:
                        print(f"{_fbhv} TTL preload failed for {_ttl_file}.")
                print(f"{_fbhv} TTL preload (Phase 1 / Phase 3): folder={_ttl_folder} keys={_preloaded_keys}")
            if _scripted_schedule_enabled:
                print(f"{_fbhv} Phase1 planner enabled: schedule={_scripted_schedule if _scripted_schedule else '[]'} speed_cap={_scripted_speed_cap}")
            if _tactical_planner_enabled:
                print(f"{_fbhv} Phase3 tactical planner enabled (FREE_RUN / FOLLOW / SETUP_*) speed_cap={_scripted_speed_cap}")
            if _prediction_requested:
                print(f"{_fbhv} Fellow prediction enabled ([Prediction] on full-control steps)")
            if _stability_guard_enabled:
                print(f"{_fbhv} Stability guard enabled ([Guard] command safety telemetry)")
        except Exception:
            _scripted_schedule_enabled = False
            _tactical_planner_enabled = False
            _stability_guard_enabled = False
            print(f"{_fbhv} Phase1/Phase3 TTL dynamic mode disabled due to setup error.")

    # SD-41I — Gear thresholds (m/s) derived from the dSPACE Dallara AV24
    # plant model. Source files (canonical authority for these numbers):
    #   .../IAC_Project/Parameterization/MOD_Traffic/Pool/Parameter/ASM/
    #     Drivetrain/GearboxAT/GearboxAT_dallara24.xml
    #   .../IAC_Project/.../SoftECU/ShiftStrategy/ShiftStrategy_dallara24.xml
    #   .../IAC_Project/.../Drivetrain/Rear Differential/Rear Differential_dallara24.xml
    #
    # Computation: speed = Proc_n_up_acc / (gear_ratio · final_drive · 60) · π·D
    # with Proc_n_up_acc = 3850 RPM, final_drive (MainReductionGear) = 3.0,
    # tire diameter D ≈ 0.686 m (Dallara IL-15 spec).
    #
    #   Gear ratios (Map_GearRatio):  1=3.75  2=2.235  3=1.518  4=1.13  5=0.7
    #   Computed upshift speeds (m/s): 1→2≈12.3   2→3≈20.6   3→4≈30.4   4→5≈40.8
    #   Plant has NO gear 6: Map_GearRatio saturates at 0.7 for indices > 5.
    #
    # The Scenic upshift values [12, 22, 32, 42] match within ~1 m/s. Downshift
    # values are 3 m/s below upshift for hysteresis. The previous lists
    # included [42, 52] / [38, 48] entries for a phantom gear 6; removed.
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0]

    wp_last_idx = 0

    # CRITICAL: Wait for simulation to initialize and position to be available
    wait
    while not hasattr(self, 'position') or self.position is None:
        wait
    # Also wait for dSPACE heading readback so waypoint-ahead initialization uses a real heading.
    while (not hasattr(self, 'dspaceActor') or self.dspaceActor is None
           or not hasattr(self.dspaceActor, 'heading') or self.dspaceActor.heading is None):
        wait
    
    # Initialize waypoint index based on starting position
    # CRITICAL FIX: Find the first waypoint that is AHEAD of the vehicle
    wp_list_init = (self.waypoints if hasattr(self, 'waypoints') else None)
    if use_waypoints and wp_list_init and len(wp_list_init) >= 2:
        try:
            px = float(self.position.x); py = float(self.position.y)
            
            car_heading = None
            car_heading_src = None
            if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
                try:
                    car_heading = float(self.dspaceActor.heading)
                    car_heading_src = "dspaceActor.heading"
                except:
                    pass
            if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
                    car_heading_src = "self.heading"
                except:
                    pass
            if car_heading is None:
                print(f"{_fbhv} Warning: heading unavailable at init; dot-product ahead search disabled")
            
            # Step 1: Find the nearest waypoint (by distance)
            nearest_idx = 0
            best_d2 = 1e18
            for i in range(len(wp_list_init)):
                wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            
            # Step 2: If we have a valid heading, find the first waypoint AHEAD of the vehicle
            # This prevents trying to track waypoints behind the vehicle
            if car_heading is not None:
                # Vehicle forward direction (math is already imported at module level)
                veh_fx = math.cos(car_heading)
                veh_fy = math.sin(car_heading)
                
                # Starting from nearest waypoint, find first one ahead (closed loop: wrap)
                # A waypoint is "ahead" if the dot product of (vehicle_to_waypoint) and (vehicle_forward) > 0
                n_wp = len(wp_list_init)
                wp_last_idx = nearest_idx

                for off in range(0, min(100, n_wp)):
                    i = (nearest_idx + off) % n_wp
                    wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                    to_wp_x = wx - px
                    to_wp_y = wy - py
                    dot_product = to_wp_x * veh_fx + to_wp_y * veh_fy

                    if dot_product > 0:  # Waypoint is ahead
                        wp_last_idx = i
                        wp_dist = (to_wp_x*to_wp_x + to_wp_y*to_wp_y) ** 0.5
                        print(f"{_fbhv} Initialized: starting at ({px:.2f}, {py:.2f}), heading={car_heading*180/math.pi:.1f}deg (src={car_heading_src})")
                        print(f"  Found first waypoint AHEAD: index={wp_last_idx} at ({wx:.2f}, {wy:.2f}), distance={wp_dist:.2f}m")
                        print(f"  Dot product={dot_product:.2f} (positive means ahead)")
                        break
                else:
                    wp_last_idx = nearest_idx
                    print(f"{_fbhv} Warning: No waypoint ahead found in search window, using nearest waypoint {nearest_idx}")
            else:
                # No heading available, use nearest waypoint
                wp_last_idx = nearest_idx
                print(f"{_fbhv} Initialized (no heading): nearest waypoint index={nearest_idx}, distance={best_d2**0.5:.2f}m")
        except Exception as e:
            print(f"{_fbhv} Warning: Could not initialize waypoint index: {e}, starting from index 0")
            wp_last_idx = 0
        
        # Initialize waypoint progress tracking
        if not hasattr(self, '_waypoints_passed'):
            self._waypoints_passed = 0

    while True:
        _fl_mpc = getattr(self, '_follow_mpc_log_prefix', '[FollowRacingLineMPC]')
        # Calculate Control Signals
        # SD-10g: per-tick wall-clock measurement for runtime analysis. Records
        # the wall time at tick start. Emitted as [TickTime] at end of tick
        # alongside the sim time, so log analysis can correlate sim moments
        # with wall-clock slowdowns. Process-relative seconds (vs absolute
        # epoch) is more readable in log greps.
        _wall_tick_start = _wallclock_time.perf_counter()
        if not hasattr(self.__class__, '_wall_process_start'):
            self.__class__._wall_process_start = _wall_tick_start
        _wall_t_now_s = _wall_tick_start - self.__class__._wall_process_start
        # SD-10l: per-section accumulators (ms). Reset every tick. Emitted as
        # [TickBreakdown] alongside [TickTime] for each control tick. Used to
        # find which section eats the budget on big-tick events (e.g. first
        # ttl_switch where tick_ms jumps to ~600ms — Shapely was ruled out by
        # SD-10k pre-warm experiment, so the cost is elsewhere).
        _sec_segmap_ms = 0.0
        _sec_assess_opp_ms = 0.0
        _sec_predict_ms = 0.0
        _sec_assess_race_ms = 0.0
        _sec_planner_ms = 0.0
        _sec_lon_ms = 0.0
        _sec_lat_ms = 0.0
        current_speed = (self.speed if self.speed is not None else 0)
        _sim = simulation()
        # Planner schedule time is in simulation seconds based on simulation timestep.
        _dt_sim = getattr(_sim, 'timestep', None)
        if _dt_sim is None or _dt_sim <= 0:
            _dt_sim = getattr(_sim, 'control_dt', None)
        if _dt_sim is None or _dt_sim <= 0:
            _dt_sim = float(getattr(_sim, 'control_period', 0.05) or 0.05)
        _sim_time_s = float(getattr(_sim, 'currentTime', 0)) * float(_dt_sim)
        _ctrl_dt = getattr(_sim, 'control_dt', None)
        if _ctrl_dt is None or _ctrl_dt <= 0:
            _ctrl_dt = getattr(_sim, 'control_period', None)
        if _ctrl_dt is None or _ctrl_dt <= 0:
            _ctrl_dt = float(_dt_sim)
        if _scripted_schedule_enabled:
            _desired_ttl = _scripted_active_ttl
            for _t_switch, _ttl_sel in _scripted_schedule:
                if _sim_time_s >= _t_switch:
                    _desired_ttl = _ttl_sel
                else:
                    break
            if _desired_ttl != _scripted_active_ttl:
                if _desired_ttl in _scripted_ttl_cache:
                    _prev_ttl = _scripted_active_ttl
                    _region_new, _pts_new = _scripted_ttl_cache[_desired_ttl]
                    self.ttl = _region_new
                    self.waypoints = list(_pts_new)
                    self.ttl_selection = _desired_ttl
                    self.ttlFileName = _PHASE1_TTL_FILE_BY_SELECTION.get(_desired_ttl, getattr(self, 'ttlFileName', None))
                    self._waypoint_segment_map = None
                    self._last_valid_segment_id = None
                    self._last_valid_segment_name = ""
                    self._cached_cumulative_dist_wp_idx = 0
                    self._cached_cumulative_dist_to_wp = 0.0
                    self._waypoint_progress = 0.0
                    self._waypoint_progress_idx = 0
                    wp_last_idx = 0
                    _scripted_active_ttl = _desired_ttl
                    print(f"{_fl_mpc} [Phase1Planner] t={_sim_time_s:.2f}s ttl_switch {_prev_ttl}->{_scripted_active_ttl}")
                else:
                    if _desired_ttl not in _scripted_warned_missing_ttl:
                        print(f"{_fl_mpc} [Phase1Planner] TTL '{_desired_ttl}' not preloaded; staying on '{_scripted_active_ttl}'.")
                        _scripted_warned_missing_ttl.add(_desired_ttl)
            self.active_ttl = _scripted_active_ttl

        # Non-control-step fast path: exit before any heavy work (no waypoint/profile, NumPy, readbacks, progress, controller assembly)
        _run_full_control = (simulation().is_control_step if hasattr(simulation(), 'is_control_step') else True) or not hasattr(self, '_last_final_steer')
        if not _run_full_control:
            self._fastpath_ticks = getattr(self, '_fastpath_ticks', 0) + 1
            _fc = getattr(self, '_full_control_ticks', 0)
            _fp = getattr(self, '_fastpath_ticks', 0)
            if (_fc + _fp) % 100 == 0 and (_fc + _fp) > 0:
                print(f"{_fl_mpc} full_control_ticks={_fc} fastpath_ticks={_fp}")
            final_steer = getattr(self, '_last_final_steer', 0.0)
            final_throttle = getattr(self, '_last_final_throttle', 0.0)
            final_brake = getattr(self, '_last_final_brake', 0.0)
            # Pit mode (segment-based): coast when speed >= 35 mph, else cap throttle at 50%
            PIT_LIMIT_MS = 15.646   # 35 mph — same as PIT_MAX_SPEED_MS in full-control block
            if getattr(self, '_in_pit_by_segment', False):
                if current_speed >= PIT_LIMIT_MS:
                    final_throttle = 0.0
                else:
                    final_throttle = min(final_throttle, 0.5)
            _hz_floor_fast = float(getattr(self, "_hazard_brake_floor", 0.0) or 0.0)
            if _hz_floor_fast > 0.0:
                final_throttle = 0.0
                final_brake = max(float(final_brake), _hz_floor_fast)
            _fast_actions = [SetSteerAction(final_steer), SetThrottleAction(final_throttle), SetBrakeAction(final_brake)]
            take _fast_actions
            wait
            continue

        self._full_control_ticks = getattr(self, '_full_control_ticks', 0) + 1
        
        # --- Waypoint Management for MPC ---
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        
        # Ensure position is available
        if not hasattr(self, 'position') or self.position is None:
            wait
            continue
        
        px = float(self.position.x); py = float(self.position.y)
        car_heading = None
        # Prefer dSPACE/ControlDesk readback heading if available
        if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
            try:
                car_heading = float(self.dspaceActor.heading)
            except:
                pass
        if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
            try:
                car_heading = float(self.heading)
            except:
                pass
        _bt = getattr(simulation(), 'behavior_timing', None)
        if _bt is not None:
            _bt.start_step()
            _bt.start_section('state_unpack')
        # Full control path (waypoint/profile/MPC); we only reach here when not on fast path
        if True:
            # Update waypoint index for MPC
            # PROGRESS-BASED ADVANCEMENT (from suggestion.md): Advance based on arc-length progress
            # Instead of radius-based advancement, track cumulative distance along waypoints
            # and advance when we've progressed past a waypoint segment
            old_wp_idx = wp_last_idx
            if use_waypoints and wp_list and len(wp_list) >= 2:
                if _bt is not None:
                    _bt.end_section('state_unpack')
                    _bt.start_section('path_progress')
                # When MPC rejects or association is bad, reset wp_last_idx to nearest waypoint (full scan) to avoid stale index and oscillations
                _md = getattr(_lat_controller, '_log_match_dist_m', None)
                _gr = getattr(_lat_controller, '_log_gate_reason', None)
                if (_md is not None and _md > 5.0) or _gr in ('too_far', 's_jump'):
                    nearest_idx = 0
                    best_d2 = 1e18
                    for i in range(len(wp_list)):
                        wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                        d2 = (px - wx)**2 + (py - wy)**2
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = i
                    wp_last_idx = nearest_idx
                    # Invalidate cumulative-dist cache so it is recomputed from new wp_last_idx
                    self._cached_cumulative_dist_wp_idx = 0
                    self._cached_cumulative_dist_to_wp = 0.0
                # Build segment map once (main + pit): from OpenDRIVE track or from TTL waypoints (same as visualization).
                if not hasattr(self, '_waypoint_segment_map') or self._waypoint_segment_map is None:
                    _sec_t_segmap_start = _wallclock_time.perf_counter()
                    try:
                        scene = simulation().scene
                        params = getattr(scene, 'params', None) or {}
                        main_ttl = getattr(scene, '_main_ttl_waypoints', None)
                        if main_ttl is not None:
                            pit_ttl = getattr(scene, '_pit_ttl_waypoints', None) or []
                            self._waypoint_segment_map = build_waypoint_segment_map_from_ttl(main_ttl, pit_ttl, waypoints=wp_list)
                            self._last_valid_segment_id = None
                            self._last_valid_segment_name = ""
                            num_seg = len(set(seg_id for seg_id, _ in self._waypoint_segment_map)) if self._waypoint_segment_map else 0
                            print(f"{_fbhv} Segment map built from TTL waypoints ({num_seg} segments, main+pit, overlap=main); ring-strict segment filtering active")
                        else:
                            track = params.get('track') or getattr(scene, 'track', None)
                            if track is not None:
                                self._waypoint_segment_map = build_waypoint_segment_map(wp_list, track)
                                self._last_valid_segment_id = None
                                self._last_valid_segment_name = ""
                                num_seg = len(set(seg_id for seg_id, _ in self._waypoint_segment_map)) if self._waypoint_segment_map else 0
                                print(f"{_fbhv} Segment map built from OpenDRIVE ({num_seg} segments, main+pit); ring-strict segment filtering active")
                            else:
                                self._waypoint_segment_map = None
                                print(f"{_fbhv} Segment map not built (no track and no TTL waypoints); log will show segment ?")
                    except Exception as e:
                        self._waypoint_segment_map = None
                        print(f"{_fbhv} Segment map not built: {e}; log will show segment ?")
                    _sec_segmap_ms += (_wallclock_time.perf_counter() - _sec_t_segmap_start) * 1000.0
                try:
                    # Initialize progress tracking if needed
                    if not hasattr(self, '_waypoint_progress'):
                        self._waypoint_progress = 0.0  # Cumulative distance along waypoints
                        self._waypoint_progress_idx = 0  # Waypoint index at last progress update
                    if not hasattr(self, '_cached_cumulative_dist_wp_idx'):
                        self._cached_cumulative_dist_wp_idx = 0
                        self._cached_cumulative_dist_to_wp = 0.0
                    # Cumulative distance to current waypoint index (O(1) update when wp_last_idx advances)
                    # Path is closed: segment i is (wp_list[i], wp_list[(i+1) % n]); wrap last -> first.
                    n_wp = len(wp_list)
                    cached_idx = getattr(self, '_cached_cumulative_dist_wp_idx', 0)
                    cached_dist = getattr(self, '_cached_cumulative_dist_to_wp', 0.0)
                    if wp_last_idx <= cached_idx:
                        if wp_last_idx < cached_idx:
                            cumulative_dist_to_wp = 0.0
                            for i in range(wp_last_idx):
                                wp0 = wp_list[i]
                                wp1 = wp_list[(i + 1) % n_wp]
                                dx = float(wp1[0]) - float(wp0[0])
                                dy = float(wp1[1]) - float(wp0[1])
                                seg_len = (dx*dx + dy*dy) ** 0.5
                                cumulative_dist_to_wp += seg_len
                            self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                            self._cached_cumulative_dist_wp_idx = wp_last_idx
                        else:
                            cumulative_dist_to_wp = cached_dist
                    else:
                        cumulative_dist_to_wp = cached_dist
                        for i in range(cached_idx, wp_last_idx):
                            wp0 = wp_list[i]
                            wp1 = wp_list[(i + 1) % n_wp]
                            dx = float(wp1[0]) - float(wp0[0])
                            dy = float(wp1[1]) - float(wp0[1])
                            seg_len = (dx*dx + dy*dy) ** 0.5
                            cumulative_dist_to_wp += seg_len
                        self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                        self._cached_cumulative_dist_wp_idx = wp_last_idx

                    # Project vehicle position onto current waypoint segment (closed loop: segment wp_last_idx -> (wp_last_idx+1) % n_wp)
                    s_0 = cumulative_dist_to_wp  # Default: progress to segment start
                    wp0 = wp_list[wp_last_idx]
                    wp1 = wp_list[(wp_last_idx + 1) % n_wp]
                    x0, y0 = float(wp0[0]), float(wp0[1])
                    x1, y1 = float(wp1[0]), float(wp1[1])
                    seg_dx = x1 - x0
                    seg_dy = y1 - y0
                    seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                    
                    if seg_len > 1e-6:
                        # Project vehicle position onto segment
                        wx = px - x0
                        wy = py - y0
                        u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                        u_proj = max(0.0, min(1.0, u_proj))
                        
                        # Current progress s_0 = cumulative distance to segment start + progress along segment
                        s_0 = cumulative_dist_to_wp + u_proj * seg_len
                        
                        # Advance waypoint index based on progress (wrap last -> first for closed loop)
                        while True:
                            segment_end_dist = cumulative_dist_to_wp + seg_len
                            if s_0 >= segment_end_dist - 0.5:  # Small threshold (0.5m) to handle numerical issues
                                wp_last_idx = (wp_last_idx + 1) % n_wp
                                cumulative_dist_to_wp = 0.0 if wp_last_idx == 0 else segment_end_dist
                                # Next segment
                                wp0 = wp_list[wp_last_idx]
                                wp1 = wp_list[(wp_last_idx + 1) % n_wp]
                                x0, y0 = float(wp0[0]), float(wp0[1])
                                x1, y1 = float(wp1[0]), float(wp1[1])
                                seg_dx = x1 - x0
                                seg_dy = y1 - y0
                                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                                if seg_len > 1e-6:
                                    wx = px - x0
                                    wy = py - y0
                                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                                    u_proj = max(0.0, min(1.0, u_proj))
                                    s_0 = cumulative_dist_to_wp + u_proj * seg_len
                                else:
                                    seg_len = 1e-6
                            else:
                                break
                    else:
                        # Degenerate segment - advance to next (with wrap)
                        wp_last_idx = (wp_last_idx + 1) % n_wp
                        cumulative_dist_to_wp = 0.0 if wp_last_idx == 0 else (cumulative_dist_to_wp + seg_len)
                    # Keep cache in sync when wp_last_idx was advanced in the while loop or degenerate branch
                    self._cached_cumulative_dist_to_wp = cumulative_dist_to_wp
                    self._cached_cumulative_dist_wp_idx = wp_last_idx

                    # Update progress tracking
                    self._waypoint_progress = s_0
                    self._waypoint_progress_idx = wp_last_idx

                    # Calculate distance to current waypoint for logging
                    current_wp_dist = None
                    if wp_last_idx < len(wp_list):
                        wp_x, wp_y = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                        dx = px - wp_x; dy = py - wp_y
                        current_wp_dist = (dx*dx + dy*dy) ** 0.5

                    # Log current waypoint
                    if wp_last_idx < len(wp_list):
                        current_wp = wp_list[wp_last_idx]
                        current_wp_x, current_wp_y = float(current_wp[0]), float(current_wp[1])
                        if wp_last_idx != old_wp_idx:
                            # Initialize waypoint progress tracking
                            if not hasattr(self, '_waypoints_passed'):
                                self._waypoints_passed = 0
                            self._waypoints_passed += 1
                            progress_pct = ((self._waypoints_passed % len(wp_list)) / len(wp_list)) * 100.0 if len(wp_list) > 0 else 0.0
                            sim = simulation()
                            ctrl_dt = getattr(sim, 'control_dt', None)
                            if ctrl_dt is None or ctrl_dt <= 0:
                                ctrl_dt = getattr(sim, 'control_period', None)
                            if ctrl_dt is None or ctrl_dt <= 0:
                                ctrl_dt = float(getattr(sim, 'timestep', 0.05))
                            step_wp = getattr(self, '_behavior_step_count', 0)
                            t_wp = step_wp * ctrl_dt
                            _smap_w = getattr(self, '_waypoint_segment_map', None)
                            _raw_w = get_segment_at_waypoint(wp_last_idx, _smap_w) if _smap_w else None
                            _path_w = "pit" if (_raw_w and _raw_w[1] and _raw_w[1].startswith("pit ")) else "main"
                            _lid_w = getattr(self, '_last_valid_segment_id', None)
                            _lname_w = getattr(self, '_last_valid_segment_name', "") or ""
                            _eid, _ename, _ = get_segment_at_waypoint_ring_strict(wp_last_idx, _smap_w, _path_w, _lid_w, _lname_w)
                            self._last_valid_segment_id = _eid
                            self._last_valid_segment_name = _ename or ""
                            seg_str = f" {get_segment_label(_eid, _ename)}" if (_eid is not None or _ename) else " segment ?"
                            print(f"{_fbhv} t={t_wp:.2f}s WAYPOINT HIT: index {old_wp_idx} -> {wp_last_idx} at ({current_wp_x:.2f}, {current_wp_y:.2f}), distance={current_wp_dist:.2f}m{seg_str}")
                            print(f"{_fbhv} t={t_wp:.2f}s Progress: {self._waypoints_passed} waypoints passed ({progress_pct:.1f}% of {len(wp_list)} total waypoints)")

                except Exception as e:
                    print(f"{_fbhv} Warning: Waypoint finder error: {e}")
                if _bt is not None:
                    _bt.end_section('path_progress')
            
            # --- Longitudinal Control (MPC) ---
            # To-Do D: Use projection-based CTE (same geometry as MPCC) everywhere. CTE magnitude for speed
            # comes from previous step's waypoint-based e_y (MPCC uses same polyline and projection).
            cte_mag_for_speed = getattr(self, '_last_waypoint_cte_for_speed', 0.0)
            # Legacy CTE: fallback when MPC does not provide e_y (e.g. no waypoints or exception)
            self._legacy_cte_this_tick = None
            if use_waypoints and wp_list and len(wp_list) >= 2:
                try:
                    nearest_idx = wp_last_idx
                    best_d2 = 1e18
                    search_window = 50
                    for i in range(max(0, wp_last_idx - search_window), min(len(wp_list), wp_last_idx + search_window)):
                        wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                        d2 = (px - wx)**2 + (py - wy)**2
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = i
                    if abs(nearest_idx - wp_last_idx) > search_window * 0.8:
                        for i in range(max(0, wp_last_idx - 2*search_window), min(len(wp_list), wp_last_idx + 2*search_window)):
                            wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                            d2 = (px - wx)**2 + (py - wy)**2
                            if d2 < best_d2:
                                best_d2 = d2
                                nearest_idx = i
                    next_idx = (nearest_idx + 1) % len(wp_list)
                    if len(wp_list) >= 2:
                        x0, y0 = float(wp_list[nearest_idx][0]), float(wp_list[nearest_idx][1])
                        x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                        seg_dx = x1 - x0
                        seg_dy = y1 - y0
                        seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                        if seg_len > 1e-6:
                            u_proj = max(0.0, min(1.0, ((px - x0)*seg_dx + (py - y0)*seg_dy) / (seg_len*seg_len)))
                            proj_x = x0 + u_proj * seg_dx
                            proj_y = y0 + u_proj * seg_dy
                            nx = -seg_dy / seg_len
                            ny = seg_dx / seg_len
                            if car_heading is not None:
                                seg_heading = math.atan2(seg_dy, seg_dx)
                                hd = math.atan2(math.sin(seg_heading - car_heading), math.cos(seg_heading - car_heading))
                                if abs(hd) > math.pi / 2:
                                    nx, ny = -nx, -ny
                            self._legacy_cte_this_tick = (px - proj_x)*nx + (py - proj_y)*ny
                except Exception:
                    pass
            if self._legacy_cte_this_tick is None and hasattr(self, 'ttl') and self.ttl is not None and hasattr(self.ttl, 'signedDistanceTo'):
                try:
                    self._legacy_cte_this_tick = self.ttl.signedDistanceTo(self.position)
                except Exception:
                    self._legacy_cte_this_tick = 0.0
            if self._legacy_cte_this_tick is None:
                self._legacy_cte_this_tick = 0.0
            if _bt is not None:
                _bt.start_section('waypoint_speed_grade')
            # Universal max speed limit: Reduced for better robustness without elevation data
            MAX_SPEED_LIMIT_MS = 62.58  # 140 mph in m/s (~225 km/h) for IAC vehicle capability
            PIT_MAX_SPEED_MS = 15.646   # 35 mph in m/s for pit lane (cap only; we coast in pit)
            # Pit lane: coast at 35 mph; only add throttle when "too slow" to avoid brake/throttle oscillation
            PIT_COAST_TOO_SLOW_MS = 10.0   # below this speed we add a bit of throttle
            PIT_COAST_MIN_TARGET_MS = 15.646  # 35 mph — pit target and coast threshold (match PIT_MAX_SPEED_MS)
            # Segment (and thus pit vs main): use raw segment at waypoint so we never rely on ModelDesk route for control.
            _smap = getattr(self, '_waypoint_segment_map', None)
            _raw_seg = get_segment_at_waypoint(wp_last_idx, _smap) if _smap else None
            _seg_name_raw = (_raw_seg[1] or "") if _raw_seg else ""
            in_pit_by_segment = _seg_name_raw.startswith("pit ")
            _path = "pit" if in_pit_by_segment else "main"
            _lid = getattr(self, '_last_valid_segment_id', None)
            _lname = getattr(self, '_last_valid_segment_name', "") or ""
            _eff_id, _eff_name, _transition = get_segment_at_waypoint_ring_strict(wp_last_idx, _smap, _path, _lid, _lname)
            self._last_valid_segment_id = _eff_id
            self._last_valid_segment_name = _eff_name or ""
            # RC-7a: parse curve/straight type from segment name. Stash both current and
            # lookahead-segment type for telemetry. RC-7b will wire these into the speed
            # planner / behavior decisions. Lookahead is wp_idx + 25 (~ 25 m at 1 m wp
            # spacing); a typical ego speed of 25-40 m/s spans 25 m in ~0.6-1.0 s, which is
            # roughly the time it takes to bleed speed before a corner -- the right horizon
            # for "is there a curve coming?" decisions.
            self._segment_type_at_wp = _parse_segment_type_from_name(_eff_name)
            _wp_ahead_idx = (wp_last_idx + 25) % len(wp_list) if (wp_list and len(wp_list) > 0) else wp_last_idx
            _ahead_seg = get_segment_at_waypoint(_wp_ahead_idx, _smap) if _smap else None
            _ahead_name = (_ahead_seg[1] or "") if _ahead_seg else ""
            self._segment_type_ahead = _parse_segment_type_from_name(_ahead_name)
            self._segment_id_ahead = (_ahead_seg[0] if _ahead_seg else None)
            # Pit exit/enter: update ModelDesk route and _route (for logging/transition only); control uses segment, not route
            if _transition == "pit_exit" and self is simulation().scene.egoObject:
                self._route = RACING_MODE_MAIN
                if hasattr(simulation(), "request_ego_route"):
                    simulation().request_ego_route(self, RACING_MODE_MAIN)
            elif _transition == "pit_enter" and self is simulation().scene.egoObject:
                self._route = RACING_MODE_PIT
                if hasattr(simulation(), "request_ego_route"):
                    simulation().request_ego_route(self, RACING_MODE_PIT)
            segment_name_current = _eff_name
            # SD-10c: pit_mode hysteresis. F7 trace showed ego briefly flagged as
            # pit_mode=True during a SETUP_LEFT lateral swing (~9.55s, 15.85s,
            # 17.35s — each tick that the segment classifier glanced at a
            # waypoint near the pit/main boundary), causing pit_mode_guard to
            # force FREE_RUN and abort the setup. Result: ttl ping-pong
            # (5x left↔optimal in 11.6s) and 287x pit_mode_guard events.
            # Hysteresis: require N consecutive ticks of the same value before
            # the latched pit_mode flips. Eliminates single-tick segment-classifier
            # noise without affecting genuine pit-lane entry/exit (which lasts
            # many seconds).
            _PIT_HYSTERESIS_TICKS = 3
            _pit_run = int(getattr(self, '_pit_mode_consecutive_count', 0) or 0)
            _pit_latched = bool(getattr(self, '_pit_mode_latched', False))
            if bool(in_pit_by_segment) == _pit_latched:
                _pit_run = 0
            else:
                _pit_run += 1
                if _pit_run >= _PIT_HYSTERESIS_TICKS:
                    _pit_latched = bool(in_pit_by_segment)
                    _pit_run = 0
            self._pit_mode_consecutive_count = _pit_run
            self._pit_mode_latched = _pit_latched
            # Pit mode for speed limit and throttle: latched value only.
            pit_mode = _pit_latched
            self._in_pit_by_segment = pit_mode

            # --- Phase 6 orchestration shells (state -> planner -> guard) ---
            # --- Phase 7 fellow next-step prediction (ego + nearest fellow) ---
            # --- Phase 8 assessment + dynamic gap (current/predicted fellow state) ---
            self._tactical_speed_cap = None
            _ego_scene = getattr(simulation().scene, 'egoObject', None)
            # RC-5: include _tactical_planner_enabled in the gate. When tactical is on
            # but prediction/assessment are off, the planner used to receive None for
            # _a8 (silent fallback to raw OpponentSituation). Now the assessment block
            # runs whenever any smart feature is on.
            if (self is _ego_scene) and (_prediction_requested or _assessment_enabled or _tactical_planner_enabled):
                _nearest_o6 = None
                _nearest_d6 = None
                _nearest_vs6 = 0.0
                _sit6 = None
                _p7r = None
                _a8 = None
                try:
                    _objs6 = getattr(simulation().scene, 'objects', [])
                    _best62 = None
                    for _ob6 in _objs6:
                        if _ob6 is self:
                            continue
                        if not hasattr(_ob6, 'position') or _ob6.position is None:
                            continue
                        _ox6 = float(_ob6.position.x)
                        _oy6 = float(_ob6.position.y)
                        _dx6 = _ox6 - px
                        _dy6 = _oy6 - py
                        _d62 = _dx6 * _dx6 + _dy6 * _dy6
                        if _best62 is None or _d62 < _best62:
                            _best62 = _d62
                            _nearest_o6 = _ob6
                            _nearest_d6 = _d62 ** 0.5
                            _nearest_vs6 = float(getattr(_ob6, 'speed', 0.0) or 0.0)
                except Exception:
                    _nearest_o6 = None
                    _nearest_d6 = None
                    _nearest_vs6 = 0.0

                if _nearest_o6 is not None and car_heading is not None:
                    _sec_t_assess_opp_start = _wallclock_time.perf_counter()
                    _ox6 = float(_nearest_o6.position.x)
                    _oy6 = float(_nearest_o6.position.y)
                    _prog6 = getattr(self, '_waypoint_progress', None)
                    _smap6 = getattr(self, '_waypoint_segment_map', None)
                    _sid6 = getattr(self, '_last_valid_segment_id', None)
                    _snm6 = getattr(self, '_last_valid_segment_name', '') or ''
                    _Lap6 = None
                    if use_waypoints and wp_list is not None and len(wp_list) >= 2:
                        _Lap6 = polyline_lap_length_m(wp_list)
                    _prev_ov6 = getattr(self, '_opponent_overlap_state', 'clear_ahead')
                    _sit6, _new_ov6 = assess_nearest_opponent(
                        (px, py),
                        float(car_heading),
                        float(current_speed),
                        (_ox6, _oy6),
                        _nearest_vs6,
                        ego_progress_s_m=_prog6,
                        waypoints=wp_list if use_waypoints else None,
                        lap_length_m=_Lap6,
                        segment_map=_smap6,
                        ego_wp_idx=wp_last_idx,
                        segment_id=_sid6,
                        segment_name=_snm6,
                        curvature_ahead_max=float(getattr(self, "_last_curvature_ahead_for_tactical", 0.0) or 0.0),
                        previous_overlap_state=_prev_ov6,
                    )
                    self._opponent_overlap_state = _new_ov6
                    _sec_assess_opp_ms += (_wallclock_time.perf_counter() - _sec_t_assess_opp_start) * 1000.0

                if _prediction_requested:
                    _sec_t_predict_start = _wallclock_time.perf_counter()
                    if _nearest_o6 is None:
                        if hasattr(self, '_fellow_predictor'):
                            self._fellow_predictor.reset()
                    else:
                        if not hasattr(self, '_fellow_predictor'):
                            self._fellow_predictor = FellowPredictor()
                        _fpx7 = float(_nearest_o6.position.x)
                        _fpy7 = float(_nearest_o6.position.y)
                        _fprog7 = getattr(_nearest_o6, '_waypoint_progress', None)
                        try:
                            _p7r = self._fellow_predictor.step(
                                _sim_time_s,
                                _fpx7,
                                _fpy7,
                                fellow_progress_s_m=_fprog7,
                                dt_pred_s=float(_ctrl_dt),
                            )
                            print(format_prediction_log_line(_sim_time_s, _p7r))
                        except Exception as _e_p7:
                            print(f"{_fbhv} Prediction step failed: {_e_p7}")
                            _p7r = None
                    _sec_predict_ms += (_wallclock_time.perf_counter() - _sec_t_predict_start) * 1000.0

                if _assessment_enabled and car_heading is not None:
                    _sec_t_assess_race_start = _wallclock_time.perf_counter()
                    if not hasattr(self, '_assessment_state'):
                        self._assessment_state = RaceSituationState()
                    _pred_xy8 = None
                    if _p7r is not None:
                        _pred_xy8 = (float(_p7r.fellow_pred_x), float(_p7r.fellow_pred_y))
                    _a8, _a8_state = assess_race_situation(
                        sit=_sit6,
                        ego_speed_mps=float(current_speed),
                        ego_xy=(float(px), float(py)),
                        ego_heading_rad=float(car_heading),
                        predicted_opp_xy=_pred_xy8,
                        state=self._assessment_state,
                    )
                    self._assessment_state = _a8_state
                    _sec_assess_race_ms += (_wallclock_time.perf_counter() - _sec_t_assess_race_start) * 1000.0
                    self._assessment_gap_ok = bool(getattr(_a8, "gap_ok", True))
                    self._assessment_overlap_flag = bool(getattr(_a8, "overlap_flag", False))
                    self._assessment_closing_flag = bool(getattr(_a8, "closing_flag", False))
                    try:
                        self._assessment_emergency_risk_01 = float(getattr(_a8, "emergency_risk_01", 0.0) or 0.0)
                    except Exception:
                        self._assessment_emergency_risk_01 = 0.0
                    print(format_assessment_log_line(_sim_time_s, _a8))
            
            # --- Phase 3 tactical planner (TTL + follow cap; uses prior-step curvature lookahead) ---
            _p10_ttl_switch_blocked = False
            if _tactical_planner_enabled and self is getattr(simulation().scene, 'egoObject', None):
                if not hasattr(self, '_tactical_tp_state'):
                    self._tactical_tp_state = TacticalPlannerState()
                if _stability_guard_enabled and not hasattr(self, '_guard_state'):
                    self._guard_state = StabilityGuardState(
                        last_ttl=str(_scripted_active_ttl or "optimal")
                    )
                _k_prev = float(getattr(self, '_last_curvature_ahead_for_tactical', 0.0) or 0.0)
                _nearest_o3 = None
                _nearest_d3 = None
                _nearest_vs3 = 0.0
                try:
                    _objs3 = getattr(simulation().scene, 'objects', [])
                    _ego_h3 = car_heading if car_heading is not None else 0.0
                    _efx3 = math.cos(_ego_h3)
                    _efy3 = math.sin(_ego_h3)
                    _best32 = None
                    for _ob3 in _objs3:
                        if _ob3 is self:
                            continue
                        if not hasattr(_ob3, 'position') or _ob3.position is None:
                            continue
                        _ox3 = float(_ob3.position.x)
                        _oy3 = float(_ob3.position.y)
                        _dx3 = _ox3 - px
                        _dy3 = _oy3 - py
                        _d22 = _dx3 * _dx3 + _dy3 * _dy3
                        if _best32 is None or _d22 < _best32:
                            _best32 = _d22
                            _nearest_o3 = _ob3
                            _nearest_d3 = _d22 ** 0.5
                            _nearest_vs3 = float(getattr(_ob3, 'speed', 0.0) or 0.0)
                except Exception:
                    _nearest_o3 = None
                _sit3 = None
                if _nearest_o3 is not None and car_heading is not None:
                    _ox3 = float(_nearest_o3.position.x)
                    _oy3 = float(_nearest_o3.position.y)
                    _prog3 = getattr(self, '_waypoint_progress', None)
                    _smap3 = getattr(self, '_waypoint_segment_map', None)
                    _sid3 = getattr(self, '_last_valid_segment_id', None)
                    _snm3 = getattr(self, '_last_valid_segment_name', '') or ''
                    _Lap3 = None
                    if use_waypoints and wp_list is not None and len(wp_list) >= 2:
                        _Lap3 = polyline_lap_length_m(wp_list)
                    _prev_ov3 = getattr(self, '_opponent_overlap_state', 'clear_ahead')
                    _sit3, _new_ov3 = assess_nearest_opponent(
                        (px, py),
                        float(car_heading),
                        float(current_speed),
                        (_ox3, _oy3),
                        _nearest_vs3,
                        ego_progress_s_m=_prog3,
                        waypoints=wp_list if use_waypoints else None,
                        lap_length_m=_Lap3,
                        segment_map=_smap3,
                        ego_wp_idx=wp_last_idx,
                        segment_id=_sid3,
                        segment_name=_snm3,
                        curvature_ahead_max=_k_prev,
                        previous_overlap_state=_prev_ov3,
                    )
                    self._opponent_overlap_state = _new_ov3
                _a_rel = getattr(_a8, "fellow_relation", None) if _a8 is not None else None
                _a_gap_ok = getattr(_a8, "gap_ok", None) if _a8 is not None else None
                _a_opt_open = getattr(_a8, "optimal_open", None) if _a8 is not None else None
                _a_left_open = getattr(_a8, "left_open", None) if _a8 is not None else None
                _a_right_open = getattr(_a8, "right_open", None) if _a8 is not None else None
                _a_closing = getattr(_a8, "closing_flag", None) if _a8 is not None else None
                _a_emerg_risk = getattr(_a8, "emergency_risk_01", None) if _a8 is not None else None
                # SD-3c: thread optimal/left/right TTL polylines and ego/opp arc-lengths
                # into the planner so it can run pass_window_check geometric look-ahead
                # before initiating a SETUP. Skipped (None) if any TTL is missing — the
                # planner falls back to its prior preference logic when polylines absent.
                _opt_wp = None
                _left_wp = None
                _right_wp = None
                _opt_lap_len = None
                _opt_ego_s = None
                _opt_opp_s = None
                if _scripted_ttl_cache is not None and "optimal" in _scripted_ttl_cache:
                    _opt_wp = _scripted_ttl_cache["optimal"][1]
                    _opt_lap_len = polyline_lap_length_m(_opt_wp) if _opt_wp else None
                    if _opt_lap_len and _opt_lap_len > 0.0:
                        _opt_ego_s = _arc_length_project_xy(float(px), float(py), _opt_wp)
                        if _nearest_o3 is not None:
                            _opt_opp_s = _arc_length_project_xy(
                                float(_nearest_o3.position.x),
                                float(_nearest_o3.position.y),
                                _opt_wp,
                            )
                if _scripted_ttl_cache is not None and "left" in _scripted_ttl_cache:
                    _left_wp = _scripted_ttl_cache["left"][1]
                if _scripted_ttl_cache is not None and "right" in _scripted_ttl_cache:
                    _right_wp = _scripted_ttl_cache["right"][1]
                # SD-11d: pull a multi-step fellow trajectory from the predictor
                # if available. Used by the strategy pipeline inside the planner.
                # The horizon comes from config so .scenic files can override.
                _fellow_traj_for_strategy = None
                if hasattr(self, '_fellow_predictor') and _nearest_o3 is not None:
                    try:
                        _fellow_traj_for_strategy = self._fellow_predictor.trajectory(
                            horizon_s=float(_tactical_config.strategy_horizon_s),
                            sample_dt_s=float(_tactical_config.strategy_sample_dt_s),
                        )
                    except Exception as _e_traj:
                        _fellow_traj_for_strategy = None
                _sec_t_planner_start = _wallclock_time.perf_counter()
                _mode3, _ttl3, _cap3, _reason3 = tactical_planner_step_v1(
                    self._tactical_tp_state,
                    _sit3,
                    has_opponent=_nearest_o3 is not None,
                    ego_speed_mps=float(current_speed),
                    opponent_speed_mps=float(_nearest_vs3 or 0.0),
                    sim_time_s=float(_sim_time_s),
                    pit_mode=bool(pit_mode),
                    config=_tactical_config,
                    assessment_relation=_a_rel,
                    assessment_gap_ok=_a_gap_ok,
                    assessment_optimal_open=_a_opt_open,
                    assessment_left_open=_a_left_open,
                    assessment_right_open=_a_right_open,
                    assessment_closing_flag=_a_closing,
                    assessment_emergency_risk_01=_a_emerg_risk,
                    optimal_waypoints=_opt_wp,
                    side_waypoints_left=_left_wp,
                    side_waypoints_right=_right_wp,
                    ego_s_m=_opt_ego_s,
                    opp_s_m=_opt_opp_s,
                    lap_length_m=_opt_lap_len,
                    # SD-7: ego's actual physical TTL — source of truth for
                    # PathPredict's polyline selection. Pre-SD-7, the planner
                    # derived ego_track from state.mode which doesn't disambiguate
                    # ABORT_PASS (post-COMMIT, ego may still be on side TTL for
                    # ~1s while abort_keep_ttl_lat_m holds the side line).
                    ego_active_ttl=str(_scripted_active_ttl or "optimal"),
                    fellow_trajectory=_fellow_traj_for_strategy,
                    # SD-31: curvature on the active TTL (set by the previous
                    # tick's curvature scan; lags by one control_period). Used
                    # by tactical_planner to clip COMMIT_PASS_* speed caps so
                    # the longitudinal MPC isn't asked for a speed the tires
                    # can't honor in the upcoming curve.
                    curvature_ahead_max=float(getattr(self, "_last_curvature_ahead_for_tactical", 0.0) or 0.0),
                )
                _sec_planner_ms += (_wallclock_time.perf_counter() - _sec_t_planner_start) * 1000.0
                _mode_tac = _mode3
                _ttl_tac = _ttl3
                _cap_tac = _cap3
                if _stability_guard_enabled and _ttl_tac in _scripted_ttl_cache:
                    _ttl_guard_state = getattr(self, "_guard_state", None)
                    if _ttl_guard_state is None:
                        _ttl_guard_state = StabilityGuardState(last_ttl=str(_scripted_active_ttl or "optimal"))
                        self._guard_state = _ttl_guard_state
                    _ttl_guarded, _p10_ttl_switch_blocked = stability_guard_handle_ttl_switch(
                        _ttl_guard_state,
                        config=_guard_config,
                        sim_time_s=float(_sim_time_s),
                        current_ttl=str(_scripted_active_ttl or "optimal"),
                        requested_ttl=str(_ttl_tac or _scripted_active_ttl),
                        planner_state=str(_mode_tac or "FREE_RUN"),  # RC-6: planner intent wins during COMMIT/ABORT
                    )
                    if _p10_ttl_switch_blocked and _ttl_guarded != _ttl_tac:
                        _ttl_tac = _ttl_guarded
                        _eff_reason = "guard_ttl_switch_blocked"
                if _ttl_tac != _scripted_active_ttl and _ttl_tac in _scripted_ttl_cache:
                    if apply_ttl_key_to_agent(self, _ttl_tac, _scripted_ttl_cache, _PHASE1_TTL_FILE_BY_SELECTION):
                        _p3 = _scripted_active_ttl
                        _scripted_active_ttl = _ttl_tac
                        wp_last_idx = 0
                        wp_list = list(self.waypoints)
                        # SD-37: removed legacy [Tactical] ttl_switch line.
                        # TTL switches are already canonically logged by
                        # [Phase0Event] type=ttl_switch (~line 2936), and
                        # the per-tick mode is in [CtrlTrace] planner=...
                        # ttl=... fields. This duplicate added 1 line per
                        # switch with no unique information.
                self.active_ttl = _scripted_active_ttl
                # Phase 9 authority path owns speed cap end-to-end.
                self._tactical_speed_cap = float(_cap_tac) if _cap_tac is not None else None
                self._tactical_last_mode = _mode_tac
                _eff_reason = _reason3
                if _p10_ttl_switch_blocked:
                    _eff_reason = "guard_ttl_switch_blocked"
                self._phase_effective_planner_state = str(_canonical_mode(_mode_tac) or "FREE_RUN")
                self._phase_effective_ttl = str(_scripted_active_ttl or "optimal")
                self._phase_effective_reason = str(_eff_reason or "none")
                # SD-20a: parallel record emit for [Strategy]. The planner already
                # printed it (in tactical_planner.py); we emit the same data
                # structurally so monitors can read it without regex. Skip when
                # the planner had insufficient inputs (selected_name == "").
                _strat_name_rec = str(getattr(self._tactical_tp_state, "strategy_selected_name", "") or "")
                if _strat_name_rec:
                    _record_event('Strategy', {
                        't': float(_sim_time_s),
                        'selected': _strat_name_rec,
                        'reason': str(getattr(self._tactical_tp_state, "strategy_selected_reason", "") or ""),
                        'clearances': dict(getattr(self._tactical_tp_state, "strategy_min_clearances", {}) or {}),
                        'progress': dict(getattr(self._tactical_tp_state, "strategy_reachable_progress", {}) or {}),
                    })
                # SD-4b: emit [PathPredict] telemetry for the predicted_collision
                # computation done inside tactical_planner_step_v1. Used to verify
                # that the gate fires on F4-style sudden-stop scenarios and stays
                # silent on F2/F3 alongside-but-clear windows.
                _pc_st = self._tactical_tp_state
                if bool(getattr(_pc_st, "predicted_collision_available", False)):
                    print(
                        f"{_fl_mpc} [PathPredict] t={_sim_time_s:.2f}s "
                        f"predicted_collision={1 if _pc_st.predicted_collision else 0} "
                        f"ego_track={_pc_st.predicted_collision_ego_track} "
                        f"opp_track={_pc_st.predicted_collision_opp_track} "
                        f"min_clear={_pc_st.predicted_collision_min_clear_m:.3f} "
                        f"closest_t={_pc_st.predicted_collision_closest_t_s:.2f}s "
                        f"breach={_pc_st.predicted_collision_breach_count}"
                    )
                # Stash on self for downstream brake-trigger consumers (SD-4c/4d).
                self._predicted_collision = bool(getattr(_pc_st, "predicted_collision", False))
                self._predicted_collision_available = bool(getattr(_pc_st, "predicted_collision_available", False))
                _cap9 = f"{self._tactical_speed_cap:.2f}" if self._tactical_speed_cap is not None else "na"
                _arel9 = str(_a_rel or "na")
                _agap9 = (
                    "na"
                    if _a_gap_ok is None
                    else ("1" if bool(_a_gap_ok) else "0")
                )
                _aopt9 = (
                    "na"
                    if _a_opt_open is None
                    else ("1" if bool(_a_opt_open) else "0")
                )
                _alft9 = (
                    "na"
                    if _a_left_open is None
                    else ("1" if bool(_a_left_open) else "0")
                )
                _argt9 = (
                    "na"
                    if _a_right_open is None
                    else ("1" if bool(_a_right_open) else "0")
                )
                _mode9 = _canonical_mode(_mode3)
                print(
                    f"{_fl_mpc} [Planner] t={_sim_time_s:.2f}s planner_state={_mode9} "
                    f"chosen_ttl={_scripted_active_ttl} target_speed_cap={_cap9} decision_reason={_reason3} "
                    f"assessment_relation={_arel9} assessment_gap_ok={_agap9} "
                    f"assessment_optimal_open={_aopt9} assessment_left_open={_alft9} assessment_right_open={_argt9}"
                )
                if _commit_enabled:
                    _commit_st = getattr(self._tactical_tp_state, "commit", None)
                    _p11_commit_trigger = str(getattr(_commit_st, "trigger", "none") or "none") if _commit_st is not None else "none"
                    _p11_abort_trigger = str(getattr(_commit_st, "abort_trigger", "none") or "none") if _commit_st is not None else "none"
                    _p11_pass_success = bool(getattr(_commit_st, "pass_success", False)) if _commit_st is not None else False
                    _p11_abort_success = bool(getattr(_commit_st, "abort_success", False)) if _commit_st is not None else False
                    _p11_post_state = str(getattr(_commit_st, "post_event_state", "none") or "none") if _commit_st is not None else "none"
                    _p11_commit_cand = int(getattr(_commit_st, "candidate_count", 0)) if _commit_st is not None else 0
                    # SD-14: protected_follow_active field deleted in SD-13c.
                    # The log slot stays at 0 for backward-compat with downstream
                    # log parsers; can be removed once those are updated.
                    _seg_ctx = str(getattr(_sit3, "segment_context", "none") or "none") if _sit3 is not None else "none"
                    _seg_modifier = str(getattr(self._tactical_tp_state, "segment_modifier", "normal") or "normal")
                    print(
                        f"{_fl_mpc} [Commit] t={_sim_time_s:.2f}s planner_state={_canonical_mode(_mode3)} "
                        f"chosen_ttl={_scripted_active_ttl} decision_reason={_reason3} "
                        f"commit_trigger={_p11_commit_trigger} abort_trigger={_p11_abort_trigger} "
                        f"pass_success={1 if _p11_pass_success else 0} abort_success={1 if _p11_abort_success else 0} "
                        f"post_event_state={_p11_post_state} "
                        f"commit_cand_count={_p11_commit_cand} protected_follow=0 "
                        f"seg_ctx={_seg_ctx} seg_modifier={_seg_modifier}"
                    )
                    _record_event('Commit', {
                        't': float(_sim_time_s),
                        'planner_state': _canonical_mode(_mode3),
                        'chosen_ttl': str(_scripted_active_ttl),
                        'decision_reason': str(_reason3),
                        'commit_trigger': _p11_commit_trigger,
                        'abort_trigger': _p11_abort_trigger,
                        'pass_success': bool(_p11_pass_success),
                        'abort_success': bool(_p11_abort_success),
                        'post_event_state': _p11_post_state,
                        'commit_cand_count': int(_p11_commit_cand),
                        'seg_ctx': _seg_ctx,
                        'seg_modifier': _seg_modifier,
                    })
                # SD-37: removed legacy [Tactical] periodic mode dump.
                # Mode/ttl/cap are already logged every tick in [CtrlTrace]
                # (planner=, ttl=) and per-tick [Planner] events; the every-
                # 50-step coarse dump added no information.
            
            # --- Curvature-based speed gate (from suggestion.md) ---
            # Formula: v_max(s) = sqrt(a_y_max / (|κ(s)| + ε))
            # v_ref = min(v_desired, min_{s∈[s_0, s_0+L]} v_max(s))
            # This ensures vehicle enters turns at appropriate speed (Laguna Seca: see turns early to avoid run-off)
            curvature_speed_limit = target_speed  # Default: no reduction
            curvature_ahead_max = 0.0  # Max curvature magnitude over lookahead (for speed limits)
            curvature_ahead_max_signed = 0.0  # Task 3: signed kappa at apex (left > 0, right < 0) for downstream
            max_lateral_accel = 8.0  # m/s² (conservative for indoor sim, can be configured)
            curvature_epsilon = 0.001  # Small epsilon to avoid division by zero
            curvature_speed_margin = 0.96  # Use 96% of theoretical v_max (IAC race car with slicks)
            
            if use_waypoints and wp_list and len(wp_list) >= 3:
                try:
                    # Compute curvature-based speed limit over MPC horizon (waypoint indices wrap at end of lap)
                    # Look ahead: ensure we see turns early enough to brake (Laguna Seca Corkscrew/hairpins)
                    horizon = _lon_controller.config.mpc_prediction_horizon if hasattr(_lon_controller, 'config') else 35
                    dt_mpc = _lon_controller.config.mpc_prediction_dt if hasattr(_lon_controller, 'config') else 0.05
                    lookahead_dist = current_speed * horizon * dt_mpc  # Distance over MPC horizon
                    if lookahead_dist < 10.0:
                        lookahead_dist = 25.0  # Minimum at very low speed
                    # At high speed, need enough distance to brake before turn (avoid run-off).
                    # Braking from v to v_turn at slew_down m/s takes (v - v_turn)/slew_down seconds; distance ~ v * T.
                    # At 46 m/s, 7 m/s/s slew: need ~5.5 s -> ~250 m to see turn and slow to ~8 m/s for sharp bend.
                    min_lookahead_for_braking = 85.0   # m when 15 < speed <= 25
                    if current_speed > 40.0:
                        min_lookahead_for_braking = 250.0  # m at very high speed: see sharp turn in time (e.g. k=0.1) without capping max speed
                    elif current_speed > 25.0:
                        min_lookahead_for_braking = 120.0  # m at high speed
                    if lookahead_dist < min_lookahead_for_braking and current_speed > 15.0:
                        lookahead_dist = min_lookahead_for_braking
                    # When speed is high, use at least 120 m lookahead so sharp bends (e.g. segment 43) are fully seen and we slow in time.
                    if current_speed > 20.0 and lookahead_dist < 120.0:
                        lookahead_dist = max(lookahead_dist, 120.0)
    
                    lookahead_idx = wp_last_idx
                    accumulated_dist = 0.0
                    min_v_max = target_speed  # Track minimum v_max over horizon
                    n_wp = len(wp_list)
                    
                    # Sample curvature along horizon; wrap waypoints so near end-of-lap we see the straight after the loop
                    sample_points = []
                    while accumulated_dist < lookahead_dist:
                        next_idx = (lookahead_idx + 1) % n_wp
                        x0, y0 = float(wp_list[lookahead_idx][0]), float(wp_list[lookahead_idx][1])
                        x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                        seg_dx = x1 - x0; seg_dy = y1 - y0
                        seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                        if seg_len < 1e-6:
                            lookahead_idx = next_idx
                            continue
                        sample_points.append((lookahead_idx, accumulated_dist))
                        accumulated_dist += seg_len
                        lookahead_idx = next_idx
                        if lookahead_idx == wp_last_idx and len(sample_points) > 1:
                            break  # wrapped full lap
                    
                    # Compute curvature at each sample point (use modulo so indices 0 and n_wp-1 are valid)
                    for sample_idx, sample_dist in sample_points:
                        i0 = (sample_idx - 1) % n_wp
                        i1 = sample_idx % n_wp
                        i2 = (sample_idx + 1) % n_wp
                        p0 = (float(wp_list[i0][0]), float(wp_list[i0][1]))
                        p1 = (float(wp_list[i1][0]), float(wp_list[i1][1]))
                        p2 = (float(wp_list[i2][0]), float(wp_list[i2][1]))
                        # Compute curvature (3-point method)
                        v1x = p1[0] - p0[0]; v1y = p1[1] - p0[1]
                        v2x = p2[0] - p1[0]; v2y = p2[1] - p1[1]
                        cross = v1x * v2y - v1y * v2x
                        len1 = (v1x*v1x + v1y*v1y) ** 0.5
                        len2 = (v2x*v2x + v2y*v2y) ** 0.5
                        if len1 > 1e-6 and len2 > 1e-6:
                            avg_len = (len1 + len2) / 2.0
                            if avg_len > 1e-6:
                                # Task 3: keep signed curvature (cross2D: left turn > 0, right turn < 0); magnitude for speed formulas
                                kappa_signed = 2.0 * cross / (len1 * len2 * avg_len)
                                abs_kappa = abs(kappa_signed)
                                if abs_kappa > curvature_ahead_max:
                                    curvature_ahead_max = abs_kappa
                                    curvature_ahead_max_signed = kappa_signed  # apex signed kappa
                                # Apply speed gate formula: v_max = sqrt(a_y_max / (|κ| + ε)); apply safety margin
                                v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + curvature_epsilon)) ** 0.5
                                if v_max_at_kappa < min_v_max:
                                    min_v_max = v_max_at_kappa
                    
                    # Expose signed curvature at apex for downstream (commit/approach logic)
                    self._curvature_ahead_max_signed = curvature_ahead_max_signed
                    # Apply minimum v_max over horizon
                    curvature_speed_limit = min_v_max
                    # Slow-in for sharp turns: when any significant curvature is ahead, cap speed more aggressively
                    # so we are already slow when the turn tightens (avoids "too fast, didn't turn in time").
                    # Very sharp turns (κ > 0.08): use 74% margin to reduce high CTE (e.g. segment 43 run).
                    if curvature_ahead_max > 0.015:
                        if curvature_ahead_max > 0.08:
                            slow_in_margin = 0.82   # Very sharp: stricter entry
                        elif curvature_ahead_max > 0.05:
                            slow_in_margin = 0.88
                        else:
                            slow_in_margin = 0.92
                        v_max_slow_in = slow_in_margin * (max_lateral_accel / (curvature_ahead_max + curvature_epsilon)) ** 0.5
                        if v_max_slow_in < curvature_speed_limit:
                            curvature_speed_limit = v_max_slow_in
                    curvature_speed_cap = curvature_speed_limit  # Task 4: single curvature-based speed cap
                except Exception as e:
                    curvature_speed_cap = target_speed
                    pass
            else:
                curvature_speed_cap = target_speed
            self._last_curvature_ahead_for_tactical = float(curvature_ahead_max)
            
            # --- CTE-based speed reduction (for MPC speed reference) ---
            # Generic: no TTL or segment IDs; only |CTE| so we slow when off-line on any track.
            # When CTE is large, cap target speed so the car can recover instead of running off.
            #
            # SD-41K: suppress the small-CTE bands (≤ 5 m) during planner-driven
            # lateral merges. The CTE cap was designed for "ego is accidentally
            # off-line, slow down to recover" — but during COMMIT_PASS_*,
            # HOLD_PASS_*, or ABORT_PASS (and the ~2-second window after
            # exiting them while ego physically converges back to the new
            # active TTL), the off-line state is *expected and managed* by
            # the planner. Throttling ego mid-merge for those CTE values
            # made F3R look "scared at start" (CTE 0.98m → cap 8 m/s during
            # the lateral shift to the left TTL) and produced the unstable
            # merge-back the user observed (FREE_RUN after pass with ego
            # still 3+m off optimal → CTE cap 7-8 + lateral MPC fighting to
            # converge → throttle/brake oscillation).
            #
            # Above 5 m CTE the cap still fires — that range is genuine
            # off-track / run-off territory that the planner doesn't intend
            # to put ego in even during a merge.
            _phase_state = str(getattr(self, '_phase_effective_planner_state', '') or '')
            _in_lateral_merge_now = _phase_state in (
                COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT,
                HOLD_PASS_LEFT, HOLD_PASS_RIGHT,
                ABORT_PASS,
            )
            if _in_lateral_merge_now:
                self._last_lateral_merge_sim_t = float(_sim_time_s)
            _time_since_merge = float(_sim_time_s) - float(
                getattr(self, '_last_lateral_merge_sim_t', -1.0e9)
            )
            _recent_merge_window_s = 2.0
            _suppress_small_cte_cap = (
                _in_lateral_merge_now or _time_since_merge < _recent_merge_window_s
            )
            if cte_mag_for_speed >= 10.0:
                # 10m+ CTE: strong decel and hard cap so we don't maintain high speed off-track
                cte_target_speed = min(3.0, max(0.0, current_speed - 4.0))
            elif cte_mag_for_speed >= 5.0:
                # 5-10m CTE: cap speed (was current_speed -> run-off). Cap at 5 m/s so MPC brakes.
                # SD-41K: keep this band even during lateral merges — 5+m CTE is genuinely
                # off-track territory that the planner doesn't intend.
                cte_target_speed = min(5.0, current_speed)
            elif cte_mag_for_speed >= 3.0:
                # 3-5m CTE: limit to 5 m/s (earlier recovery, any track)
                cte_target_speed = target_speed if _suppress_small_cte_cap else 5.0
            elif cte_mag_for_speed >= 2.0:
                # 2-3m CTE: limit to 6 m/s
                cte_target_speed = target_speed if _suppress_small_cte_cap else 6.0
            elif cte_mag_for_speed >= 1.5:
                # 1.5-2m CTE: limit to 6 m/s (early intervention)
                cte_target_speed = target_speed if _suppress_small_cte_cap else 6.0
            elif cte_mag_for_speed >= 1.0:
                # 1.0-1.5m CTE: limit to 7 m/s
                cte_target_speed = target_speed if _suppress_small_cte_cap else 7.0
            elif cte_mag_for_speed >= 0.5:
                # 0.5-1.0m CTE: limit to 8 m/s
                cte_target_speed = target_speed if _suppress_small_cte_cap else 8.0
            elif cte_mag_for_speed >= cte_stop_threshold:
                # At 50m+ CTE: aim for very low speed (encourages heavy braking)
                cte_target_speed = target_speed * 0.1
            elif cte_mag_for_speed >= cte_slowdown_threshold:
                # Between 15-50m: aim for 30% of target speed (encourages braking)
                factor = 0.3
                cte_target_speed = target_speed * factor
            elif cte_mag_for_speed >= cte_throttle_reduction_max:
                # Between 10-15m: linear from 50% to 30% of target speed
                factor = 0.5 - ((cte_mag_for_speed - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.2
                cte_target_speed = target_speed * factor
            else:
                # CTE < 0.5m: use full target speed (but still respect max speed limit)
                cte_target_speed = target_speed
            
            # --- Combine CTE and curvature speed limits (take minimum) ---
            # RC-6: explicit cap dict + binding_speed_cap telemetry. Previously the implicit
            # min() chain made it hard to attribute speed-cap behavior in logs. Now the
            # binding cap is recorded on self so [CtrlTrace] (or any future log line) can
            # surface which cap was active. Stash on self regardless of pit_mode.
            _p3cap = getattr(self, '_tactical_speed_cap', None)
            _speed_caps = {
                'cte': float(cte_target_speed),
                'curvature': float(curvature_speed_cap),
                'global': float(MAX_SPEED_LIMIT_MS),
            }
            if _scripted_speed_cap is not None:
                _speed_caps['phase1'] = float(_scripted_speed_cap)
            if _p3cap is not None:
                _speed_caps['tactical'] = float(_p3cap)
            effective_target_speed = min(_speed_caps.values())
            self._binding_speed_cap = min(_speed_caps, key=_speed_caps.get)
            # Pit mode (racing library): fixed coast target, no slew — drive differently than main
            if pit_mode:
                if current_speed < PIT_COAST_TOO_SLOW_MS:
                    effective_target_speed = min(PIT_MAX_SPEED_MS, PIT_COAST_MIN_TARGET_MS, curvature_speed_cap)
                else:
                    effective_target_speed = min(PIT_COAST_MIN_TARGET_MS, PIT_MAX_SPEED_MS, curvature_speed_cap)
                self._last_effective_target_speed = float(effective_target_speed)
            else:
                # --- Slew-rate limit on speed reference (smooth ramps into/out of turns) ---
                # When CTE is large, allow faster slew-down so the CTE speed cap takes effect quickly (generic run-off fix).
                dt_slew = _lon_controller.config.mpc_prediction_dt if hasattr(_lon_controller, 'config') else 0.05
                slew_down_ms = 12.0 if cte_mag_for_speed >= 3.0 else 7.0   # faster ramp-down when off-line
                _in_pass_maneuver = str(getattr(self, '_phase_effective_planner_state', '')) in (COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT, SETUP_LEFT, SETUP_RIGHT)
                slew_up_ms = 12.0 if _in_pass_maneuver else 8.0  # faster acceleration; pass maneuvers get max ramp
                # RC-7b: when ego is in a curve OR a curve is coming up (lookahead 25 wp ~ 25m),
                # don't allow the speed reference to ramp UP. This implements the racing-line
                # "carry momentum through the curve, brake before, throttle on exit" pattern by
                # preventing the planner from slamming throttle to v_target between two curves
                # (e.g. F2_tactical t=104-108 chicane oscillation: brake, release, throttle 1.0,
                # brake again, throttle 1.0). Speed CAN still drop (slew_down unchanged), so
                # safety-side responsiveness is preserved. Only "more throttle while curve nearby"
                # is suppressed.
                _seg_at = str(getattr(self, '_segment_type_at_wp', None) or '')
                _seg_ahead = str(getattr(self, '_segment_type_ahead', None) or '')
                _curve_nearby = (_seg_at == 'curve') or (_seg_ahead == 'curve')
                if _curve_nearby and not _in_pass_maneuver:
                    slew_up_ms = 0.0  # hold or drop -- no ramp up while curve is in scope
                self._curve_nearby_telemetry = _curve_nearby  # for [CtrlTrace]
                if not hasattr(self, '_last_effective_target_speed'):
                    self._last_effective_target_speed = float(effective_target_speed)
                last_eff = float(self._last_effective_target_speed)
                # SD-32A (post-SD-41F): raise slew-down rate to ~1.5g when the
                # CURVATURE cap is binding and asking for deceleration so the cap
                # is honored within ~0.4s instead of ~0.85s. Originally this also
                # fired for the "tactical" cap, but Stage C wired v_ref_profile
                # straight from PlannerReference.vx_mps; the slew limiter no
                # longer affects what the MPC tracks in tactical mode (it only
                # mutates the telemetry-only effective_target_speed). The
                # "tactical" check was deleted as dead work. The curvature path
                # remains load-bearing for non-tactical scenarios where
                # effective_target_speed still drives v_ref_profile.
                _bind = str(getattr(self, '_binding_speed_cap', '') or '')
                _safety_binding = _bind == "curvature"
                _decelerating = float(effective_target_speed) < last_eff
                if _safety_binding and _decelerating:
                    slew_down_ms = max(slew_down_ms, 15.0)
                effective_target_speed = max(last_eff - slew_down_ms * dt_slew, min(last_eff + slew_up_ms * dt_slew, float(effective_target_speed)))
                # SD-41C: SD-40 hard-ceiling clamp removed. The clamp was a
                # band-aid for the brain-leg gap — when planner cap dropped,
                # the slew limiter held effective_target_speed at the prior
                # value and the MPC kept accelerating. SD-40 forced eff <= cap
                # in one tick. With Stage C, v_ref_profile is sourced from
                # PlannerReference.vx_mps directly (which already composed all
                # caps with no slew), so the slew limiter no longer affects
                # what the MPC tracks in tactical mode — the clamp is redundant.
                # The slew limiter and effective_target_speed remain, but only
                # matter for the legacy non-tactical fallback path.
                self._last_effective_target_speed = float(effective_target_speed)

            # --- SD-41B: emit dense PlannerReference (compatibility shim) ---
            # Built every tick when the tactical planner is on. Stage B is a
            # non-consuming shim: the reference is stored on self for Stage C
            # (longitudinal MPC) and Stage D (lateral MPC) to consume. Until
            # those stages land, the per-tick effective_target_speed chain
            # above remains authoritative. The [PlannerRef] log line lets us
            # diff vx_mps[0] (planner-composed cap, no slew) against the
            # actual effective_target_speed (post-slew, post-clamp) to verify
            # the contract before flipping consumers.
            if _tactical_planner_enabled and not pit_mode:
                _ref_ttl_key = str(_scripted_active_ttl or "optimal")
                _ref_chosen_wp = None
                _ref_lap_len = 0.0
                _ref_ego_s = 0.0
                if _scripted_ttl_cache is not None and _ref_ttl_key in _scripted_ttl_cache:
                    _ref_chosen_wp = _scripted_ttl_cache[_ref_ttl_key][1]
                    _ref_lap_len = polyline_lap_length_m(_ref_chosen_wp) if _ref_chosen_wp else 0.0
                    if _ref_lap_len and _ref_lap_len > 0.0:
                        _ego_s_proj = _arc_length_project_xy(float(px), float(py), _ref_chosen_wp)
                        _ref_ego_s = float(_ego_s_proj) if _ego_s_proj is not None else 0.0
                _ref_horizon = _lon_controller.config.mpc_prediction_horizon if hasattr(_lon_controller, 'config') else 35
                _ref_dt = _lon_controller.config.mpc_prediction_dt if hasattr(_lon_controller, 'config') else 0.05
                self._planner_traj_id = int(getattr(self, '_planner_traj_id', 0)) + 1
                _prev_ref = getattr(self, '_planner_reference', None)
                _prev_vx0 = float(_prev_ref.vx_mps[0]) if _prev_ref is not None and _prev_ref.vx_mps.size > 0 else None
                self._planner_reference = _planner_build_reference(
                    mode=str(_mode_tac or "FREE_RUN"),
                    ttl_key=_ref_ttl_key,
                    decision_reason=str(_eff_reason or ""),
                    planner_cap_mps=self._tactical_speed_cap,
                    ego_x=float(px),
                    ego_y=float(py),
                    ego_speed_mps=float(current_speed),
                    ego_s_m=float(_ref_ego_s),
                    chosen_waypoints=_ref_chosen_wp,
                    lap_length_m=float(_ref_lap_len),
                    cte_cap_mps=float(cte_target_speed),
                    curvature_cap_mps=float(curvature_speed_cap),
                    global_max_mps=float(MAX_SPEED_LIMIT_MS),
                    scripted_cap_mps=float(_scripted_speed_cap) if _scripted_speed_cap is not None else None,
                    racing_line_target_mps=float(target_speed),
                    mpc_horizon_n=int(_ref_horizon),
                    mpc_dt_s=float(_ref_dt),
                    sim_time_s=float(_sim_time_s),
                    traj_id=int(self._planner_traj_id),
                    prev_vx0_mps=_prev_vx0,
                )
                _ref0 = float(self._planner_reference.vx_mps[0])
                print(
                    f"{_fl_mpc} [PlannerRef] t={_sim_time_s:.2f}s "
                    f"traj_id={self._planner_traj_id} mode={self._planner_reference.mode} "
                    f"ttl={self._planner_reference.ttl_key} "
                    f"vx0={_ref0:.2f} eff={effective_target_speed:.2f} "
                    f"binding={self._planner_reference.binding_cap_source} "
                    f"horizon={int(self._planner_reference.horizon_length())}"
                )

                # SD-41E (skeleton): the safety supervisor's pre-MPC
                # reference-swap was implemented and tested but produced an
                # F14 regression — the safe-stop ramp's tail-of-horizon
                # zeros caused the MPC to brake more aggressively than the
                # planner's ABORT_PASS already wanted, perturbing ego's
                # trajectory enough to trip a pre-existing pit_mode false
                # positive in the segment classifier (SD-12a). The
                # `should_swap_for_emergency` and `swap_reference_for_emergency`
                # helpers in scenic.domains.racing.safety.stability_guard
                # remain available for future iteration. Until they are
                # called here, Stage E is functionally just (a) auto-enabling
                # the post-MPC stability guard when tactical is on and
                # (b) shipping the helper functions for later use. The
                # SD-36 panic-brake bypass at the post-MPC site continues
                # to provide emergency authority and is kept by Stage F.

            # --- Build speed reference profile for MPC ---
            horizon = _lon_controller.config.mpc_prediction_horizon
            # Pit mode: constant profile at coast target (no curvature lookahead)
            if pit_mode:
                v_ref_profile = [float(effective_target_speed)] * horizon
                v_ref_profile = [min(v, PIT_MAX_SPEED_MS) for v in v_ref_profile]
            else:
                # Main mode: profile from effective_target_speed, then reduce for upcoming turns
                v_ref_profile = [float(effective_target_speed)] * horizon
            # SD-41C: when the tactical planner is on, source v_ref_profile
            # from PlannerReference.vx_mps directly. The planner already
            # composed all caps (cte, curvature, global, scripted, tactical)
            # into vx_mps with NO slew or hard-ceiling band-aid, so the MPC
            # tracks the brain's intent at the same tick it's set. The
            # per-step curvature reduction below still runs on top, so any
            # apex tighter than the planner's binding cap will pull the
            # profile down further. Non-tactical scenarios (no PlannerReference)
            # fall through to the legacy effective_target_speed source above.
            if not pit_mode and _tactical_planner_enabled:
                _ref_for_mpc = getattr(self, '_planner_reference', None)
                if _ref_for_mpc is not None and _ref_for_mpc.vx_mps is not None and _ref_for_mpc.vx_mps.size > 0:
                    _vx = _ref_for_mpc.vx_mps
                    if _vx.size >= horizon:
                        v_ref_profile = [float(v) for v in _vx[:horizon]]
                    else:
                        # Pad with the last value to fill the horizon.
                        _last = float(_vx[-1])
                        v_ref_profile = [float(v) for v in _vx] + [_last] * (horizon - _vx.size)
            # If we have waypoints (main mode only; pit already has constant profile), build a speed profile that reduces speed for upcoming turns (cached + vectorized)
            if not pit_mode and use_waypoints and wp_list and len(wp_list) >= 2:
                # Stable cache key (shape + first/last waypoint) so cache hits when racing line is unchanged
                wp_cache_key = (len(wp_list), (float(wp_list[0][0]), float(wp_list[0][1])), (float(wp_list[-1][0]), float(wp_list[-1][1])))
                self._wp_cache_tick = getattr(self, '_wp_cache_tick', 0) + 1
                # Build/reuse curvature cache (2D cumdist + per-waypoint curve_vmax)
                if getattr(self, '_wp_curve_cache_key', None) != wp_cache_key:
                    nwp = len(wp_list)
                    if nwp >= 2:
                        # Closed-loop segment lengths (including last->first)
                        seg_len_xy = np.zeros(nwp, dtype=np.float64)
                        for i in range(nwp):
                            j = (i + 1) % nwp
                            x0, y0 = float(wp_list[i][0]), float(wp_list[i][1])
                            x1, y1 = float(wp_list[j][0]), float(wp_list[j][1])
                            dx, dy = x1 - x0, y1 - y0
                            seg_len_xy[i] = max((dx*dx + dy*dy) ** 0.5, 1e-9)
                        cum = np.zeros(nwp + 1, dtype=np.float64)
                        cum[1:] = np.cumsum(seg_len_xy)
                        self._wp_seg_len_xy = seg_len_xy
                        self._wp_cumdist_xy = cum
                        self._wp_total_len_xy = float(cum[-1])
                        # Curvature vmax per waypoint (closed-loop: modulo neighbors)
                        curve_vmax_list = []
                        for i in range(nwp):
                            i0 = (i - 1) % nwp
                            i1 = i
                            i2 = (i + 1) % nwp
                            p0 = (float(wp_list[i0][0]), float(wp_list[i0][1]))
                            p1 = (float(wp_list[i1][0]), float(wp_list[i1][1]))
                            p2 = (float(wp_list[i2][0]), float(wp_list[i2][1]))
                            v1x = p1[0] - p0[0]; v1y = p1[1] - p0[1]
                            v2x = p2[0] - p1[0]; v2y = p2[1] - p1[1]
                            cross = v1x * v2y - v1y * v2x
                            len1 = (v1x*v1x + v1y*v1y) ** 0.5
                            len2 = (v2x*v2x + v2y*v2y) ** 0.5
                            if len1 > 1e-6 and len2 > 1e-6:
                                avg_len = (len1 + len2) / 2.0
                                abs_kappa = abs(2.0 * cross / (len1 * len2 * avg_len))
                                v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + curvature_epsilon)) ** 0.5
                                curve_vmax_list.append(v_max_at_kappa)
                            else:
                                curve_vmax_list.append(1e6)
                        self._wp_curve_vmax = np.asarray(curve_vmax_list, dtype=np.float64)
                        self._wp_curve_cache_key = wp_cache_key
                        self._wp_curve_cache_rebuilds = getattr(self, '_wp_curve_cache_rebuilds', 0) + 1
                else:
                    self._wp_curve_cache_hits = getattr(self, '_wp_curve_cache_hits', 0) + 1
                try:
                    if getattr(self, '_wp_curve_vmax', None) is not None:
                        # Vectorized: dist_vec, modulo lookahead for closed loop, gather curve_vmax, apply min
                        dt = _lon_controller.config.mpc_prediction_dt
                        dist_vec = current_speed * (np.arange(1, horizon + 1, dtype=np.float64)) * dt
                        nwp = len(wp_list)
                        base_idx = int(wp_last_idx) % nwp
                        base_cum = self._wp_cumdist_xy[base_idx]
                        L = self._wp_total_len_xy
                        target_abs = base_cum + dist_vec
                        target_mod = np.mod(target_abs, L)
                        seg_idx = np.searchsorted(self._wp_cumdist_xy, target_mod, side='right') - 1
                        seg_idx = np.clip(seg_idx, 0, nwp - 1)
                        cap_profile = self._wp_curve_vmax[seg_idx]
                        v_ref_profile = np.minimum(np.asarray(v_ref_profile, dtype=np.float64), cap_profile).tolist()
                except Exception as e:
                    # If profile building fails, use constant speed
                    pass
            
            # Task 4: enforce curvature-based speed cap on entire profile (main mode; pit already constant)
            if not pit_mode:
                v_ref_profile = [min(v, curvature_speed_cap) for v in v_ref_profile]
            # Apply max speed limit to profile (pit mode: already capped above; main: 140 mph cap)
            if not pit_mode:
                v_ref_profile = [min(v, MAX_SPEED_LIMIT_MS) for v in v_ref_profile]
            
            # --- Build grade profile for longitudinal MPC (if 3D waypoints available) ---
            grade_profile = None
            if use_waypoints and wp_list and len(wp_list) >= 2:
                # --- Build/reuse waypoint geometry cache --- (same stable key as curvature block)
                wp_cache_key = (len(wp_list), (float(wp_list[0][0]), float(wp_list[0][1])), (float(wp_list[-1][0]), float(wp_list[-1][1])))
                if getattr(self, '_wp_grade_cache_key', None) != wp_cache_key:
                    nwp = len(wp_list)
                    if nwp >= 2:
                        # Closed-loop grade segments (including last->first)
                        seg_len_3d = []
                        seg_grade = []
                        for i in range(nwp):
                            j = (i + 1) % nwp
                            wp0 = wp_list[i]
                            wp1 = wp_list[j]
                            x0, y0 = float(wp0[0]), float(wp0[1])
                            x1, y1 = float(wp1[0]), float(wp1[1])
                            z0 = float(wp0[2]) if len(wp0) >= 3 else 0.0
                            z1 = float(wp1[2]) if len(wp1) >= 3 else 0.0

                            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
                            L3 = (dx*dx + dy*dy + dz*dz) ** 0.5
                            Lxy = (dx*dx + dy*dy) ** 0.5

                            seg_len_3d.append(max(L3, 1e-9))
                            seg_grade.append(math.atan2(dz, Lxy) if Lxy > 1e-6 else 0.0)

                        self._wp_seg_len_3d = np.asarray(seg_len_3d, dtype=np.float64)
                        self._wp_seg_grade = np.asarray(seg_grade, dtype=np.float64)

                        cum = np.zeros(nwp + 1, dtype=np.float64)
                        cum[1:] = np.cumsum(self._wp_seg_len_3d)
                        self._wp_cumdist_3d = cum
                        self._wp_total_len_3d = float(cum[-1])

                        self._wp_grade_cache_key = wp_cache_key
                        self._wp_grade_cache_rebuilds = getattr(self, '_wp_grade_cache_rebuilds', 0) + 1
                else:
                    self._wp_grade_cache_hits = getattr(self, '_wp_grade_cache_hits', 0) + 1
                if getattr(self, '_wp_cache_tick', 0) % 100 == 0:
                    ch = getattr(self, '_wp_curve_cache_hits', 0); cr = getattr(self, '_wp_curve_cache_rebuilds', 0)
                    gh = getattr(self, '_wp_grade_cache_hits', 0); gr = getattr(self, '_wp_grade_cache_rebuilds', 0)
                    print(f"[Waypoint cache] tick={getattr(self, '_wp_cache_tick', 0)}  curve: hits={ch} rebuilds={cr}  grade: hits={gh} rebuilds={gr}")
                if len(wp_list[0]) >= 3:
                    horizon = _lon_controller.config.mpc_prediction_horizon
                    dt_mpc = _lon_controller.config.mpc_prediction_dt

                    dist_vec = current_speed * (np.arange(1, horizon + 1, dtype=np.float64)) * dt_mpc

                    nwp = len(wp_list)
                    base_idx = int(wp_last_idx) % nwp
                    base_cum = self._wp_cumdist_3d[base_idx]
                    L = self._wp_total_len_3d

                    target_abs = base_cum + dist_vec
                    target_mod = np.mod(target_abs, L)

                    seg_idx = np.searchsorted(self._wp_cumdist_3d, target_mod, side='right') - 1
                    seg_idx = np.clip(seg_idx, 0, nwp - 1)

                    grade_profile = self._wp_seg_grade[seg_idx].astype(np.float64, copy=False)
            if _bt is not None:
                _bt.end_section('waypoint_speed_grade')
            
            # --- Use MPC for throttle/brake control ---
            # Get current acceleration (estimate from speed if not available)
            dt = getattr(simulation(), 'control_dt', None)
            if dt is None:
                dt = simulation().timestep * max(1, getattr(simulation(), '_control_interval', 1))
            current_accel = 0.0
            if hasattr(self, '_prev_speed_mpc'):
                current_accel = (current_speed - self._prev_speed_mpc) / dt
                current_accel = max(-15.0, min(20.0, current_accel))  # Clamp to reasonable range
            self._prev_speed_mpc = current_speed
            
            # Build vehicle state for MPC
            vehicle_state_mpc = {
                'speed': current_speed,
                'acceleration': current_accel,
                'gear': getattr(self, 'gear', 0) if manage_gears and hasattr(self, 'setGear') else None
            }
            
            # Compute throttle/brake using MPC (with grade compensation if available)
            try:
                _sec_t_lon_start = _wallclock_time.perf_counter()
                throttle_mpc, brake_mpc = _lon_controller.run_step(
                    vehicle_state_mpc,
                    v_ref_profile,
                    None,  # curvature_profile not used in simplified model
                    grade_profile  # Road grade profile for gravity compensation
                )
                _sec_lon_ms += (_wallclock_time.perf_counter() - _sec_t_lon_start) * 1000.0
                throttle_mpc = float(throttle_mpc)
                brake_mpc = float(brake_mpc)
            except Exception as ex:
                print(f"{_fbhv} MPC longitudinal error: {ex}, using fallback")
                # Fallback: simple proportional control. SD-41C: prefer the
                # planner reference's vx[0] when available so the fallback
                # honors the brain even when the MPC solver bails.
                _fb_target = float(effective_target_speed)
                if v_ref_profile:
                    _fb_target = float(v_ref_profile[0])
                speed_error = _fb_target - current_speed
                if speed_error > 0:
                    throttle_mpc = min(1.0, speed_error * 0.1)
                    brake_mpc = 0.0
                else:
                    throttle_mpc = 0.0
                    brake_mpc = min(1.0, abs(speed_error) * 0.1)
            
            # Use MPC outputs (will be processed by CTE-aware safety envelope below)
            throttle_pid = throttle_mpc  # Keep variable name for compatibility with existing code
            
            # -------------------------------
            # CTE-aware longitudinal safety
            # -------------------------------
    
            # Hysteresis state tracking
            if not hasattr(self, '_was_cte_large'):
                self._was_cte_large = False
    
            _sim_cte = simulation()
            _is_non_ego_fellow = (
                getattr(_sim_cte.scene, 'egoObject', None) is not None
                and self is not _sim_cte.scene.egoObject
            )
            # Fellows (v,d plant): heavy CTE brake kills speed -> weak bicycle yaw -> lateral MPC cannot recover
            _cte_enter_large = 7.5 if _is_non_ego_fellow else 4.5
            _cte_exit_large = 6.5 if _is_non_ego_fellow else 3.5
            _cte_very_large_m = 14.0 if _is_non_ego_fellow else 10.0
    
            if cte_mag_for_speed >= _cte_enter_large:
                self._was_cte_large = True
            elif cte_mag_for_speed < _cte_exit_large:
                self._was_cte_large = False
    
            SPEED_THRESHOLD_FOR_BRAKE = 2.0
            MIN_THROTTLE_WHEN_STOPPED = 0.10
    
            # Output knobs (generic: no TTL/segment; avoid run-off by braking sooner when |CTE| is large)
            MAX_BRAKE_NORMAL = 0.25        # MPC brake cap when CTE is small
            MAX_BRAKE_LARGE_CTE = 0.65     # 4.5-10m CTE: allow stronger brake to recover
            MAX_BRAKE_VERY_LARGE = 0.90    # 10m+ CTE: strong brake, still not instant 1.0
            BRAKE_SLEW = 0.15              # per step
    
            cte_brake = 0.0
            throttle_override = None
    
            very_large = (cte_mag_for_speed >= _cte_very_large_m)
            large = (self._was_cte_large or cte_mag_for_speed >= _cte_enter_large)
    
            if very_large:
                # Far off-track: strong brake so we don't maintain speed (generic fix for run-off)
                if _is_non_ego_fellow:
                    if current_speed > 4.0:
                        throttle_override = 0.0
                        cte_brake = 0.28
                    elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                        throttle_override = 0.0
                        cte_brake = 0.18
                    else:
                        throttle_override = MIN_THROTTLE_WHEN_STOPPED
                        cte_brake = 0.0
                elif current_speed > 4.0:
                    throttle_override = 0.0
                    cte_brake = 0.50
                elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                    throttle_override = 0.0
                    cte_brake = 0.30
                else:
                    throttle_override = MIN_THROTTLE_WHEN_STOPPED
                    cte_brake = 0.0
            elif large:
                # 4.5-10m CTE (ego): meaningful brake at higher speed. Fellow: lighter so MPC keeps authority.
                if _is_non_ego_fellow:
                    if current_speed > 8.0:
                        throttle_override = None
                        cte_brake = 0.06
                    elif current_speed > 5.0:
                        throttle_override = None
                        cte_brake = 0.04
                    elif current_speed > 3.0:
                        throttle_override = None
                        cte_brake = 0.02
                    else:
                        throttle_override = None
                        cte_brake = 0.0
                elif current_speed > 8.0:
                    throttle_override = 0.0
                    cte_brake = 0.25
                elif current_speed > 5.0:
                    throttle_override = 0.0
                    cte_brake = 0.15
                elif current_speed > 3.0:
                    throttle_override = 0.0
                    cte_brake = 0.05
                else:
                    throttle_override = None
                    cte_brake = 0.0
    
            # Apply throttle override if set
            if throttle_override is not None:
                throttle_mpc = throttle_override
    
            # -------------------------------
            # Brake cap + merge + SLEW-LIMIT (drop-in replacement)
            # -------------------------------
    
            # Compute a brake cap depending on CTE regime
            if very_large:
                brake_cap = MAX_BRAKE_VERY_LARGE
            elif large:
                brake_cap = MAX_BRAKE_LARGE_CTE
            else:
                brake_cap = MAX_BRAKE_NORMAL
    
            # Merge CTE brake + MPC brake (both capped)
            raw_brake = max(float(cte_brake), float(brake_mpc))
            raw_brake = min(float(raw_brake), float(brake_cap))
    
            # ---- Guard: do not "slam brake" at (near) standstill ----
            STOP_SPEED = 0.6  # m/s (tune: 0.3~1.0)
            if current_speed <= STOP_SPEED:
                raw_brake = 0.0
    
            # ---- Slew-limit brake to avoid spikes (apply slower, release faster) ----
            BRAKE_SLEW_UP = 0.12     # max increase per step (0..1 scale)
            BRAKE_SLEW_DOWN = 0.20   # max decrease per step
    
            if not hasattr(self, "_last_raw_brake"):
                self._last_raw_brake = float(raw_brake)
    
            prev_brake = float(self._last_raw_brake)
            db = float(raw_brake) - prev_brake
            limited = False
    
            if db > BRAKE_SLEW_UP:
                raw_brake = prev_brake + BRAKE_SLEW_UP
                limited = True
            elif db < -BRAKE_SLEW_DOWN:
                raw_brake = prev_brake - BRAKE_SLEW_DOWN
                limited = True
    
            raw_brake = max(0.0, min(float(brake_cap), float(raw_brake)))
            self._last_raw_brake = float(raw_brake)
            final_brake = max(0.0, min(1.0, raw_brake))
    
            # --- Lateral Control (MPC) ---
            sim = simulation()
            is_fellow_mpc_virt = (
                getattr(sim.scene, "egoObject", None) is not None
                and self is not sim.scene.egoObject
                and getattr(sim, "mpc_config", None) is not None
            )
            _ctrl_dt_virt = getattr(sim, "control_dt", None)
            if _ctrl_dt_virt is None or _ctrl_dt_virt <= 0:
                _ctrl_dt_virt = float(sim.timestep) * max(1, getattr(sim, "_control_interval", 1))

            # Build vehicle state for MPC (assembly + read_state_from_controldesk)
            vehicle_state = {
                'x': px,
                'y': py,
                'yaw': car_heading if car_heading is not None else 0.0,
                'speed': current_speed,
            }

            # Scenic: bare names / importlib -> self.* ; use builtin __import__ only.
            if is_fellow_mpc_virt:
                __import__(
                    "scenic.domains.racing.mpc.fellow_virtual_mpc_state",
                    fromlist=["fellow_virt_prepare_for_scenic"],
                ).fellow_virt_prepare_for_scenic(
                    self,
                    vehicle_state,
                    wp_list,
                    use_waypoints,
                    wp_last_idx,
                    car_heading,
                )
    
            # Add gear information for MPC (check before gear change logic)
            if manage_gears and hasattr(self, 'setGear'):
                current_gear = getattr(self, 'gear', 0)
                vehicle_state['gear'] = current_gear
    
            # Add optional yaw_rate if available (fellow: plant rate often not meaningful for MPC)
            if not is_fellow_mpc_virt and hasattr(self, 'angularVelocity') and self.angularVelocity is not None:
                try:
                    vehicle_state['yaw_rate'] = float(self.angularVelocity.z) if hasattr(self.angularVelocity, 'z') else 0.0
                except Exception:
                    pass
    
            # Ego: steering feedback from ControlDesk. Fellow: virtual steer (no Vesi steer path).
            if not is_fellow_mpc_virt and hasattr(sim, 'mpc_config') and sim.mpc_config:
                from scenic.domains.racing.mpc.io_adapter import read_state_from_controldesk
                try:
                    cd_state = read_state_from_controldesk(sim, self)
                    if 'steer_actual' in cd_state:
                        vehicle_state['steer_actual'] = cd_state['steer_actual']   # match the same sign convention as command
    
                except Exception:
                    # If reading fails, MPC will use previous state estimate
                    pass
    
            # Convert waypoints to list of tuples for MPC (preserve 3D if available); cache to avoid rebuild every tick
            _wp_src = wp_list
            _wp_src_id = id(_wp_src)
            _wp_len = len(_wp_src) if _wp_src else 0

            cache_ok = (
                hasattr(self, '_waypoints_for_mpc_cache_id') and
                self._waypoints_for_mpc_cache_id == _wp_src_id and
                getattr(self, '_waypoints_for_mpc_cache_len', -1) == _wp_len
            )

            if cache_ok:
                waypoints_for_mpc = self._waypoints_for_mpc_cache
            else:
                if _wp_src and len(_wp_src) >= 2:
                    is_3d_waypoints = len(_wp_src[0]) >= 3
                    if is_3d_waypoints:
                        waypoints_for_mpc = tuple((float(wp[0]), float(wp[1]), float(wp[2])) for wp in _wp_src)
                    else:
                        waypoints_for_mpc = tuple((float(wp[0]), float(wp[1])) for wp in _wp_src)
                else:
                    waypoints_for_mpc = None

                self._waypoints_for_mpc_cache = waypoints_for_mpc
                self._waypoints_for_mpc_cache_id = _wp_src_id
                self._waypoints_for_mpc_cache_len = _wp_len

            # Compute steering using MPC (mpc_total in [LoopOther] from record_lateral_mpc_ms + record_longitudinal_mpc_ms)
            # Pass behavior's progress index (wp_last_idx) so MPC searches locally; MPC owns chosen segment (last_seg_idx), we do not sync back.
            try:
                # Corridor-aware MPC: pass per-waypoint LEFT/RIGHT distances if the TTL was
                # loaded in race_common 20-column format (attach_ttl populates these). Falls
                # back to plain line-tracking when bounds are unavailable. See docs/frames.md.
                _ttl_left_dist = getattr(self, 'ttl_left_dist_m', None)
                _ttl_right_dist = getattr(self, 'ttl_right_dist_m', None)
                _sec_t_lat_start = _wallclock_time.perf_counter()
                steer_mpc = _lat_controller.run_step(
                    vehicle_state,
                    waypoints_for_mpc,
                    wp_last_idx if (use_waypoints and wp_list and len(wp_list) >= 2) else None,
                    cte_magnitude=cte_mag_for_speed,
                    v_ref_profile=v_ref_profile,  # Same trajectory as longitudinal: smooth turns, avoid over-steer then correct
                    curvature_ahead_max=curvature_ahead_max,  # For deadzone eligibility: never deadzone in moderate curvature (curv_ahead_max < curv_deadzone_max)
                    left_dist_per_wp=_ttl_left_dist,
                    right_dist_per_wp=_ttl_right_dist,
                )
                _sec_lat_ms += (_wallclock_time.perf_counter() - _sec_t_lat_start) * 1000.0
                steer_mpc = float(steer_mpc)
    
            except Exception as e:
                print(f"{_fbhv} MPC error: {e}, using fallback")
                steer_mpc = 0.0
            if is_fellow_mpc_virt:
                __import__(
                    "scenic.domains.racing.mpc.fellow_virtual_mpc_state",
                    fromlist=["fellow_virt_step_for_scenic"],
                ).fellow_virt_step_for_scenic(
                    self,
                    float(steer_mpc),
                    current_speed,
                    getattr(_lat_controller, "_log_kappa_ref_at_proj", None),
                    float(_ctrl_dt_virt),
                    float(sim.mpc_config.wheel_base),
                    float(sim.mpc_config.steer_tau),
                )
            if _bt is not None:
                _bt.start_section('cmd_post')
            # Keep waypoint-based CTE (e_y_mpc) for mismatch fallback (used when behavior CTE disagrees)
            if getattr(_lat_controller, '_log_mpc_e_y', None) is not None:
                self._last_waypoint_cte_for_speed = abs(getattr(_lat_controller, '_log_mpc_e_y'))
    
            # --- CTE-aware safety envelope (for throttle/brake) ---
            # Single definition: MPC e_y (projection onto chosen segment) when available; legacy only as fallback when MPC did not run.
            _ey = getattr(_lat_controller, '_log_mpc_e_y', None)
            if _ey is not None:
                cte = float(_ey)
            else:
                cte = float(getattr(self, '_legacy_cte_this_tick', 0.0))
            cte_mag = abs(cte)
            # One ff_log line per MPC tick (feedforward + command side: u_norm, delta_cmd_rad post-clamp, current_delta_max)
            _lc_ff = _lat_controller
            _seg_id = getattr(_lc_ff, '_log_segment_id', None)
            _v_ff = getattr(_lc_ff, '_log_v', None)
            _kap_ff = getattr(_lc_ff, '_log_kappa_ref_at_proj', None)
            _dff = getattr(_lc_ff, '_log_delta_ff', None)
            _dfb = getattr(_lc_ff, '_log_delta_fb', None)
            _dtot = getattr(_lc_ff, '_log_delta_total', None)
            _dcmd = getattr(_lc_ff, '_log_delta_cmd_rad', None)
            _dmax = getattr(_lc_ff, '_log_current_delta_max', None)
            _sraw = getattr(_lc_ff, '_log_steer_mpc_raw', None)
            _slpf = getattr(_lc_ff, '_log_steer_after_lpf', None)
            _seg_s = str(_seg_id) if _seg_id is not None else "?"
            _v_s = f"{_v_ff:.3f}" if _v_ff is not None else "?"
            _kap_s = f"{_kap_ff:.4f}" if _kap_ff is not None else "?"
            _dff_s = f"{_dff:.4f}" if _dff is not None else "?"
            _dfb_s = f"{_dfb:.4f}" if _dfb is not None else "?"
            _dtot_s = f"{_dtot:.4f}" if _dtot is not None else "?"
            _dcmd_s = f"{_dcmd:.4f}" if _dcmd is not None else "?"
            _dmax_s = f"{_dmax:.4f}" if _dmax is not None else "?"
            _sraw_s = f"{_sraw:.3f}" if _sraw is not None else "?"
            _slpf_s = f"{_slpf:.3f}" if _slpf is not None else "?"
            _log_step = getattr(self, '_behavior_step_count', 0) + 1
            if _log_step % 50 == 0:
                print(f"{_fl_mpc} ff_log segment_id={_seg_s} v={_v_s} kappa_ref_at_proj={_kap_s} delta_ff={_dff_s} delta_fb={_dfb_s} delta_total={_dtot_s} delta_cmd_rad={_dcmd_s} current_delta_max={_dmax_s} u_norm_mpc={_sraw_s} u_norm_lpf={_slpf_s}")
                # Task 1: signed curvature sanity log (every 50 steps)
                _kappa_ahead = getattr(_lc_ff, '_log_kappa_ref_ahead_signed', None)
                _s_ref = getattr(_lc_ff, '_log_s_ref', None)
                _kappa_ahead_s = f"{_kappa_ahead:.4f}" if _kappa_ahead is not None else "?"
                _s_ref_s = f"{_s_ref:.3f}" if _s_ref is not None else "?"
                print(f"{_fl_mpc} CURV_SANITY kappa_ref_at_proj={_kap_s} kappa_ref_ahead_signed={_kappa_ahead_s} segment_id={_seg_s} s_ref={_s_ref_s}")
            # Heading diff for steering conditioning (segment direction vs vehicle heading, wrapped to [-pi, pi])
            heading_diff = 0.0
            if car_heading is not None and use_waypoints and wp_list and len(wp_list) >= 2:
                n_wp = len(wp_list)
                next_idx = (wp_last_idx + 1) % n_wp
                x0, y0 = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                x1, y1 = float(wp_list[next_idx][0]), float(wp_list[next_idx][1])
                seg_heading = math.atan2(y1 - y0, x1 - x0)
                heading_diff = math.atan2(math.sin(seg_heading - car_heading), math.cos(seg_heading - car_heading))
            local_throttle_limit = throttle_limit
            final_brake = 0.0
    
            # Plan: steering = road wheel angle (rad). Single source of truth delta_max (racing.constants).
            # Progressive throttle reduction based on CTE magnitude
            if cte_mag >= cte_stop_threshold:
                local_throttle_limit = 0.0
                final_brake = 1.0
                steer_mpc = (-0.5 * DELTA_MAX_RAD) if cte > 0 else (0.5 * DELTA_MAX_RAD)
            elif cte_mag >= cte_slowdown_threshold:
                local_throttle_limit = min(local_throttle_limit, 0.3)
                final_brake = min(1.0, (cte_mag - cte_slowdown_threshold) / (cte_stop_threshold - cte_slowdown_threshold))
            elif cte_mag >= cte_throttle_reduction_max:
                throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.7
                local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
                local_throttle_limit = max(local_throttle_limit, 0.3)
            elif cte_mag >= cte_throttle_reduction_start:
                throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_start) / (cte_throttle_reduction_max - cte_throttle_reduction_start)) * (1.0 - min_throttle_at_large_cte / throttle_limit)
                local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
                local_throttle_limit = max(local_throttle_limit, min_throttle_at_large_cte)
    
            # Speed-based throttle reduction when CTE is large
            if cte_mag >= cte_throttle_reduction_start and current_speed > 3.0:
                if current_speed <= 8.0:
                    speed_penalty = (current_speed - 3.0) / 10.0
                else:
                    speed_penalty = 0.5 + ((current_speed - 8.0) / 10.0) * 0.3
                speed_penalty = min(0.8, speed_penalty)
                local_throttle_limit = local_throttle_limit * (1.0 - speed_penalty)
    
            # Additional throttle reduction for moderate CTE (2-4m) when speed is high
            if cte_mag >= 2.0 and cte_mag < 5.0 and current_speed > 4.0:
                if current_speed <= 6.0:
                    moderate_cte_penalty = (current_speed - 4.0) / 4.0
                else:
                    moderate_cte_penalty = 0.5 + ((current_speed - 6.0) / 4.0) * 0.3
                moderate_cte_penalty = min(0.8, moderate_cte_penalty)
                local_throttle_limit = local_throttle_limit * (1.0 - moderate_cte_penalty)
    
            
            # --- Steering: MPC is single owner of clamp/rate limit (mpc_lateral.py). Safety backup below. ---
            final_steer = max(-DELTA_MAX_RAD, min(DELTA_MAX_RAD, float(steer_mpc)))
            final_steer_ds = final_steer   # rad; IO adapter converts to dSPACE steering_wheel_deg in simulator
            # fix.md Option 1: heading-based yaw rate (avoids yaw-rate channel stuck at 0)
            _psi_rad = float(car_heading) if car_heading is not None else None
            _dt = float(simulation().timestep) if hasattr(simulation(), 'timestep') else 0.05
            _psi_prev = getattr(self, '_psi_prev_steer_cal', None)
            _yaw_rps_est = None
            if _psi_rad is not None and _psi_prev is not None and _dt > 1e-6:
                _dpsi = _psi_rad - _psi_prev
                _dpsi = math.atan2(math.sin(_dpsi), math.cos(_dpsi))  # wrapToPi
                _yaw_rps_est = _dpsi / _dt
            self._psi_prev_steer_cal = _psi_rad
            # Unit-inference analysis: prefer yaw_rps_est when channel yaw_rate is zero/missing (fix.md)
            _L_wb = getattr(_lat_controller.config, 'wheel_base', 2.97)
            _delta_cmd_rad = float(final_steer)
            _kappa_pred = math.tan(_delta_cmd_rad) / _L_wb if _L_wb > 1e-6 else 0.0
            _speed_mps = float(current_speed) if current_speed is not None else 0.0
            _yaw_rate_ch = vehicle_state.get('yaw_rate', None) if vehicle_state else None
            _speed_floor = 0.5
            _yaw_rps_use = _yaw_rps_est if (_yaw_rate_ch is None or abs(float(_yaw_rate_ch)) < 1e-6) else _yaw_rate_ch
            if _yaw_rps_use is not None:
                _yaw_rps_use = float(_yaw_rps_use)
            _kappa_meas = (_yaw_rps_use / max(_speed_mps, _speed_floor)) if _yaw_rps_use is not None else None
            _curv_err = (_kappa_meas - _kappa_pred) if _kappa_meas is not None else None
            _kappa_ratio = (_kappa_meas / _kappa_pred) if (_kappa_meas is not None and abs(_kappa_pred) > 1e-3) else None
            _psi_s = f"{_psi_rad:.4f}" if _psi_rad is not None else "?"
            _yaw_est_s = f"{_yaw_rps_est:.4f}" if _yaw_rps_est is not None else "?"
            _kmeas_s = f"{_kappa_meas:.5f}" if _kappa_meas is not None else "?"
            _cerr_s = f"{_curv_err:.5f}" if _curv_err is not None else "?"
            _krat_s = f"{_kappa_ratio:.4f}" if _kappa_ratio is not None else "?"
            _steer_cal_tick = getattr(self, '_steer_cal_log_count', 0)
            _log_step_plant = getattr(self, '_behavior_step_count', 0) + 1
            if _log_step_plant % 50 == 0:
                print(f"[STEER_CAL] v={_speed_mps:.3f} psi={_psi_s} yaw_rps_est={_yaw_est_s} delta_cmd_rad={_delta_cmd_rad:.4f} L={_L_wb:.3f} kappa_meas={_kmeas_s} kappa_pred={_kappa_pred:.5f} curv_err={_cerr_s} kappa_ratio={_krat_s}")
                print(f"[PLANT] speed_mps={_speed_mps:.3f} kappa_pred={_kappa_pred:.4f} kappa_meas={_kmeas_s} curvature_error={_cerr_s}")
            self._steer_cal_log_count = _steer_cal_tick + 1
    
            # Ensure local_throttle_limit doesn't exceed base throttle_limit
            local_throttle_limit = min(local_throttle_limit, throttle_limit)
            
            # Use MPC throttle/brake outputs (already processed by CTE-aware safety above)
            final_throttle = max(0.0, min(local_throttle_limit, throttle_mpc))
            final_brake = raw_brake  # Already merged and capped above
            
            # Hard speed limit: pit mode vs main (racing library mode drives which limit applies)
            current_speed_limit_ms = PIT_MAX_SPEED_MS if pit_mode else MAX_SPEED_LIMIT_MS
            SPEED_LIMIT_DEADBAND = 0.5  # m/s: trigger brake when speed > limit+deadband; release when speed < limit-deadband
            speed_limit_applied_this_step = False
            was_limit_active = getattr(self, '_speed_limit_active', False)
            if current_speed > current_speed_limit_ms + SPEED_LIMIT_DEADBAND:
                self._speed_limit_active = True
            if current_speed < current_speed_limit_ms - SPEED_LIMIT_DEADBAND:
                self._speed_limit_active = False
            # Pit mode: when over limit we only coast (no brake); throttle is zeroed in pit block when speed >= 35 mph
            apply_pit_limit = pit_mode and current_speed > PIT_MAX_SPEED_MS
            apply_main_limit = (not pit_mode) and self._speed_limit_active and current_speed > current_speed_limit_ms - SPEED_LIMIT_DEADBAND
            if apply_main_limit:
                speed_limit_applied_this_step = True
                speed_excess = current_speed - current_speed_limit_ms
                if speed_excess > 2.0:
                    final_throttle = 0.0
                    final_brake = max(final_brake, 0.5)
                elif speed_excess > 1.0:
                    final_throttle = 0.0
                    final_brake = max(final_brake, 0.3)
                else:
                    final_throttle = max(0.0, final_throttle * 0.5)
                    final_brake = max(final_brake, 0.1)
                step_for_log = getattr(self, '_behavior_step_count', 0) + 1
                print(f"[Speed Limit] step={step_for_log} Speed {current_speed:.2f}m/s exceeds limit {current_speed_limit_ms:.1f}m/s (main), applying brake={final_brake:.3f}")
            elif apply_pit_limit:
                # Pit: over 35 mph — coast only (no brake); throttle is zeroed below when speed >= PIT_MAX_SPEED_MS
                speed_limit_applied_this_step = False
                step_for_log = getattr(self, '_behavior_step_count', 0) + 1
                print(f"[Speed Limit] step={step_for_log} Speed {current_speed:.2f}m/s exceeds limit {current_speed_limit_ms:.1f}m/s (pit 35mph, coast only)")

            # Phase 9 structural hazard authority:
            # If tactical planner is in explicit safety-follow reasons and assessment still
            # reports unsafe overlap/closing/gap pressure, force longitudinal suppression.
            _eff_state_now = str(getattr(self, "_phase_effective_planner_state", "") or "")
            _eff_reason_now = str(getattr(self, "_phase_effective_reason", "") or "")
            _hz_gap_bad = not bool(getattr(self, "_assessment_gap_ok", True))
            _hz_overlap = bool(getattr(self, "_assessment_overlap_flag", False))
            _hz_closing = bool(getattr(self, "_assessment_closing_flag", False))
            _hz_risk = float(getattr(self, "_assessment_emergency_risk_01", 0.0) or 0.0)
            _hz_reason_gate = _eff_reason_now in (
                "protected_follow_envelope",
                "contact_recovery_hold",
                "proximity_hazard_follow",
                "gap_not_ok_follow",
            )
            # SD-4d: gate hazard brake floor on predicted-path-collision when
            # available. Snapshot heuristics (overlap, gap_bad, closing, risk)
            # remain as the fast-fail filter; predicted_collision is the AUTHORITY.
            # Falls back to today's snapshot logic when polylines weren't threaded
            # (test mode / legacy callers).
            _hz_predicted_collision = bool(getattr(self, "_predicted_collision", False))
            _hz_predicted_collision_available = bool(getattr(self, "_predicted_collision_available", False))
            _hz_snapshot = (
                (not pit_mode)
                and _tactical_planner_enabled
                and _assessment_enabled
                and (_eff_state_now == "FOLLOW")
                and _hz_reason_gate
                and (_hz_overlap or (_hz_gap_bad and (_hz_closing or _hz_risk >= 0.70)))
            )
            if _hz_predicted_collision_available:
                _hazard_active = bool(_hz_snapshot and _hz_predicted_collision)
            else:
                _hazard_active = bool(_hz_snapshot)
            _hazard_brake_floor = 0.0
            if _hazard_active:
                if _hz_overlap:
                    _hazard_brake_floor = 0.45
                elif _hz_risk >= 0.85:
                    _hazard_brake_floor = 0.35
                else:
                    _hazard_brake_floor = 0.25
                final_throttle = 0.0
                final_brake = max(float(final_brake), float(_hazard_brake_floor))
                print(
                    f"{_fl_mpc} [Hazard] t={_sim_time_s:.2f}s active=1 reason={_eff_reason_now} "
                    f"overlap={1 if _hz_overlap else 0} closing={1 if _hz_closing else 0} "
                    f"gap_ok={1 if (not _hz_gap_bad) else 0} risk_01={_hz_risk:.3f} "
                    f"brake_floor={_hazard_brake_floor:.2f}"
                )
            self._hazard_brake_floor = float(_hazard_brake_floor)
    
            # Global mutual exclusion: never command throttle and brake at the same time.
            # When slowing (e.g. for a turn), lift throttle and brake only—no simultaneous throttle+brake.
            BRAKE_THROTTLE_EXCLUSION_THRESHOLD = 0.05  # treat as "active" above this
            if final_brake > BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                final_throttle = 0.0
            elif final_throttle > BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                final_brake = 0.0
    
            # Throttle ramp after brake release: avoid jumping from full brake to full throttle (generic, any TTL)
            THROTTLE_RAMP_STEPS = 12   # ~0.6 s at 0.05 s/step
            THROTTLE_RAMP_START = 0.2  # throttle cap at first step after brake
            was_braking = getattr(self, '_last_brake_heavy', False)
            self._last_brake_heavy = (final_brake > 0.2)
            ramp_remaining = getattr(self, '_throttle_ramp_steps_remaining', 0)
            if was_braking and final_brake < BRAKE_THROTTLE_EXCLUSION_THRESHOLD:
                ramp_remaining = THROTTLE_RAMP_STEPS
            if ramp_remaining > 0:
                # Linear ramp: cap throttle so it increases smoothly over THROTTLE_RAMP_STEPS
                step_in_ramp = THROTTLE_RAMP_STEPS - ramp_remaining
                throttle_cap = THROTTLE_RAMP_START + (1.0 - THROTTLE_RAMP_START) * (step_in_ramp / THROTTLE_RAMP_STEPS)
                final_throttle = min(final_throttle, throttle_cap)
                self._throttle_ramp_steps_remaining = ramp_remaining - 1
            else:
                self._throttle_ramp_steps_remaining = 0
            # Pit mode: cap throttle at 50%; when speed >= 35 mph use zero throttle to coast (don't push past limit)
            PIT_MAX_THROTTLE = 0.5
            if pit_mode:
                if current_speed >= PIT_COAST_MIN_TARGET_MS:
                    final_throttle = 0.0
                else:
                    final_throttle = min(final_throttle, PIT_MAX_THROTTLE)
                # Debug: log pit mode every 50 steps (speed, throttle, target 35 mph)
                _step_num = getattr(self, '_behavior_step_count', 0) + 1
                if _step_num % 50 == 0 or _step_num == 1:
                    _t_log = _step_num * (getattr(simulation(), 'control_dt', None) or getattr(simulation(), 'timestep', 0.05))
                    _reason = "coast (speed>=35mph)" if current_speed >= PIT_COAST_MIN_TARGET_MS else f"cap {PIT_MAX_THROTTLE}"
                    print(f"[Pit] step={_step_num} t={_t_log:.2f}s pit_mode=True speed={current_speed:.2f}m/s target=35mph throttle={final_throttle:.2f} ({_reason})")
            if _stability_guard_enabled and self is getattr(simulation().scene, 'egoObject', None):
                _p10_state = getattr(self, "_guard_state", None)
                if _p10_state is None:
                    _p10_state = StabilityGuardState(
                        last_ttl=str(_scripted_active_ttl or "optimal")
                    )
                    self._guard_state = _p10_state
                _p10_guard = stability_guard_step(
                    _p10_state,
                    config=_guard_config,
                    sim_time_s=float(_sim_time_s),
                    control_dt_s=float(_ctrl_dt),
                    planner_state=str(getattr(self, "_phase_effective_planner_state", "FREE_RUN") or "FREE_RUN"),
                    active_ttl=str(getattr(self, "_phase_effective_ttl", _scripted_active_ttl) or _scripted_active_ttl),
                    decision_reason=str(getattr(self, "_phase_effective_reason", "none") or "none"),
                    steer_cmd_rad=float(final_steer),
                    throttle_cmd=float(final_throttle),
                    brake_cmd=float(final_brake),
                    pit_mode=bool(pit_mode),
                    gap_ok=bool(getattr(self, "_assessment_gap_ok", True)),
                    overlap_flag=bool(getattr(self, "_assessment_overlap_flag", False)),
                    closing_flag=bool(getattr(self, "_assessment_closing_flag", False)),
                    emergency_risk_01=float(getattr(self, "_assessment_emergency_risk_01", 0.0) or 0.0),
                    ttl_switch_blocked=bool(_p10_ttl_switch_blocked),
                    # SD-4b: thread the predicted-collision result from the planner.
                    # Used by SD-4d to gate EMERGENCY_STABLE entry on predicted-path-collision
                    # (defaults to False here so it has no effect until SD-4d rewires).
                    predicted_collision=bool(getattr(self, "_predicted_collision", False)),
                    predicted_collision_available=bool(getattr(self, "_predicted_collision_available", False)),
                )
                final_steer = float(_p10_guard.steer_cmd_rad)
                # SD-36: panic-brake authority during EMERGENCY_STABLE. The guard
                # outputs a brake floor (0.30/0.45/0.60 depending on overlap +
                # closing flags) when predicted_collision fires. Without this
                # explicit override, downstream brake_cap re-clips and slew
                # limits earlier in this tick cap the brake at 0.25 (MAX_BRAKE_NORMAL).
                # Forcing throttle to 0 and using the guard's brake floor as a
                # *hard floor* (max with whatever else was set) ensures the
                # safety layer's authority is real, not just advisory.
                if bool(_p10_guard.emergency_stable_mode):
                    final_throttle = 0.0
                    final_brake = max(float(_p10_guard.brake_cmd), float(final_brake))
                else:
                    final_throttle = float(_p10_guard.throttle_cmd)
                    final_brake = float(_p10_guard.brake_cmd)
                # SD-38: defense-in-depth mutual exclusion AFTER the guard.
                # The pre-guard mutual exclusion (line 2387-2390) only catches
                # conflicts that exist before the guard runs. The guard itself
                # can re-introduce both-active commands (e.g. reapproach_hold's
                # cap-throttle + floor-brake combo). This final check ensures
                # the controller NEVER emits both throttle and brake. Brake
                # wins because if anything in the chain decided "brake",
                # there's a safety reason for it -- shedding throttle is
                # always safer than the alternative.
                if final_brake > 0.05 and final_throttle > 0.05:
                    final_throttle = 0.0
                # SD-38: idle-creep brake floor at near-stop speeds. A real
                # car with engaged gear creeps forward due to idle TORQUE
                # (not constant speed); the actual effect depends on road
                # grade -- uphill the engine drag dominates and the car
                # decelerates without brake; flat ground gives slow creep;
                # downhill gravity + idle torque both push forward and
                # demand stronger brake to hold. The dSPACE plant model has
                # similar behavior.
                #
                # GRADE LIMITATION: this 0.20 floor is a flat-road
                # approximation. On the actual Laguna Seca-style track with
                # significant elevation changes (the Corkscrew has ~6% grade),
                # this is too aggressive uphill (wastes brake) and too gentle
                # downhill (may not hold against gravity). When the controller
                # is exercised on grade, this should become grade-aware:
                #   floor = 0.20 + max(0, -grade_pct * 0.05)  (more brake downhill)
                # Read grade from ttl waypoints' z-values vs ego s-position.
                # For now (mostly-flat F-bank scenarios), 0.20 works.
                if bool(_p10_guard.guard_active) and current_speed < 2.0:
                    final_brake = max(float(final_brake), 0.20)
                    final_throttle = 0.0
                self._phase_effective_planner_state = str(_p10_guard.planner_state or getattr(self, "_phase_effective_planner_state", "FREE_RUN"))
                self._phase_effective_ttl = str(_p10_guard.active_ttl or getattr(self, "_phase_effective_ttl", _scripted_active_ttl))
                self._phase_effective_reason = str(_p10_guard.decision_reason or getattr(self, "_phase_effective_reason", "none"))
                self._guard_active = bool(_p10_guard.guard_active)
                self._guard_reason = str(_p10_guard.guard_reason or "none")
                self._guard_steer_limited = bool(_p10_guard.steer_limited)
                self._guard_brake_limited = bool(_p10_guard.brake_limited)
                self._guard_ttl_switch_blocked = bool(_p10_guard.ttl_switch_blocked)
                self._guard_emergency_stable_mode = bool(_p10_guard.emergency_stable_mode)
                print(format_stability_guard_log_line(_sim_time_s, _p10_guard))
                _record_event('Guard', {
                    't': float(_sim_time_s),
                    'guard_active': bool(_p10_guard.guard_active),
                    'guard_reason': str(_p10_guard.guard_reason or "none"),
                    'steer_limited': bool(_p10_guard.steer_limited),
                    'brake_limited': bool(_p10_guard.brake_limited),
                    'ttl_switch_blocked': bool(_p10_guard.ttl_switch_blocked),
                    'emergency_stable_mode': bool(_p10_guard.emergency_stable_mode),
                })
            # RC-1: consolidated controller-trace per tick. Read-only telemetry; safe to remove.
            # Captures POST-stability-guard (truly final) commands plus the upstream values
            # the executor used. Some fields are populated by code paths that don't always
            # run (e.g. _assessment_*); read defensively via getattr so we never crash the
            # control loop with telemetry. CTE and curvature_ahead read from self attrs
            # (locals can be stale 0 if the speed-gate try block was skipped).
            try:
                _ct_locs = locals()
                _ct_cte = float(getattr(self, '_last_waypoint_cte_for_speed', 0.0) or 0.0)
                _ct_k = float(getattr(self, '_last_curvature_ahead_for_tactical', 0.0) or 0.0)
                print(
                    f"[CtrlTrace] t={_sim_time_s:.2f}s "
                    f"v={current_speed:.2f} cte={_ct_cte:.2f} "
                    f"k_ahead={_ct_k:.4f} "
                    f"k_signed={float(getattr(self, '_curvature_ahead_max_signed', 0.0) or 0.0):.4f} "
                    f"brake_cap={float(_ct_locs.get('brake_cap', 0.0) or 0.0):.2f} "
                    f"brake_mpc={float(_ct_locs.get('brake_mpc', 0.0) or 0.0):.3f} "
                    f"cte_brake={float(_ct_locs.get('cte_brake', 0.0) or 0.0):.3f} "
                    f"final_brake={final_brake:.3f} final_throttle={final_throttle:.3f} final_steer={final_steer:+.3f} "
                    f"planner={str(getattr(self, '_phase_effective_planner_state', 'FREE_RUN') or 'FREE_RUN')} "
                    f"ttl={str(getattr(self, '_phase_effective_ttl', _scripted_active_ttl) or 'optimal')} "
                    f"ttl_blocked={int(bool(_p10_ttl_switch_blocked))} "
                    f"gap_ok={int(bool(getattr(self, '_assessment_gap_ok', True)))} "
                    f"overlap={int(bool(getattr(self, '_assessment_overlap_flag', False)))} "
                    f"risk={float(getattr(self, '_assessment_emergency_risk_01', 0.0) or 0.0):.3f} "
                    f"seg={getattr(self, '_last_valid_segment_id', None)}/"
                    f"{str(getattr(self, '_segment_type_at_wp', None) or 'na')} "
                    f"seg_ahead={getattr(self, '_segment_id_ahead', None)}/"
                    f"{str(getattr(self, '_segment_type_ahead', None) or 'na')} "
                    f"curve_hold={int(bool(getattr(self, '_curve_nearby_telemetry', False)))}"
                )
                # SD-10g: end-of-tick wall-clock measurement. Records:
                #   wall_t = process-relative wall seconds since first tick
                #   tick_ms = compute time spent on this control step
                # Use perf_counter() for monotonic high-res timing. The delta
                # is measured AT THE LAST emit so it captures the full tick
                # cost: planner + guard + ctrl + telemetry. If a future
                # optimization wants finer breakdown, add intermediate marks.
                _wall_tick_end = _wallclock_time.perf_counter()
                _wall_tick_ms = (_wall_tick_end - _wall_tick_start) * 1000.0
                print(
                    f"[TickTime] t={_sim_time_s:.2f}s "
                    f"wall_t={_wall_t_now_s:.3f}s "
                    f"tick_ms={_wall_tick_ms:.2f}"
                )
                _record_event('TickTime', {
                    't': float(_sim_time_s),
                    'wall_t_s': float(_wall_t_now_s),
                    'tick_ms': float(_wall_tick_ms),
                })
                # SD-10l: per-section breakdown for the same tick. The sum of
                # the section ms below is typically less than tick_ms — the
                # remainder ("other") is everything not individually wrapped:
                # waypoint indexing, segment-classification logic, log
                # formatting, control-action dispatch, IPC bridge sync.
                _sec_sum_ms = (_sec_segmap_ms + _sec_assess_opp_ms
                               + _sec_predict_ms + _sec_assess_race_ms
                               + _sec_planner_ms + _sec_lon_ms + _sec_lat_ms)
                _sec_other_ms = max(0.0, _wall_tick_ms - _sec_sum_ms)
                print(
                    f"[TickBreakdown] t={_sim_time_s:.2f}s "
                    f"segmap={_sec_segmap_ms:.2f} "
                    f"assess_opp={_sec_assess_opp_ms:.2f} "
                    f"predict={_sec_predict_ms:.2f} "
                    f"assess_race={_sec_assess_race_ms:.2f} "
                    f"planner={_sec_planner_ms:.2f} "
                    f"lon={_sec_lon_ms:.2f} "
                    f"lat={_sec_lat_ms:.2f} "
                    f"other={_sec_other_ms:.2f}"
                )
            except Exception as _ct_e:
                if not getattr(self, '_ctrltrace_warned', False):
                    print(f"[CtrlTrace] disabled (first error): {type(_ct_e).__name__}: {_ct_e}")
                    self._ctrltrace_warned = True
            self._last_final_steer = final_steer
            self._last_final_throttle = final_throttle
            self._last_final_brake = final_brake
            self._last_curvature_ahead_max = curvature_ahead_max

        # Store CTE for debugging
        self._current_cte = cte

        # ---- Detailed drive logging (heavy brake / near stop / speed drop) ----
        # Use step_for_log (not _step) to avoid shadowing the behavior's _step() method
        # Timestamp t = step * control_dt for systematic comparison across runs
        sim = simulation()
        ctrl_dt = getattr(sim, 'control_dt', None)
        if ctrl_dt is None or ctrl_dt <= 0:
            ctrl_dt = getattr(sim, 'control_period', None)
        if ctrl_dt is None or ctrl_dt <= 0:
            ctrl_dt = float(getattr(sim, 'timestep', 0.05))
        step_for_log = getattr(self, '_behavior_step_count', 0) + 1
        t_log = step_for_log * ctrl_dt
        _last_speed = getattr(self, '_last_speed', None)
        if final_brake > 0.25:
            cte_show = float(cte) if cte is not None else 0.0
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Heavy brake: speed={current_speed:.2f}m/s brake={final_brake:.3f} throttle={final_throttle:.3f} | speed_limit={speed_limit_applied_this_step} cte={cte_show:.2f}m brake_mpc={brake_mpc:.3f}")
        if current_speed is not None and current_speed < 6.0:
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Low speed: speed={current_speed:.2f}m/s brake={final_brake:.3f} throttle={final_throttle:.3f}")
        if _last_speed is not None and current_speed is not None and (current_speed - _last_speed) < -4.0:
            print(f"[Drive] t={t_log:.2f}s step={step_for_log} Speed drop: from {_last_speed:.2f} to {current_speed:.2f} m/s (delta={current_speed - _last_speed:.2f})")
        self._last_speed = float(current_speed) if current_speed is not None else 0.0
        
        # Build Action List
        actions_to_take = [
            SetSteerAction(final_steer),
            SetThrottleAction(final_throttle),
            SetBrakeAction(final_brake)
        ]

        # Gear Logic: proactive downshift before turns + speed-based shifts
        gear_changed = False
        new_gear = None
        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', 0) 
            
            if current_gear < 1:
                actions_to_take.append(SetGearAction(1))
                self.gear = 1
                gear_changed = True
                new_gear = 1
                print(f"  [Gear] Shifting from {current_gear} to 1 (starting from neutral)")
            
            elif current_speed is not None:
                # Proactive downshift before turns (curvature-ahead aware)
                curvature_very_tight = 0.08   # 1/m, tight turn -> prefer gear 1
                curvature_tight = 0.05        # 1/m, turn -> prefer one gear lower
                proactive_downshift = None
                if curvature_ahead_max >= curvature_very_tight and current_gear >= 2 and current_speed < 12.0:
                    proactive_downshift = 1  # 2->1 before very tight turn
                elif curvature_ahead_max >= curvature_tight and current_gear >= 3 and current_speed < 20.0:
                    proactive_downshift = current_gear - 1  # 3->2 (or 4->3, etc.) before turn
                if proactive_downshift is not None:
                    new_gear = proactive_downshift
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Proactive downshift from {current_gear} to {new_gear} (curvature_ahead={curvature_ahead_max:.3f} 1/m, speed={current_speed:.2f} m/s)")
                elif current_gear < 5 and current_speed > gear_up_thresholds[min(current_gear, 4)]:
                    new_gear = current_gear + 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting up from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")
                elif current_gear > 1 and current_speed < gear_down_thresholds[min(current_gear - 1, 4)]:
                    new_gear = current_gear - 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting down from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")

        # Debug Print
        if hasattr(self, '_behavior_step_count'):
            self._behavior_step_count += 1
        else:
            self._behavior_step_count = 0
        
        gear_val = getattr(self, 'gear', 0)
        self._last_final_gear = gear_val if gear_val >= 1 else 1
        # Per-step opponent/contact tracking (ego + fellow assumption): detect
        # overlap/near at control cadence, not only in the 50-step summary block.
        nearest_obj = None
        nearest_dist = None
        nearest_rel_speed = None
        nearest_rel_longitudinal = None
        try:
            _objs = getattr(simulation().scene, 'objects', [])
            _ego_h = car_heading if car_heading is not None else 0.0
            _ego_fx = math.cos(_ego_h)
            _ego_fy = math.sin(_ego_h)
            _ego_speed = float(current_speed if current_speed is not None else 0.0)
            _ego_race = getattr(self, "raceNumber", None)
            _best_d2 = None
            for _obj in _objs:
                if _obj is self:
                    continue
                if not hasattr(_obj, 'position') or _obj.position is None:
                    continue
                _obj_race = getattr(_obj, "raceNumber", None)
                # In benchmark scenes, treat race-numbered peer cars as valid opponents.
                # This avoids accidental nearest-object selection from non-vehicle objects.
                if _ego_race is not None:
                    if _obj_race is None:
                        continue
                    if _obj_race == _ego_race:
                        continue
                _ox = float(_obj.position.x)
                _oy = float(_obj.position.y)
                _dx = _ox - px
                _dy = _oy - py
                _d2 = _dx * _dx + _dy * _dy
                if _best_d2 is None or _d2 < _best_d2:
                    _best_d2 = _d2
                    nearest_obj = _obj
                    nearest_dist = _d2 ** 0.5
                    _ov = float(getattr(_obj, 'speed', 0.0) or 0.0)
                    nearest_rel_speed = _ov - _ego_speed
                    nearest_rel_longitudinal = _dx * _ego_fx + _dy * _ego_fy
        except Exception:
            nearest_obj = None
            nearest_dist = None
            nearest_rel_speed = None
            nearest_rel_longitudinal = None

        _p0_eval = getattr(simulation().scene, "params", None) or {}
        if _p0_eval.get("eval_gt_dist_log", True):
            _gt_d_step = read_eval_gt_dist_object_1_m(simulation())
            _oeps_step = float(_p0_eval.get("eval_obb_overlap_eps_m", EVAL_DEFAULT_OBB_OVERLAP_EPS_M))
            _hnear_step = float(_p0_eval.get("eval_hull_near_m", EVAL_DEFAULT_HULL_NEAR_M))
            _sclose_step = float(_p0_eval.get("eval_sensor_close_m", EVAL_DEFAULT_SENSOR_CLOSE_M))
            _obb_sep_step = None
            try:
                if nearest_obj is not None and car_heading is not None:
                    _oh_step = eval_heading_rad(nearest_obj)
                    if _oh_step is not None:
                        _eL_step, _eW_step = eval_vehicle_length_width_m(self)
                        _oL_step, _oW_step = eval_vehicle_length_width_m(nearest_obj)
                        _obb_sep_step = obb_separation_distance_m(
                            float(px),
                            float(py),
                            float(car_heading),
                            _eL_step,
                            _eW_step,
                            float(nearest_obj.position.x),
                            float(nearest_obj.position.y),
                            float(_oh_step),
                            _oL_step,
                            _oW_step,
                        )
            except Exception:
                _obb_sep_step = None
            _risk_step, _cflags_step = classify_eval_contact(
                _obb_sep_step,
                _gt_d_step,
                overlap_eps_m=_oeps_step,
                hull_near_m=_hnear_step,
                sensor_close_m=_sclose_step,
            )
            if _risk_step == "overlap" or _risk_step == "near":
                _os_step = f"{_obb_sep_step:.3f}" if _obb_sep_step is not None else "na"
                _ds_step = f"{float(_gt_d_step):.3f}" if eval_dspace_dist_object_1_valid(_gt_d_step) else "na"
                _opp_d_step = f"{nearest_dist:.3f}" if nearest_dist is not None else "na"
                _opp_rel_v_step = f"{nearest_rel_speed:.3f}" if nearest_rel_speed is not None else "na"
                _opp_rel_s_step = (
                    f"{nearest_rel_longitudinal:.3f}" if nearest_rel_longitudinal is not None else "na"
                )
                _ov_dbg = str(getattr(self, "_opponent_overlap_state", "unknown"))
                _seg_dbg = str(getattr(self, "_last_valid_segment_name", "") or "unknown")
                _ahead_dbg = (
                    1 if (nearest_rel_longitudinal is not None and nearest_rel_longitudinal > 0.0) else 0
                )
                # SD-37: collapsed EvalEvent + EvalEventDiag into a single
                # canonical contact-event line. The previous two-line format
                # emitted ~312 lines per F14 run with redundant t / severity /
                # bbox_gap_m / dspace_obj1_m fields. Folded EvalEventDiag's
                # unique fields (ego_speed_mps, opp_center_dist_m, rel_speed_mps,
                # rel_longitudinal_m, overlap_state, seg, ahead_hint) into
                # EvalEvent. Downstream parsers (metrics.py:_records_extract)
                # only consumed bbox_gap_m + type=eval_contact from EvalEvent,
                # so adding diagnostic fields is backward-compatible.
                print(
                    f"[EvalEvent] t={t_log:.2f}s type=eval_contact severity={_risk_step} "
                    f"bbox_gap_m={_os_step} dspace_obj1_m={_ds_step} "
                    f"dspace_valid={1 if _cflags_step.get('dspace_valid', False) else 0} "
                    f"ego_speed_mps={float(current_speed):.3f} opp_center_dist_m={_opp_d_step} "
                    f"rel_speed_mps={_opp_rel_v_step} rel_longitudinal_m={_opp_rel_s_step} "
                    f"overlap_state={_ov_dbg} seg={_seg_dbg} ahead_hint={_ahead_dbg}"
                )
                _record_event('EvalEvent', {
                    't': float(t_log),
                    'type': 'eval_contact',
                    'severity': str(_risk_step),
                    'bbox_gap_m': (float(_obb_sep_step) if _obb_sep_step is not None else None),
                    'dspace_obj1_m': (float(_gt_d_step) if eval_dspace_dist_object_1_valid(_gt_d_step) else None),
                    'dspace_valid': bool(_cflags_step.get('dspace_valid', False)),
                    'ego_speed_mps': float(current_speed),
                    'opp_center_dist_m': (float(nearest_dist) if nearest_dist is not None else None),
                    'rel_speed_mps': (float(nearest_rel_speed) if nearest_rel_speed is not None else None),
                    'rel_longitudinal_m': (float(nearest_rel_longitudinal) if nearest_rel_longitudinal is not None else None),
                    'overlap_state': _ov_dbg,
                    'seg': _seg_dbg,
                    'ahead_hint': int(_ahead_dbg),
                })
        # Log step summary every 50 steps; include t= and OpenDRIVE-based segment for segment performance analysis
        if self._behavior_step_count % 50 == 0:
            sim = simulation()
            ctrl_dt = getattr(sim, 'control_dt', None)
            if ctrl_dt is None or ctrl_dt <= 0:
                ctrl_dt = getattr(sim, 'control_period', None)
            if ctrl_dt is None or ctrl_dt <= 0:
                ctrl_dt = float(getattr(sim, 'timestep', 0.05))
            t_log = self._behavior_step_count * ctrl_dt
            _eid_log = getattr(self, '_last_valid_segment_id', None)
            _ename_log = getattr(self, '_last_valid_segment_name', "") or ""
            segment_str = f" {get_segment_label(_eid_log, _ename_log)}" if (_eid_log is not None or _ename_log) else " segment ?"
            print(f"{_fl_mpc} t={t_log:.2f}s Step {self._behavior_step_count}: pos=({px:.2f},{py:.2f}) speed={current_speed:.2f}m/s CTE={cte:.3f}m steer={final_steer:.3f} throttle={final_throttle:.3f} brake={final_brake:.3f} gear={gear_val} curv_ahead={curvature_ahead_max:.3f}{segment_str}")
            # Phase 0 telemetry (baseline visibility): active TTL, planner mode, ego s/speed,
            # nearest-opponent relative metrics, and event markers (switch/near-miss/collision/off-track).
            _ttl_label = None
            _ttl_file = getattr(self, 'ttlFileName', None)
            _ttl_sel = getattr(self, 'ttl_selection', None)
            if isinstance(_ttl_sel, str) and _ttl_sel:
                _ttl_label = _ttl_sel
            elif isinstance(_ttl_file, str) and _ttl_file:
                _tf = _ttl_file.lower()
                if "left" in _tf:
                    _ttl_label = "left"
                elif "right" in _tf:
                    _ttl_label = "right"
                elif "pit" in _tf:
                    _ttl_label = "pit"
                else:
                    _ttl_label = "optimal"
            else:
                _ttl_label = "unknown"
            _planner_mode = getattr(self, 'strategy_type', None) or "follow_mpc"
            # Use MPC controller progress estimate if available; keep None-safe for early ticks.
            _ego_s = getattr(_lat_controller, '_log_s_ref', None)

            # Detect TTL switch event.
            _last_ttl_label = getattr(self, '_baseline_last_ttl_label', None)
            if _last_ttl_label is None:
                self._baseline_last_ttl_label = _ttl_label
            elif _last_ttl_label != _ttl_label:
                print(f"[Phase0Event] t={t_log:.2f}s type=ttl_switch from={_last_ttl_label} to={_ttl_label}")
                self._baseline_last_ttl_label = _ttl_label

            _opp_dist_s = f"{nearest_dist:.2f}" if nearest_dist is not None else "na"
            _opp_rel_v_s = f"{nearest_rel_speed:.2f}" if nearest_rel_speed is not None else "na"
            _opp_rel_s_s = f"{nearest_rel_longitudinal:.2f}" if nearest_rel_longitudinal is not None else "na"
            _ego_s_s = f"{_ego_s:.2f}" if _ego_s is not None else "na"
            print(
                f"[Phase0] t={t_log:.2f}s ttl={_ttl_label} planner_mode={_planner_mode} "
                f"ego_s={_ego_s_s} ego_speed={current_speed:.2f} "
                f"nearest_opp_ds={_opp_rel_s_s} nearest_opp_rel_speed={_opp_rel_v_s} nearest_opp_dist={_opp_dist_s}"
            )
            # Evaluation-only: dSPACE Object_Sensor_3D + IAC OBB gap (not used for control).
            _p0 = getattr(simulation().scene, "params", None) or {}
            if _p0.get("eval_gt_dist_log", True):
                _gt_d = read_eval_gt_dist_object_1_m(sim)
                _gt_valid = eval_dspace_dist_object_1_valid(_gt_d)
                _obb_sep = None
                _center_minus_obb = None
                _obb_minus_gt = None
                _oeps = float(_p0.get("eval_obb_overlap_eps_m", EVAL_DEFAULT_OBB_OVERLAP_EPS_M))
                _hnear = float(_p0.get("eval_hull_near_m", EVAL_DEFAULT_HULL_NEAR_M))
                _sclose = float(_p0.get("eval_sensor_close_m", EVAL_DEFAULT_SENSOR_CLOSE_M))
                try:
                    if nearest_obj is not None and car_heading is not None:
                        _oh = eval_heading_rad(nearest_obj)
                        if _oh is not None:
                            _eL, _eW = eval_vehicle_length_width_m(self)
                            _oL, _oW = eval_vehicle_length_width_m(nearest_obj)
                            _obb_sep = obb_separation_distance_m(
                                float(px),
                                float(py),
                                float(car_heading),
                                _eL,
                                _eW,
                                float(nearest_obj.position.x),
                                float(nearest_obj.position.y),
                                float(_oh),
                                _oL,
                                _oW,
                            )
                            if nearest_dist is not None:
                                _center_minus_obb = float(nearest_dist) - float(_obb_sep)
                            if _gt_valid and _obb_sep is not None:
                                _obb_minus_gt = float(_obb_sep) - float(_gt_d)
                except Exception:
                    pass
                _risk, _cflags = classify_eval_contact(
                    _obb_sep,
                    _gt_d,
                    overlap_eps_m=_oeps,
                    hull_near_m=_hnear,
                    sensor_close_m=_sclose,
                )
                _os = f"{_obb_sep:.3f}" if _obb_sep is not None else "na"
                _cmo = f"{_center_minus_obb:.3f}" if _center_minus_obb is not None else "na"
                _omg = f"{_obb_minus_gt:.3f}" if _obb_minus_gt is not None else "na"
                _gtr = f"{_gt_d:.3f}" if _gt_d is not None else "na"
                _dcmg = "na"
                if nearest_dist is not None and _gt_valid:
                    _dcmg = f"{float(nearest_dist) - float(_gt_d):.3f}"
                elif nearest_dist is not None and not _gt_valid:
                    _dcmg = "na(invalid_gt)"
                _nds = f"{nearest_dist:.3f}" if nearest_dist is not None else "na"
                print(
                    f"[EvalGT] t={t_log:.2f}s dspace_obj1_raw_m={_gtr} dspace_valid={1 if _gt_valid else 0} "
                    f"bbox_gap_m={_os} nearest_opp_center_dist_m={_nds} "
                    f"center_minus_bbox_m={_cmo} center_minus_gt_m={_dcmg} bbox_minus_gt_m={_omg}"
                )
                _record_event('EvalGT', {
                    't': float(t_log),
                    'dspace_obj1_raw_m': (float(_gt_d) if _gt_d is not None else None),
                    'dspace_valid': bool(_gt_valid),
                    'bbox_gap_m': (float(_obb_sep) if _obb_sep is not None else None),
                    'nearest_opp_center_dist_m': (float(nearest_dist) if nearest_dist is not None else None),
                    'center_minus_bbox_m': (float(_center_minus_obb) if _center_minus_obb is not None else None),
                    'bbox_minus_gt_m': (float(_obb_minus_gt) if _obb_minus_gt is not None else None),
                })
                # SD-37: removed [EvalContact] emission. It was a sample-cadence
                # (~12 lines/run) duplicate of EvalEvent's overlap/near
                # severity, with redundant fields (bbox_gap_m, dspace_valid)
                # already in EvalEvent and EvalGT. The overlap_hull/sensor flags
                # weren't consumed by metrics.py; they were captured by
                # _record_event but never queried by _records_extract. Strict
                # dead code, removed.
            # Phase 2: race-semantics opponent state (planner inputs; same cadence as Phase 0 line).
            try:
                if nearest_obj is not None and car_heading is not None:
                    _ox = float(nearest_obj.position.x)
                    _oy = float(nearest_obj.position.y)
                    _ov = float(getattr(nearest_obj, "speed", 0.0) or 0.0)
                    _prog = getattr(self, "_waypoint_progress", None)
                    _smap_p2 = getattr(self, "_waypoint_segment_map", None)
                    _sid = getattr(self, "_last_valid_segment_id", None)
                    _snm = getattr(self, "_last_valid_segment_name", "") or ""
                    _Lap = None
                    if use_waypoints and wp_list is not None and len(wp_list) >= 2:
                        _Lap = polyline_lap_length_m(wp_list)
                    _prev_ov = getattr(self, "_opponent_overlap_state", "clear_ahead")
                    _sit_p2, _new_ov = assess_nearest_opponent(
                        (px, py),
                        float(car_heading),
                        float(current_speed),
                        (_ox, _oy),
                        _ov,
                        ego_progress_s_m=_prog,
                        waypoints=wp_list if use_waypoints else None,
                        lap_length_m=_Lap,
                        segment_map=_smap_p2,
                        ego_wp_idx=wp_last_idx,
                        segment_id=_sid,
                        segment_name=_snm,
                        curvature_ahead_max=float(curvature_ahead_max),
                        previous_overlap_state=_prev_ov,
                    )
                    self._opponent_overlap_state = _new_ov
                    print(format_opponent_log_line(t_log, _sit_p2))
                # SD-37: removed legacy [Phase2] opponent=none and assess_error
                # fallback emissions. The success path uses format_opponent_log_line
                # (which emits [Phase2] with full opponent state), and absence
                # of an opponent or an assessment error is already captured by
                # missing [Assessment] lines downstream — no separate diagnostic
                # tag needed for the failure-mode case.
            except Exception:
                pass
            if cte is not None and abs(float(cte)) >= 10.0:
                print(f"[Phase0Event] t={t_log:.2f}s type=off_track cte_m={float(cte):.3f}")
            # Ref continuity / gate logging (todo1: match_dist, gate ACCEPT/REJECT, s_ref/dS_ref/s_jump_flag, segment_prev->new, stick_blocked, e_y_mpc vs cte_behavior)
            _lc = _lat_controller
            _md = getattr(_lc, '_log_match_dist_m', None)
            _gs = getattr(_lc, '_log_gate_status', '?')
            _gr = getattr(_lc, '_log_gate_reason', None)
            _sr = getattr(_lc, '_log_s_ref', None)
            _ds = getattr(_lc, '_log_delta_s_ref', None)
            _sj = getattr(_lc, '_log_s_jump_flag', False)
            _sp = getattr(_lc, '_log_segment_prev', None)
            _sn = getattr(_lc, '_log_segment_new', None)
            _st = getattr(_lc, '_log_stick_blocked', False)
            _ey = getattr(_lc, '_log_mpc_e_y', None)
            _md_s = f"{_md:.3f}" if _md is not None else "?"
            _gr_s = f" reason={_gr}" if _gr else ""
            _sr_s = f"{_sr:.2f}" if _sr is not None else "?"
            _ds_s = f"{_ds:.3f}" if _ds is not None else "?"
            _seg_s = f"{_sp}->{_sn}" if _sp is not None else f"->{_sn}"
            _ey_s = f"{_ey:.3f}" if _ey is not None else "?"
            cte_b = float(cte) if cte is not None else 0.0
            print(f"{_fl_mpc} ref_log match_dist_m={_md_s} gate={_gs}{_gr_s} s_ref={_sr_s} dS_ref={_ds_s} s_jump_flag={1 if _sj else 0} seg={_seg_s} stick={1 if _st else 0} e_y_mpc={_ey_s} cte_behavior={cte_b:.3f}")
            # Task 4: Quick projection check — match_dist_m, proj_xy, ego_xy, segment_id progression (stuck = segment_id stops advancing or match_dist spikes at left-right transition)
            _rp = getattr(_lc, '_log_ref_point', None)
            _ego = getattr(_lc, '_log_ego', None)
            _proj_s = f"({_rp[0]:.3f},{_rp[1]:.3f})" if _rp is not None and len(_rp) >= 2 else "?"
            _ego_xy_s = f"({_ego[0]:.3f},{_ego[1]:.3f})" if _ego is not None and len(_ego) >= 2 else "?"
            print(f"{_fl_mpc} projection_check match_dist_m={_md_s} proj_xy={_proj_s} veh_xy={_ego_xy_s} segment_id={_seg_s}")
            # Task 1: projection continuity — s_ref, segment_id, proj_xy, match_dist; ensure s doesn't jump and projection doesn't hop
            _s_ok = 1 if getattr(_lc, '_log_s_ref_continuous', True) else 0
            _hop_ok = 1 if not getattr(_lc, '_log_proj_hop', False) else 0
            _cont_ok = 1 if getattr(_lc, '_log_projection_continuity_ok', True) else 0
            print(f"{_fl_mpc} projection_continuity s_ref={_sr_s} segment_id={_seg_s} proj_xy={_proj_s} match_dist_m={_md_s} s_ok={_s_ok} proj_hop_ok={_hop_ok} continuity_ok={_cont_ok}")
            _stuck_hint = False
            if _md is not None and _md > 5.0:
                _stuck_hint = True  # match_dist spike
            if _sp is not None and _sn is not None and _sp == _sn and _ds is not None and abs(_ds) < 0.01 and _md is not None and _md > 2.0:
                _stuck_hint = True  # segment not advancing with significant match_dist
            if _stuck_hint:
                print(f"{_fl_mpc} projection_check STUCK? (segment_id not advancing or match_dist spike)")
            # CTE cross-check log: cte_to_waypoints = e_y_mpc (single source); cte_behavior is same when MPC ran
            _rp = getattr(_lc, '_log_ref_point', None)
            _ego = getattr(_lc, '_log_ego', None)
            _cte_wp_s = _ey_s
            _rp_s = f"({_rp[0]:.3f},{_rp[1]:.3f})" if _rp is not None and len(_rp) >= 2 else "?"
            _ego_s = f"({_ego[0]:.3f},{_ego[1]:.3f})" if _ego is not None and len(_ego) >= 2 else "?"
            print(f"{_fl_mpc} ct_crosscheck cte_to_waypoints={_cte_wp_s} cte_behavior={cte_b:.3f} ref_point={_rp_s} vehicle={_ego_s}")
            if _ey is not None:
                self._last_waypoint_cte_for_speed = abs(_ey)
            # To-Do C: Polyline identity (behavior CTE, MPCC waypoints, segment map) — n_pts, first/last, total length, id
            _pts_b = wp_list if (use_waypoints and wp_list) else None
            if _pts_b is None or len(_pts_b) == 0:
                _pw = "behavior_cte_polyline: n=0"
            else:
                _n_b = len(_pts_b)
                _f_b = (float(_pts_b[0][0]), float(_pts_b[0][1])) if len(_pts_b[0]) >= 2 else (0, 0)
                _l_b = (float(_pts_b[-1][0]), float(_pts_b[-1][1])) if len(_pts_b[-1]) >= 2 else (0, 0)
                _len_b = 0.0
                for _i in range(len(_pts_b) - 1):
                    _a, _b = _pts_b[_i], _pts_b[_i + 1]
                    _len_b += ((float(_b[0]) - float(_a[0]))**2 + (float(_b[1]) - float(_a[1]))**2) ** 0.5
                _pw = f"behavior_cte_polyline: id={id(_pts_b)} n={_n_b} first=({_f_b[0]:.2f},{_f_b[1]:.2f}) last=({_l_b[0]:.2f},{_l_b[1]:.2f}) length={_len_b:.2f}m"
            _pts_m = waypoints_for_mpc if waypoints_for_mpc else None
            if _pts_m is None or len(_pts_m) == 0:
                _pm = "mpcc_waypoint_polyline: n=0"
            else:
                _n_m = len(_pts_m)
                _f_m = (float(_pts_m[0][0]), float(_pts_m[0][1])) if len(_pts_m[0]) >= 2 else (0, 0)
                _l_m = (float(_pts_m[-1][0]), float(_pts_m[-1][1])) if len(_pts_m[-1]) >= 2 else (0, 0)
                _len_m = 0.0
                for _i in range(len(_pts_m) - 1):
                    _a, _b = _pts_m[_i], _pts_m[_i + 1]
                    _len_m += ((float(_b[0]) - float(_a[0]))**2 + (float(_b[1]) - float(_a[1]))**2) ** 0.5
                _pm = f"mpcc_waypoint_polyline: id={id(_pts_m)} n={_n_m} first=({_f_m[0]:.2f},{_f_m[1]:.2f}) last=({_l_m[0]:.2f},{_l_m[1]:.2f}) length={_len_m:.2f}m"
            _smap = getattr(self, '_waypoint_segment_map', None)
            if _smap is not None and len(_smap) > 0:
                _seg_first = _smap[0]
                _seg_last = _smap[-1]
                _sm_s = f"segment_map: id={id(_smap)} n={len(_smap)} first_seg=({_seg_first[0]},{_seg_first[1]}) last_seg=({_seg_last[0]},{_seg_last[1]})"
            else:
                _sm_s = "segment_map: (none)"
            print(f"{_fl_mpc} polyline_check {_pw} | {_pm} | {_sm_s}")
            # ff_log is printed every MPC tick (see above) with segment_id, v, kappa_ref_at_proj, delta_ff, delta_fb, delta_total, steer_mpc_raw, steer_after_lpf

        # Supplement log (Todo2): deadzone decision, association, curvature, steering — every 10 ticks or when deadzone state changes
        _lc2 = _lat_controller
        _dz_app = getattr(_lc2, '_log_deadzone_applied', False)
        _last_dz = getattr(self, '_last_deadzone_applied', None)
        _dz_changed = (_last_dz is not None and _dz_app != _last_dz)
        self._last_deadzone_applied = _dz_app
        if (self._behavior_step_count % 10 == 0) or _dz_changed:
            _dz_m = getattr(_lc2, '_log_dz_cte_m', None)
            _cte_used = getattr(_lc2, '_log_cte_used_for_control', None)
            _cte_raw = getattr(_lc2, '_log_cte_raw', None)
            _reason = getattr(_lc2, '_log_deadzone_reason', '?')
            _match_d = getattr(_lc2, '_log_match_dist_m', None)
            _gate_ok = getattr(_lc2, '_log_gate_accept', None)
            _seg_id = getattr(_lc2, '_log_segment_id', None)
            _kappa = getattr(_lc2, '_log_kappa_ref_at_proj', None)
            _creg = getattr(_lc2, '_log_curv_regime', '?')
            _s_raw = getattr(_lc2, '_log_steer_mpc_raw', None)
            _s_caps = getattr(_lc2, '_log_steer_after_caps', None)
            _s_lpf = getattr(_lc2, '_log_steer_after_lpf', None)
            _s_rate = getattr(_lc2, '_log_steer_rate', None)
            _curv_ahd = curvature_ahead_max
            _dz_m_s = f"{_dz_m:.3f}" if _dz_m is not None else "?"
            _cte_used_s = f"{_cte_used:.3f}" if _cte_used is not None else "?"
            _cte_raw_s = f"{_cte_raw:.3f}" if _cte_raw is not None else "?"
            _match_s = f"{_match_d:.3f}" if _match_d is not None else "?"
            _kappa_s = f"{_kappa:.3f}" if _kappa is not None else "?"
            _curv_ahd_s = f"{_curv_ahd:.3f}" if _curv_ahd is not None else "?"
            _s_raw_s = f"{_s_raw:.3f}" if _s_raw is not None else "?"
            _s_caps_s = f"{_s_caps:.3f}" if _s_caps is not None else "?"
            _s_lpf_s = f"{_s_lpf:.3f}" if _s_lpf is not None else "?"
            _s_rate_s = f"{_s_rate:.3f}" if _s_rate is not None else "?"
            print(f"{_fl_mpc} deadzone_log deadzone_applied={_dz_app} dz_cte_m={_dz_m_s} cte_used_for_control={_cte_used_s} cte_raw={_cte_raw_s} deadzone_reason={_reason} match_dist_m={_match_s} gate_accept={_gate_ok} segment_id={_seg_id} curv_ahead_max={_curv_ahd_s} curv_regime={_creg} kappa_ref_at_proj={_kappa_s} steer_mpc_raw={_s_raw_s} steer_after_caps={_s_caps_s} steer_after_lpf={_s_lpf_s} steer_rate={_s_rate_s}")

        if _bt is not None:
            _bt.end_section('cmd_post')
        # Execute all actions together
        take actions_to_take

behavior PitStopBehavior(manage_gears=True):
    """Execute a pit stop using racing-specific systems.
    
    This behavior demonstrates the use of racing-specific actions like
    pit limiter and ERS deployment.
    """
    
    # Enter pit lane with speed limiter
    take PitLimiterAction(activate=True)
    do FollowRacingLineMPCBehavior(target_speed=20, manage_gears=manage_gears)
    
    # Stop for pit stop
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take PitLimiterAction(activate=False)

behavior OvertakingBehavior(target_car, aggressive=False):
    """Attempt to overtake target car using racing systems.
    
    This behavior uses DRS and ERS systems for overtaking maneuvers.
    
    Args:
        target_car: The car to overtake
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    # Close the gap
    while (distance from self to target_car) > 5:
        do FollowRacingLineMPCBehavior(target_speed=35)
    
    # Execute overtake with racing systems
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)
        take DRSAction(activate=True)
    
    # Move to side and accelerate
    take SetThrottleAction(1.0)
    
    # Complete overtake
    do FollowRacingLineMPCBehavior() until (distance from self to target_car) > 10
    
    # Return to racing line
    do FollowRacingLineMPCBehavior()

behavior DefensiveBehavior():
    """Defend position using racing-specific systems.
    
    This behavior uses traction control and brake bias adjustments
    for defensive driving.
    """
    
    # Adjust racing systems for defense
    take TractionControlAction(level=8)  # More conservative TC
    take BrakeBiasAction(bias=0.6)  # More front bias for stability
    
    # Follow racing line defensively
    do FollowRacingLineMPCBehavior(target_speed=25)


## Decision tree behaviors (for race decision engine integration)

behavior FlagBasedSpeedBehavior(speed_type="green", speed_limit=None, manage_gears=True):
    """Set speed based on flag type (decision tree behavior).
    
    This behavior sets the speed limit based on race flags (yellow, green, etc.)
    and applies it to the vehicle.
    
    Args:
        speed_type: Speed type string - "yellow", "double_yellow", "green", "round", etc.
        speed_limit: Speed limit in m/s (if None, uses default for speed_type)
    """
    
    # Set speed limit based on type
    if speed_limit is None:
        # Default speeds (can be overridden with params)
        speed_limits = {
            "pit_crawl": 10.0,
            "pit_lane": 20.0,
            "pit_road": 25.0,
            "yellow": 40.0,
            "double_yellow": 90.0,
            "green": 120.0,
            "round": 120.0,
            "stop": 0.0
        }
        speed_limit = speed_limits.get(speed_type, 120.0)
    
    take SetSpeedLimitAction(speed_limit=speed_limit, speed_type=speed_type)
    
    # Apply speed limit via FollowRacingLineMPCBehavior
    do FollowRacingLineMPCBehavior(target_speed=speed_limit, manage_gears=manage_gears)


behavior LaneSelectionBehavior(ttl_selection="race", manage_gears=True):
    """Select TTL based on attacker/defender flags (decision tree behavior).
    
    This behavior selects the appropriate TTL (left for defender, right for attacker,
    race for optimal) and sets the speed accordingly.
    
    Args:
        ttl_selection: TTL selection string - "left", "right", "race", "optimal", or "pit"
    """
    
    take SetTTLSelectionAction(selection=ttl_selection)
    
    # Set speed based on selection (green speed for racing, slower for pit)
    if ttl_selection == "pit":
        do FlagBasedSpeedBehavior(speed_type="pit_lane", speed_limit=20.0, manage_gears=manage_gears)
    else:
        do FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0, manage_gears=manage_gears)


behavior ARTStackControlBehavior():
    """Wrapper behavior: do not control the vehicle; let the ART stack control it.
    
    Use this when the ego is driven by the ART (Automated Racing Technology) stack
    (e.g. VKS, race decision engine) instead of Scenic controllers. The behavior
    runs every step but does not send throttle, brake, or steering commands, so
    Scenic does not overwrite ART's control outputs.
    
    Similar in spirit to other shell/wrapper behaviors (e.g. LaneSelectionBehavior
    that delegates to FlagBasedSpeedBehavior).
    """
    while True:
        wait


behavior StopBehavior(stop_type="safe"):
    """Stop car with specified stop type (decision tree behavior).
    
    This behavior implements emergency, immediate, or safe stop behavior.
    
    Args:
        stop_type: Stop type string - "emergency", "immediate", or "safe"
    """
    
    take StopCarAction(stop_type=stop_type)
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")


behavior FollowModeBehavior(target_car, target_gap=31.0, manage_gears=True, use_waypoints=True, lookahead=20.0):
    """Follow another car maintaining target gap (decision tree behavior).
    
    This behavior implements follow mode strategy where the car maintains
    a target gap distance to the car ahead.
    
    Args:
        target_car: The car to follow
        target_gap: Target gap distance in meters
    """
    
    take SetStrategyAction(strategy_type="follow_mode")
    take SetTargetGapAction(gap=target_gap, gap_type="attacker_preparing")
    
    # Get controllers
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    past_steer_angle = 0
    # SD-41I: see the matching block in FollowRacingLineMPCBehavior for the
    # plant-derived rationale. Keep the two lists in sync.
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0]
    
    # Waypoint state (nearest index), used only if waypoints are available
    wp_last_idx = 0
    
    while True:
        # Compute gap to target car
        current_gap = distance from self to target_car
        
        # Compute speed error based on gap
        gap_error = current_gap - target_gap
        
        # Adjust speed to maintain gap
        if gap_error > 5.0:  # Too far, speed up
            target_speed = (target_car.speed if target_car.speed is not None else 0) + 2.0
        elif gap_error < -5.0:  # Too close, slow down
            target_speed = (target_car.speed if target_car.speed is not None else 0) - 2.0
        else:
            target_speed = target_car.speed if target_car.speed is not None else 0
        
        # Clamp to max speed
        target_speed = min(target_speed, self.maxSpeed if hasattr(self, 'maxSpeed') else 120.0)
        
        # Get TTL to follow
        line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else (track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad))
        
        # Cross-track error (waypoint-targeted if available)
        cte = None
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        if use_waypoints and wp_list and len(wp_list) >= 2:
            px = float(self.position.x); py = float(self.position.y)
            nearest_idx = 0; best_d2 = 1e18
            for i in range(max(0, wp_last_idx - 25), min(len(wp_list), wp_last_idx + 26)):
                wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            wp_last_idx = nearest_idx
            Ld = float(lookahead)
            tgt_idx = nearest_idx; rem = Ld; j = nearest_idx
            n_wp_la = len(wp_list)
            steps = 0
            while rem > 0.0 and steps < n_wp_la:
                j_next = (j + 1) % n_wp_la
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j_next][0]), float(wp_list[j_next][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    j = j_next; steps += 1; continue
                if rem <= seg_len:
                    u = rem / seg_len
                    # projection for signed error
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
                    break
                else:
                    rem -= seg_len; j = j_next; tgt_idx = j; steps += 1
            if cte is None:
                # Closed loop: last segment index is len(wp_list)-1; next wp is (k0+1)%n_wp
                n_wp_fb = len(wp_list)
                k0 = max(0, min(n_wp_fb - 1, wp_last_idx))
                j1 = (k0 + 1) % n_wp_fb
                x0, y0 = float(wp_list[k0][0]), float(wp_list[k0][1])
                x1, y1 = float(wp_list[j1][0]), float(wp_list[j1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    cte = 0.0
                else:
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
        if cte is None:
            cte = line.signedDistanceTo(self.position)
        current_speed = (self.speed if self.speed is not None else 0)
        speed_error = target_speed - current_speed

        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', None)
            if current_gear is None or current_gear < 1:
                take SetGearAction(1)
                self.gear = 1
                current_gear = 1
            elif current_speed is not None:
                if current_gear < 5 and current_speed > gear_up_thresholds[current_gear]:
                    take SetGearAction(current_gear + 1)
                    self.gear = current_gear + 1
                    current_gear = self.gear
                elif current_gear > 1 and current_speed < gear_down_thresholds[current_gear - 1]:
                    take SetGearAction(current_gear - 1)
                    self.gear = current_gear - 1
                    current_gear = self.gear
        
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer


behavior PitLaneBehavior(manage_gears=True):
    """Handle pit lane speeds (decision tree behavior).
    
    This behavior implements pit lane speed limits: pit crawl (10 m/s),
    pit lane (20 m/s), and pit road (25 m/s).
    """
    
    # Determine which pit zone we're in (simplified - would need location detection)
    # For now, use pit_lane speed as default
    take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
    take SetTTLSelectionAction(selection="pit")
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")
    
    # Apply pit lane speed
    do FollowRacingLineMPCBehavior(target_speed=20.0, manage_gears=manage_gears)


behavior SimpleRaceBehavior(manage_gears=True, use_waypoints=True,
                           out_of_bounds_tolerance=5.0):
    """Simplified race decision tree behavior.
    
    Priority-based decision making:
    1. Emergency stop (if out of bounds)
    2. Pit lane behavior (if in pit lane)
    3. Green flag behavior (normal racing)
    
    Args:
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based steering for :obj:`FollowRacingLineMPCBehavior`
        out_of_bounds_tolerance: Distance tolerance for out-of-bounds check (meters)
    """
    
    while True:
        # ============================================================
        # PRIORITY 1: Emergency Stop Check (Out of Bounds)
        # ============================================================
        
        # Check if car is still within track bounds
        # Option 1: Check if position is in road region
        is_in_bounds = road.contains(self.position) if hasattr(road, 'contains') else True
        
        # Option 2: Check distance to road (more lenient)
        if not is_in_bounds:
            # Check if we're close enough to road (within tolerance)
            distance_to_road = road.distanceTo(self.position) if hasattr(road, 'distanceTo') else 0.0
            is_in_bounds = distance_to_road <= out_of_bounds_tolerance
        
        # Emergency stop if out of bounds
        if not is_in_bounds:
            take StopCarAction(stop_type="emergency")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            # Emergency stop - exit behavior
            break
        
        # ============================================================
        # PRIORITY 2: Pit Lane vs Green Flag
        # ============================================================
        
        # Check if we're in pit lane
        in_pit_lane = False
        if hasattr(track, 'pitLaneRoad') and track.pitLaneRoad:
            in_pit_lane = track.pitLaneRoad.contains(self.position)
        
        if in_pit_lane:
            # PIT LANE BEHAVIOR
            take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
            take SetTTLSelectionAction(selection="pit")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute pit lane behavior
            do FollowRacingLineMPCBehavior(target_speed=20.0, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints)
        else:
            # GREEN FLAG BEHAVIOR (Normal Racing)
            green_speed = 120.0  # Default green speed (m/s)
            take SetSpeedLimitAction(speed_limit=green_speed, speed_type="green")
            take SetTTLSelectionAction(selection="race")  # Use race TTL
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute green flag behavior
            do FollowRacingLineMPCBehavior(target_speed=green_speed, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints)
        
        wait  # Wait one timestep before re-evaluating