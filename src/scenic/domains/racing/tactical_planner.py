"""Tactical planner — overtake state machine + brake-trigger gating.

State machine (SD-3d expanded with HOLD):
    FREE_RUN ─→ FOLLOW ──→ SETUP_PASS_{L,R} ──→ COMMIT_PASS_{L,R}
                  ↑                                  │
                  │                                  ↓ (relation flips behind
                  │                                   AND still side-by-side)
                  │                            HOLD_PASS_{L,R}
                  │                                  │
                  └─── ABORT_PASS ←─────── any decisive maneuver ──→ FREE_RUN
                                          (predicted-collision interrupts)

Maps opponent situation + race assessment to (TTL choice, optional speed cap,
decision_reason). Called each control cycle from the racing behavior.

Brake-trigger gating (post-SD-4):
  All snapshot heuristics (overlap_flag, gap_ok, closing_flag, risk thresholds)
  go through ``_apply_predicted_collision_gate`` which combines them with the
  predicted-path-collision check (path_collision_predicted from pass_geometry.py).
  During decisive maneuvers (SETUP/COMMIT/HOLD), snapshot is suppressed entirely;
  only predicted_collision can interrupt. This delivers the user's "decisive
  overtake" requirement: pick a path, floor it, brake only on PREDICTED collision.

Δv-derived gates (SD-3b): SETUP/COMMIT entry gap thresholds and HOLD release
distance are linear functions of Δv = max(0.5, ego_speed - opp_speed), capped
to a sane band. High Δv → wider initiation gap (swerve out far); low Δv →
tighter (wait closer).

Geometric look-ahead (SD-3c): SETUP→COMMIT entry consults pass_window_check
to reject pass attempts whose pass-side TTL converges with fellow's path
within the predicted pass duration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

from scenic.domains.racing.assessment.pass_geometry import (
    pass_window_check,
    path_collision_predicted,
    select_tracks_for_state,
)
from scenic.domains.racing.situation_assessment import OpponentSituation

FREE_RUN = "FREE_RUN"
FOLLOW = "FOLLOW"
SETUP_LEFT = "SETUP_LEFT"
SETUP_RIGHT = "SETUP_RIGHT"
SETUP_PASS_LEFT = "SETUP_PASS_LEFT"
SETUP_PASS_RIGHT = "SETUP_PASS_RIGHT"
COMMIT_PASS_LEFT = "COMMIT_PASS_LEFT"
COMMIT_PASS_RIGHT = "COMMIT_PASS_RIGHT"
ABORT_PASS = "ABORT_PASS"
# SD-3d: post-pass hold states. Ego stays on the COMMIT-side TTL after
# relation flips behind, until merge-back is geometrically + longitudinally safe.
HOLD_PASS_LEFT = "HOLD_PASS_LEFT"
HOLD_PASS_RIGHT = "HOLD_PASS_RIGHT"


# SD-4c: ego is in an active overtake maneuver (lateral shift / pass / merge-hold)
# when in any of these states. During such a maneuver, snapshot-heuristic brake
# triggers (overlap_flag, gap_ok, closing_flag, risk thresholds) are SUPPRESSED —
# only path_collision_predicted has authority to interrupt. Rationale: once we
# decide to pass, hesitating in parallel is more dangerous than committing.
def _is_decisive_maneuver(planner_state: str) -> bool:
    return str(planner_state or "") in (
        SETUP_LEFT, SETUP_RIGHT,
        SETUP_PASS_LEFT, SETUP_PASS_RIGHT,
        COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT,
        HOLD_PASS_LEFT, HOLD_PASS_RIGHT,
    )


# SD-5: factor the SD-4c gate pattern repeated at 5 sites in tactical_planner_step_v1
# (proximity_hazard, release_hazard, overlap_hazard_now, hard_hazard_now,
# safety_pressure). Single source of truth for "snapshot heuristic + predicted
# collision agreement" — the user-facing semantic for SD-4 decisive-overtake.
def _apply_predicted_collision_gate(
    snapshot: bool,
    *,
    planner_state: str,
    predicted_collision: bool,
    predicted_collision_available: bool,
) -> bool:
    """Combine a snapshot heuristic with the predicted-path-collision check.

    Three regimes:
      1. predicted_collision_available=False → fall back to snapshot only
         (preserves backward-compat for tests / callers that don't thread polylines).
      2. _is_decisive_maneuver(planner_state)=True → snapshot SUPPRESSED;
         only predicted_collision can fire the trigger.
      3. Otherwise (FOLLOW / FREE_RUN / ABORT) → require BOTH snapshot AND
         predicted_collision. Snapshot is a fast-fail filter; predicted is authority.
    """
    if not bool(predicted_collision_available):
        return bool(snapshot)
    if _is_decisive_maneuver(planner_state):
        return bool(predicted_collision)
    return bool(snapshot and predicted_collision)


@dataclass
class TacticalPlannerConfig:
    """Tunable thresholds (meters, seconds, m/s)."""

    relevance_dist_m: float = 95.0
    blocked_longitudinal_m: float = 42.0
    blocked_distance_m: float = 62.0
    pass_safe_risk_max: float = 0.48
    pass_requires_straight: bool = True
    setup_flip_cooldown_s: float = 4.0
    follow_speed_margin_mps: float = 2.5
    follow_tight_gap_m: float = 10.0
    follow_tight_headway_s: float = 0.6
    blocked_headway_s: float = 1.8
    setup_min_headway_s: float = 0.6
    hard_ttc_s: float = 1.4
    # SD-2g: dropped from 3.0 to 0.3 to break the matched-speed FOLLOW deadlock.
    # The 3.0 was unsatisfiable from steady-state FOLLOW (cap=opp+follow_margin → MPC
    # brakes-for-distance once gap ≤ safe_gap → closing decays to ~0). The "realistic
    # overtake needs differential" intent is already enforced by SD-2e's COMMIT cap
    # (=opp+commit_speed_margin_mps=8) which GUARANTEES actual closing during the pass.
    # The pass_safe gate just needs a feasibility floor: closing >= 0.3 means ego is
    # not falling behind, so once SETUP raises the cap to opp+setup_speed_margin (4.5),
    # the pass becomes physically realizable.
    pass_min_relative_speed_mps: float = 0.3
    follow_tight_cap_scale: float = 0.92
    setup_min_distance_m: float = 9.0
    setup_partial_overlap_lateral_m: float = 1.8
    setup_reentry_cooldown_s: float = 0.5
    contact_recovery_hold_s: float = 1.0
    protected_follow_release_cycles: int = 2
    setup_commit_entry_cycles: int = 2
    setup_commit_hold_s: float = 1.2
    setup_commit_min_closing_mps: float = 0.3
    pass_intent_entry_cycles: int = 2
    pass_intent_hold_s: float = 1.6
    lateral_path_lock_hold_s: float = 1.6
    # Commit/abort lifecycle
    commit_abort_enabled: bool = False
    commit_entry_cycles: int = 2
    # SD-3f: extended from 1.2 to 2.5 s to match the look-ahead window. Empirical
    # math: at commit_speed_margin_mps=16 the pass needs ~2 s to complete a 15 m
    # gap. 1.2 s was hard-aborting before relation flipped, leaving ego stuck
    # alongside fellow indefinitely (the F2_tactical "run parallel" failure).
    commit_hold_s: float = 2.5
    abort_hold_s: float = 0.9
    abort_risk_01: float = 0.55
    abort_ttc_s: float = 0.4   # tight: only abort on near-imminent collision
    # SD-2d: while ego is on a commit-side TTL and the fellow is still laterally
    # within this radius, ABORT keeps the commit-side TTL instead of reverting to
    # optimal — so an abort decelerates rather than swerving across the fellow.
    abort_keep_ttl_lat_m: float = 3.0
    ahead_relax_free_run_enabled: bool = True
    ahead_relax_min_gap_m: float = 32.0
    ahead_relax_max_risk_01: float = 0.12
    stationary_overlap_relief_enabled: bool = True
    stationary_opp_speed_mps: float = 1.5
    stationary_overlap_relief_lateral_m: float = 2.0
    commit_success_time_s: float = 2.0
    commit_max_speed_mps: float = 999.0  # effectively disabled; use opposing_commit_cooldown instead
    opposing_commit_cooldown_s: float = 4.0
    # SD-3b: SETUP/COMMIT entry distances are now Δv-derived. Replaces the old
    # constants setup_max_longitudinal_m / commit_max_longitudinal_m with a linear
    # function of Δv = ego_speed - opp_speed, clamped to a sane band.
    #
    # Formula: gap_max(Δv) = clamp(floor, ceiling, slope * Δv + intercept)
    #
    # Anchors at observed F2_tactical successful pass (Δv ≈ 5 m/s):
    #   SETUP: 1.4*5 + 14  = 21 m  (was 28 — slightly tighter)
    #   COMMIT: 0.9*5 + 8  = 12.5 m (was 22 — much tighter, real "outbraking" range)
    # Anchors at high Δv (Δv=15: ego catching slow fellow on long straight):
    #   SETUP: 1.4*15 + 14 = 35 m  (swerve out far — gap closes fast)
    #   COMMIT: 0.9*15 + 8 = 21.5 m
    # Anchors at low Δv (Δv=2: matched-speed startup):
    #   SETUP: 1.4*2 + 14  = 16.8 m (wait closer)
    #   COMMIT: 0.9*2 + 8  = 9.8 m
    setup_gap_dv_slope: float = 1.4
    setup_gap_dv_intercept_m: float = 14.0
    setup_gap_dv_floor_m: float = 12.0
    setup_gap_dv_ceiling_m: float = 42.0
    commit_gap_dv_slope: float = 0.9
    commit_gap_dv_intercept_m: float = 8.0
    commit_gap_dv_floor_m: float = 8.0
    commit_gap_dv_ceiling_m: float = 30.0
    # SD-3b: HOLD release longitudinal-clearance formula (Stage 4 will use it).
    # hold_release_long_m(Δv) = vehicle_length + 1.5 + Δv * slope, where the
    # vehicle_length+buffer is folded into the intercept (4.88 + 1.5 = 6.4).
    # Δv=5: 6.4 + 1.5 = 7.9 m. Δv=15: 6.4 + 4.5 = 10.9 m (longer hold for fast pass).
    hold_release_long_dv_slope: float = 0.3
    hold_release_long_intercept_m: float = 6.4
    # SD-3b: HOLD safety floor — force release after this many seconds even if
    # geometric exit conditions never resolve. Should not normally fire.
    hold_max_s: float = 4.0
    # SD-3b: speed cap during HOLD. Cap = max(ego_speed_at_entry, opp_speed + this).
    hold_speed_floor_margin_mps: float = 1.5
    # SD-3b: if |sit.lateral_m| > this when COMMIT success fires, we are already
    # laterally clear — go straight to FREE_RUN, skip HOLD. Below this, enter HOLD.
    # SD-5 note: distinct from pass_geometry.DEFAULT_MIN_LAT_CLEARANCE_M=1.6m
    # (which is the side-by-side safety used by pass_window_check for the
    # OVERTAKE itself). merge_safe_lat_m is wider because the post-pass merge
    # is a transient lateral move where we want a larger safety pad before
    # claiming "no HOLD needed".
    merge_safe_lat_m: float = 2.5
    # SD-2e/3f: bounded speed margin during COMMIT. SD-2e set this to 8 m/s
    # to prevent the right-TTL convergence overshoot. With SD-3c/3f's look-ahead
    # now vouching for geometry (pass_window_check rejects converging sides
    # before commit fires), we can safely raise the margin so ego actually
    # COMPLETES the pass within commit_hold_s instead of running parallel.
    # Empirical math (F2): gap_at_commit ≈ 12-15m, commit_hold_s = 2.5s.
    # Required Δv to clear with 5m post-pass buffer: (12+5)/2.5 = 6.8 m/s minimum,
    # (15+5)/2.5 = 8 m/s. We use 16 m/s to give 2× headroom for slew-rate ramp-up.
    commit_speed_margin_mps: float = 16.0
    # SD-2f: SETUP needs enough closing speed to satisfy pass_min_relative_speed_mps=3.0
    # (otherwise SETUP holds forever, never reaching COMMIT). follow_speed_margin_mps=2.5
    # gives only 2.5 m/s closing — below the minimum. Use 4.5 m/s during SETUP so
    # closing-speed gate clears and the cycle progresses SETUP→COMMIT cleanly.
    setup_speed_margin_mps: float = 4.5
    # SD-2f: hard timeout — if SETUP holds this long without COMMIT firing, bail back
    # to FOLLOW on optimal. Prevents the "stuck on side TTL while approaching" failure
    # mode from F2_tactical first attempt (4 sec SETUP without COMMIT → contact).
    setup_max_hold_s: float = 2.5
    # SD-10a: configurable hysteresis thresholds. Pre-SD-10a these were hardcoded
    # `setup_candidate_count<3` and `follow_pressure_count>=3` literals scattered
    # in the planner body, which made them invisible to test overrides and config
    # tuning. Surfaced as proper config fields so they're discoverable and tunable.
    setup_entry_persistence_cycles: int = 3
    follow_pressure_threshold_cycles: int = 3
    # SD-6: commit_approach_risk_max=0.10 deleted. It was a closing+risk snapshot gate
    # that fired from the very first tick of SETUP (risk grows above 0.10 immediately
    # when ego closes on fellow), so COMMIT could never fire on F2-style overtakes.
    # Redundant defense — geometry is validated by SD-3c look-ahead (_commit_geom_ok)
    # and active brake authority is SD-4 predicted_collision. Removed per SD-6.
    # Segment-conditioned tactical intelligence
    segment_aware_enabled: bool = False
    corner_body_blocks_commit: bool = True
    corner_entry_commit_risk_max: float = 0.30


# SD-3b: Δv-derived gate helpers. All clamped to a sane band so a freak
# Δv (e.g. negative or huge) doesn't produce nonsensical gap thresholds.
def _dv_setup_gap_max_m(config: "TacticalPlannerConfig", dv_mps: float) -> float:
    raw = float(config.setup_gap_dv_slope) * float(dv_mps) + float(config.setup_gap_dv_intercept_m)
    return max(float(config.setup_gap_dv_floor_m),
               min(float(config.setup_gap_dv_ceiling_m), raw))


def _dv_commit_gap_max_m(config: "TacticalPlannerConfig", dv_mps: float) -> float:
    raw = float(config.commit_gap_dv_slope) * float(dv_mps) + float(config.commit_gap_dv_intercept_m)
    return max(float(config.commit_gap_dv_floor_m),
               min(float(config.commit_gap_dv_ceiling_m), raw))


def _dv_hold_release_long_m(config: "TacticalPlannerConfig", dv_mps: float) -> float:
    return float(config.hold_release_long_intercept_m) + float(config.hold_release_long_dv_slope) * max(0.0, float(dv_mps))


@dataclass
class CommitPlannerState:
    """State for the commit/abort pass lifecycle sub-machine."""
    side: str = ""                # current committed side ("left" / "right" / "")
    candidate_count: int = 0      # hysteresis counter for commit entry gate
    until_s: float = -1.0e9      # dwell timer: stay committed until this time
    start_s: float = -1.0e9      # timestamp when commit was entered (for success check)
    abort_until_s: float = -1.0e9  # hold timer for ABORT_PASS state
    last_side: str = ""           # side of the most recently completed commit (for cooldown)
    last_exit_s: float = -1.0e9   # sim time when last commit exited (for opposing cooldown)
    # SD-3d: HOLD phase state. Records when ego entered HOLD and ego's speed
    # at entry — used to compute the HOLD speed cap (freeze gain at entry,
    # don't accelerate to optimal target during the merge-back window).
    hold_entry_s: float = -1.0e9
    hold_speed_at_entry_mps: float = 0.0
    hold_pass_side: str = ""       # "left" or "right" — which side TTL HOLD is on
    # Per-cycle event flags (for logging; never drive control logic)
    trigger: str = "none"
    abort_trigger: str = "none"
    pass_success: bool = False
    abort_success: bool = False
    post_event_state: str = "none"


@dataclass
class TacticalPlannerState:
    mode: str = FREE_RUN
    last_setup_side: str = "left"
    last_flip_sim_time_s: float = -1.0e9
    last_setup_exit_sim_time_s: float = -1.0e9
    setup_candidate_side: str = ""
    setup_candidate_count: int = 0
    follow_pressure_count: int = 0
    protected_follow_active: bool = False
    protected_follow_clear_count: int = 0
    recovery_hold_until_s: float = -1.0e9
    setup_commit_side: str = ""
    setup_commit_candidate_count: int = 0
    setup_commit_until_s: float = -1.0e9
    pass_intent_side: str = ""
    pass_intent_candidate_count: int = 0
    pass_intent_until_s: float = -1.0e9
    lateral_path_lock_side: str = ""
    lateral_path_lock_until_s: float = -1.0e9
    commit: CommitPlannerState = field(default_factory=CommitPlannerState)
    segment_modifier: str = "normal"  # "blocked" / "conservative" / "relaxed" / "normal"
    # SD-2f: timestamp when SETUP was entered. Used for the setup_max_hold_s timeout
    # bail-out (back to FOLLOW on optimal) when SETUP doesn't reach COMMIT in time.
    setup_entry_s: float = -1.0e9
    # SD-4b: predicted-collision result from the most recent planner tick.
    # When predicted_collision_available is False, polylines weren't provided and
    # downstream brake-trigger gates fall back to today's snapshot logic.
    predicted_collision: bool = False
    predicted_collision_available: bool = False
    predicted_collision_min_clear_m: float = 0.0
    predicted_collision_closest_t_s: float = 0.0
    predicted_collision_breach_count: int = 0
    predicted_collision_ego_track: str = ""
    predicted_collision_opp_track: str = ""


def apply_ttl_key_to_agent(agent, ttl_key: str, ttl_cache: dict, file_by_sel: Dict[str, str]) -> bool:
    """Switch ego TTL polyline and invalidate segment/progress caches (Scenic agent)."""
    if ttl_key not in ttl_cache:
        return False
    region_new, pts_new = ttl_cache[ttl_key]
    agent.ttl = region_new
    agent.waypoints = list(pts_new)
    agent.ttl_selection = ttl_key
    agent.ttlFileName = file_by_sel.get(ttl_key, getattr(agent, "ttlFileName", None))
    agent._waypoint_segment_map = None
    agent._last_valid_segment_id = None
    agent._last_valid_segment_name = ""
    agent._cached_cumulative_dist_wp_idx = 0
    agent._cached_cumulative_dist_to_wp = 0.0
    agent._waypoint_progress = 0.0
    agent._waypoint_progress_idx = 0
    return True


def tactical_planner_step(
    state: TacticalPlannerState,
    sit: Optional[OpponentSituation],
    *,
    has_opponent: bool,
    ego_speed_mps: float,
    opponent_speed_mps: float,
    sim_time_s: float,
    pit_mode: bool,
    config: TacticalPlannerConfig,
) -> Tuple[str, str, Optional[float]]:
    """Backward-compatible API — return ``(mode, ttl_key, speed_cap_mps_or_None)``."""
    mode, ttl_key, cap, _reason = tactical_planner_step_v1(
        state,
        sit,
        has_opponent=has_opponent,
        ego_speed_mps=ego_speed_mps,
        opponent_speed_mps=opponent_speed_mps,
        sim_time_s=sim_time_s,
        pit_mode=pit_mode,
        config=config,
    )
    return mode, ttl_key, cap


def _canonical_mode(mode: str) -> str:
    if mode == SETUP_LEFT:
        return SETUP_PASS_LEFT
    if mode == SETUP_RIGHT:
        return SETUP_PASS_RIGHT
    return mode


def tactical_planner_step_v1(
    state: TacticalPlannerState,
    sit: Optional[OpponentSituation],
    *,
    has_opponent: bool,
    ego_speed_mps: float,
    opponent_speed_mps: float,
    sim_time_s: float,
    pit_mode: bool,
    config: TacticalPlannerConfig,
    assessment_relation: Optional[str] = None,
    assessment_gap_ok: Optional[bool] = None,
    assessment_optimal_open: Optional[bool] = None,
    assessment_left_open: Optional[bool] = None,
    assessment_right_open: Optional[bool] = None,
    assessment_closing_flag: Optional[bool] = None,
    assessment_emergency_risk_01: Optional[float] = None,
    # SD-3c: optional polyline/arc-length kwargs for the pass_window_check
    # geometric look-ahead. When all five are provided, SETUP entry rejects
    # any pass-side TTL whose geometry converges with the fellow's path
    # within the predicted pass duration. Default None preserves existing
    # tests (look-ahead skipped, prior preference logic stands).
    optimal_waypoints: Optional[Sequence] = None,
    side_waypoints_left: Optional[Sequence] = None,
    side_waypoints_right: Optional[Sequence] = None,
    ego_s_m: Optional[float] = None,
    opp_s_m: Optional[float] = None,
    lap_length_m: Optional[float] = None,
    # SD-7: ego's actual current physical TTL ("optimal"/"left"/"right"). The
    # planner state alone doesn't disambiguate (e.g. ABORT_PASS may be on
    # a side TTL post-COMMIT before reverting to optimal). Without this,
    # PathPredict walked the wrong polyline during ABORT_PASS → false
    # collision predictions → spurious EMERGENCY_STABLE → parallel braking.
    # Defaults to None for backward compat (falls back to state-derived).
    ego_active_ttl: Optional[str] = None,
) -> Tuple[str, str, Optional[float], str]:
    """Return ``(mode, ttl_key, speed_cap, decision_reason)``.

    Accepts assessment inputs (relation, gap safety, corridor openness, emergency risk)
    from the race situation assessment layer and maps them into a TTL choice.

    SD-3c: when polyline/s kwargs are provided, the SETUP-entry decision
    consults ``pass_window_check`` to reject geometrically-doomed passes
    before initiating the lateral shift.
    """
    def _follow_result(reason: str) -> Tuple[str, str, Optional[float], str]:
        state.mode = FOLLOW
        cap = float(opponent_speed_mps) + config.follow_speed_margin_mps
        if sit is not None:
            ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
            dynamic_tight_gap = max(
                float(config.follow_tight_gap_m),
                ref_speed * float(config.follow_tight_headway_s),
            )
        else:
            dynamic_tight_gap = float(config.follow_tight_gap_m)
        if sit is not None and sit.distance_m < dynamic_tight_gap:
            cap = min(
                cap,
                float(opponent_speed_mps) * config.follow_tight_cap_scale + 0.5,
            )
        cap = max(3.0, cap)
        return FOLLOW, "optimal", cap, reason

    def _setup_result(side: str, reason: str) -> Tuple[str, str, Optional[float], str]:
        state.last_setup_side = side
        # SD-2f: cap during SETUP must allow closing speed >= pass_min_relative_speed_mps,
        # otherwise pass_safe stays False forever and COMMIT never fires (observed
        # F2_tactical first attempt: SETUP cap=opp+2.5 → closing=2.5 < pass_min=3.0 →
        # pass_safe=False → 4 sec SETUP → contact). Use setup_speed_margin_mps which
        # defaults to 4.5 (1.5 m/s of headroom over the 3.0 minimum).
        cap: Optional[float] = None
        if relation_ahead and sit is not None:
            cap = max(3.0, float(opponent_speed_mps) + float(config.setup_speed_margin_mps))
        # SD-2f: stamp SETUP entry time for the timeout bail-out below.
        if state.mode not in (SETUP_LEFT, SETUP_RIGHT):
            state.setup_entry_s = float(sim_time_s)
        if side == "left":
            state.mode = SETUP_LEFT
            return SETUP_LEFT, "left", cap, reason
        state.mode = SETUP_RIGHT
        return SETUP_RIGHT, "right", cap, reason

    def _clear_safety_latches() -> None:
        state.protected_follow_active = False
        state.protected_follow_clear_count = 0
        state.recovery_hold_until_s = -1.0e9

    def _clear_setup_commit() -> None:
        state.setup_commit_side = ""
        state.setup_commit_candidate_count = 0
        state.setup_commit_until_s = -1.0e9

    def _clear_pass_intent() -> None:
        state.pass_intent_side = ""
        state.pass_intent_candidate_count = 0
        state.pass_intent_until_s = -1.0e9

    def _clear_lateral_lock() -> None:
        state.lateral_path_lock_side = ""
        state.lateral_path_lock_until_s = -1.0e9

    def _clear_commit_lifecycle() -> None:
        if state.commit.side in ("left", "right"):
            state.commit.last_side = state.commit.side
            state.commit.last_exit_s = float(sim_time_s)
        state.commit.side = ""
        state.commit.candidate_count = 0
        state.commit.until_s = -1.0e9
        state.commit.start_s = -1.0e9
        state.commit.abort_until_s = -1.0e9

    def _reset_commit_cycle_flags() -> None:
        state.commit.trigger = "none"
        state.commit.abort_trigger = "none"
        state.commit.pass_success = False
        state.commit.abort_success = False
        state.commit.post_event_state = "none"

    # SD-3b: speed differential drives SETUP/COMMIT/HOLD distance gates.
    # Floored at 0.5 so matched-speed FOLLOW (ego ≈ opp) still produces sane
    # gap thresholds via the formula intercept. Capped above only by physics.
    dv_mps = max(0.5, float(ego_speed_mps) - float(opponent_speed_mps))

    # SD-4b: compute predicted-collision once per tick, store on state.
    # Used by brake-trigger gates (SD-4c/4d) AND emitted as [PathPredict] log.
    # Falls back to "no collision" when polyline kwargs are absent (existing
    # tests that don't thread polylines preserve their snapshot-fallback path).
    pc_available = bool(
        optimal_waypoints is not None
        and ego_s_m is not None
        and opp_s_m is not None
        and lap_length_m is not None
        and float(lap_length_m) > 0.0
    )
    pc_collision = False
    pc_diag: Dict = {}
    # SD-10d: stationary-fellow PathPredict bypass.
    #
    # Root cause of F9 parking failure (full_stack_20260427_104218):
    # PathPredict walks `opp` on the OPTIMAL polyline at `opp_s + opp_speed*t`.
    # When opp is stationary AND laterally OFF the optimal line (e.g. F9 fellow
    # parked on roadside at lat=-5.5m, opp_s projection onto optimal stays at
    # a fixed point), the walker compares ego's predicted xy to that PROJECTED
    # opp position — NOT to the fellow's actual roadside xy. If the side TTL
    # ego walks happens to curve near the projected-opp point, PathPredict
    # reports collision (e.g. F9 t=5.85s: ego_track=left, min_clear=1.22m,
    # predicted_collision=1) even though the real fellow is 5.5m away.
    #
    # That false-positive cascaded: 292x predicted_collision=1 → 584x
    # EMERGENCY_STABLE → ego forced to v=0 → "parked next to stationary fellow".
    #
    # Fix: when opp is stationary AND laterally clear of the racing line,
    # there is no collision risk regardless of polyline geometry. Bypass
    # PathPredict and report no collision.
    if (
        pc_available
        and sit is not None
        and float(opponent_speed_mps) <= float(config.stationary_opp_speed_mps)
        and abs(float(sit.lateral_m)) > float(config.stationary_overlap_relief_lateral_m)
    ):
        pc_collision = False
        pc_diag = {
            "reason": "stationary_lateral_clear_bypass",
            "min_clear_m": abs(float(sit.lateral_m)),
            "closest_t_s": 0.0,
            "breach_count": 0,
        }
        # Flag track names so [PathPredict] log still emits a useful entry.
        state.predicted_collision_ego_track = "bypass"
        state.predicted_collision_opp_track = "bypass"
    elif pc_available:
        # SD-7: ego_active_ttl is the source of truth for which polyline ego
        # is physically on. Pre-SD-7 we derived it from state.mode, but
        # ABORT_PASS doesn't encode the side (SD-2d keeps the commit-side TTL
        # during abort while still side-by-side, so ego may be on right/left
        # for up to ~1s after ABORT triggered). Mis-deriving ego_track caused
        # PathPredict to walk the wrong polyline → spurious collision predictions
        # → EMERGENCY_STABLE → user-visible parallel braking on F2.
        if str(ego_active_ttl or "") in ("optimal", "left", "right"):
            _ego_ttl_resolved = str(ego_active_ttl)
        else:
            # Fallback for legacy callers that don't thread ego_active_ttl:
            # derive from state.mode (incomplete for ABORT_PASS but better than nothing).
            _ego_ttl_resolved = (
                "left" if state.mode in (SETUP_PASS_LEFT, COMMIT_PASS_LEFT, HOLD_PASS_LEFT, SETUP_LEFT)
                else "right" if state.mode in (SETUP_PASS_RIGHT, COMMIT_PASS_RIGHT, HOLD_PASS_RIGHT, SETUP_RIGHT)
                else ("left" if (state.mode == ABORT_PASS and state.commit.side == "left")
                      else ("right" if (state.mode == ABORT_PASS and state.commit.side == "right")
                            else "optimal"))
            )
        ego_track_name, opp_track_name = select_tracks_for_state(
            str(state.mode or "FREE_RUN"),
            _ego_ttl_resolved,
        )
        ego_track = optimal_waypoints if ego_track_name == "optimal" else (
            side_waypoints_left if ego_track_name == "left" else side_waypoints_right
        )
        opp_track = optimal_waypoints  # opp always on optimal by select_tracks_for_state
        if ego_track is not None and opp_track is not None:
            pc_collision, pc_diag = path_collision_predicted(
                ego_track=ego_track,
                opp_track=opp_track,
                ego_s_m=float(ego_s_m),
                ego_speed_mps=float(ego_speed_mps),
                opp_s_m=float(opp_s_m),
                opp_speed_mps=float(opponent_speed_mps),
                lap_length_m=float(lap_length_m),
            )
            state.predicted_collision_ego_track = ego_track_name
            state.predicted_collision_opp_track = opp_track_name
        else:
            pc_available = False
    state.predicted_collision = bool(pc_collision)
    state.predicted_collision_available = bool(pc_available)
    state.predicted_collision_min_clear_m = float(pc_diag.get("min_clear_m", 0.0))
    state.predicted_collision_closest_t_s = float(pc_diag.get("closest_t_s", 0.0))
    state.predicted_collision_breach_count = int(pc_diag.get("breach_count", 0))

    def _commit_result(side: str, reason: str) -> Tuple[str, str, Optional[float], str]:
        # SD-2e: cap speed during COMMIT so ego maintains a controlled differential
        # (~8 m/s closing) instead of charging at full target_speed. Removes the
        # over-acceleration that caused F2_tactical's right-TTL convergence overshoot.
        # The cap only applies while the fellow is still ahead; once relation flips
        # to behind, the COMMIT branch transitions to FREE_RUN (cap=None) at line 522.
        cap: Optional[float] = None
        if relation_ahead and sit is not None:
            cap = max(3.0, float(opponent_speed_mps) + float(config.commit_speed_margin_mps))
        # SD-2f: leaving SETUP — clear the entry timestamp so a future SETUP gets a
        # fresh timeout window.
        state.setup_entry_s = -1.0e9
        if side == "left":
            state.mode = COMMIT_PASS_LEFT
            return COMMIT_PASS_LEFT, "left", cap, reason
        state.mode = COMMIT_PASS_RIGHT
        return COMMIT_PASS_RIGHT, "right", cap, reason

    def _abort_result(reason: str) -> Tuple[str, str, Optional[float], str]:
        state.mode = ABORT_PASS
        # SD-2d: don't switch TTL during abort while ego is still side-by-side.
        # Reverting to optimal at gap=8m / lat=2m carries ego LATERALLY across the
        # fellow (observed F2_tactical t=7.35s: ttl_switch right→optimal caused
        # ego's left side to clip the fellow's right side at t=7.50s).
        # Keep the commit-side TTL until either relation flips (fellow behind) or
        # lateral overlap clears, so the abort becomes a deceleration rather than
        # a steering swerve into the obstacle.
        abort_ttl = "optimal"
        commit_side_now = str(state.commit.side or "")
        if (
            commit_side_now in ("left", "right")
            and relation_ahead
            and sit is not None
            and abs(float(sit.lateral_m)) < float(config.abort_keep_ttl_lat_m)
        ):
            abort_ttl = commit_side_now
        return ABORT_PASS, abort_ttl, None, reason

    _reset_commit_cycle_flags()

    def _is_proximity_hazard() -> bool:
        if sit is None:
            return False
        overlap_hazard = sit.overlap_state in ("partial_overlap", "side_by_side")
        # Distance-based hazard only applies when opponent is ahead; an opponent
        # behind ego is receding and does not constitute a forward proximity hazard.
        if not relation_ahead:
            return bool(overlap_hazard)
        ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
        dynamic_blocked_gap = max(
            float(config.follow_tight_gap_m),
            float(config.blocked_distance_m),
            ref_speed * float(config.blocked_headway_s),
        )
        close_hazard = sit.distance_m < dynamic_blocked_gap
        snapshot = bool(overlap_hazard or close_hazard)
        return _apply_predicted_collision_gate(
            snapshot,
            planner_state=state.mode,
            predicted_collision=state.predicted_collision,
            predicted_collision_available=state.predicted_collision_available,
        )

    def _is_release_hazard() -> bool:
        if sit is None:
            return False
        overlap_hazard = sit.overlap_state in ("partial_overlap", "side_by_side")
        # Opponent behind ego is their responsibility — no forward hazard.
        if not relation_ahead:
            return bool(overlap_hazard)
        ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
        dynamic_tight_gap = max(
            float(config.follow_tight_gap_m),
            ref_speed * float(config.follow_tight_headway_s),
        )
        # Suppress tight-gap hazard when there is an asymmetric opening (fellow on a
        # parallel TTL).  The car is not on a collision course with ego's path; blocking
        # pass success on proximity alone prevents the overtake from ever completing.
        tight_gap_hazard = sit.distance_m < dynamic_tight_gap and not asymmetric_opening
        snapshot = bool(overlap_hazard or tight_gap_hazard)
        return _apply_predicted_collision_gate(
            snapshot,
            planner_state=state.mode,
            predicted_collision=state.predicted_collision,
            predicted_collision_available=state.predicted_collision_available,
        )

    if pit_mode:
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        _clear_commit_lifecycle()
        return FREE_RUN, "optimal", None, "pit_mode_guard"

    if not has_opponent or sit is None:
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        _clear_commit_lifecycle()
        return FREE_RUN, "optimal", None, "no_opponent"

    if sit.distance_m > config.relevance_dist_m:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        _clear_commit_lifecycle()
        return FREE_RUN, "optimal", None, "opponent_far_free_run"

    relation_ahead = bool(sit.ahead)
    if isinstance(assessment_relation, str):
        if assessment_relation == "ahead":
            relation_ahead = True
        elif assessment_relation == "behind":
            relation_ahead = False
    left_open_now = True if assessment_left_open is None else bool(assessment_left_open)
    right_open_now = True if assessment_right_open is None else bool(assessment_right_open)
    opening_any = bool(left_open_now or right_open_now)
    overlap_hazard_raw = sit.overlap_state in ("partial_overlap", "side_by_side")
    stationary_overlap_relief = bool(
        bool(config.stationary_overlap_relief_enabled)
        and overlap_hazard_raw
        and relation_ahead
        and opening_any
        and (not bool(assessment_closing_flag))
        and (opponent_speed_mps <= float(config.stationary_opp_speed_mps))
        and (
            abs(float(sit.lateral_m))
            >= float(config.stationary_overlap_relief_lateral_m)
        )
    )
    overlap_hazard_now_snapshot = bool(overlap_hazard_raw and (not stationary_overlap_relief))
    overlap_hazard_now = _apply_predicted_collision_gate(
        overlap_hazard_now_snapshot,
        planner_state=state.mode,
        predicted_collision=state.predicted_collision,
        predicted_collision_available=state.predicted_collision_available,
    )
    if overlap_hazard_now:
        state.recovery_hold_until_s = max(
            float(state.recovery_hold_until_s),
            float(sim_time_s) + float(config.contact_recovery_hold_s),
        )
        state.protected_follow_active = True
        state.protected_follow_clear_count = 0
    in_recovery_hold = float(sim_time_s) < float(state.recovery_hold_until_s)
    # emergency_risk_high is computed after asymmetric_opening (below)
    asymmetric_opening = bool(left_open_now) ^ bool(right_open_now)
    # During a parallel-TTL pass (asymmetric_opening), risk is already dampened by fly-by
    # geometry.  Use a higher threshold so fly-by-dampened TTC pressure (~0.5) does not
    # trigger the same gates designed for co-linear rear-end danger (~0.48).
    _risk_threshold = (
        0.75 if asymmetric_opening else float(config.pass_safe_risk_max)
    )
    emergency_risk_high = bool(
        assessment_emergency_risk_01 is not None
        and float(assessment_emergency_risk_01) >= _risk_threshold
    )
    ref_speed_mps = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
    dynamic_tight_gap_m = max(
        float(config.follow_tight_gap_m),
        ref_speed_mps * float(config.follow_tight_headway_s),
    )
    dynamic_blocked_gap_m = max(
        float(config.blocked_distance_m),
        ref_speed_mps * float(config.blocked_headway_s),
    )
    dynamic_setup_min_gap_m = max(
        float(config.setup_min_distance_m),
        ref_speed_mps * float(config.setup_min_headway_s),
    )
    closing_speed_pos_mps = max(0.0, float(sit.closing_speed_mps))
    ttc_s = (
        (float(sit.distance_m) / closing_speed_pos_mps)
        if closing_speed_pos_mps > 1.0e-6
        else 1.0e9
    )
    opening_window_available = bool(
        (left_open_now or right_open_now)
        and asymmetric_opening
        and (not emergency_risk_high)
        # When a clear asymmetric opening exists (fellow on a parallel TTL), the
        # gap_ok+closing_flag pair must not block the window — we are not on a
        # collision course with that car.
        and (not ((assessment_gap_ok is False) and bool(assessment_closing_flag) and not asymmetric_opening))
        and (not _is_release_hazard())
        and (not in_recovery_hold)
    )
    hard_hazard_now_snapshot = bool(
        in_recovery_hold
        or overlap_hazard_now
        # Proximity/TTC terms suppressed for parallel-TTL fellows (asymmetric_opening).
        # A fellow on a side TTL is not on ego's collision path; distance alone must not
        # trigger a hard hazard and disable lateral-lock during the pass.
        or (sit.distance_m < dynamic_tight_gap_m and not asymmetric_opening)
        or (ttc_s < float(config.hard_ttc_s) and not asymmetric_opening)
        or emergency_risk_high
        or ((assessment_gap_ok is False) and (not opening_window_available))
    )
    hard_hazard_now = _apply_predicted_collision_gate(
        hard_hazard_now_snapshot,
        planner_state=state.mode,
        predicted_collision=state.predicted_collision,
        predicted_collision_available=state.predicted_collision_available,
    )
    lateral_lock_side = str(state.lateral_path_lock_side or "")
    lateral_lock_active = bool(
        lateral_lock_side in ("left", "right")
        and float(sim_time_s) < float(state.lateral_path_lock_until_s)
        and (not hard_hazard_now)
    )

    safety_pressure_snapshot = bool(
        ((assessment_gap_ok is False) and (not opening_window_available))
        or (bool(assessment_closing_flag) and (not opening_window_available))
        or emergency_risk_high
        # Proximity and TTC terms suppressed when an asymmetric opening exists: the
        # fellow is on a parallel TTL and is not a collision threat on ego's path.
        # Without this suppression, dynamic_tight_gap (~18 m at 20 m/s) fires for any
        # close parallel-TTL car, re-latching protected_follow every SETUP cycle and
        # preventing ego from accelerating during the committed pass.
        or (sit.distance_m < dynamic_tight_gap_m and not asymmetric_opening)
        or (ttc_s < float(config.hard_ttc_s) and not asymmetric_opening)
    )
    safety_pressure = _apply_predicted_collision_gate(
        safety_pressure_snapshot,
        planner_state=state.mode,
        predicted_collision=state.predicted_collision,
        predicted_collision_available=state.predicted_collision_available,
    )
    commit_enabled = bool(config.commit_abort_enabled)
    hard_abort_hazard_snapshot = bool(
        in_recovery_hold
        or overlap_hazard_now
        or emergency_risk_high
        or (ttc_s < float(config.abort_ttc_s))
        or (
            assessment_emergency_risk_01 is not None
            and float(assessment_emergency_risk_01) >= float(config.abort_risk_01)
        )
    )
    # SD-8: gate hard_abort_hazard on predicted_collision (mirror SD-4c).
    # The TTC term in the snapshot fires on raw distance/closing without
    # considering lateral separation — so it triggered abort during F2's
    # COMMIT_PASS_RIGHT at t=7.00 (gap=5.5m, closing=15 m/s → ttc=0.36s
    # < abort_ttc_s=0.4) even though ego had 4m of lateral clearance on
    # the right TTL. PathPredict correctly reported no collision; the
    # snapshot fired anyway. Fixing it via the same gate as SD-4c.
    hard_abort_hazard = _apply_predicted_collision_gate(
        hard_abort_hazard_snapshot,
        planner_state=state.mode,
        predicted_collision=state.predicted_collision,
        predicted_collision_available=state.predicted_collision_available,
    )
    soft_abort_pressure = bool(
        (assessment_gap_ok is False)
        and bool(assessment_closing_flag)
        and (not opening_window_available)
    )

    # Segment-conditioned modifier — computed once, stored on state for logging.
    # "blocked"       — corner_body: no new protected-follow release or commit entry
    # "conservative"  — corner_entry: tighter collision-risk gate on commit entry
    # "relaxed"       — straight: no additional restrictions
    # "normal"        — corner_exit or no segment data: no additional restrictions
    _seg_ctx_raw = str(getattr(sit, "segment_context", "") or "") if sit is not None else ""
    if config.segment_aware_enabled:
        if _seg_ctx_raw == "corner_body":
            _seg_modifier = "blocked"
        elif _seg_ctx_raw == "corner_entry":
            _seg_modifier = "conservative"
        elif _seg_ctx_raw == "straight":
            _seg_modifier = "relaxed"
        else:
            _seg_modifier = "normal"
    else:
        _seg_modifier = "normal"
    state.segment_modifier = _seg_modifier

    if commit_enabled and state.mode in (COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT):
        # Clear safety latches at the start of every COMMIT cycle.  protected_follow_active
        # may have been set during SETUP by safety_pressure when the parallel-TTL fellow was
        # close.  If it carries into COMMIT, the hazard brake floor prevents ego from
        # accelerating past the fellow.  True hazards are handled by hard_abort_hazard.
        _clear_safety_latches()
        commit_side = "left" if state.mode == COMMIT_PASS_LEFT else "right"
        state.commit.side = commit_side
        # Track when we first entered commit so we can measure elapsed commit time.
        if float(state.commit.start_s) < 0.0:
            state.commit.start_s = float(sim_time_s)
        if float(state.commit.until_s) <= float(sim_time_s):
            state.commit.until_s = float(sim_time_s) + float(config.commit_hold_s)
        commit_hold_active = bool(float(sim_time_s) < float(state.commit.until_s))
        if hard_abort_hazard or (soft_abort_pressure and (not commit_hold_active)):
            state.commit.start_s = -1.0e9
            state.commit.abort_until_s = max(
                float(state.commit.abort_until_s),
                float(sim_time_s) + float(config.abort_hold_s),
            )
            state.commit.abort_trigger = "commit_invalidated_hazard"
            state.commit.post_event_state = ABORT_PASS
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _abort_result("abort_commit_invalidated")
        # Classic success: ego has physically overtaken the fellow.
        # SD-3d: instead of immediately switching to optimal TTL (which carries
        # ego LATERALLY across the fellow at the moment of merge — the bug we
        # observed in F2_tactical), enter HOLD on the same side TTL when ego is
        # still in lateral-merge danger. HOLD waits until both longitudinal and
        # geometric merge-back checks pass before releasing to FREE_RUN.
        if (not relation_ahead) and (not _is_release_hazard()) and (not in_recovery_hold):
            state.commit.pass_success = True
            still_side_by_side = bool(
                sit is not None
                and abs(float(sit.lateral_m)) < float(config.merge_safe_lat_m)
            )
            if still_side_by_side:
                # Enter HOLD on the same side TTL.
                state.commit.hold_entry_s = float(sim_time_s)
                state.commit.hold_speed_at_entry_mps = float(ego_speed_mps)
                state.commit.hold_pass_side = commit_side
                state.commit.post_event_state = (
                    HOLD_PASS_LEFT if commit_side == "left" else HOLD_PASS_RIGHT
                )
                state.mode = (
                    HOLD_PASS_LEFT if commit_side == "left" else HOLD_PASS_RIGHT
                )
                # Don't clear commit lifecycle yet — HOLD reuses commit.last_side.
                _clear_setup_commit()
                _clear_pass_intent()
                _clear_lateral_lock()
                hold_cap = max(
                    float(state.commit.hold_speed_at_entry_mps),
                    float(opponent_speed_mps) + float(config.hold_speed_floor_margin_mps),
                )
                return state.mode, commit_side, hold_cap, "hold_pass_entry"
            # Already laterally clear (e.g. F6/F7 parallel-TTL passes) — go straight to FREE_RUN.
            state.commit.post_event_state = FREE_RUN
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            return FREE_RUN, "optimal", None, "pass_success_free_run"
        # TTL-clear success: ego has been stably committed to the open passing TTL
        # long enough that the manoeuvre is complete. Covers parallel-TTL scenarios
        # (F6/F7) where the fellow cruises at comparable speed and never physically
        # drops behind ego within the benchmark window.
        passing_side_open = bool(left_open_now if commit_side == "left" else right_open_now)
        commit_elapsed_s = float(sim_time_s) - float(state.commit.start_s)
        if (
            passing_side_open
            and not hard_abort_hazard
            and not overlap_hazard_now
            and not relation_ahead
            and commit_elapsed_s >= float(config.commit_success_time_s)
        ):
            state.commit.pass_success = True
            state.commit.post_event_state = FREE_RUN
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            return FREE_RUN, "optimal", None, "pass_success_ttl_clear"
        if commit_side == "left":
            return _commit_result("left", "commit_pass_left_hold")
        return _commit_result("right", "commit_pass_right_hold")

    # SD-3d: HOLD branch — ego successfully passed but is still in lateral merge
    # danger. Stay on the side TTL until BOTH the longitudinal-clearance gate
    # (delta_s_behind ≥ hold_release_long_m) AND the geometric merge-back check
    # (pass_window_check("merge_back")) clear. This is the "convoluted exit" the
    # user explicitly requested — not a constant K·Δv.
    if commit_enabled and state.mode in (HOLD_PASS_LEFT, HOLD_PASS_RIGHT):
        hold_side = state.commit.hold_pass_side or (
            "left" if state.mode == HOLD_PASS_LEFT else "right"
        )
        # Hard timeout — degenerate fallback (should not normally fire).
        hold_elapsed_s = float(sim_time_s) - float(state.commit.hold_entry_s)
        if hold_elapsed_s > float(config.hold_max_s):
            state.commit.post_event_state = FREE_RUN
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            state.commit.hold_entry_s = -1.0e9
            state.commit.hold_pass_side = ""
            return FREE_RUN, "optimal", None, "hold_timeout_force_release"
        # Hard hazard during HOLD → ABORT (e.g. fellow drafts back alongside).
        if relation_ahead or overlap_hazard_now:
            state.commit.abort_trigger = "hold_hazard_reappeared"
            state.commit.post_event_state = ABORT_PASS
            state.commit.abort_until_s = max(
                float(state.commit.abort_until_s),
                float(sim_time_s) + float(config.abort_hold_s),
            )
            state.commit.hold_entry_s = -1.0e9
            state.commit.hold_pass_side = ""
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _abort_result("abort_hold_hazard")
        # Compute the merge-back exit gate.
        # Longitudinal: ego must be far enough ahead that fellow can't rear-end during merge.
        delta_s_behind_m = -float(sit.delta_s_m) if sit is not None else 0.0
        long_clear_required_m = _dv_hold_release_long_m(config, dv_mps)
        long_ok = bool(delta_s_behind_m >= long_clear_required_m)
        # Geometric: when polylines provided, also verify merge-back path doesn't
        # intersect fellow's predicted trajectory.
        merge_geom_ok = True
        if (
            optimal_waypoints is not None
            and ego_s_m is not None
            and opp_s_m is not None
            and lap_length_m is not None
            and float(lap_length_m) > 0.0
        ):
            merge_geom_ok, _diag = pass_window_check(
                "merge_back",
                ego_s_m=float(ego_s_m),
                ego_speed_mps=float(ego_speed_mps),
                opp_s_m=float(opp_s_m),
                opp_speed_mps=float(opponent_speed_mps),
                optimal_waypoints=optimal_waypoints,
                side_waypoints=optimal_waypoints,  # unused in merge_back mode
                lap_length_m=float(lap_length_m),
                pass_duration_s=1.0,  # short window — merge takes <1 s
            )
        if long_ok and merge_geom_ok:
            state.commit.post_event_state = FREE_RUN
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            state.commit.hold_entry_s = -1.0e9
            state.commit.hold_pass_side = ""
            return FREE_RUN, "optimal", None, "hold_release_merge_safe"
        # Stay in HOLD on the side TTL with the bounded speed cap.
        hold_cap = max(
            float(state.commit.hold_speed_at_entry_mps),
            float(opponent_speed_mps) + float(config.hold_speed_floor_margin_mps),
        )
        return state.mode, hold_side, hold_cap, "hold_pass_hold"

    if commit_enabled and state.mode == ABORT_PASS:
        # Early release: opponent already behind and no active hazard → pass completed,
        # no need to hold abort timer (prevents unnecessary hard braking after passing).
        if (not relation_ahead) and (not overlap_hazard_now) and (not in_recovery_hold):
            state.commit.abort_success = True
            # SD-8: mirror SD-3d HOLD entry from the COMMIT-success branch.
            # When ego completes the pass during the abort_hold window AND is still
            # laterally side-by-side (|sit.lateral_m| < merge_safe_lat_m), enter HOLD
            # on the same side TTL instead of switching back to optimal IMMEDIATELY.
            # The immediate switch was the F2 "barely clears" failure mode: ABORT
            # triggered on TTC snapshot mid-pass, ego still on right TTL, relation
            # flipped behind, abort_passed_free_run fired with ttl_switch right→optimal
            # at lat=1.95m → near-contact during merge.
            commit_side_for_hold = str(state.commit.side or state.commit.last_side or "")
            still_side_by_side = bool(
                sit is not None
                and abs(float(sit.lateral_m)) < float(config.merge_safe_lat_m)
                and commit_side_for_hold in ("left", "right")
            )
            if still_side_by_side:
                state.commit.hold_entry_s = float(sim_time_s)
                state.commit.hold_speed_at_entry_mps = float(ego_speed_mps)
                state.commit.hold_pass_side = commit_side_for_hold
                state.commit.post_event_state = (
                    HOLD_PASS_LEFT if commit_side_for_hold == "left" else HOLD_PASS_RIGHT
                )
                state.mode = (
                    HOLD_PASS_LEFT if commit_side_for_hold == "left" else HOLD_PASS_RIGHT
                )
                _clear_setup_commit()
                _clear_pass_intent()
                _clear_lateral_lock()
                hold_cap = max(
                    float(state.commit.hold_speed_at_entry_mps),
                    float(opponent_speed_mps) + float(config.hold_speed_floor_margin_mps),
                )
                return state.mode, commit_side_for_hold, hold_cap, "hold_pass_entry_from_abort"
            # Already laterally clear OR no commit side recorded → original behavior.
            state.commit.post_event_state = FREE_RUN
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            return FREE_RUN, "optimal", None, "abort_passed_free_run"
        if hard_abort_hazard:
            state.commit.abort_until_s = max(
                float(state.commit.abort_until_s),
                float(sim_time_s) + float(config.abort_hold_s),
            )
        if float(sim_time_s) < float(state.commit.abort_until_s):
            state.commit.post_event_state = ABORT_PASS
            return _abort_result("abort_hold")
        if (not relation_ahead) and (not _is_release_hazard()) and (not in_recovery_hold):
            state.commit.abort_success = True
            state.commit.post_event_state = FREE_RUN
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            _clear_commit_lifecycle()
            state.mode = FREE_RUN
            return FREE_RUN, "optimal", None, "abort_recovered_free_run"
        if (not hard_abort_hazard) and (
            (assessment_gap_ok is True)
            or (opening_window_available and (not bool(assessment_closing_flag)))
        ):
            state.commit.abort_success = True
            state.commit.post_event_state = FOLLOW
            _clear_commit_lifecycle()
            return _follow_result("abort_success_follow")
        state.commit.post_event_state = FOLLOW
        _clear_commit_lifecycle()
        return _follow_result("abort_recover_follow")

    if relation_ahead and safety_pressure:
        state.protected_follow_active = True
        state.protected_follow_clear_count = 0
    elif state.protected_follow_active:
        # Structural opening-release rule:
        # keep FOLLOW while hazards are present, but allow release into setup
        # when a pass opening is stably clear even if opponent remains ahead.
        opening_stably_clear = bool(
            relation_ahead
            and opening_window_available
            and ((assessment_gap_ok is True) or asymmetric_opening)
            and ((not bool(assessment_closing_flag)) or asymmetric_opening)
            and not (
                bool(assessment_closing_flag)
                and assessment_emergency_risk_01 is not None
                and float(assessment_emergency_risk_01) > 0.25
            )
            and not (config.segment_aware_enabled and _seg_modifier == "blocked")
        )
        clear_now = opening_stably_clear or (
            (not relation_ahead) and (not _is_release_hazard()) and (not in_recovery_hold)
        )
        if clear_now:
            state.protected_follow_clear_count += 1
        else:
            state.protected_follow_clear_count = 0
        if state.protected_follow_clear_count >= int(config.protected_follow_release_cycles):
            state.protected_follow_active = False
            state.protected_follow_clear_count = 0

    if in_recovery_hold:
        _clear_commit_lifecycle()
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count += 1
        return _follow_result("contact_recovery_hold")

    if state.protected_follow_active:
        _clear_commit_lifecycle()
        _commit_side_pre = str(state.setup_commit_side or "")
        _intent_side_pre = str(state.pass_intent_side or "")
        _lock_side_pre = str(state.lateral_path_lock_side or "")
        _hold_side_pre = _lock_side_pre if _lock_side_pre in ("left", "right") else (
            _commit_side_pre if _commit_side_pre in ("left", "right") else _intent_side_pre
        )
        _hold_active_pre = bool(
            _hold_side_pre in ("left", "right")
            and (
                lateral_lock_active
                or float(sim_time_s) < float(state.setup_commit_until_s)
                or float(sim_time_s) < float(state.pass_intent_until_s)
            )
            and (not in_recovery_hold)
            and (not emergency_risk_high)
            and (not _is_release_hazard())
        )
        if _hold_active_pre:
            return _setup_result(_hold_side_pre, f"lateral_path_lock_{_hold_side_pre}_hold")
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count += 1
        return _follow_result("protected_follow_envelope")

    if not relation_ahead:
        _clear_commit_lifecycle()
        if _is_proximity_hazard():
            state.setup_candidate_side = ""
            state.setup_candidate_count = 0
            state.follow_pressure_count += 1
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _follow_result("proximity_hazard_follow")
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "opponent_behind_free_run"

    blocked = (
        sit.longitudinal_m > 0.0
        and sit.longitudinal_m < config.blocked_longitudinal_m
        and sit.distance_m < dynamic_blocked_gap_m
    )
    if assessment_gap_ok is False:
        blocked = True
    # When commit is enabled: do not drop to FREE_RUN on "large gap" alone while the
    # assessment still has the fellow ahead. Otherwise ego loses pass/follow discipline
    # and can run off-track in opponent-aware scenarios (F4/F6/F7).
    if (not blocked) and commit_enabled and relation_ahead:
        # Relaxation: if the fellow is clearly ahead, non-closing, low-risk, and there
        # is a large opening, allow FREE_RUN instead of forcing conservative FOLLOW.
        # Prevents slow startup lock-in against far / roadside opponents (e.g. F9).
        opening_available = bool(
            (assessment_optimal_open is True)
            or (assessment_left_open is True)
            or (assessment_right_open is True)
        )
        relax_to_free_run = bool(
            bool(config.ahead_relax_free_run_enabled)
            and (assessment_gap_ok is True)
            and (assessment_closing_flag is False)
            and opening_available
            and (sit.distance_m >= float(config.ahead_relax_min_gap_m))
            and (
                (assessment_emergency_risk_01 is not None)
                and (
                    float(assessment_emergency_risk_01)
                    <= float(config.ahead_relax_max_risk_01)
                )
            )
        )
        if not relax_to_free_run:
            blocked = True
    if not blocked:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        _clear_commit_lifecycle()
        return FREE_RUN, "optimal", None, "opponent_not_blocking"

    straight_ok = (not config.pass_requires_straight) or (sit.segment_context == "straight")
    overlap_side = sit.overlap_state == "side_by_side"
    overlap_partial_unsafe = (
        sit.overlap_state == "partial_overlap"
        and (
            sit.distance_m < config.setup_min_distance_m
            or abs(float(sit.lateral_m)) < config.setup_partial_overlap_lateral_m
        )
    )
    overlap_unsafe_for_setup = overlap_side or overlap_partial_unsafe
    close_for_setup = sit.distance_m < dynamic_setup_min_gap_m
    # SD-2c: prefer the assessment-dampened risk over raw collision_risk_01.
    # The raw value spikes on closing-speed terms even when fly-by geometry makes
    # the rear-end interpretation wrong; assessment_emergency_risk_01 already
    # applies _longitudinal_opening_dampen so it reflects the actual pass risk.
    # Falls back to raw when assessment is disabled.
    effective_risk_01 = (
        float(assessment_emergency_risk_01)
        if assessment_emergency_risk_01 is not None
        else float(sit.collision_risk_01)
    )
    # SD-2g: "ego can close" is a FEASIBILITY check, not an actual-state check.
    # In matched-speed FOLLOW (ego held to opp+follow_margin, MPC braking-for-distance)
    # the actual closing speed decays to ~0. The old gate (closing >= pass_min) was
    # then unsatisfiable, locking ego in FOLLOW forever. Instead, ask: under SETUP's
    # raised cap (=opp+setup_speed_margin_mps), would ego be closing fast enough?
    # If yes, the pass is feasible — let SETUP fire so ego accelerates and the actual
    # closing materializes. Real differential is enforced by SD-2e's COMMIT cap, not here.
    pass_can_close = bool(
        float(ego_speed_mps) + float(config.setup_speed_margin_mps)
        >= float(opponent_speed_mps) + float(config.pass_min_relative_speed_mps)
    )
    pass_safe = (
        straight_ok
        and effective_risk_01 <= config.pass_safe_risk_max
        and not overlap_unsafe_for_setup
        and not (close_for_setup and sit.overlap_state == "side_by_side")
        and pass_can_close
    )
    if bool(assessment_closing_flag):
        pass_safe = pass_safe and asymmetric_opening
    if assessment_emergency_risk_01 is not None:
        pass_safe = pass_safe and (not emergency_risk_high)
    left_open = True if assessment_left_open is None else bool(assessment_left_open)
    right_open = True if assessment_right_open is None else bool(assessment_right_open)
    if sit.lateral_relation == "left":
        preferred_side = "right"
    elif sit.lateral_relation == "right":
        preferred_side = "left"
    else:
        preferred_side = state.last_setup_side
    if preferred_side == "left" and not left_open and right_open:
        preferred_side = "right"
    elif preferred_side == "right" and not right_open and left_open:
        preferred_side = "left"
    if assessment_optimal_open is False and assessment_gap_ok is False:
        pass_safe = pass_safe and (left_open or right_open)
    if (not left_open) and (not right_open):
        pass_safe = False

    # Structural safety hold: sustained close-gap pressure blocks setup attempts.
    in_follow_pressure = bool(
        ((assessment_gap_ok is False) and (not opening_window_available))
        or (bool(assessment_closing_flag) and (not opening_window_available))
        or (sit.distance_m < dynamic_tight_gap_m and not asymmetric_opening)
        or (ttc_s < float(config.hard_ttc_s) and not asymmetric_opening)
    )
    if in_follow_pressure:
        state.follow_pressure_count += 1
    else:
        state.follow_pressure_count = max(0, int(state.follow_pressure_count) - 1)

    pass_intent_active = bool(
        state.pass_intent_side in ("left", "right")
        and float(sim_time_s) < float(state.pass_intent_until_s)
    )
    commit_active = bool(
        state.setup_commit_side in ("left", "right")
        and float(sim_time_s) < float(state.setup_commit_until_s)
    )

    # Arm pass intent from FOLLOW/SETUP when opening is consistently good.
    # SD-2f: also require gap proximity — pass_intent should not arm if fellow
    # is too far away. Otherwise the pass_intent → commit_active → setup chain
    # promotes ego into SETUP from far range, defeating the SETUP gap gate below.
    intent_candidate_ok = bool(
        pass_safe
        and opening_window_available
        and preferred_side in ("left", "right")
        and (closing_speed_pos_mps >= float(config.setup_commit_min_closing_mps))
        and (not in_recovery_hold)
        and (
            sit is None
            or float(sit.longitudinal_m) <= _dv_setup_gap_max_m(config, dv_mps)
        )
    )
    if intent_candidate_ok:
        if state.pass_intent_side != preferred_side:
            state.pass_intent_side = preferred_side
            state.pass_intent_candidate_count = 1
        else:
            state.pass_intent_candidate_count += 1
        if state.pass_intent_candidate_count >= int(config.pass_intent_entry_cycles):
            state.pass_intent_until_s = max(
                float(state.pass_intent_until_s),
                float(sim_time_s) + float(config.pass_intent_hold_s),
            )
            pass_intent_active = True
    else:
        state.pass_intent_candidate_count = max(0, int(state.pass_intent_candidate_count) - 1)

    # Promote pass intent into setup commit hold as soon as side-consistent setup intent exists.
    if pass_intent_active:
        intent_side = str(state.pass_intent_side or preferred_side)
        if intent_side not in ("left", "right"):
            intent_side = preferred_side
        if intent_side in ("left", "right"):
            if state.setup_commit_side != intent_side:
                state.setup_commit_side = intent_side
                state.setup_commit_candidate_count = 1
            else:
                state.setup_commit_candidate_count += 1
            if state.setup_commit_candidate_count >= int(config.setup_commit_entry_cycles):
                state.setup_commit_until_s = max(
                    float(state.setup_commit_until_s),
                    float(sim_time_s) + float(config.setup_commit_hold_s),
                )
                commit_active = True

    hard_commit_hazard = bool(
        in_recovery_hold
        or overlap_unsafe_for_setup
        or (sit.distance_m < dynamic_tight_gap_m and not asymmetric_opening)
        or (ttc_s < float(config.hard_ttc_s) and not asymmetric_opening)
        or emergency_risk_high
        or ((assessment_gap_ok is False) and (not opening_window_available))
    )
    if (pass_intent_active or commit_active) and hard_commit_hazard:
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        pass_intent_active = False
        commit_active = False
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)

    if commit_active:
        side = str(state.setup_commit_side or preferred_side)
        if side not in ("left", "right"):
            side = preferred_side
        if side in ("left", "right"):
            state.lateral_path_lock_side = side
            state.lateral_path_lock_until_s = max(
                float(state.lateral_path_lock_until_s),
                float(sim_time_s) + float(config.lateral_path_lock_hold_s),
            )
            if commit_enabled:
                side_open = bool(left_open if side == "left" else right_open)
                _seg_blocks_commit = (
                    config.segment_aware_enabled
                    and bool(config.corner_body_blocks_commit)
                    and _seg_modifier == "blocked"
                )
                _seg_tightens_commit = (
                    config.segment_aware_enabled
                    and _seg_modifier == "conservative"
                    and sit is not None
                    and float(sit.collision_risk_01) > float(config.corner_entry_commit_risk_max)
                )
                _commit_speed_ok = float(ego_speed_mps) <= float(config.commit_max_speed_mps)
                _opposing_commit_cooling = bool(
                    str(state.commit.last_side) in ("left", "right")
                    and side != str(state.commit.last_side)
                    and (
                        float(sim_time_s) - float(state.commit.last_exit_s)
                        < float(config.opposing_commit_cooldown_s)
                    )
                )
                # Gap gate: only commit when close enough to the fellow (1-2 car lengths).
                # Prevents premature commits from 30+ m away that can't complete before
                # safety gates re-engage.
                _commit_gap_ok = bool(
                    float(sit.longitudinal_m) <= _dv_commit_gap_max_m(config, dv_mps)
                )
                # SD-3f: also run pass_window_check on the commit_active path. The
                # late-SETUP-entry look-ahead (SD-3c) was bypassed when the planner
                # took the pass_intent → commit_active chain, leaving F2_tactical's
                # converging-corner geometry undetected. When polylines unavailable,
                # fail open (preserves backward compat with tests that don't pass them).
                _commit_geom_ok = True
                if (
                    optimal_waypoints is not None
                    and ego_s_m is not None
                    and opp_s_m is not None
                    and lap_length_m is not None
                    and float(lap_length_m) > 0.0
                ):
                    side_wp = side_waypoints_left if side == "left" else side_waypoints_right
                    if side_wp is not None:
                        _commit_geom_ok, _diag = pass_window_check(
                            side,
                            ego_s_m=float(ego_s_m),
                            ego_speed_mps=float(ego_speed_mps),
                            opp_s_m=float(opp_s_m),
                            opp_speed_mps=float(opponent_speed_mps),
                            optimal_waypoints=optimal_waypoints,
                            side_waypoints=side_wp,
                            lap_length_m=float(lap_length_m),
                        )
                # SD-6: dropped the (closing_flag AND risk > 0.10) gate.
                # Geometry is validated by _commit_geom_ok (SD-3c look-ahead);
                # actual collision is gated by SD-4 predicted_collision (which
                # routes through hard_abort_hazard and EMERGENCY_STABLE). The
                # risk-snapshot gate was over-defensive and never permitted
                # commit during normal closing approaches.
                commit_candidate_ok = bool(
                    side_open
                    and pass_safe
                    and opening_window_available
                    and (not hard_abort_hazard)
                    and (not in_recovery_hold)
                    and not _seg_blocks_commit
                    and not _seg_tightens_commit
                    and _commit_speed_ok
                    and not _opposing_commit_cooling
                    and _commit_gap_ok
                    and _commit_geom_ok
                )
                if commit_candidate_ok:
                    if state.commit.side != side:
                        state.commit.side = side
                        state.commit.candidate_count = 1
                    else:
                        state.commit.candidate_count += 1
                else:
                    state.commit.candidate_count = 0
                if state.commit.candidate_count >= int(config.commit_entry_cycles):
                    state.commit.trigger = f"setup_chain_commit_{side}"
                    state.commit.post_event_state = COMMIT_PASS_LEFT if side == "left" else COMMIT_PASS_RIGHT
                    state.commit.until_s = max(
                        float(state.commit.until_s),
                        float(sim_time_s) + float(config.commit_hold_s),
                    )
                    return _commit_result(side, f"commit_pass_{side}")
            return _setup_result(side, f"setup_commit_{side}_hold")

    if not pass_safe:
        if commit_enabled and state.mode in (COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT):
            state.commit.abort_trigger = "pass_safe_lost"
            state.commit.post_event_state = ABORT_PASS
            state.commit.abort_until_s = max(
                float(state.commit.abort_until_s),
                float(sim_time_s) + float(config.abort_hold_s),
            )
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _abort_result("abort_pass_safe_lost")
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        if assessment_gap_ok is False:
            return _follow_result("gap_not_ok_follow")
        return _follow_result("ahead_blocking_follow")

    # Keep FOLLOW while pressure is sustained even if one frame appears pass-safe.
    if state.follow_pressure_count >= int(config.follow_pressure_threshold_cycles):
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        return _follow_result("follow_pressure_hold")

    # Avoid setup chatter: once we exit SETUP due to unsafe/blocked geometry,
    # hold FOLLOW briefly before allowing a fresh setup entry.
    if state.mode not in (SETUP_LEFT, SETUP_RIGHT):
        if float(sim_time_s) - float(state.last_setup_exit_sim_time_s) < config.setup_reentry_cooldown_s:
            state.setup_candidate_side = ""
            state.setup_candidate_count = 0
            return _follow_result("setup_reentry_cooldown_hold")

    # SD-2f: don't enter SETUP from too far away. SETUP performs the lateral shift
    # to a side TTL; doing it from 40+ m means ego converges with fellow on a side
    # line for several seconds before COMMIT can fire. Stay in FOLLOW until close
    # enough that SETUP→COMMIT can complete in 1-2 cycles.
    if (
        state.mode not in (SETUP_LEFT, SETUP_RIGHT)
        and sit is not None
        and float(sit.longitudinal_m) > _dv_setup_gap_max_m(config, dv_mps)
    ):
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        return _follow_result("setup_too_far_follow")

    # SD-2f: SETUP timeout. If SETUP has held for setup_max_hold_s without reaching
    # COMMIT (typically because closing speed is too low or geometry is bad), bail
    # back to FOLLOW on optimal. Prevents the "stuck on side TTL while approaching"
    # failure mode (F2_tactical first attempt: 4 sec SETUP → contact).
    if state.mode in (SETUP_LEFT, SETUP_RIGHT) and float(state.setup_entry_s) > -1.0e8:
        if float(sim_time_s) - float(state.setup_entry_s) > float(config.setup_max_hold_s):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
            state.setup_entry_s = -1.0e9
            state.setup_candidate_side = ""
            state.setup_candidate_count = 0
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _follow_result("setup_timeout_follow")

    target_side = preferred_side

    if state.mode == SETUP_LEFT and target_side == "right":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "left"
            flip_held = True
        else:
            state.last_flip_sim_time_s = sim_time_s
            flip_held = False
    elif state.mode == SETUP_RIGHT and target_side == "left":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "right"
            flip_held = True
        else:
            state.last_flip_sim_time_s = sim_time_s
            flip_held = False
    else:
        flip_held = False

    # Setup entry persistence: require side-consistent setup signal before entering setup.
    if state.mode not in (SETUP_LEFT, SETUP_RIGHT):
        if state.setup_candidate_side != target_side:
            state.setup_candidate_side = target_side
            state.setup_candidate_count = 1
            return _follow_result("setup_candidate_collect")
        state.setup_candidate_count += 1
        if state.setup_candidate_count < int(config.setup_entry_persistence_cycles):
            return _follow_result("setup_candidate_collect")
    else:
        state.setup_candidate_side = target_side
        state.setup_candidate_count = 0

    # SD-3c: geometric look-ahead. If the candidate pass-side TTL converges
    # with fellow's path inside the predicted pass duration, reject this side
    # before initiating the lateral shift. If both sides fail, stay in FOLLOW.
    # Polyline kwargs are Optional — when not provided (e.g. by existing tests),
    # the check is skipped and the prior preference logic stands.
    polyline_inputs_present = (
        optimal_waypoints is not None
        and ego_s_m is not None
        and opp_s_m is not None
        and lap_length_m is not None
        and float(lap_length_m) > 0.0
    )
    if polyline_inputs_present and state.mode not in (SETUP_LEFT, SETUP_RIGHT):
        def _side_window_ok(s: str) -> Tuple[bool, dict]:
            wp = side_waypoints_left if s == "left" else side_waypoints_right
            if wp is None:
                return True, {"reason": "no_side_polyline"}
            return pass_window_check(
                s,
                ego_s_m=float(ego_s_m),
                ego_speed_mps=float(ego_speed_mps),
                opp_s_m=float(opp_s_m),
                opp_speed_mps=float(opponent_speed_mps),
                optimal_waypoints=optimal_waypoints,
                side_waypoints=wp,
                lap_length_m=float(lap_length_m),
            )
        target_ok, target_diag = _side_window_ok(target_side)
        if not target_ok:
            other = "left" if target_side == "right" else "right"
            other_ok, _other_diag = _side_window_ok(other)
            if other_ok:
                # Switch sides — the geometry permits the opposite side.
                target_side = other
            else:
                # Both sides rejected — pass is geometrically infeasible. Stay in FOLLOW.
                state.setup_candidate_side = ""
                state.setup_candidate_count = 0
                _clear_setup_commit()
                _clear_pass_intent()
                _clear_lateral_lock()
                return _follow_result("pass_window_unsafe_both_sides")

    if state.setup_commit_side and state.setup_commit_side != target_side:
        _clear_setup_commit()
    if state.pass_intent_side and state.pass_intent_side != target_side:
        _clear_pass_intent()
    if state.lateral_path_lock_side and state.lateral_path_lock_side != target_side:
        _clear_lateral_lock()
    if target_side == "left":
        reason = "setup_flip_cooldown_hold" if flip_held else "setup_left_open"
        state.lateral_path_lock_side = "left"
        state.lateral_path_lock_until_s = max(
            float(state.lateral_path_lock_until_s),
            float(sim_time_s) + float(config.lateral_path_lock_hold_s),
        )
        return _setup_result("left", reason)
    reason = "setup_flip_cooldown_hold" if flip_held else "setup_right_open"
    state.lateral_path_lock_side = "right"
    state.lateral_path_lock_until_s = max(
        float(state.lateral_path_lock_until_s),
        float(sim_time_s) + float(config.lateral_path_lock_hold_s),
    )
    return _setup_result("right", reason)


__all__ = [
    "FREE_RUN",
    "FOLLOW",
    "SETUP_LEFT",
    "SETUP_RIGHT",
    "SETUP_PASS_LEFT",
    "SETUP_PASS_RIGHT",
    "COMMIT_PASS_LEFT",
    "COMMIT_PASS_RIGHT",
    "ABORT_PASS",
    "CommitPlannerState",
    "TacticalPlannerConfig",
    "TacticalPlannerState",
    "_canonical_mode",
    "apply_ttl_key_to_agent",
    "tactical_planner_step",
    "tactical_planner_step_v1",
]
