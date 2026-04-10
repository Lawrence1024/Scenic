"""Tests for Phase 4 pass commit / shield layer."""

from scenic.domains.racing.pass_commit_shield import (
    ABORT_PASS,
    COMMIT_PASS_LEFT,
    COMMIT_PASS_RIGHT,
    EMERGENCY_AVOID,
    PassShieldConfig,
    PassShieldState,
    pass_shield_step,
)
from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import (
    FOLLOW,
    FREE_RUN,
    SETUP_LEFT,
    SETUP_RIGHT,
    TacticalPlannerConfig,
)


def _sit(**kwargs):
    defaults = dict(
        ahead=True,
        delta_s_m=15.0,
        delta_s_source="heading_proxy",
        lateral_relation="on_line",
        closing_speed_mps=2.0,
        overlap_state="clear_ahead",
        collision_risk_01=0.2,
        segment_context="straight",
        distance_m=18.0,
        longitudinal_m=15.0,
        lateral_m=1.0,
    )
    defaults.update(kwargs)
    return OpponentSituation(**defaults)


def test_emergency_triggers_emergency_avoid():
    st = PassShieldState()
    s = _sit(collision_risk_01=0.95, segment_context="straight")
    m, ttl, cap, reason = pass_shield_step(
        st,
        s,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=25.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == EMERGENCY_AVOID and ttl == "optimal" and cap is not None and reason == "emergency_risk"


def test_abort_setup_when_risk_high():
    st = PassShieldState()
    s = _sit(collision_risk_01=0.95, segment_context="straight")
    m, ttl, cap, reason = pass_shield_step(
        st,
        s,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=25.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == EMERGENCY_AVOID and reason == "emergency_risk"

    st2 = PassShieldState()
    s2 = _sit(collision_risk_01=0.75, segment_context="straight")
    m2, ttl2, cap2, reason2 = pass_shield_step(
        st2,
        s2,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=25.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(emergency_risk_01=0.99, abort_setup_risk_01=0.70),
    )
    assert m2 == ABORT_PASS and ttl2 == "optimal" and reason2 == "setup_risk"


def test_commit_after_dwell_on_straight_right():
    st = PassShieldState()
    cfg = PassShieldConfig(commit_dwell_s=0.2, commit_max_risk_01=0.5)
    tac = TacticalPlannerConfig()
    s = _sit(collision_risk_01=0.1, segment_context="straight", lateral_relation="right")
    m, ttl, cap, r0 = pass_shield_step(
        st,
        s,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=0.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    assert m == SETUP_RIGHT and r0 is None

    s2 = _sit(collision_risk_01=0.1, segment_context="straight", lateral_relation="right")
    m2, ttl2, cap2, r2 = pass_shield_step(
        st,
        s2,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=0.5,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    assert m2 == COMMIT_PASS_RIGHT and ttl2 == "right" and r2 == "commit_dwell_right"


def test_commit_after_dwell_on_straight_left():
    st = PassShieldState()
    cfg = PassShieldConfig(commit_dwell_s=0.2, commit_max_risk_01=0.5)
    tac = TacticalPlannerConfig()
    s = _sit(collision_risk_01=0.1, segment_context="straight", lateral_relation="left")
    pass_shield_step(
        st,
        s,
        SETUP_LEFT,
        "left",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=0.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    s2 = _sit(collision_risk_01=0.1, segment_context="straight", lateral_relation="left")
    m2, ttl2, _, r2 = pass_shield_step(
        st,
        s2,
        SETUP_LEFT,
        "left",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=0.5,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    assert m2 == COMMIT_PASS_LEFT and ttl2 == "left" and r2 == "commit_dwell_left"


def test_abort_committed_corner_corridor():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "right"
    tac = TacticalPlannerConfig()
    s = _sit(
        collision_risk_01=0.60,
        segment_context="corner_body",
    )
    m, ttl, cap, reason = pass_shield_step(
        st,
        s,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=5.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=PassShieldConfig(),
    )
    assert m == ABORT_PASS and ttl == "optimal" and cap is not None and reason == "corridor"
    assert st.commit_active is False


def test_abort_committed_partial_overlap():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "left"
    tac = TacticalPlannerConfig()
    s = _sit(
        collision_risk_01=0.60,
        segment_context="straight",
        overlap_state="partial_overlap",
    )
    m, ttl, cap, reason = pass_shield_step(
        st,
        s,
        SETUP_LEFT,
        "left",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=5.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=PassShieldConfig(),
    )
    assert m == ABORT_PASS and reason == "overlap"


def test_hold_commit_right():
    st = PassShieldState()
    cfg = PassShieldConfig(commit_dwell_s=0.05, commit_max_risk_01=0.5)
    tac = TacticalPlannerConfig()
    s0 = _sit(collision_risk_01=0.1, segment_context="straight")
    pass_shield_step(
        st,
        s0,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=0.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    s1 = _sit(collision_risk_01=0.1, segment_context="straight")
    m1, _, _, _ = pass_shield_step(
        st,
        s1,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    assert m1 == COMMIT_PASS_RIGHT
    s2 = _sit(collision_risk_01=0.1, segment_context="straight")
    m2, _, _, _ = pass_shield_step(
        st,
        s2,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=1.1,
        pit_mode=False,
        tactical_config=tac,
        shield_config=cfg,
    )
    assert m2 == COMMIT_PASS_RIGHT


def test_passthrough_free_run():
    st = PassShieldState()
    s = _sit(distance_m=120.0, longitudinal_m=80.0)
    m, ttl, cap, r = pass_shield_step(
        st,
        s,
        FREE_RUN,
        "optimal",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=28.0,
        sim_time_s=0.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == FREE_RUN and r is None


def test_follow_clears_commit():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "right"
    s = _sit()
    m, _, _, _ = pass_shield_step(
        st,
        s,
        FOLLOW,
        "optimal",
        20.0,
        has_opponent=True,
        ego_speed_mps=25.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == FOLLOW
    assert st.commit_active is False


def test_free_run_clears_commit():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "left"
    s = _sit()
    m, _, _, _ = pass_shield_step(
        st,
        s,
        FREE_RUN,
        "optimal",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=28.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == FREE_RUN
    assert st.commit_active is False


def test_pit_mode_passthrough_clears_commit():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "right"
    s = _sit()
    m, ttl, _, _ = pass_shield_step(
        st,
        s,
        FOLLOW,
        "optimal",
        18.0,
        has_opponent=True,
        ego_speed_mps=10.0,
        opponent_speed_mps=8.0,
        sim_time_s=1.0,
        pit_mode=True,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == FOLLOW and ttl == "optimal"
    assert st.commit_active is False


def test_no_opponent_skips_shield():
    st = PassShieldState()
    st.commit_active = True
    m, ttl, cap, r = pass_shield_step(
        st,
        _sit(),
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=False,
        ego_speed_mps=30.0,
        opponent_speed_mps=0.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == SETUP_RIGHT and r is None
    assert st.commit_active is False


def test_sit_none_skips_shield():
    st = PassShieldState()
    m, ttl, _, r = pass_shield_step(
        st,
        None,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=25.0,
        sim_time_s=1.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == SETUP_RIGHT and r is None


def test_corner_entry_triggers_corridor_abort_when_committed():
    st = PassShieldState()
    st.commit_active = True
    st.commit_side = "right"
    s = _sit(collision_risk_01=0.55, segment_context="corner_entry")
    m, _, _, reason = pass_shield_step(
        st,
        s,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        ego_speed_mps=35.0,
        opponent_speed_mps=30.0,
        sim_time_s=2.0,
        pit_mode=False,
        tactical_config=TacticalPlannerConfig(),
        shield_config=PassShieldConfig(),
    )
    assert m == ABORT_PASS and reason == "corridor"
