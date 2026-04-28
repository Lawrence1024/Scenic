"""SD-11a: unit tests for FellowPredictor.trajectory() multi-step CV extrapolation."""

import math

import pytest

from scenic.domains.racing.prediction import FellowPredictor


def _seed_constant_velocity(p, vx, vy, *, dt=0.05, n=3):
    """Feed n+1 observations with constant velocity (vx, vy)."""
    for i in range(n + 1):
        t = i * dt
        x = vx * t
        y = vy * t
        p.step(t, x, y, fellow_progress_s_m=None, dt_pred_s=dt)


def test_trajectory_length_matches_horizon_over_dt_plus_one():
    p = FellowPredictor()
    _seed_constant_velocity(p, vx=10.0, vy=0.0, dt=0.05, n=3)
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    # int(2.0 / 0.5) + 1 = 5 samples at t = {0, 0.5, 1.0, 1.5, 2.0}
    assert len(samples) == 5
    assert [s[0] for s in samples] == pytest.approx([0.0, 0.5, 1.0, 1.5, 2.0])


def test_trajectory_index_one_matches_step_dt_prediction():
    """samples[1] (at t=dt) should match what step(dt_pred_s=dt) would output for the same history."""
    p = FellowPredictor()
    dt_pred = 0.5
    # Build identical histories in two predictors. p_step calls step(dt_pred); p_traj calls trajectory.
    p_step = FellowPredictor()
    p_traj = FellowPredictor()
    obs = [(0.00, 0.0, 0.0), (0.05, 0.5, 0.0), (0.10, 1.0, 0.0), (0.15, 1.5, 0.0)]
    for (t, x, y) in obs[:-1]:
        p_step.step(t, x, y, fellow_progress_s_m=None, dt_pred_s=dt_pred)
        p_traj.step(t, x, y, fellow_progress_s_m=None, dt_pred_s=dt_pred)
    # Final step on p_step uses dt_pred_s=dt_pred so its output is the dt_pred-ahead prediction.
    r_step = p_step.step(obs[-1][0], obs[-1][1], obs[-1][2],
                         fellow_progress_s_m=None, dt_pred_s=dt_pred)
    p_traj.step(obs[-1][0], obs[-1][1], obs[-1][2], fellow_progress_s_m=None, dt_pred_s=dt_pred)
    samples = p_traj.trajectory(horizon_s=1.0, sample_dt_s=dt_pred)
    # samples[1] is at t = dt_pred from now; should match step(dt_pred) prediction.
    assert samples[1][1] == pytest.approx(r_step.fellow_pred_x, abs=1e-6)
    assert samples[1][2] == pytest.approx(r_step.fellow_pred_y, abs=1e-6)


def test_trajectory_constant_velocity_preserves_direction():
    p = FellowPredictor()
    _seed_constant_velocity(p, vx=10.0, vy=5.0, dt=0.05, n=4)
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    # Each sample should advance ~ (vx*dt, vy*dt) past the prior.
    for i in range(1, len(samples)):
        dx = samples[i][1] - samples[i - 1][1]
        dy = samples[i][2] - samples[i - 1][2]
        assert dx == pytest.approx(10.0 * 0.5, abs=0.05)
        assert dy == pytest.approx(5.0 * 0.5, abs=0.05)


def test_trajectory_empty_history_returns_empty_list():
    p = FellowPredictor()
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    assert samples == []


def test_trajectory_single_observation_returns_zero_motion():
    p = FellowPredictor()
    p.step(0.0, 7.0, -3.0, fellow_progress_s_m=None, dt_pred_s=0.05)
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    assert len(samples) == 5
    for t_off, x, y, s in samples:
        assert x == pytest.approx(7.0)
        assert y == pytest.approx(-3.0)
        assert s is None


def test_trajectory_stationary_fellow_returns_constant_xy():
    p = FellowPredictor()
    # Simulate stationary fellow at (10, 20).
    for i in range(5):
        p.step(i * 0.05, 10.0, 20.0, fellow_progress_s_m=None, dt_pred_s=0.05)
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    for t_off, x, y, _s in samples:
        assert x == pytest.approx(10.0, abs=1e-6)
        assert y == pytest.approx(20.0, abs=1e-6)


def test_trajectory_includes_s_when_progress_was_supplied():
    p = FellowPredictor()
    # Constant longitudinal speed: 20 m/s along s.
    for i in range(4):
        t = i * 0.05
        p.step(t, 0.0, 0.0, fellow_progress_s_m=20.0 * t, dt_pred_s=0.05)
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    assert len(samples) == 5
    # s should advance 20 m/s * 0.5 s = 10 m per sample.
    for i in range(1, len(samples)):
        ds = samples[i][3] - samples[i - 1][3]
        assert ds == pytest.approx(10.0, abs=0.5)


def test_trajectory_zero_horizon_returns_only_current():
    p = FellowPredictor()
    _seed_constant_velocity(p, vx=10.0, vy=0.0, dt=0.05, n=3)
    samples = p.trajectory(horizon_s=0.0, sample_dt_s=0.5)
    assert len(samples) == 1
    assert samples[0][0] == 0.0
