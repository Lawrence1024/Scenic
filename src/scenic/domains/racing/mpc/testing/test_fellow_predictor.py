"""Unit tests for Phase 7 ``FellowPredictor`` (CV extrapolation + baselines)."""

import math

import pytest

from scenic.domains.racing.prediction import FellowPredictor


def test_constant_velocity_three_steps():
    p = FellowPredictor()
    dt = 0.05
    # First observation: no prior prediction to score
    r0 = p.step(0.0, 0.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    assert r0.prediction_error_next_step is None
    assert r0.prediction_error_zero_motion is None
    # Second observation: previous forward pose defaulted to first sample -> error = displacement
    r1 = p.step(dt, 1.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    assert r1.prediction_error_next_step == pytest.approx(1.0)
    assert r1.fellow_pred_x == 2.0
    assert r1.fellow_pred_y == 0.0
    # Third observation: CV segment (0,0)->(1,0) predicts 2.0; realized 2.0 -> zero error
    r2 = p.step(2 * dt, 2.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    assert r2.prediction_error_next_step == pytest.approx(0.0, abs=1e-9)
    assert r2.fellow_pred_x == pytest.approx(3.0)


def test_reset_clears_errors():
    p = FellowPredictor()
    dt = 0.05
    p.step(0.0, 0.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    p.step(dt, 1.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    p.step(2 * dt, 2.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    p.reset()
    r = p.step(0.0, 5.0, 5.0, fellow_progress_s_m=None, dt_pred_s=dt)
    assert r.prediction_error_next_step is None


def test_zero_motion_baseline_matches_displacement():
    p = FellowPredictor()
    dt = 0.05
    p.step(0.0, 0.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    r = p.step(dt, 3.0, 4.0, fellow_progress_s_m=None, dt_pred_s=dt)
    assert r.prediction_error_zero_motion is not None
    assert math.isclose(r.prediction_error_zero_motion, 5.0, rel_tol=0.0, abs_tol=1e-6)


def test_recency_weighting_deemphasizes_old_samples():
    p = FellowPredictor()
    dt = 0.05
    # Old point is far away in both time and position.
    p.step(0.0, 100.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    p.step(0.50, 0.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    p.step(0.55, 1.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    r = p.step(0.60, 2.0, 0.0, fellow_progress_s_m=None, dt_pred_s=dt)
    # Prediction should stay near recent trend (around x=3.0), not be dragged by old outlier.
    assert r.fellow_pred_x > 2.6
