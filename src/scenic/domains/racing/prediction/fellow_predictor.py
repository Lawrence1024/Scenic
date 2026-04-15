"""One-step-ahead fellow pose prediction from short pose history (Phase 7)."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass
class FellowPredictorStepResult:
    """Outputs for one control-cycle prediction update."""

    fellow_pred_x: float
    fellow_pred_y: float
    fellow_pred_s: Optional[float]
    prediction_error_next_step: Optional[float]
    prediction_error_zero_motion: Optional[float]
    prediction_error_hold_last: Optional[float]


class FellowPredictor:
    """Short-horizon fellow pose prediction with baselines for benchmarking.

    Uses CV extrapolation from the last segment, blended with a recency-weighted
    history trend so very old samples influence less than recent ones. Baselines
    compare against the same realized pose using simpler forecasts.

    - **zero motion:** predicted next pose = previous pose
    - **hold last velocity:** CV from the prior segment to the current observation time
    """

    def __init__(
        self,
        *,
        history_len: int = 8,
        history_decay_s: float = 0.20,
        history_max_age_s: float = 0.40,
    ) -> None:
        self._history_len = max(3, int(history_len))
        self._history_decay_s = max(1e-3, float(history_decay_s))
        self._history_max_age_s = max(self._history_decay_s, float(history_max_age_s))
        self._hist: Deque[Tuple[float, float, float, Optional[float]]] = deque(
            maxlen=self._history_len
        )
        self._last_pred_xy: Optional[Tuple[float, float]] = None
        self._last_pred_s: Optional[float] = None

    def _recency_weighted_velocity_xy(self) -> Optional[Tuple[float, float]]:
        if len(self._hist) < 2:
            return None
        t_ref = float(self._hist[-1][0])
        tw_sum = 0.0
        t_sum = 0.0
        x_sum = 0.0
        y_sum = 0.0
        weighted_rows: list[Tuple[float, float, float, float]] = []
        for ti, xi, yi, _si in self._hist:
            age = max(0.0, t_ref - float(ti))
            if age > self._history_max_age_s:
                continue
            wi = math.exp(-age / self._history_decay_s)
            tw_sum += wi
            t_sum += wi * float(ti)
            x_sum += wi * float(xi)
            y_sum += wi * float(yi)
            weighted_rows.append((wi, float(ti), float(xi), float(yi)))
        if tw_sum <= 1e-12:
            return None
        t_mean = t_sum / tw_sum
        x_mean = x_sum / tw_sum
        y_mean = y_sum / tw_sum
        var_t = 0.0
        cov_tx = 0.0
        cov_ty = 0.0
        for wi, ti, xi, yi in weighted_rows:
            dt = ti - t_mean
            var_t += wi * dt * dt
            cov_tx += wi * dt * (xi - x_mean)
            cov_ty += wi * dt * (yi - y_mean)
        if var_t <= 1e-12:
            return None
        return (cov_tx / var_t, cov_ty / var_t)

    def _recency_weighted_velocity_s(self) -> Optional[float]:
        rows: list[Tuple[float, float]] = []
        for ti, _xi, _yi, si in self._hist:
            if si is not None:
                rows.append((float(ti), float(si)))
        if len(rows) < 2:
            return None
        t_ref = rows[-1][0]
        tw_sum = 0.0
        t_sum = 0.0
        s_sum = 0.0
        weighted_rows: list[Tuple[float, float, float]] = []
        for ti, si in rows:
            age = max(0.0, t_ref - ti)
            if age > self._history_max_age_s:
                continue
            wi = math.exp(-age / self._history_decay_s)
            tw_sum += wi
            t_sum += wi * ti
            s_sum += wi * si
            weighted_rows.append((wi, ti, si))
        if tw_sum <= 1e-12:
            return None
        t_mean = t_sum / tw_sum
        s_mean = s_sum / tw_sum
        var_t = 0.0
        cov_ts = 0.0
        for wi, ti, si in weighted_rows:
            dt = ti - t_mean
            var_t += wi * dt * dt
            cov_ts += wi * dt * (si - s_mean)
        if var_t <= 1e-12:
            return None
        return cov_ts / var_t

    def reset(self) -> None:
        self._hist.clear()
        self._last_pred_xy = None
        self._last_pred_s = None

    def step(
        self,
        sim_time_s: float,
        x: float,
        y: float,
        *,
        fellow_progress_s_m: Optional[float],
        dt_pred_s: float,
    ) -> FellowPredictorStepResult:
        """Update history with the *current* observation and emit predictions / errors."""

        t = float(sim_time_s)
        px = float(x)
        py = float(y)
        ps = float(fellow_progress_s_m) if fellow_progress_s_m is not None else None

        err_next: Optional[float] = None
        err_zero: Optional[float] = None
        err_hold: Optional[float] = None

        if self._last_pred_xy is not None:
            lx, ly = self._last_pred_xy
            err_next = math.hypot(px - lx, py - ly)

        if self._hist:
            px_prev, py_prev = self._hist[-1][1], self._hist[-1][2]
            err_zero = math.hypot(px - px_prev, py - py_prev)
            if len(self._hist) >= 2:
                t0, x0, y0, _ = self._hist[-2]
                t1, x1, y1, _ = self._hist[-1]
                dt_seg = float(t1 - t0)
                if dt_seg > 1e-9:
                    vx = (x1 - x0) / dt_seg
                    vy = (y1 - y0) / dt_seg
                    dt_to_now = float(t) - float(t1)
                    pred_hold_x = float(x1) + vx * dt_to_now
                    pred_hold_y = float(y1) + vy * dt_to_now
                    err_hold = math.hypot(px - pred_hold_x, py - pred_hold_y)

        self._hist.append((t, px, py, ps))

        pred_x, pred_y = px, py
        pred_s: Optional[float] = ps
        if len(self._hist) >= 2:
            t0, x0, y0, s0 = self._hist[-2]
            t1, x1, y1, s1 = self._hist[-1]
            dt_seg = float(t1 - t0)
            if dt_seg > 1e-9:
                vx_seg = (x1 - x0) / dt_seg
                vy_seg = (y1 - y0) / dt_seg
                v_hist = self._recency_weighted_velocity_xy()
                vx_curr = vx_seg
                vy_curr = vy_seg
                if v_hist is not None:
                    # Keep the latest segment dominant while using recent history
                    # to reduce sensitivity to single-sample jitter.
                    w_hist = 0.35
                    vx_curr = (1.0 - w_hist) * vx_seg + w_hist * v_hist[0]
                    vy_curr = (1.0 - w_hist) * vy_seg + w_hist * v_hist[1]

                dt_f = float(dt_pred_s)
                pred_x = float(x1) + vx_curr * dt_f
                pred_y = float(y1) + vy_curr * dt_f
                if s1 is not None and s0 is not None:
                    vs_seg = (float(s1) - float(s0)) / dt_seg
                    vs_curr = vs_seg
                    vs_hist = self._recency_weighted_velocity_s()
                    if vs_hist is not None:
                        w_hist_s = 0.35
                        vs_curr = (1.0 - w_hist_s) * vs_seg + w_hist_s * vs_hist
                    pred_s = float(s1) + vs_curr * dt_f

        self._last_pred_xy = (float(pred_x), float(pred_y))
        self._last_pred_s = pred_s

        return FellowPredictorStepResult(
            fellow_pred_x=float(pred_x),
            fellow_pred_y=float(pred_y),
            fellow_pred_s=pred_s,
            prediction_error_next_step=err_next,
            prediction_error_zero_motion=err_zero,
            prediction_error_hold_last=err_hold,
        )


def _fmt_opt(x: Optional[float]) -> str:
    if x is None:
        return "na"
    if not math.isfinite(float(x)):
        return "na"
    return f"{float(x):.4f}"


def format_phase7_prediction_log_line(sim_time_s: float, r: FellowPredictorStepResult) -> str:
    ps = _fmt_opt(r.fellow_pred_s)
    return (
        f"[Phase7Prediction] t={sim_time_s:.2f}s fellow_pred_x={r.fellow_pred_x:.4f} "
        f"fellow_pred_y={r.fellow_pred_y:.4f} fellow_pred_s={ps} "
        f"prediction_error_next_step={_fmt_opt(r.prediction_error_next_step)} "
        f"prediction_error_zero_motion={_fmt_opt(r.prediction_error_zero_motion)} "
        f"prediction_error_hold_last={_fmt_opt(r.prediction_error_hold_last)}"
    )
