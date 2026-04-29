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

    def _yaw_rate_from_history(self) -> Optional[float]:
        """SD-27a: estimate fellow's yaw rate from heading history.

        Uses per-segment heading = atan2(dy, dx) over the last ~max_age_s of
        history, unwraps, then runs a recency-weighted linear regression of
        unwrapped heading vs time. Returns None if too few segments or motion
        is too small to extract a reliable heading (caller falls back to CV).

        Pure observation-based: makes no assumption that fellow is on any
        particular polyline. Captures whatever turn rate fellow's actual
        cartesian trajectory exhibits.
        """
        if len(self._hist) < 3:
            return None

        # Compute per-segment headings from successive history points.
        seg_headings: list[Tuple[float, float]] = []  # (t_mid, theta)
        hist_list = list(self._hist)
        for k in range(1, len(hist_list)):
            t0, x0, y0, _ = hist_list[k - 1]
            t1, x1, y1, _ = hist_list[k]
            dt_seg = float(t1) - float(t0)
            if dt_seg <= 1e-9:
                continue
            dx = float(x1) - float(x0)
            dy = float(y1) - float(y0)
            if math.hypot(dx, dy) < 1e-3:
                # Motion too small — heading is noise.
                continue
            seg_headings.append((0.5 * (float(t0) + float(t1)), math.atan2(dy, dx)))

        if len(seg_headings) < 2:
            return None

        # Unwrap headings so the regression sees a continuous angle.
        theta_unw = [seg_headings[0][1]]
        for i in range(1, len(seg_headings)):
            d = seg_headings[i][1] - seg_headings[i - 1][1]
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            theta_unw.append(theta_unw[-1] + d)

        # Recency-weighted linear regression of unwrapped theta vs time.
        t_ref = seg_headings[-1][0]
        rows: list[Tuple[float, float, float]] = []
        for (ti, _), th in zip(seg_headings, theta_unw):
            age = max(0.0, t_ref - ti)
            if age > self._history_max_age_s:
                continue
            wi = math.exp(-age / self._history_decay_s)
            rows.append((wi, ti, th))
        if len(rows) < 2:
            return None
        tw_sum = sum(w for w, _, _ in rows)
        if tw_sum <= 1e-12:
            return None
        t_mean = sum(w * t for w, t, _ in rows) / tw_sum
        th_mean = sum(w * th for w, _, th in rows) / tw_sum
        var_t = sum(w * (t - t_mean) ** 2 for w, t, _ in rows)
        cov_tth = sum(w * (t - t_mean) * (th - th_mean) for w, t, th in rows)
        if var_t <= 1e-12:
            return None
        return cov_tth / var_t

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

    def trajectory(
        self,
        horizon_s: float,
        sample_dt_s: float,
        *,
        use_ctr: bool = True,
        ctr_min_yaw_rate_rad_s: float = 1e-3,
    ) -> list[Tuple[float, float, float, Optional[float]]]:
        """SD-11a / SD-27a: multi-step extrapolation over [0, horizon_s].

        Walks forward from the most recent observation. Default model is
        constant-turn-rate (CTR) using the fellow's observed yaw rate from
        recent heading history; if yaw rate can't be estimated or is below
        ``ctr_min_yaw_rate_rad_s``, falls back to constant velocity (CV).

        CTR makes ZERO assumption about fellow being on a racing line —
        the propagated turn rate is whatever fellow's actual heading
        history shows. Pre-SD-27 (CV-only) smeared fellow off curving
        racing lines, opening fictional gaps that drove the strategy
        simulator to predict safe pass clearances that didn't exist.

        Args:
          horizon_s: total prediction horizon (seconds).
          sample_dt_s: sampling interval (seconds).
          use_ctr: when True (default), uses CTR if yaw rate is
            estimable; when False, always uses CV (legacy behaviour).
          ctr_min_yaw_rate_rad_s: yaw rates below this magnitude collapse
            to CV (avoids dividing by ~0). Default 1e-3 rad/s ≈ 0.06°/s.

        Returns a list of ``(t_offset_s, x, y, s_or_None)`` tuples with
        ``t_offset_s`` in {0, dt, 2*dt, ..., k*dt} where ``k = floor(horizon_s/dt)``.
        Length is ``k + 1``. ``samples[0]`` is the current observation (no motion);
        ``samples[i]`` for ``i >= 1`` is the extrapolated pose at ``i*dt``.

        Degenerate cases:
          - No history: returns ``[]``.
          - Single observation OR zero-length last segment: returns the
            current observation repeated at every sample (zero motion).

        Used by SD-11b's strategy simulator to predict where the fellow
        will be over the planning horizon.
        """
        if not self._hist:
            return []

        h = max(0.0, float(horizon_s))
        dt = max(1e-6, float(sample_dt_s))
        n_steps = int(h / dt)

        t_now, x_now, y_now, s_now = self._hist[-1]

        vx = 0.0
        vy = 0.0
        vs: Optional[float] = None
        if len(self._hist) >= 2:
            t0, x0, y0, s0 = self._hist[-2]
            t1, x1, y1, s1 = self._hist[-1]
            dt_seg = float(t1 - t0)
            if dt_seg > 1e-9:
                vx_seg = (x1 - x0) / dt_seg
                vy_seg = (y1 - y0) / dt_seg
                v_hist = self._recency_weighted_velocity_xy()
                if v_hist is not None:
                    w_hist = 0.35
                    vx = (1.0 - w_hist) * vx_seg + w_hist * v_hist[0]
                    vy = (1.0 - w_hist) * vy_seg + w_hist * v_hist[1]
                else:
                    vx = vx_seg
                    vy = vy_seg
                if s1 is not None and s0 is not None:
                    vs_seg = (float(s1) - float(s0)) / dt_seg
                    vs_hist = self._recency_weighted_velocity_s()
                    if vs_hist is not None:
                        w_hist_s = 0.35
                        vs = (1.0 - w_hist_s) * vs_seg + w_hist_s * vs_hist
                    else:
                        vs = vs_seg

        # SD-27a: estimate yaw rate from heading history; fall back to CV
        # if the estimate is too noisy / motion too small.
        yaw_rate: Optional[float] = None
        if use_ctr:
            yaw_rate = self._yaw_rate_from_history()
        v_speed = math.hypot(vx, vy)
        if (
            yaw_rate is not None
            and abs(yaw_rate) >= float(ctr_min_yaw_rate_rad_s)
            and v_speed > 1e-3
        ):
            theta0 = math.atan2(vy, vx)
            radius = v_speed / yaw_rate  # signed; negative for CW turns
        else:
            yaw_rate = None
            theta0 = 0.0
            radius = 0.0

        samples: list[Tuple[float, float, float, Optional[float]]] = []
        for i in range(n_steps + 1):
            t_off = i * dt
            if yaw_rate is not None:
                # CTR: circular arc from (x_now, y_now) at heading theta0,
                # speed v_speed, turn rate yaw_rate.
                angle = theta0 + yaw_rate * t_off
                x_i = float(x_now) + radius * (math.sin(angle) - math.sin(theta0))
                y_i = float(y_now) - radius * (math.cos(angle) - math.cos(theta0))
            else:
                x_i = float(x_now) + vx * t_off
                y_i = float(y_now) + vy * t_off
            s_i: Optional[float] = None
            if s_now is not None:
                if vs is not None:
                    s_i = float(s_now) + vs * t_off
                else:
                    s_i = float(s_now)
            samples.append((float(t_off), float(x_i), float(y_i), s_i))
        return samples

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


def format_prediction_log_line(sim_time_s: float, r: FellowPredictorStepResult) -> str:
    ps = _fmt_opt(r.fellow_pred_s)
    return (
        f"[Prediction] t={sim_time_s:.2f}s fellow_pred_x={r.fellow_pred_x:.4f} "
        f"fellow_pred_y={r.fellow_pred_y:.4f} fellow_pred_s={ps} "
        f"prediction_error_next_step={_fmt_opt(r.prediction_error_next_step)} "
        f"prediction_error_zero_motion={_fmt_opt(r.prediction_error_zero_motion)} "
        f"prediction_error_hold_last={_fmt_opt(r.prediction_error_hold_last)}"
    )
