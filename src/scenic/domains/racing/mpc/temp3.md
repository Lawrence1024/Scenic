## 2A) Fix curvature cache to be closed-loop

### File: `racing/behaviors.scenic`

### Around line ~1010–1047 (the `_wp_curve_vmax` and `_wp_cumdist_xy` usage)

You currently compute curvature vmax with special casing:

```python
if i == 0 or i == nwp - 1:
    curve_vmax_list.append(1e6)
else:
    p0 = wp[i-1]; p1 = wp[i]; p2 = wp[i+1]
```

**Replace the entire curvature loop with modulo neighbors (closed loop):**

```python
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
```

## 2B) Fix `_wp_cumdist_xy` to include the last→first segment and wrap lookahead distances

Wherever you build `_wp_cumdist_xy` (it’s used here):

```python
base_idx = max(0, min(wp_last_idx, len(wp_list) - 2))
base_cum = self._wp_cumdist_xy[base_idx]
target_abs = base_cum + dist_vec
wp_end_idx = np.searchsorted(self._wp_cumdist_xy, target_abs, side='right') - 1
```

This is **open-path logic**. Replace it with **loop logic**:

### Build closed-loop cumulative distance once

When you (re)build the XY cache, build segment lengths for *all* segments including the last→first:

```python
nwp = len(wp_list)
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
```

### Use modulo distances for lookahead

Replace the non-wrapping portion with:

```python
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
```
