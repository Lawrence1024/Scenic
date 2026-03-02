### File: `racing/behaviors.scenic`

### Around line ~1067–1118

You currently do:

```python
for i in range(nwp - 1):
    wp0 = wp_list[i]
    wp1 = wp_list[i + 1]
    ...
self._wp_seg_grade = np.asarray(seg_grade)
self._wp_cumdist_3d = cum  # length nwp, built from nwp-1 segs
...
base_idx = max(0, min(wp_last_idx, len(wp_list) - 2))
...
seg_idx = np.clip(..., base_idx, len(self._wp_seg_grade) - 1)
grade_profile = self._wp_seg_grade[seg_idx]
```

This is also open-path and will break at the seam.

### Fix grade segments to length `nwp` using wrap

Replace the loop with:

```python
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
```

### Fix grade lookahead selection to wrap

Replace the base/clamp/searchsorted chunk with:

```python
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
```