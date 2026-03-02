### File: `racing/behaviors.scenic`

### Around line ~521–537 (your init “first waypoint AHEAD” search)

**Current code (non-wrapping, clamps at end):**

```python
for i in range(nearest_idx, min(len(wp_list_init), nearest_idx + 100)):
    wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
    ...
    if dot_product > 0:
        wp_last_idx = i
        ...
        break
else:
    wp_last_idx = nearest_idx
```

**Replace with (wraps properly):**

```python
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
        print(f"[FollowRacingLineMPCBehavior] Initialized: starting at ({px:.2f}, {py:.2f}), heading={car_heading*180/math.pi:.1f}deg (src={car_heading_src})")
        print(f"  Found first waypoint AHEAD: index={wp_last_idx} at ({wx:.2f}, {wy:.2f}), distance={wp_dist:.2f}m")
        print(f"  Dot product={dot_product:.2f} (positive means ahead)")
        break
else:
    wp_last_idx = nearest_idx
    print(f"[FollowRacingLineMPCBehavior] Warning: No waypoint ahead found in search window, using nearest waypoint {nearest_idx}")
```