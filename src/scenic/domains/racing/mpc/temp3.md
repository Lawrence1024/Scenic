### 3) Cache `waypoints_for_mpc` conversion in `racing/behaviors.scenic`

You currently rebuild the tuple list every control tick:

```python
waypoints_for_mpc = [(float(wp[0]), float(wp[1]), ... ) for wp in wp_list]
```

That’s pure Python allocation overhead on a hot path.

### Low-risk optimization

Cache the converted waypoint list and only rebuild if the source list changes.

#### Suggested pattern

```python
# Before converting
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
```
