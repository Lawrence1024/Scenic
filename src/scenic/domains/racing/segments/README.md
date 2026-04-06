# Racing library structure and segments

This README describes the **generic structure of the racing library** and the role of **track segments** within it. Segments provide waypoint-to-segment mapping and curve/straight (or conventional) labeling used by behaviors and MPC.

---

## Racing library structure (overview)

The racing domain (`scenic.domains.racing`) extends the driving domain with closed-loop circuits, pit lanes, racing controllers, and a single control contract so that behaviors, MPC, and simulators do not give contradicting instructions.

```
domains/racing/
├── constants.py              # Single source of truth: DELTA_MAX_RAD, THETA_SW_MAX_DEG, R
├── README.md                  # Racing domain reference (includes control contract)
├── behaviors.scenic          # FollowRacingLineMPCBehavior (MPC line follow), fellow (v,d) plants, etc.
├── actions.py                # Racing actions (SetMaxSpeed, SetTTL, SetGear, …)
├── simulators.py             # RacingSimulation: getRacingControllers(use_mpc=…), sets _racing_steer_units
├── model.scenic, tracks.py   # RacingCar, RacingTrack, regions
├── segments/                  # This folder: waypoint–segment map, curve/straight or conventional labels
│   ├── segment_map.py        # build_waypoint_segment_map, get_segment_at_waypoint, get_segment_label
│   ├── visualize_racing_segments.py
│   └── README.md
└── mpc/                       # MPCC lateral + longitudinal MPC; single owner of lateral clamp/rate limit
    ├── mpc_lateral.py, mpc_longitudinal.py, reference_builder.py, config.py, …
    └── README.md              # MPC-focused documentation
```

**Control flow (short):** Behaviors get controllers from `getRacingControllers(agent, use_mpc=True|False)`. PID path uses normalized steering [-1, 1]; MPC path uses road wheel angle in rad. Constants live in `constants.py`; rad→dSPACE conversion only in the simulator’s `steer_io`. See [racing README – Control contract](../README.md#control-contract).

**Where segments fit:** Behaviors (e.g. `FollowRacingLineMPCBehavior`) and MPC use waypoints and a **segment map** to know which track segment each waypoint belongs to. That supports logging (e.g. “segment 6 curve”), per-segment analysis, and MPC reference continuity (segment selection, gate, stick). Segment map is built from the same waypoints/centerline used for the racing line; it does not drive control by itself—it labels and indexes.

---

## Segments: purpose and contents

- **Deterministic segment IDs:** For the same map and curvature threshold, segment boundaries and IDs are the same every run. Enables comparing runs and TTLs by segment (e.g. “segment 6 curve”).
- **Waypoint-to-segment mapping:** Each waypoint gets a `(segment_id, segment_name)` used in behaviors and in MPC (e.g. segment map passed into lateral MPC for reference building and logging).

| File | Role |
|------|------|
| `segment_map.py` | Core logic: `build_waypoint_segment_map` (OpenDRIVE), `build_waypoint_segment_map_from_ttl` (TTL — same pipeline as visualization); curve/straight via `_build_curve_straight_segments`; overlap rule for TTL (main wins); `get_segment_at_waypoint`, `get_segment_label`. |
| `visualize_racing_segments.py` | Standalone script: OpenDRIVE or `--ttl-folder PATH`. TTL: one overlaid plot (main full, pit pit-only; overlap removed from pit). Same segmentation and overlap rule as `build_waypoint_segment_map_from_ttl`. |
| `__init__.py` | Re-exports public API: `from scenic.domains.racing.segments import build_waypoint_segment_map`, etc. |

---

## Segment modes

1. **Curve/straight (default)**  
   Derived from centerline curvature. Where curvature exceeds `CURVATURE_THRESHOLD` (~0.015 1/m), the track is labeled “curve”; otherwise “straight”. Each segment name is prefixed by road type: **main straight**, **main curve**, **pit straight**, **pit curve**. Consecutive same-type regions are merged. Yields many segments for fine-grained analysis and pit speed enforcement.

2. **Conventional Laguna Seca**  
   When `use_curvature_segments` is False and `use_conventional_laguna` is True and the track has two main roads, fixed named sections (Front Straight+T1, Andretti Hairpin, Corkscrew, etc.) are used.

3. **Coarse**  
   If neither applies, one segment per main racing road (segment id only, no name).

---

## Building the segment map

1. **OpenDRIVE:** `build_waypoint_segment_map(waypoints, track, ...)` — segments from the map’s centerlines and ring topology.

2. **TTL waypoints:** `build_waypoint_segment_map_from_ttl(main_ttl_waypoints, pit_ttl_waypoints=None, waypoints=None, ...)` — segments from TTL polylines only. Uses the **same pipeline as the visualization**: build polyline(s) from waypoints, segment each with `_build_curve_straight_segments` (same `CURVATURE_THRESHOLD`, `MIN_SEGMENT_LENGTH`, `MAX_STRAIGHT_ABSORB_INTO_CURVE`). In overlap (e.g. Andretti, Corkscrew) waypoints are assigned to **main** (`TTL_OVERLAP_MAIN_WINS_M`), so segment boundaries and main-vs-pit match what you see in `visualize_racing_segments --ttl-folder`. Behaviors use this when the scene has `_main_ttl_waypoints` (set by the simulator when `ttlFolder` is set).

Both return `_SegmentMapWithSequences`; `get_segment_at_waypoint`, `get_ring_segment_ids`, and behaviors work unchanged.

**TTL pipeline (same in visualization and Scenic):**
1. Build main polyline from `main_ttl_waypoints`, pit polyline from `pit_ttl_waypoints` (`_ttl_polyline_from_waypoints`).
2. Segment each polyline with `_build_curve_straight_segments` (same `CURVATURE_THRESHOLD`, `MIN_SEGMENT_LENGTH`, `MAX_STRAIGHT_ABSORB_INTO_CURVE` as OpenDRIVE).
3. Overlap rule: waypoint within `TTL_OVERLAP_MAIN_WINS_M` of main (or closer to main than pit) → assign to main segment (Andretti, Corkscrew count as main); else assign to pit segment.
4. Visualization draws main in full and pit only where pit-only (overlap removed from pit), so what you see matches the segment map used by behaviors when `ttlFolder` is set.

---

## Usage

- **From Scenic (e.g. behaviors):**  
  `from scenic.domains.racing.segments import build_waypoint_segment_map, build_waypoint_segment_map_from_ttl, get_segment_at_waypoint, get_segment_label`  
  Use `build_waypoint_segment_map(wp_list, track)` when you have an OpenDRIVE track, or `build_waypoint_segment_map_from_ttl(main_ttl, pit_ttl, waypoints=wp_list)` when using TTL waypoints (when `ttlFolder` is set). Then `get_segment_at_waypoint(wp_idx, segment_map)` / `get_segment_label(...)` for logging or MPC.

- **Visualization:**  
  From repo root:  
  `python -m scenic.domains.racing.segments.visualize_racing_segments [--map PATH]` or `--ttl-folder PATH` for TTL (one overlaid plot: main full, pit pit-only).  
  `python -m scenic.domains.racing.segments.visualize_ttl_segments [--ttl-folder PATH] [-o FILE]` for TTL-only (two subplots).

---

## Notes

- **Main racing roads** include ordinary roads plus **junction connecting roads** for the outer loop. If you do not set `main_loop_connecting_road_ids`, at each junction the code picks the connecting road that **smoothly** continues the main loop (smallest total angle change at the two connection points). If you set `main_loop_connecting_road_ids=(24, 34)` (OpenDRIVE IDs), those links are used instead.
- **Pit lane** links: if you do not set `pit_connecting_road_ids`, at each junction any connecting road that has the pit road in its junction endpoints is taken as the pit link. If you set `pit_connecting_road_ids=(25, 30)`, those links are used instead.
- The segment map **always includes main and pit roads** so segment labels are consistent for both lap and pitlane TTLs.
- Only `(x, y)` from waypoints is used; projection onto the nearest road centerline (main or pit) gives arc length `s` for segment lookup.
- Segments are **map-based:** same map and threshold → same segments across runs.

---

## Related documentation

- [Racing README – Control contract](../README.md#control-contract) – Steering units, constants, simulator contract.
- [../README.md](../README.md) – Full racing domain reference (objects, actions, behaviors, simulator implementation).
- [../mpc/README.md](../mpc/README.md) – MPC formulation, config, and how MPC uses segments/reference.
- [simulators/dspace/README.md](../../../simulators/dspace/README.md) – dSPACE integration (placement, steering IO, control application).
