# Coordinate frames in the dSPACE racing pipeline

This is the canonical reference for how coordinates flow through the Scenic ↔ dSPACE
racing stack. **Read this before touching `placement.py`, `readback.py`, or any TTL CSV.**

## TL;DR

```
        Scenic XODR (xy, m)         dSPACE RD (xyz, m)        dSPACE Frenet (s, t/d, m)
                  \                       |                              ↑
       XODR.geoReference                   | dSPACE GPS_CALC              | along centerline
       (+proj=tmerc, exact)                | (per-vehicle readback)       |
                    \                      |                              |
                     v                     v                              |
                     +---------> GPS (lon, lat, alt) <---------------------+
                                            ^
                                            |
                                     TTL CSV (xyz)
                                     Source files: lat/lon (ttl_optimal.csv)
                                     Projected:    XODR-xy (ttl_*_xodr.csv)
```

GPS is the canonical anchor. Every other frame is defined by its transform to/from GPS.
Bilateral conversions (e.g. XODR → RD) are compositions through GPS.

## The five frames

| Frame | Units | Source of truth | Live consumers |
|---|---|---|---|
| **GPS** | lat°, lon°, alt m (WGS84) | physical world | `read_ego_gps`, `read_fellow_gps` (`controldesk/readback.py:78–126`); source TTLs `ttl_optimal.csv` etc. |
| **Scenic XODR (xy)** | m | XODR `<header>` `<geoReference>` proj string | every `.scenic` file's `at (x, y)`; Scenic `Network`; `_road_index` |
| **dSPACE RD (xyz)** | m | dSPACE project's internal track origin | VEOS plant readback (`Pos_x/y/z_Vehicle_CoorSys_E`); fellow x/y arrays |
| **dSPACE Frenet (s, t)** | m | ModelDesk route reference line + lateral offset | ModelDesk placement (`seq.StartPosition = s`, `AdditionalLateralOffset = t`) |
| **TTL CSV (xyz)** | m or deg | Source GPS files OR XODR-projected files | MPC racing-line reference; `(s, t)` projection centerline |

## The conversions

### XODR-xy ↔ GPS

Defined by the XODR `<header>` `<geoReference>` proj string. For `LGS_v1.xodr`:

```
+proj=tmerc +lat_0=36.5869133 +lon_0=-121.755903 +k=1 +x_0=0 +y_0=0 +datum=WGS84
```

This is plain Transverse Mercator centered at the GPS origin. Use `pyproj.Transformer`
in both directions. **Exact, no calibration needed.**

`tools/frames/verify_xodr_rd_alignment.py` reads this proj string and exposes both
directions; you can copy that pattern into runtime code for Phase B.

### dSPACE RD ↔ GPS

dSPACE outputs GPS for every vehicle via `GPS_CALC`:

- Ego: `Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC/{Longitude_deg, Latitude_deg, Heading_deg}`
- Fellow: `Platform()://ASM_Traffic/Model Root/VesiInterface/VehicleSensors/ground_truth/GPS_POSITION/GPS_CALC/{Longitude_deg, Latitude_deg, Heading_deg}` (indexed by fellow index)

These come from dSPACE's internal track GPS metadata. To convert in the other direction
(GPS → RD), fit an affine transform from sampled `(GPS, RD)` pairs. The infrastructure
exists in `src/scenic/domains/racing/gnss_transform.py`:

- `GNSSLocalTransform` — affine transform with origin reference
- `fit_transform_from_csv` — calibrate from sample CSV
- `save_calibration` / `load_calibration` — persistence

**Whether a calibration is needed at all depends on Phase A.1.** If `LGS_v1.xodr`-xy
equals dSPACE-RD-xy (single-source), then the GPS→RD transform is identity and we
don't need to compose through GPS in the hot path. The verification utility writes
its conclusion at the end of every run.

### dSPACE RD ↔ Frenet (s, t/d)

`s` = arc length along the **ModelDesk route reference line** (R1 = Pit, R2 = Lap).
`t` (Scenic-side) and `d` (dSPACE-side) are **the same quantity**: signed lateral
offset from that reference line, positive = left of route direction, negative = right.

The reference lines are loaded from two CSVs:
- `assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv` — **ModelDesk R2 reference line**
- `assets/ttls/LS_ENU_TTL_CSV/ttl_pitlane.csv` — **ModelDesk R1 reference line**

> **What these files actually are** (verified 2026-04-26 via constant-d fellow-drive
> measurement on LGS_v1; see `tools/frames/measure_centerline_from_drive.py`):
>
> Despite the file names, these are **not geometric road centerlines** and **not the
> optimal racing line**. Track-width signature against race_common boundaries:
> total width = 11.9 m ± 1.9 m (matches the ~11.5 m race track) but asymmetry stdev
> = 7.95 m, range ±12 m — an alternating apex-cutting pattern characteristic of a
> racing line. They are also ~2 m offset from `ttl_optimal_xodr.csv`, so they are not
> *the* optimal line either.
>
> They are the **dSPACE ModelDesk R1 / R2 nominal route reference polylines** — a
> project-internal default driving path baked into the dSPACE/ModelDesk project,
> expressed in RD coordinates. Arc-length along these polylines IS the `s` value that
> `seq.StartPosition` expects.

**Why the empirical files are stable across XODR changes** (this is load-bearing):

The dSPACE RD frame is GPS-anchored at the dSPACE-project origin, **independent** of
any XODR file. ModelDesk R1/R2 are RD-coordinate polylines stored inside the dSPACE
project. Loading a different XODR (e.g. `LagunaSeca.xodr` → `LGS_v1.xodr`) changes
the XODR's `<geoReference>` GPS anchor and shape, but does NOT touch the dSPACE
project's GPS anchor or its routes. The pre-LGS_v1 empirical measurements were
verified bit-perfect against fresh LGS_v1 measurements (R2 p95 = 4 mm, R1 max = 11 mm
in the pit zone) — confirming the files still point at the same physical (lat/lon)
locations because the dSPACE-side GPS anchor never moved.

The (-6.101, -50.761) calibration is just the gap between the XODR's GPS anchor and
the dSPACE project's GPS anchor; both are fixed in physical space.

**Why XODR-derived files (`ttl_main_road_xodr.csv`, `ttl_pitlane_xodr.csv`) are NOT
drop-in replacements** for placement-time projection: they sample the OpenDRIVE road
reference (geometric center of the road), not a racing-line-shaped path. Mean offset
~3-4 m laterally vs ModelDesk R1/R2. They are correct for region/segment-map use
(B5) but cannot anchor `seq.StartPosition` without per-route s-offset calibration.

The **inverse** direction (Frenet → RD) is what ModelDesk does internally when given
`seq.StartPosition = s`. We don't invert it explicitly in our code.

**Re-measure when the dSPACE project changes routes** (not when the XODR changes):
- `examples/racing/calibration/measure_lgs_v1_centerline.scenic` — drives 5 R2 + 3
  R1 fellows at constant d; logs per-step RD readback to
  `tools/frames/data/lgs_v1_centerline_drive.csv`.
- `tools/frames/measure_centerline_from_drive.py` — extracts d=0 polylines and
  3-way diffs new vs old empirical vs XODR-derived.

### TTL CSV

There are three TTL CSV families in `assets/ttls/LS_ENU_TTL_CSV/`:

- **GPS source files** (`ttl_optimal.csv`, `ttl_left.csv`, `ttl_right.csv`, `ttl_pit.csv`):
  header `latitude,longitude,altitude`. Source of truth for the racing lines, authored once.
- **XODR-projected racing lines** (`ttl_optimal_xodr.csv`, `ttl_left_xodr.csv`,
  `ttl_right_xodr.csv`, `ttl_pit_xodr.csv`): header `x,y,z` in XODR-xy. Generated from
  the GPS sources by projecting through the current XODR's `<geoReference>`.
- **Centerline CSVs** (`ttl_main_road.csv`, `ttl_pitlane.csv` empirical OR
  `ttl_main_road_xodr.csv`, `ttl_pitlane_xodr.csv` derived): header `x,y,z`. Only
  these two are loaded by `placement.py` for the `(s, t)` projection.

The racing-line CSVs are consumed by the MPC controller; the centerline CSVs are
consumed by the placement-time projection only.

## Yaw conventions

Scenic: 0° = North = +y, increasing counterclockwise.
dSPACE: 0° = East = +x, increasing counterclockwise.

`placement.py:317, 321, 594, 599` apply `dspace_yaw = scenic_yaw - π/2` when sending
heading to ModelDesk. `controldesk/readback.py:287` normalizes the read-back yaw via
`atan2(sin, cos)`. These will be consolidated into `frames.yaw_scenic_to_rd` /
`frames.yaw_rd_to_scenic` in Phase B.

## The "weird offset" history

The pre-Phase-A workflow was:

1. Measure RD↔GPS empirically by driving and recording (RD-xy, GPS) pairs. Saved to
   `assets/maps/dSPACE/Laguna_Seca_transform.json`.
2. Measure the centerline empirically by driving along the route and recording xy.
   Saved to `ttl_main_road.csv` / `ttl_pitlane.csv`.
3. Project the GPS source TTLs to XODR-xy using the calibrated transform.

This worked for the OLD map (`LagunaSeca.xodr`, vendor=`Scenic_XODR_generation`) because
that map was *generated from* the empirically-measured TTLs — by construction the XODR-xy
frame equaled the dSPACE-RD-xy frame.

For the new map (`LGS_v1.xodr`, vendor=`MathWorks`), the XODR was generated independently
of dSPACE's measurement. Whether XODR-xy equals RD-xy is now an empirical question, not
a tautology. That's what Phase A.1 verifies. If equal, single-source assumption survives.
If not, GPS becomes the canonical anchor and we compose through it.

`Laguna_Seca_transform.json` and the `coordinate_transform.py` machinery built around it
are dead at runtime (`_coordinate_transform = None` in `simulator.py:656`); they are slated
for Phase C deletion.

## Phase A.1 calibration result (measured 2026-04-26)

| Property | Value |
|---|---|
| Model | pure translation (constant offset) |
| `translation_xy_xodr_to_rd` | `(-6.101, -50.761)` m |
| Single sample residual | 51.13 m magnitude |
| Stored at | `assets/maps/dSPACE/LGS_v1_gps_rd_calibration.json` |

**What this means**: dSPACE's internal RD origin is anchored at a slightly different GPS
point than `LGS_v1.xodr`'s `<geoReference>` declares. The OLD `LagunaSeca.xodr` was
auto-generated to match dSPACE's offset, so the single-source assumption held; the new
MathWorks XODR was generated to the canonical race_common origin, so a translation is
required when crossing the frame boundary.

The translation is applied in three live places (Phase A.6 / A.7):
- `placement.py` `place_ego` / `place_fellow`: XODR→RD before projecting onto the
  empirical centerline (which lives in RD frame).
- `controldesk/readback.py` `read_ego_state` / `read_fellow_state`: RD→XODR after reading
  back from dSPACE so `dspaceActor.position` is in XODR frame for visualization.
- `fellow/commands.py` `compute_fellow_ttl_geometric_d_m`: XODR→RD before projecting
  onto the centerline for the (v, d) plant fellow lateral controller.
- `vehicle/fellow_racing_line_lateral.py` `build_racing_line_delta_table`: optimal
  TTL points (NEW XODR frame) translated to RD before projecting onto the empirical
  centerline at table-build time. Cache key includes the XODR basename so multi-map
  runs don't share a stale table.

## Track-region visualization (Phase A.8)

`domains/racing/segments/track_regions.py` `build_track_regions_from_ttl` previously
defaulted to the empirical `ttl_main_road.csv` / `ttl_pitlane.csv` for the `mainTrack` /
`pitTrack` PolygonalRegion that the Scenic --2d viewer renders as the drivable area.
Two issues with that:

1. The empirical `ttl_main_road.csv` is in dSPACE RD frame, not XODR frame, so the
   polygon was drawn ~51 m offset from the LGS_v1 XODR map.
2. The empirical path is NOT actually a centerline — its distance to the race_common
   inside boundary varies from 1.9 m to 55 m depending on location. Buffering it ±6 m
   produces a lopsided polygon, not the actual track outline.

Phase A.8 changes the default so `build_track_regions_from_ttl` prefers
`ttl_main_road_xodr.csv` / `ttl_pitlane_xodr.csv` (XODR-derived true centerline,
NEW XODR frame, ±6 m buffer matches the race_common ~12 m track width) when present,
falling back to the empirical files if not. This makes the viewer's drivable-area
overlay align with the XODR map and with ego's position.

The XODR-derived centerlines are produced once via
`tools/frames/derive_centerline_from_xodr.py` from `LGS_v1.xodr`'s `<planView>`
`<line>`/`<arc>`/`<spiral>` primitives.

## race_common ground-truth track boundaries

For sanity-checking placement against the canonical track geometry, the race_common
geofence CSVs are mirrored into the assets folder:

- `assets/ttls/LS_ENU_TTL_CSV/track_inside.csv` (4964 pts)
- `assets/ttls/LS_ENU_TTL_CSV/track_outside.csv` (5000 pts)
- `assets/ttls/LS_ENU_TTL_CSV/pit_inside.csv` (4497 pts)
- `assets/ttls/LS_ENU_TTL_CSV/pit_outside.csv` (5000 pts)

All four are in the canonical `LS_ENU` (East-North-Up) frame, anchored at GPS
`(36.5869133, -121.7559026, 231.9349051)` — essentially the same origin as
`LGS_v1.xodr`'s `<geoReference>`. So **ENU ≈ XODR** for these files. To compare
against dSPACE RD-frame data, translate by `(-6.101, -50.761)` (ENU → RD); otherwise
distances appear ~50 m larger than reality. Source:
`/home/bklfh/ros_ws/race_common/src/external/common/race_metadata/geo_fences/LS_ENU_TTL_CSV/`.

`tools/frames/check_ego_in_bounds.py` parses an F-bank log file and reports per-sample
distance to the inside/outside boundaries plus an in-track flag. Used to verify the
Phase A.6/A.7 fixes against ground truth.

## Corridor-aware MPC (Phase B - 2026-04-26)

### Why
The new race_common-derived TTLs cut close to the curb on corner exits because they're a
"max-speed optimal racing line." A pure line-tracking MPC follows the line exactly, so the
ego's car body extends off-track in tight sections (proven via F0 BoundsCheck: racing line
at `d_out=2.06m` vs IAC car half-width `0.965m` -> body at `d_out=1.10m`, off the curb).

ART/race_common's Frenet planner doesn't have this problem because it uses the racing line
as a *reference* and applies an exponential boundary barrier
(`exp(-q * dist / half_width)`, `q=4.0`) plus a hard geofence check. The racing line is
the ideal but the planner pulls inward when the body would go off-track.

### How (Scenic-side implementation)
The Scenic MPC is OSQP-based (pure quadratic). race_common's exponential barrier doesn't
fit directly, so we use a **quadratic surrogate** with the same effect:

1. Per-step, compute corridor midpoint offset from the racing line:
   `mid_offset_k = (d_left_k - d_right_k) / 2.0`
   (sign matches Scenic e_y: positive = left of racing line)
2. Per-step, compute "safe half-corridor" — how much e_y excursion the car body can
   tolerate before hitting the safety threshold:
   `safe_half_k = (d_left_k + d_right_k)/2 - vehicle_half_width - safety_margin`
3. Per-step weight scales with inverse-square of safe-half:
   `w_b_k = min(w_corridor_max, w_corridor_base / safe_half_k**2)`
   - When `safe_half_k` is large (line centered in wide corridor), `w_b_k` is tiny -> racing line wins.
   - When `safe_half_k` is small (line near boundary), `w_b_k` dominates -> pulls toward midpoint.
4. Add quadratic cost: `w_b_k * (e_y_k - mid_offset_k)**2` to the OSQP P matrix and q vector.

This is purely quadratic (constants per step), slots into the existing QP without slack
variables or constraint changes. It mirrors race_common's behavior to first order.

### Where (file map)
| Concern | File | Notes |
|---|---|---|
| race_common 20-column TTL format | `assets/ttls/LS_ENU_TTL_CSV/ttl_*_xodr_full.csv` | mirrored from `/home/bklfh/ros_ws/race_common/.../race_metadata/ttls/LS_ENU_TTL_CSV/`; ttl_17 -> optimal, ttl_2g -> left, ttl_9g -> right, ttl27 -> pit |
| TTL column enum | `src/scenic/simulators/dspace/ttl/loader.py` `TtlColumn` | mirrors race_common's `ttl.hpp` |
| Full-format loader | `src/scenic/simulators/dspace/ttl/loader.py::load_ttl_full` | returns dict with racing line + bounds + speed/curvature + metadata; None if format is just 3-col x,y,z |
| Auto-pickup of `_full.csv` sibling | `src/scenic/simulators/dspace/ttl/loader.py::_autodetect_full_ttl_filename` | scenarios that point at `ttl_optimal_xodr.csv` get the corridor MPC for free if `ttl_optimal_xodr_full.csv` exists next to it |
| Per-waypoint bound distances on object | `attach_ttl` -> `obj.ttl_left_dist_m`, `obj.ttl_right_dist_m` | euclidean distance from racing line point to LEFT/RIGHT bound point |
| Per-horizon interpolation helper | `src/scenic/domains/racing/mpc/mpc_lateral.py::_interpolate_bounds_at_horizon` | linear interp on cumulative arc-length, periodic wrap |
| MPC cost addition | `src/scenic/domains/racing/mpc/mpc_lateral.py::_build_qp_matrices` | accepts `left_dist_horizon`/`right_dist_horizon` kwargs; identity if None or feature disabled |
| Config knobs | `src/scenic/domains/racing/mpc/config.py` | `corridor_barrier_enabled`, `corridor_barrier_weight`, `corridor_safety_margin_m`, `vehicle_half_width_m`, `corridor_barrier_weight_max` |
| Behavior call site | `src/scenic/domains/racing/behaviors.scenic` `lat_controller.run_step` (line 1772) | passes `left_dist_per_wp` / `right_dist_per_wp` from object attrs |

### Tunable parameters (defaults, post-deadzone fix)
- `corridor_activation_clearance_m = 1.5` -- body clearance above which the cost is fully off (hard deadzone). On a typical 12 m race track most of the lap has clearance > 1.5 m, so the racing-line tracker wins exactly. Below this threshold, weight ramps up cubically.
- `corridor_barrier_weight_max = 5000.0` -- weight at zero body clearance.
- `vehicle_half_width_m = 0.965` -- IAC AV-21 (1.93 m / 2).

### Post-mortem: Phase B corridor cost interaction with longitudinal MPC
The original Phase B formula (`w_b_k = w_base / safe_half_k**2`) fired everywhere because even on wide straights `w_b_k` was comparable to `w_ey`. CTE drifted to ~+1.2 m, longitudinal MPC's wp_last_idx tracking got thrown off, ego braked spuriously and stayed at ~12 m/s. Replaced with hard-deadzone cubic ramp -- straights now identity, corridor cost only fires on corner exits where the line genuinely approaches a curb. Speed regression resolved.

### Status (as of 2026-04-26)
Currently inactive (Path C is in effect): F-bank scenarios use the OLD `_og` racing lines (translated to NEW XODR frame) which don't have a `_full.csv` sibling, so the loader auto-pickup doesn't fire and the MPC's corridor barrier silently stays off (identity). The corridor MPC code path is present and tested; it activates if a `_full.csv` sibling is restored.

## XODR-native track regions (Phase B.5 - 2026-04-26)

`mainTrack` and `pitTrack` PolygonalRegions are now derived directly from XODR road geometry by default (was: empirical centerline TTL CSVs buffered by ±6 m / ±1.5 m).

### Why
- The empirical `ttl_main_road.csv` and `ttl_pitlane.csv` came from driving sessions in dSPACE -- slow to produce, brittle when the map changed, and not actually a geometric centerline (varied 1.9-55 m from race_common's inner edge).
- LGS_v1.xodr has all the road geometry needed; using it as the source of truth means changing the map automatically updates the regions.
- Verified XODR-derived `mainTrack` matches race_common's `outside.csv` polygon within **0.83 m mean / 3.33 m max** at sampled boundary points; areas agree to 99.9% (XODR 21,764 m² vs TTL 21,739 m²).

### Implementation
- `domains/racing/segments/track_regions.py::create_track_regions` switched default to `build_track_regions_from_opendrive`. Legacy TTL path retained as fallback when XODR missing or `prefer_ttl_track_regions=True`.
- `domains/racing/model.scenic` exposes `param preferTtlTrackRegions = False` for opt-back-in.
- `simulators/dspace/simulator.py` `createEgoInSimulator` now gates `scene._main_ttl_waypoints` / `_pit_ttl_waypoints` setattr on the same flag. With default `False`, the behavior's segment-map builder falls through to `build_waypoint_segment_map(wp_list, track)` (XODR-native).

### What's still TTL-dependent (out of B.5 scope)
- `placement.py::_route_pref_from_ttl_distances` uses `ttl_main_road.csv` / `ttl_pitlane.csv` to detect Lap vs Pit at placement time.
- (s, t) projection at placement uses `_road_index_ttl` built from the same empirical CSVs. Affects the `s` value sent to ModelDesk.

These could move to XODR-native too, but require care because the `s` value sent to ModelDesk needs to align with dSPACE's internal route reference. Deferred to Phase C cleanup.

### Verification
`tools/frames/verify_xodr_native_regions.py` -- run anytime to confirm XODR-derived regions still match race_common bounds.

## File map

| Concern | File |
|---|---|
| GPS↔XODR projection (proj string) | `LGS_v1.xodr` `<geoReference>` |
| GPS↔RD calibration (if needed) | `assets/maps/dSPACE/LGS_v1_gps_rd_calibration.json` (Phase A) |
| Affine transform class | `src/scenic/domains/racing/gnss_transform.py` |
| Calibration fit script | `src/scenic/domains/racing/ttl_processing/fit_gnss_rd_calibration.py` |
| Centerline derivation utility | `tools/frames/derive_centerline_from_xodr.py` |
| Alignment verification utility | `tools/frames/verify_xodr_rd_alignment.py` |
| Ego-start derivation utility | `tools/frames/derive_ego_start.py` |
| Placement-time (s, t) projection | `src/scenic/simulators/dspace/geometry/route_projection.py` |
| ModelDesk placement | `src/scenic/simulators/dspace/modeldesk/placement.py` |
| Ego/fellow readback | `src/scenic/simulators/dspace/controldesk/readback.py` |
| Phase B target consolidation | `src/scenic/simulators/dspace/geometry/frames.py` (NEW) |
| Elevation backfill (RC-Z) | `tools/frames/add_elevation_from_race_common.py` |
| Elevation reference data | `tools/frames/data/race_common_ttl_17.csv` (race_common's 20-col ttl_17 vendored locally) |

## Note on TTL z-column (RC-Z, 2026-04-26)

The `ttl_*_xodr.csv` files now carry **real elevation** in the z column
(range -5.33 to +49.54 m, 54.86 m vertical span — Corkscrew alone drops ~18 m).
Previously z was near-flat (~0.06 m range) which silently no-op'd the longitudinal
MPC's grade compensation (`gravity_force = mass * g * sin(grade)` at
`mpc_longitudinal.py:399`). The file format is unchanged (still 3-column
`x,y,z`); user-authored TTLs are not required to provide rich data, but the
controller will use whatever is provided. See
[`docs/racing_controller_cleanup.md`](racing_controller_cleanup.md) for the
broader RC-* cycle context.
