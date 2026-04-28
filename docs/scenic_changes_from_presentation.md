# Code-change list surfaced while preparing the Scenic core review presentation

A running ledger of code warts and design questions that came up while
writing the talk. None of these are blockers for delivering the talk;
they're follow-up work where rehearsing the explanation forced an
honest look at the current implementation.

Ordered roughly by how badly they bite. Each item: the current
behavior, why it's wrong (or worth questioning), and the rough fix
shape. Resolved items move to the `## Done` section at the bottom so
the numbered list stays useful as a forward-looking ledger.

---

## 1. mainTrack / pitTrack don't use XODR road width

*Resolved in SD-19a. See `## Done` section below.*

---

## 2. ttlRegion buffer is too large for "place on TTL" semantics

*Resolved in SD-19b. See `## Done` section below.*

---

## 3. VerifAI monitors read parsed stdout, not in-memory state

**Current:** the falsification loop in
`src/scenic/domains/racing/benchmarks/verifai_runner.py:main()` runs
the simulator, captures stdout to `logs/sample_NNN.log` via a `_Tee`
context manager, then calls `parse_sample()` from
`sampled_runner.py` to regex-extract a `SampleMetrics` dataclass.
Monitor functions in `monitors.py` consume that dataclass and emit
a robustness scalar, which the runner feeds back to VerifAI as
`feedback` for the next sample.

**Why it's worth questioning:** the contract between the simulator
and the monitor is **the stdout log format**. If anyone changes a
log line — adds a field, renames a key, drops debounce noise — our
monitors silently break (they regex-match against literal patterns
like `[EvalGT]`, `[Commit]`, `[BoundsCheck]`).

This is also not how VerifAI is *intended* to be used. VerifAI's
clean pattern is:
- The simulator emits per-step data into an in-memory structure
  (Scenic exposes `simulation.records` for this).
- Monitor functions read directly from `simulation.records`,
  return robustness.
- No string parsing. The contract is a Python data structure.

**Fix shape:** route the per-tick eval signals through
`simulation.records` (Scenic's `record` clause / the `simulation.result.records`
dict on the Python side). The emitting code is in
`src/scenic/domains/racing/tactical_planner.py` and
`src/scenic/domains/racing/assessment/pass_geometry.py` — both
currently `print()` to stdout. Adding a parallel `simulation.record_event(...)`
call (or a simulation-level callback) keeps the human-readable log
AND gives monitors a structured channel.

Then `monitors.py` can drop its dependency on `parse_sample` and
read directly from `simulation.records`.

**Why we hadn't done it:** speed-of-delivery for SD-15/16. Log
parsing was already in `sampled_runner.py` and we wanted the
verifai_runner to share parsing infrastructure. Now that the
pipeline is landed, the in-memory route is a clean follow-up.

---

## 4. Corner-segment placement specifier missing

**Symptom:** `examples/racing/f_shared/F8_corner_entry_fellow_ahead_optimal.scenic`
and `F10`/`F11`/`F12` are nearly byte-identical to `F6`/`F7` —
**only the ego start coordinates differ**. The user can't write
"put ego at a corner entry" abstractly; they have to look up the
corner's (x, y) and hardcode it.

**Why this matters:** falsification over corner-vs-straight
behavior is a natural axis (we should be able to vary the corner
the layout starts in). Today that's a manual scenario duplication
per corner.

**Fix shape:** a `CornerSegment` region (or a more general
`Segment(curvature_range=..., length_range=...)`) that classifies
roads by curvature and exposes them as Region. Then:
```
ego = new RacingCar on cornerSegment(curvature=Range(0.05, 0.15))
```

The classifier already exists in
`src/scenic/domains/racing/segments/` (the `RaceSegment` machinery
in `segments/tracks.py` and the segment-aware planner code). What's
missing is exposing the classifier outputs as Scenic regions.

---

## 5. RacingTrack isn't directly usable as a Scenic Region

*Resolved in SD-19c. See `## Done` section below.*

---

## 6. BoundsCheck off-track measurement is unreliable

*Resolved in SD-18a. See `## Done` section below.*

---

## 7. verifai_runner should print sample-progress to the terminal even with `--quiet *>file`

*Resolved in SD-18b. See `## Done` section below.*

---

## 8. Behavior phase ordering (no change needed -- documentation only)

The 8-phase tick in `behaviors.scenic` runs in this order (the order
the user questioned during the presentation prep, then confirmed):

1. State read
2. Waypoint progress (advance index on current TTL -> gives ego `s`)
3. Tactical planner (uses `s`; picks TTL + speed cap)
4. MPC reference build (chosen-TTL waypoints over horizon)
5. Lateral MPC
6. Longitudinal MPC
7. Gear logic
8. Safety gates -> action emit

The planner runs BEFORE the MPC reference build because the planner
chooses WHICH TTL the MPC should track. This is correct as-is. The
deck initially had the order wrong; fixed in the slide-15 edit pass.

No code change required. Recording here so future reads of `behaviors.scenic`
don't regress this conventionally-good architecture.

---

## Items to revisit later (less urgent)

- **`fuelLevel` and `tireWear` on `RacingCar` are extension points
  with no live behavior.** Honest in the talk; might as well
  comment them out or move to a separate `EnduranceRacingCar`
  subclass to clarify scope.
- **MPC gear logic is rule-based, not part of the QP.** A hybrid
  MPC formulation (continuous throttle + discrete gear) would be
  cleaner. Out of scope for this cycle.
- **Falsification campaign size.** The 10-sample CE result is
  preliminary; rerun with 100+ samples to confirm the 22-23 m
  cluster is robust across seeds.

---

## Done

### #6 — BoundsCheck off-track measurement is unreliable (SD-18a)

**What was wrong.** The 50-sample CE campaign at
`results/verifai_20260428_052048/` flagged 50/50 samples as
off-track despite the dSPACE viewer showing ego visually inside
the track surface throughout. Two compounding issues:

- **Issue A — frame mismatch.** Every `[BoundsCheck]` line carried a
  ~10 m residual between the simulator's `pos=(...)` (derived from
  dSPACE actor position via `readback.rd_to_xodr` with the empirical
  `(-6.101, -50.761)` translation in `frame_calibration.py`) and the
  GPS-derived `xodr_from_gps=(...)` (from `pyproj` projection through
  the XODR's `<geoReference>`). Constant magnitude + constant
  direction = frame-transform offset, not noise. The residual pushed
  the ego position spuriously across the inner geofence.

- **Issue B (NOT touched in this fix) — `d_in=0` semantics.**
  Investigation showed the inside/outside CSVs ARE physical track
  edges (11-28 m apart), so `d_in=0` correctly means "ego touched
  the inner boundary." With Issue A fixed, `d_in=0` events are real;
  no semantic change needed.

**What landed.** `simulator.py` now prefers
`gps_to_xodr(lon, lat, xodr_path)` as the input to
`compute_bounds_distance`. The pyproj path through
`<geoReference>` is canonical by construction (no empirical fit).
The readback-derived `(_x, _y)` stays as a fallback when GPS is
unavailable. The `[BoundsCheck]` log line gained a `src=gps|readback`
field and now reports `readback_xy=(...)` + residual diagnostics.

**Verification notes.** Re-run the 50-sample campaign and confirm
`summary.csv` shows non-100% off-track. Spot-check at least 3
samples by replaying logs against the dSPACE viewer.

### #7 — verifai_runner sample-progress visibility under `--quiet *>file` (SD-18b)

**What was wrong.** `python verifai_runner.py ... --quiet *>run.log`
captured all `[VerifaiRunner] sample N/M` boundaries inside the log
file; the terminal showed nothing during the ~80-minute run.

**What landed.** Added a `_progress(text)` helper that writes through
`sys.__stdout__` with `flush()`. PowerShell `*>` redirects only
`sys.stdout` / `sys.stderr`; the original FD at `sys.__stdout__` is
untouched, so writes through it remain visible on the terminal even
under redirection. All 22 `[VerifaiRunner]` status prints (setup
banner, sample boundaries, sampled values, per-sample outcome,
violations, circuit-breaker, campaign-end summary) routed through
`_progress`. Per-sample simulator stdout (the heavy log content) is
unchanged — still flows through `_tee_stdout` to the per-sample log
file.

**Verification notes.** Run a 3-sample CE smoke with
`--quiet *>run.log`. Terminal should show
`[VerifaiRunner] === sample 1/3 ===` etc. live; `run.log` should be
~50 lines (no per-sample sim noise).

### #1 — mainTrack/pitTrack now use XODR road polygons (SD-19a)

**What was wrong.** `create_track_regions()` built `mainTrack` /
`pitTrack` by buffering each road's **centerline** by ±6 m / ±1.5 m.
This discarded XODR's per-station lane widths entirely and
treated the LGS track as if it were 12 m wide everywhere
(roughly right on straights, wrong at corner apexes).

**What landed.** `build_track_regions_from_opendrive` in
`track_regions.py` now defaults to using `Road.polygon` directly
(each `Road` inherits from `NetworkElement -> PolygonalRegion`,
so the polygon already encodes the full drivable width from XODR).
We just call `PolygonalRegion.unionAll(main_loop_roads)` and
`PolygonalRegion.unionAll(pit_roads)`, then apply the
main-wins-on-overlap rule via `pit.difference(main)`. The
buffer arguments are kept for back-compat but ignored on the
polygon path. The legacy centerline-buffer path remains as
fallback inside the same function (gated by `use_road_polygons=False`
or by polygon-side exception).

The startup log now reads:
`[TrackRegions] mainTrack/pitTrack source = XODR road polygons (width-aware drivable area; centerline-buffer fallback unused)`

**Verification notes.** Re-run F-bank or any sampled scenario; the
startup log should show the new line. Visually, mainTrack at
corner apexes should now narrow correctly instead of staying 12 m
wide. `examples/racing/sampled/smoke_on_mainTrack.scenic` is the
quickest sanity check (no simulator needed).

### #2 — ttlRegion returns PolylineRegion (SD-19b)

**What was wrong.** `ttlRegion(file)` returned a buffered
PolygonalRegion (±6 m around the racing-line centerline). The
per-vehicle TTL placement default
(`position: new Point on ttlRegion(self.ttlFileName)` on
`RacingCar`) thus sampled in a 12 m-wide envelope, not on the line.

**What landed.** `create_ttl_region_from_file` in `track_regions.py`
returns the underlying `PolylineRegion` directly (no buffer). The
`ttlRegion` helper in `model.scenic` and the `_ttl` global both
updated to drop the buffer arg. Each `new Point on ttlRegion(...)`
sample now lands EXACTLY on the racing-line waypoint chain.

**Lateral wiggle.** Without a buffer, the ego always starts
mathematically on the line. If we want a small lateral
randomization for falsification diversity, that's a future
Frenet-offset feature (Ask 1 in the deck — Scenic-core proposal).

**Verification notes.** Compile any scenario using a TTL placement
and inspect the sampled (x, y) — they should sit within
~0.001 m of the closest TTL waypoint, not scattered ±6 m.

### #5 — raceTrack alias for `new RacingCar on raceTrack` (SD-19c)

**What was wrong.** `RacingTrack` (in `segments/tracks.py`) was a
Python wrapper class around Scenic's `Network`. `_track` in
`model.scenic` referenced an instance, but `_track` was not a
`Region`, so `new RacingCar on _track` failed.

**What landed.** Added a top-level `raceTrack` alias in
`model.scenic` defined as `UnionRegion(_mainTrack, _pitTrack)`
(falling back to whichever exists if one is empty). `UnionRegion`
was already imported. Users can now write:
```scenic
new RacingCar on raceTrack   # any drivable surface, main or pit
new RacingCar on mainTrack   # just main loop
new RacingCar on pitTrack    # just pit lane
```

**Why alias and not subclass.** The plan considered making
`RacingTrack` inherit from `PolygonalRegion`. The alias approach
gives the same user benefit (clean `on raceTrack` syntax) at one
line of change, with zero risk to RacingTrack's existing
`__init__` semantics. Subclassing would have required restructuring
the init order so the parent's polygon is computed before
`_identifyRacingFeatures()` runs. Not worth the fragility for an
aesthetic improvement.

**Verification notes.** Compile any scenario with
`new RacingCar on raceTrack` and inspect — should land on either
the main loop or the pit lane.
