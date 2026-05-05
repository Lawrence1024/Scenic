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

*Resolved in SD-20. See `## Done` section below.*

---

## 4. Corner-segment placement specifier missing

*Resolved in SD-24. See `## Done` section below.*

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

### #3 — VerifAI monitors read structured records, not parsed stdout (SD-20)

**What was wrong.** The contract between the simulator and the
falsification monitor was the stdout log format: `verifai_runner.py`
captured every sample's stdout, wrote `logs/sample_NNN.log`, then
called `parse_sample()` to regex-extract a `SampleMetrics`. Any
change to a `[EvalGT]` / `[Commit]` / `[BoundsCheck]` / `[Strategy]` /
`[TickTime]` / `[EvalEvent]` log line — added field, renamed key,
debounce tweak — would silently break monitors. Also not how Scenic
or VerifAI intend the contract: Scenic exposes `simulation.records`
(a `defaultdict(list)`) precisely for in-memory monitor inputs.

**What landed (Stage 20a).** Each emit site now writes to
`simulation.records[tag]` *alongside* the existing `print()`. The
print stays as the human debug log; monitors get a structured channel.

- `behaviors.scenic` adds a small `_record_event(tag, payload)` helper
  near the top that does
  `simulation().records[tag].append((sim.currentTime, dict(payload)))`
  with a try/except so a missing simulation context (replay,
  scene-only smoke) never breaks the print.
- The helper is invoked alongside the prints for `[Commit]`,
  `[TickTime]`, `[EvalEvent]`, `[EvalEventDiag]`, `[EvalGT]`,
  `[EvalContact]` (six call sites in behaviors.scenic).
- `[Strategy]` is special: the print lives inside `tactical_planner.py`,
  but the planner is pure-Python (no veneer import). The behavior
  emits the record from its own context right after
  `tactical_planner_step_v1` returns, reading `state.strategy_*`
  fields the planner already stashes. This keeps the planner pure.
- `simulator.py` (the dSPACE simulator) appends directly to
  `self.records['BoundsCheck']` since it has the simulation as `self`;
  no helper needed.

**What landed (Stage 20b).** `parse_sample()` in `sampled_runner.py`
gained a keyword-only `records=None` argument:

- When `records` is provided, eval/timing/strategy/bounds/commit
  fields are derived structurally by walking the records dict via a
  new `_records_extract(records)` helper. Same `SampleMetrics` shape;
  no regex.
- When `records is None` (subprocess path used by `sampled_runner.py`,
  older logs) every field is regex-parsed from the log as before.
- Fields that have no record source — `lap_time_s`, `ego_start_xy`,
  `opp_start_xy`, `sampled_gap_m`, `guard_emergency_stable_count` —
  are still log-parsed regardless of the records path.
- `_run_one_simulation` in `verifai_runner.py` now returns
  `(ok, simulation)`. `main()` reads `simulation.result.records` and
  passes it as `records=` to `parse_sample`, so VerifAI's monitor
  pipeline now sources from the in-memory channel.

**Stage 20c (no change).** `sampled_runner.py`'s subprocess path runs
`scenic` as a child process and only sees the log file — no
Simulation object reachable. The regex path stays as the fallback for
that runner.

**Why this matters.** Falsifier monitors no longer depend on the
literal stdout format. A future log-line edit that doesn't change the
structured record shape no longer breaks falsification.

**Verification notes.** Re-run a small CE smoke
(`--sampler ce --monitor safety --count 5 --time 1500`). Compare its
`summary.csv` to the regex-only path by setting `records=None` in a
debugger; numeric columns should match within rounding for fields
that have a record source. Per-sample log files should still be
human-readable (the prints were not removed).

**Aside found during verification.** The regex path was double-counting
`commit_pass_*_count` and `commit_abort_pass_count` because
`decision_reason=...` substrings appear in BOTH the `[Commit]` and
`[Planner]` log lines. The records path (which only walks the `Commit`
record stream) gave the canonical count — half the regex value. SD-20
incidentally fixed this. Cross-cycle comparison: only compare SD-20+
runs to other SD-20+ runs on those two columns.

### SD-21 — Drop regex pipeline entirely; rename module to `metrics.py`

**What was wrong.** SD-20 left a regex fallback alive in `parse_sample`
for the case where `records=None`. The fallback existed only to support
the subprocess-style `sampled_runner.py` (which can't access the
simulation object). With `verifai_runner.py` now superseding the
subprocess flow via `--sampler halton/random` for uniform-coverage runs
and `--sampler ce/bo` for active falsification, the regex code was
unreachable in practice and just legacy weight.

**What landed.**

- Added `_record_event` calls at the four remaining always-regex sites
  so every `SampleMetrics` field is records-driven:
  - `placement.py`: `'EgoStart'` (one entry per run, scene setup) and
    `'FellowPlacement'` (gap_m, lat_m, s, t).
  - `behaviors.scenic`: `'Guard'` alongside the existing
    `format_stability_guard_log_line` print (so
    `guard_emergency_stable_count` no longer needs substring counting).
  - `lap_time_s` is left at `None` in `SampleMetrics` — no behavior
    emits a `LapTime` record today and the regex was already non-matching
    on the verifai_runner stdout. The field is preserved on the dataclass
    so `summary.csv`'s schema is unchanged; future scenarios can populate
    it via a record event in the lap-completion code.
- `parse_sample` is now records-only. `_decode_log`, every `_RE_*`
  constant, and the `records is None` dispatch branch were deleted.
- `sampled_runner.py` was renamed to `metrics.py` (git mv preserves
  history). Its subprocess CLI (`main`, `run_one_sample`, the argparse
  block) was deleted along with `import re / argparse / subprocess /
  shutil / sys`. What remains: `SampleMetrics`, `_records_extract`,
  `parse_sample`, `write_summary_csv`, `write_summary_text`. About
  half the original line count.
- `verifai_runner.py` and `monitors.py` import paths updated. The
  `records=None` defensive path in verifai_runner now treats a missing
  `simulation.result.records` as a failed sample (logs a progress line,
  trips the consecutive-failures circuit breaker) rather than silently
  falling back to regex on the captured stdout.
- Docs updated: `docs/falsification_pipeline.md` lost its "Two parser
  bugs" history section, the "What the summary columns actually mean"
  table now points at record tags instead of log lines, and the file
  map reflects the rename. Example .scenic docstrings (S1_falsify,
  S1_fellow_left_ahead) updated to point at `verifai_runner.py
  --sampler halton`.

**Why this matters.** The contract between the simulator and the
falsifier is now `simulation.records` — a Python data structure — and
nothing else. Adding a new metric: write a `_record_event(tag, payload)`
call alongside the existing print, and add an extractor branch in
`metrics._records_extract`. There is no second contract to keep in sync.

**Verification notes.** A 5-sample CE smoke after SD-21 should produce
the same numeric distribution as the SD-20 smoke (`summary.csv` columns
are unchanged in shape; only the implementation moved). Per-sample log
files retain the full `[Commit]`/`[Strategy]`/`[BoundsCheck]`/etc.
prints because those stayed in place — only the parser was deleted.

### SD-22 — Seed Python random + numpy.random in verifai_runner

**What was wrong.** Determinism analysis across seven seed=42
verifai_runner runs found that no two produced the same sample 1
despite identical scenic file and identical CLI seed. Sample-1
`param0` (the VerifAI-controlled gap) ranged across {27.46, 28.71,
32.93, 40.0, 41.85, 52.48, 59.63} and `ego_start_xy` varied across
the entire main loop. Root cause: `verifai_runner.py` never seeded
the RNGs. The `--seed` flag was a label-only artifact; the real
Scenic global RNG (used by `new Point on ttlRegion(...)` and other
in-place samplers) and `numpy.random` (used by VerifAI's CE/BO
samplers) both initialized from `os.urandom` each invocation.

**What landed.** In `verifai_runner.main()`, before
`scenarioFromFile`, mirror what `scenic --seed N` does at
`__main__.py:184-189` — seed both `random.seed()` and
`numpy.random.seed()` with the base value. Per-flag behaviour:

- `--seed N` provided → seed deterministically with N; reproducible.
- `--seed` omitted → auto-generate a base at startup, seed with it,
  print it loudly so the user can re-run with `--seed <printed>`
  to reproduce.

The auto-generated base is drawn from Python's default RNG (which
itself initializes from `os.urandom` on first use), so consecutive
no-seed runs produce different bases and therefore different
campaigns — i.e. "act as random when seed is not provided."

**Verification notes.** Two back-to-back runs with `--seed 42` now
produce the same sample-1 `param0`, the same `ego_start_xy`, the
same `opp_start_xy`. A run without `--seed` produces a different
sample-1 and prints `auto-generated (no --seed; reproduce with
--seed <N>)` so the user can re-trigger an interesting random run
deterministically.

### SD-23 — ManeuverTime-based warmup + tighter initial poll + settle tick

**What was wrong (after SD-22).** With seeds locked, two same-seed
runs A and B produced the same sampled layout but slightly different
ego trajectories. Trace inspection found ego started driving at
*different* `ManeuverTime` values across runs (~3.41 s in run A vs
~3.05 s in run B). Three layered wall-clock-dependent stages in the
setup chain compounded:

1. `simulator.py:setup` polls `ManeuverTime > 0` with
   `time.sleep(0.2)`. Whichever 200 ms window the sleep wakes up in
   determines the first observed `ManeuverTime`, which can be
   anywhere in [~0, ~0.4 s].
2. `arrays.ensure_fellow_arrays_initialized` advances 3 s of
   *`SimulationTime`* from wherever VEOS happens to be.
   `SimulationTime` is the free-running VEOS clock that ticks even
   when the maneuver engine is dormant, so the post-warmup
   `ManeuverTime` inherits the variance from (1) directly.
3. The break condition `mt_now >= target` lands at a tick
   boundary, so the overshoot is in [0, sim.timestep] depending on
   floating-point rounding.

Cumulative variance: ~360 ms across runs at fixed seed. Visible as
the user-reported "ego starts driving at different maneuver time."

**What landed.**

- `arrays.WARMUP_MANEUVER_TIME = 3.0 s` replaces `WARMUP_SIM_DURATION`
  as the warmup target. The loop now advances until *`ManeuverTime`*
  (the maneuver-engine clock that drives plant behaviour) crosses
  the absolute threshold, not until `SimulationTime` drifts 3 s.
- `simulator.py:setup` poll cadence dropped from 200 ms to 10 ms.
  First observed `ManeuverTime` is now bounded by one or two dSPACE
  ticks instead of 200 ms.
- Settling block: if the warmup loop breaks within half a timestep
  of the target, advance one extra tick. Both runs end in
  `[target + 0.5*ts, target + 1.5*ts]` regardless of rounding
  direction. The completion log now prints absolute `mt_final` and
  overshoot-in-ms so two same-seed runs can be diffed directly.

**Verification.** Two `--seed 42 --count 1 --time 1500` runs (SD-23
A and B) both completed with `[dSPACE] Warmup done: ManeuverTime =
3.0130 s after N advances (+0 settle); target = 3.00 s, overshoot =
13.00 ms` — `mt_final` and overshoot agree to the millisecond. The
advance count differs (298 vs 278) because the starting offset
differed (0.024 vs 0.218 s); the *end state* is invariant. First
five `[BoundsCheck]` ticks of run A vs run B are bit-identical at
t=0.00, 0.50, 2.00 s; differ by exactly 0.01 m at t=1.00 and 1.50 s
(the printed precision floor; sub-cm OSQP numerical drift).

**What this does NOT touch.** OSQP solver determinism (subnormal
flush, BLAS thread scheduling, ADMM iteration count variance) still
produces sub-ulp control differences amplified through closed-loop
dynamics into sub-cm trajectory drift. That's the noise floor under
tight tolerances and is orthogonal to the maneuver-start
synchronization addressed here. For falsification purposes it's
well below any meaningful safety threshold.

### SD-24 — Curve / straight track regions + unified `trackRegion(...)` pipeline

**What was wrong.** Item #4 on the original change-list. Falsifying
corner-vs-straight behaviour required duplicating scenarios per
corner (`F8`/`F10`/`F11`/`F12` were byte-identical to `F6`/`F7` save
for hardcoded ego `(x, y)` coordinates). There was no abstraction
for "place ego on a corner" — the curvature classifier in
`segments/segment_map.py` (`_build_curve_straight_segments`,
threshold 0.011) existed and the planner consumed it via
`_waypoint_segment_map`, but its output wasn't exposed as Scenic
regions, and there was no way to drive a falsifier along the
corner-vs-straight axis. Separately, the existing per-vehicle
placement helper (`ttlRegion(self.ttlFileName)`) was destined to grow
sibling helpers for curve and straight, splitting one decision tree
across three sibling functions.

**What landed.**

- *Stage 24a — sub-road polygon slicing* in
  `src/scenic/domains/racing/segments/track_regions.py`:
  - `slice_road_polygon_at_segments(road, segments, road_category)`
    — uses Shapely's `split()` with perpendicular cut lines at each
    interior segment boundary to partition a road's drivable polygon
    into per-segment slices. Each piece is labelled by projecting
    its centroid onto the centerline and looking up the segment that
    contains the projected `s`.
  - `build_curve_straight_regions_from_opendrive(track)` — runs the
    slicer over every main + pit road, unions the slices into six
    Scenic Regions: `curve`, `straight`, `mainCurve`, `mainStraight`,
    `pitCurve`, `pitStraight`. Applies the same "main wins on
    overlap" rule that `mainTrack`/`pitTrack` already use.
  - On Laguna Seca: 9 main curve pieces (~8.4k m²) + 11 main
    straight pieces (~32.4k m²) + 2 pit curves + 2 pit straights.
    Areas roughly match the visual track geometry.

- *Stage 24b — region exposure + unified placement pipeline* in
  `src/scenic/domains/racing/model.scenic`:
  - Six new top-level `Region` names bound after `raceTrack`.
  - `ttl_category(ttlFileName)` helper added to
    `track_regions.py` (re-used from both Scenic and the simulator)
    classifies a TTL filename as `'main'` or `'pit'` via the same
    `'pit' in name.lower()` predicate already used at
    `tracks.py:343-350` for pit-road identification.
  - `trackRegion(ttlFileName=None, segment=None)` — the unified
    9-cell decision table replacing the old `ttlRegion(...)` helper.
    `ttlRegion` is kept as a one-line backward-compat alias.
  - `RacingCar.position` default routed through `trackRegion(...)`.
    The no-segment case (the default) is byte-identical to the
    pre-SD-24 behaviour: TTL polyline if a TTL is set, mainTrack
    fallback otherwise.

- *Stage 24c — placement contradiction warning* in
  `src/scenic/simulators/dspace/modeldesk/placement.py`:
  - `_maybe_warn_placement_contradiction(sim, obj, x, y, label)`
    fires a `[Placement] [WARN]` line + a parallel
    `'PlacementContradiction'` record entry when the car's TTL
    category disagrees with the polygon classification of its
    placed `(x, y)`. The four mismatch cases (main TTL on
    `pitCurve`/`pitStraight`/`pitTrack`; pit TTL on
    `mainCurve`/`mainStraight`/`mainTrack`) are explicitly covered.
  - Routing placement through `trackRegion(...)` makes
    contradictions structurally impossible — the warning only fires
    for users who deliberately bypass the unified pipeline via
    explicit cross-product names. No rejection; the user might be
    deliberately stressing the planner.

- *Stage 24d — example scenarios:*
  - `examples/racing/sampled/smoke_on_curve.scenic` — three-car
    compile-only smoke demonstrating TTL-aware curve placement
    (ego on optimal-TTL ∩ mainCurve), TTL-agnostic curve placement
    (fellow on full curve union), and explicit cross-product
    placement (car3 on mainStraight).
  - `examples/racing/falsifiable/F_curve_falsify.scenic` —
    replaces the F8/F10/F11/F12 family of hardcoded-corner
    scenarios with a single VerifaiRange-driven scenario that
    samples uniformly across all main-loop curves on the optimal
    racing line.

**Verification.** Compile-only via Python (avoids Scenic CLI's
visualizer window): both new scenarios sample correctly, with three
seeds producing three distinct ego positions clustered around
Laguna Seca corners. Three example ego positions for
`smoke_on_curve.scenic`: (540.77, -14.52), (-57.46, -218.18),
(502.77, 8.48). Per-region areas after slicing: mainCurve = 8443.9
m² (9 pieces), mainStraight = 32359.6 m² (11 pieces), pitCurve =
935.7 m² (2 pieces), pitStraight = 6740.4 m² (2 pieces).

**Placement bank (16 cases).** Added
`src/scenic/domains/racing/benchmarks/sd24_placement_bank.py` — a
single-command verification driver that compiles a synthetic
scenario per case via `scenarioFromString`, samples N times, and
asserts (a) the ego lands inside the requested polygon and
(b) the contradiction predicate fires iff the case is an explicit
mismatch. No simulator is attached — the bank uses the same
`ttl_category != classify(x, y)` predicate as `placement.py` so the
two stay aligned. Run via:

```bash
python src/scenic/domains/racing/benchmarks/sd24_placement_bank.py \
    --samples 3 --seed 42 --log sd24_bank.log
```

Coverage: 4 explicit cross-product cases + 4 unified-pipeline cases
(`trackRegion(name, segment)` with both main and pit TTLs) + 2 bare
axis cases (`on curve`, `on straight`) + 2 default-position cases
(main TTL, pit TTL) + 4 explicit contradiction cases (the four
mismatch combinations). Final result: 16/16 pass.

**Refinement uncovered by the bank: on-TTL skip in the contradiction
predicate.** The first bank run failed `default_pit_ttl`: 2/3
samples placed on the pit TTL polyline but classified as `mainTrack`
because the pit TTL legitimately traverses the mainTrack polygon at
pit entry/exit (where the main-wins-on-overlap rule subtracts pit
from main, so points along the pit TTL near pit entry land in main
by polygon arithmetic). The contradiction predicate fired
spuriously. Fix: both `placement.py:_maybe_warn_placement_contradiction`
and the bank's local copy now skip when the placed (x, y) is within
1 m of the TTL polyline. Rationale: any placement on the TTL is
consistent with the TTL by construction — it came from
`trackRegion(...)` or the default — regardless of which side of the
polygon arithmetic claims (x, y). Same skip rule mirrors between
the bank and the simulator so they validate the same predicate.

Stage 2 warning verification with a live simulator (the
`[Placement] [WARN]` line itself, not the predicate) requires
dSPACE; a 3-sample halton smoke through `verifai_runner.py` on
`F_curve_falsify.scenic` confirmed the unmodified scenario produces
no warnings, three distinct corner placements, and the expected
overtake-attempt-vs-gap correlation. Run via the user; not
exercised in this commit.

### SD-25 — Smart-ego pathology fixes from the 30-sample falsifier campaign

Three stages landed, one reverted after the falsifier surfaced a
worse failure mode than the one it tried to fix.

**What was wrong (baseline).** A 30-sample CE falsification run on
`examples/racing/falsifiable/S1_falsify.scenic` at `--seed 42`
produced 13/30 collisions. Three distinct bugs explained the
pattern:

| Bug | Count | Symptom |
|---|---|---|
| **A — Strategy-selector tiebreak** | 10/13 | When `pass_left` and `pass_right` tied on `reachable_progress_at_horizon_m`, both had `_TIEBREAK_RANK = 1`. Python's `min()` returned the first match in iteration order — pass_left always — even when pass_right's `min_clearance_m` was visibly higher. Smart ego repeatedly drove into a fellow geometrically positioned on the left because of a deterministic tiebreak bias. |
| **B — Locked-on-stay_optimal rear-end** | 2/13 | Selector picked `stay_optimal` for ALL 600 ticks while ego accelerated to 28 m/s and rear-ended a slower fellow on the left racing line. The lateral OBB-separation metric (`min_clearance_m`) stayed >2.5 m the whole run because the optimal and left lines are parallel and >2.5 m apart at the rear-end track sections. The metric missed the longitudinal closing rate (8–20 m/s). |
| **C — Abort recovery on side TTL with no speed cap** | 1/13 | Lifecycle correctly committed `pass_left`, then correctly aborted on `commit_invalidated_hazard`. But `_abort_result` returned `cap=None` so ego kept accelerating at target_speed during the recovery and rear-ended fellow 1.15 s after the abort. |

**Final outcome: ALL THREE STAGES REVERTED.** SD-25 lands as
documentation of three failed fix attempts plus an architectural
finding about the strategy_simulator. Code state is identical to
SD-24's end state; nothing in the planner / selector changed.

**Stages REVERTED (in order).**

- **SD-25c — abort_speed_margin_mps speed cap.** Originally landed
  in commit `0fc64781`. Reverted in `ae7b7f87` after a 30-sample
  re-run with all of SD-25a/b/c active surfaced multiple severe
  off-track events that didn't exist in the baseline:
    | Sample | Off-track? | track_clearance_m | Notes |
    |---|---|---|---|
    | #003 | TRUE | -0.43 m | mild |
    | #004 | TRUE | **-8.77 m** | severe |
    | #010 | TRUE | **-20.4 m** | catastrophic, also collision |

  Sample #10's per-tick log pinpointed the mechanism: at t=9.80 s,
  abort fired (`commit_invalidated_hazard`) mid-corner. The
  `[Planner]` line at that tick read
  `target_speed_cap=10.78 = opp_speed (8.78) + abort_speed_margin_mps (2.0)`.
  Ego state at t=10.00 s: speed=26.09 m/s, gear=3, segment "main
  curve", CTE=-0.54 m. The MPC was now asked to brake from 26 →
  11 m/s **while** still tracking the LEFT TTL (per SD-2d's
  keep-commit-side-during-side-by-side) **through a corner**. The
  combination is dynamically infeasible — ego understeers, builds
  cross-track error from -0.54 m → -8 m, and exits the corner
  ~20 m off the track. The instant-cap implementation is too
  aggressive for mid-corner aborts, even though the intent (don't
  accelerate into the fellow during recovery) is right. A future
  cycle could try: apply the cap only when ego is on the OPTIMAL
  TTL during recovery, OR use a rate-limited deceleration profile.
  Both require more design work than a one-line patch.

- **SD-25b — closing-flag gate for stay_optimal.** Originally
  landed in commit `d17f2516`. Reverted in `8e559023`.
  Implementation added a `closing_on_current_line: bool = False`
  kwarg to `select_strategy` and wired `assessment_closing_flag`
  from tactical_planner; when set, excluded `stay_optimal` from the
  primary survivor set. Intended to fix the 2/13 stay_optimal-locked
  rear-end collisions in the baseline (samples #1 and #13). The
  user's read of the post-SD-25c run was that the integrated
  behaviour was "still horrible" beyond the SD-25c off-track
  events; rolled back as part of a one-step-at-a-time response to
  isolate SD-25a's effect. SD-25b's design is sound at the
  unit-bank level (it correctly excludes stay_optimal when
  closing); whether the upstream `assessment_closing_flag` fires
  appropriately in real-world dynamics is an open question for
  a future cycle.

- **SD-25a — strategy_selector.py:95 clearance tiebreak.**
  Originally landed in commit `2d48a3de`. Reverted in `26f0671c`.
  Implementation replaced the single-key
  `min(tied, key=TIEBREAK_RANK)` with the tuple key
  `(TIEBREAK_RANK, -min_clearance_m)` so that when pass_left and
  pass_right tied on `reachable_progress_at_horizon_m`, the
  higher-clearance side won the tiebreak. The 30-sample
  SD-25a-only campaign showed it working in 7/13 baseline
  collisions and producing the cleanest results we'd ever seen
  (collisions 13 → 6, completed overtakes 4 → 11, off-track 1 →
  0, worst track_clearance −15.86 m → +0.52 m). Reverted because
  the user signalled an upcoming redesign of the strategy
  abstraction and a half-fix in the codebase complicates
  before/after analysis when verifying that redesign.

**The architectural finding (the real takeaway from SD-25).**

SD-25a was a band-aid on a deeper bug. Investigation while debating
whether to keep or revert SD-25a surfaced this:

The `prediction.strategy_simulator.simulate_strategy` function
integrates ego's longitudinal arc-length `s` using **a side-blind
speed/phase schedule**. Lines 168–182 define `speed_target` and
`phase_for_t` for both pass_left and pass_right with the SAME body
— neither function takes a `side` argument. The arc-length update
at lines 268–271 then advances `s` purely from speed; nothing
about it is side-aware:

```python
ds = 0.5 * (ego_v + v_next) * sample_dt_s   # 1-D longitudinal step
ego_s += ds                                  # ego_s evolves identically for both sides
```

`side` enters the simulator only at line 215 — for projecting the
already-advanced `ego_s` onto a polyline to compute clearance
against the fellow. **Consequence:**
`reachable_progress_at_horizon_m` is bit-identical for pass_left
and pass_right almost always. The `_polyline_for_pass_phase`
helper at line 65 makes it worse for `min_clearance_m`: during the
`merge_back` phase, both pass_* strategies use the OPTIMAL
polyline. Over a 10 s horizon with ~13 of 21 samples in
merge_back, the closest-approach to fellow often falls in the
side-blind tail, so the min-clearance metric ALSO comes back
identical for both sides.

We confirmed this empirically by reading the baseline (pre-SD-25)
logs for the 5 collision samples that SD-25a couldn't fix
(#8, #10, #21, #27, #28). At every tick where pass_left was
selected over pass_right, the reported clearances were
**bit-identical**:

| Sample | t at first pass_left selection | pL clearance | pR clearance |
|---|---|---|---|
| #8  | 0.15 s | 7.73 m | 7.73 m |
| #10 | 3.65 s | 10.65 m | 10.65 m |
| #21 | 1.55 s | 3.04 m | 3.04 m |
| #27 | 1.25 s | 5.86 m | 5.86 m |
| #28 | 0.35 s | 7.18 m | 7.18 m |

When clearances are tuple-equal, SD-25a's `(rank, -clearance)` key
becomes tuple-equal too, and Python's `min()` falls back on the
same iteration-order bias as before — pass_left wins by being
defined first in `_ALL_STRATEGIES`. SD-25a was structurally
incapable of helping these samples; the simulator never gave the
selector any signal to disambiguate.

**The real bug, restated.** The strategy selector primary-ranks
on `reachable_progress_at_horizon_m` and tiebreaks on rank /
clearance. But the simulator's longitudinal integration is
side-blind, AND the merge_back phase washes out side-specific
clearance over a 10 s horizon. So on the cases that matter most
(side-by-side overtakes), neither metric distinguishes the two
sides — and the selector falls back on iteration order. There is
no one-line fix.

**Design directions for SD-26 (a real fix, not a band-aid).** The
fix has to give the selector a side-distinguishing signal that
isn't washed out by merge_back. Three viable options, ordered by
invasiveness:

1. **Restrict `min_clearance_m` to side-specific samples only.**
   Compute the metric only over `lane_change` + `alongside`
   phases; ignore `merge_back` (where both sides are on optimal).
   1-line change in `simulate_strategy`. Fixes the equal-clearance
   degeneracy at the source.
2. **Add a side-sensitive primary metric.** E.g., signed lateral
   distance from fellow at the moment of closest approach
   (positive = on the safer side). This would be inherently
   side-aware and make a tiebreak unnecessary in most cases.
   Requires a new metric on `StrategyOutcome` and a selector
   update.
3. **Add lane-change kinematics to the simulator.** A bicycle
   model that actually slows ego through tighter turns would
   make `reachable_progress_at_horizon_m` differ between sides
   when the geometry favours one direction. Most invasive but
   architecturally correct — addresses the root cause that the
   strategy_simulator's "instantaneous lane change" abstraction
   leaks information.

Option 1 is the minimum viable fix and likely the right next
step. Option 2 is the architecturally cleanest. Option 3 is
overkill for the residual collision cluster but might be worth
it as part of a broader strategy-layer redesign.

**The falsifier did its job (twice).** SD-25's value as a cycle
isn't the code that landed (none did). It's the two findings the
falsifier surfaced that no unit bank could have caught:

1. **Mid-corner abort with a hard speed cap is dynamically
   infeasible.** A unit bank can verify `select_strategy` picks the
   right answer; only a closed-loop simulation through the cosim
   bridge shows the MPC's reaction to the planner's command.
2. **The strategy_simulator's progress and clearance metrics are
   both side-blind in the residual-collision cases.** A unit
   bank fed pre-built `StrategyOutcome` lists never exercises
   the simulator that produces them; only running the actual
   simulator on real geometries shows the equal-clearance
   degeneracy.

Both findings are deferred to SD-26. Code state at the end of
SD-25 is byte-identical to the end of SD-24.

**End-state files (post all three reverts).**

- `src/scenic/domains/racing/planner/strategy_selector.py` —
  unchanged from SD-24. Single-key `min(tied, key=TIEBREAK_RANK)`
  at line 95; `select_strategy` has no `closing_on_current_line`
  parameter.
- `src/scenic/domains/racing/tactical_planner.py` — unchanged
  from SD-24. `_select_strategy` call site does not pass a
  closing flag; `_abort_result` returns `cap=None`; no
  `abort_speed_margin_mps` field.
- `src/scenic/domains/racing/benchmarks/sd25_selector_unit_bank.py`
  — file does not exist. Removed by the SD-25a/d revert.

**Verification.** Re-run the same 30-sample CE campaign at
`--seed 42` to confirm code state matches the original 13/30
baseline:

```powershell
python src/scenic/domains/racing/benchmarks/verifai_runner.py `
    examples/racing/falsifiable/S1_falsify.scenic `
    --sampler ce --monitor safety --count 30 --seed 42 --time 3000 `
    --quiet *>sd25_postrevert_baseline.log
```

Expected: 13/30 collisions matching `verifai_20260428_165255/`.
Sample-by-sample equivalence isn't strictly required (the cosim
bridge has cm-scale OSQP-driven non-determinism between runs) but
the aggregate count + dominant pathology should match.

### SD-26 — Lane-change blending in the strategy simulator

Single-stage fix targeting the dominant pathology in the SD-25
post-revert baseline (13/30 collisions): the strategy simulator was
placing ego at full lateral offset on the side polyline at `t=0+`,
producing optimistic `pass_*` clearance predictions that didn't match
the MPC's actual lateral dynamics.

**What was wrong.** From the SD-25 architectural note: at the moment
of `pass_left` commit, the simulator queried `xy_at_arclength(left_polyline, ego_s)`
— ego on the left polyline from t=0+. The actual MPC's CTE decay in
sample #8 of the baseline campaign was 4.37 m → 4.13 m → 2.75 m over
t=0.2 → 1.0 → 2.5 s — a first-order exponential with τ ≈ 3–5 s.
Predicted clearance: 7.73 m. Actual closest approach: 0.94 m.

**The fix.** Replace the instantaneous side-polyline projection with
a blended position:

```python
ego_xy(t) = α(t) * side_polyline_xy(s) + (1 − α(t)) * optimal_polyline_xy(s)
```

where `α(t) = 1 − exp(−t / τ)` with τ = 2.5 s (default, tunable via
`TacticalPlannerConfig.strategy_lane_change_tau_s`). `α(0) = 0` (ego on
optimal at the moment of commit), `α(τ) ≈ 0.63`, `α(3τ) ≈ 0.95`.

**Files (final state).**

- `src/scenic/domains/racing/prediction/strategy_simulator.py` —
  added `_blend_alpha(t_off, tau_s)` helper and replaced the
  `_polyline_for_pass_phase` ego-track lookup with a per-tick
  blended xy. `lane_change_tau_s` kwarg added to
  `simulate_strategy` (default 2.5 s, tau ≤ 0 collapses to legacy
  instantaneous behaviour for backward-compat tests).
- `src/scenic/domains/racing/tactical_planner.py` —
  `TacticalPlannerConfig.strategy_lane_change_tau_s = 2.5`, threaded
  into the `simulate_strategy` call.
- `src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py`
  — new offline regression bank; six original SD-26 cases
  (alpha math, tau=0 backward-compat, pass_left blend, pass_right
  symmetry, stay_optimal invariance, merge_back routing). Single-
  command runner with `--log` flag.

**What stayed the same.** Longitudinal `s` integration unchanged
(side-blind by design). Phase detection (`lane_change` /
`alongside` / `merge_back`) unchanged. Speed-target schedule per
phase unchanged. Strategy selector untouched (still consumes
`min_clearance_m`; the value just becomes realistic).

**Verification at SD-26 boundary.** SD-26 fixed the blend math in
isolation. The 30-sample CE campaign with SD-26 alone wasn't run
to completion — once SD-27 was in motion (a 2-sample probe surfaced
the OBB-clearance bug that SD-26's centroid metric still couldn't
catch), the campaign was re-run with SD-26 + SD-27 together. See
SD-27 entry below for the unified result.

### SD-27 — Better opponent prediction (CTR) and OBB-aware clearance

SD-27 is the unified fix for two failure modes that SD-26 alone
couldn't address. A 2-sample probe with SD-26 in place still produced
a full-overlap collision in sample 2: `pass_left` was selected with
3.95 m predicted clearance, actual outcome was full OBB overlap from
t=4.25 s to t=4.65 s.

**What was wrong (root causes uncovered after SD-26).**

1. **Cartesian-CV fellow extrapolation smeared fellow off the racing
   line.** `FellowPredictor.trajectory()` walked fellow forward at
   constant cartesian velocity from the latest observation. On
   curving sections, the predicted xy drifted off whatever line the
   fellow was actually following, opening fictional gaps. This
   directly drove the simulator to predict safe `pass_left`
   clearances when fellow was on the left racing line.

2. **The clearance metric was centroid-to-centroid, treating cars
   as points.** With IAC Dallaras at 4.88 m × 1.93 m, a 3.95 m
   centroid distance during end-to-end approach is already
   overlapping. The 2.5 m hard filter was calibrated to that point
   model and never had a chance to reject these geometries. The
   user's exact diagnosis: "we drove straight through fellow like
   it doesn't exist; doesn't that just mean our estimation is
   horrible? […] the stack should see a huge car at the left of
   the track, predict that it is still going to be on the left,
   and thus not overlap our trajectory with that."

**Architectural directive (from the user, for this fix).** Adopt
Frenet-frame / arc-following lessons from race_common, but
*without* baking in a "fellow follows the racing line" assumption
— in real racing a malfunctioning fellow can drive any line and
the smart-ego still has to avoid it. The fix has to be observation-
based.

**SD-27a — Constant-turn-rate fellow prediction.**

`src/scenic/domains/racing/prediction/fellow_predictor.py`:

- New `_yaw_rate_from_history()` method. Per-segment heading from
  successive history points, unwrap, recency-weighted linear
  regression of unwrapped θ vs time. Returns `None` if too few
  segments or motion too small.
- `trajectory()` now propagates fellow on a circular arc when yaw
  rate is estimable (`|ω| ≥ 1e-3 rad/s`):

  ```
  x(t) = x0 + (v / ω) * (sin(θ0 + ω·t) − sin(θ0))
  y(t) = y0 − (v / ω) * (cos(θ0 + ω·t) − cos(θ0))
  ```

  Otherwise falls back to the existing CV path. New kwargs
  `use_ctr=True` (default) and `ctr_min_yaw_rate_rad_s=1e-3`.

**Crucially: this makes ZERO assumption about fellow being on a
racing line.** The propagated turn rate is whatever fellow's
actual heading history shows — a fellow drifting off-line gets
predicted with the off-line yaw rate they actually exhibit.

Unit-bank verification: with circular-arc input (yaw rate 0.2 rad/s,
speed 10 m/s), CTR tracks the true arc with 0.45 m error at t=2 s;
CV diverges to 4.43 m error.

**SD-27b — OBB-aware clearance metric.**

`src/scenic/domains/racing/prediction/strategy_simulator.py`:

- New kwargs `ego_length_m`, `ego_width_m`, `fellow_length_m`,
  `fellow_width_m` (default IAC Dallara 4.88 m × 1.93 m, imported
  from `eval_geometry`).
- Per-tick clearance is now `obb_separation_distance_m` (true
  edge-to-edge minimum gap between the two oriented bounding boxes
  via point-in-OBB and segment-to-segment SAT-style checks). Reuses
  the existing `eval_geometry.obb_separation_distance_m` helper.
- **Headings are from finite-difference of the previous tick's xy.**
  Both ego heading and fellow heading are computed as
  `atan2(Δy, Δx)` between adjacent samples, so OBBs are properly
  rotated to each agent's actual direction of motion at every tick.
  At the first tick (`i=0`) there's no previous xy, so the metric
  falls back to `centroid_dist − circumradius_ego − circumradius_fellow`
  (heading-agnostic, conservative — under-reports clearance, never
  over-reports). For a 21-sample horizon (10 s at 0.5 s dt), 20 of
  21 samples use the heading-aware OBB metric.

**SD-27b — Reverse-blend during merge_back.** Symmetric to SD-26's
forward blend. Pre-SD-27 the simulator snapped ego onto the optimal
polyline at the first `merge_back` tick; with OBB clearance that
collapsed the predicted gap to ~0.1 m when ego was one car-length
ahead on the same line as fellow, breaking otherwise-safe passes.
Now α decays back to 0 over τ during merge_back (`alpha = last_alpha_pre_merge * exp(-(t - merge_back_start_t) / τ)`).

**SD-27b — Threshold recalibration.** The clearance metric's
*meaning* changed from centroid-to-centroid to edge-to-edge gap, so
the filter thresholds had to follow. Defaults dropped:

- `strategy_min_clearance_m`: 2.5 → **0.5 m**
- `strategy_soft_clearance_m`: 1.5 → **0.2 m**

Calibration intuition: a 2.5 m centroid distance with IAC-width
1.93 m cars meant ~0.6 m of physical bumper-to-bumper gap. The new
0.5 m hard threshold preserves roughly that physical safety buffer
under the new metric. Updated in
`src/scenic/domains/racing/planner/strategy_selector.py` defaults
and `TacticalPlannerConfig.strategy_min_clearance_m / strategy_soft_clearance_m`.

**Files (final state).**

- `src/scenic/domains/racing/prediction/fellow_predictor.py` —
  `_yaw_rate_from_history` helper + CTR branch in `trajectory()`.
- `src/scenic/domains/racing/prediction/strategy_simulator.py` —
  `eval_geometry` imports for IAC dimensions and the OBB helper;
  per-tick OBB clearance with finite-diff headings; reverse-blend
  in merge_back; vehicle-dimension kwargs.
- `src/scenic/domains/racing/planner/strategy_selector.py` —
  threshold defaults dropped to 0.5 / 0.2; docstring updated.
- `src/scenic/domains/racing/tactical_planner.py` — config
  defaults dropped to match.
- `src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py`
  — added 4 SD-27 cases (CTR-on-straight, CTR-on-arc, OBB lateral
  pass, OBB full overlap) and reworked the merge-back case to
  verify the reverse-blend.
- `src/scenic/domains/racing/mpc/testing/test_strategy_simulator.py`,
  `test_tactical_planner_sd11d.py` — assertion thresholds updated
  for OBB metric (5.0 m centroid → ≈3.07 m OBB; comments call out
  the metric change).

**Verification.** Full racing test suite: 151 passed. SD-26/27
unit bank: 11/11 passed (alpha math, tau-zero backward-compat,
pass_left blend, pass_right symmetry, stay_optimal invariance,
reverse-blend in merge_back, far-fellow sanity, CTR-straight,
CTR-arc, OBB lateral pass, OBB full overlap).

30-sample CE campaign at `--seed 42` with SD-26 + SD-27 active
(`verifai_20260429_104834/`):

| Metric | Pre-SD-25 baseline | SD-26 + SD-27 | Change |
|---|---:|---:|---|
| Collisions | 13 / 30 | 2 / 30 | ↓ 85% |
| Off-track | 1 | 0 | ↓ |
| Successful passes | 4 | 21 | × 5.25 |
| Pass attempts (L / R / aborted) | 898 / 438 / 473 | 37 / 3173 / 33 | left attempts ↓ 96%, aborts ↓ 93% |
| Strategy picks (stay / follow / pL / pR) | 17321 / 17 / 381 / 281 | 16536 / 157 / 486 / 821 | follow ↑ 9×, pR selections ↑ 3× |
| Worst per-sample `bbox_gap_m_min` | 0.00 m (full overlap) | 1.79 m | full overlap → safe gap |

The most striking signal in the telemetry is the L→R pass-attempt
flip (898 / 438 → 37 / 3173). In `S1_falsify.scenic` the sampled
fellow gap puts fellow on the **left** racing line. Pre-SD-27 the
simulator (cartesian-CV smearing fellow off-line + centroid metric)
reported `pass_left` ≈ 3.95 m clearance — looked safe, was selected,
collided. Post-SD-27 fellow's predicted trajectory stays on the left
line (CTR), the OBB metric correctly reports `pass_left` ≈ 0 m, the
filter rejects, the selector picks `pass_right` instead.

See `docs/falsify_results.md` for the running campaign log
(attempt 1 baseline, attempt 2 = post-SD-27, template for future
attempts).

**Remaining failures (both different design problems from what
SD-26 / SD-27 attack — out of scope here).**

- Sample 4 (seed 45): strategy selector flip-flops between
  `pass_left` / `pass_right` every tick at this track location.
  Hysteresis (`strategy_commit_cycles=2`) is too short for noisy
  alternating selection; snapshot fallback path doesn't apply the
  same hysteresis. 37 left commits all aborted, eventual contact.
- Sample 19 (seed 60): zero commits — ego stayed on `stay_optimal`
  the whole time and rear-ended fellow. Predictor alternates
  between `stay_optimal` ≈ 9 m on even ticks and ≈ 1 m on odd
  ticks (per-tick fellow-pose oscillation upstream of the
  predictor). Selector picks `stay_optimal` whenever it clears
  0.5 m; never proactively switches to `follow_fellow` when
  stay_optimal is consistently marginal.

Both expose new design questions (selector stability, marginal-
stay-optimal handling) that the prediction-correctness work surfaced
because the dominant pre-SD-27 failure mode — false-safe `pass_left`
into a fellow on the left line — is gone.

### SD-28 — S2 falsification scenario (gap + side + speed)

Strictly a scenario expansion, not a stack change. Adds three
VerifAI knobs (continuous gap, binary side / TTL pair, continuous
fellow speed) so the campaign explores a 3-dim space instead of S1's
1-dim. Sampler-agnostic (works with halton / ce / bo / random).
Captured in `examples/racing/falsifiable/S2_falsify.scenic`. Two
Scenic-mechanics tricks worth knowing:

- **Side knob synchronisation.** Lateral offset and TTL filename
  must agree per sample, but two independent VerifaiOptions would
  let CE pick inconsistent combos (`lat=+5, right TTL`). Solution:
  one `VerifaiDiscreteRange(0, 1)` routed through two
  `@distributionFunction`-decorated helpers (`_fellow_lat_for_side`,
  `_fellow_ttl_for_side`) so both properties resolve from the same
  underlying sample.
- **Behavior kwargs aren't auto-resolved.** Scenic resolves
  Distribution properties on object instances at scene-sample time,
  but Distribution kwargs to behavior constructors are captured raw
  and never sampled. Workaround: register `fellow_speed_mph` as a
  scene `param`, then have a thin wrapper behavior
  (`_FellowFollowFromParams`) read the resolved scalar from
  `simulation().scene.params` at first activation and delegate to
  `FellowFollowTTLGeometricBehavior` with a literal float.
- **Default position formula breaks on Distribution `ttlFileName`.**
  `racing_model`'s default `position: new Point on
  trackRegion(self.ttlFileName)` calls `_ttl_category()` which does
  `if not ttl_file_name` and raises `RandomControlFlowError` on a
  Distribution. Pinning `at (0, 0) with regionContainedIn everywhere`
  bypasses the formula; `_racing_st_offset` overrides the placeholder
  at simulation time via `modeldesk/placement.py`.

`verifai_runner.py` also got a `--scenic-control / --no-scenic-control`
convenience flag (BooleanOptionalAction) that overrides the scene's
`scenic_control` param without requiring `--extra-param scenic_control=...`.
Default `None` leaves the .scenic file's setting alone.

S2 attempt-1 baseline (run dir `verifai_20260429_122917/`, frozen as
the runtime-cuts reference at commit `1d7e4f7c`):

- Collisions:           6 / 30
- Off-track:            3 / 30
- Successful passes:    16
- Pass attempts L/R/A:  1180 / 1954 / 97
- Worst bbox_gap:       0.00 m
- Mean tick_ms_p50:     26.7 ms

S2 reuses the existing `safety` monitor, the existing per-sample log
schema, and the SD-27 stack. No changes to planner, simulator, or
selector.

### SD-29 — Runtime cuts (REVERTED) + wall-clock budget analysis

Two-part cycle. Part 1 attempted no-behavior-change cuts inside the
Scenic stack to recover the per-tick wall budget SD-27's OBB SAT
work consumed; the cuts shipped (`4d7b485c`) but were **reverted
(`2c2eb8ab`)** when partial verification showed sample-1 collision
+ off-track and a sample-6 spin from "crazy control commands". The
"no-behavior-change" reasoning was correct in isolation (the
conservative bound is genuinely a lower bound on SAT clearance) but
loose enough (~0.4 m gap to true SAT) that some marginal close-call
cases got pushed over the wrong side of the 0.5 m hard filter, which
re-armed the false-safe pass-left bias SD-27 had eliminated.

Part 2 measures *where* the per-tick wall budget actually goes so
future cycles know whether to keep optimising Scenic or move to the
cosim/IPC layer.

#### Part 1: no-behavior-change cuts (REVERTED)

Profiling target was the `planner` bucket in `[TickBreakdown]` logs
which went from 5.5 ms (pre-SD-27) to 10.8 ms (post-SD-27) — the
+5.3 ms is almost entirely OBB SAT distance computation (4 strategies
× 21 horizon samples = 84 SAT calls per planner tick, each ~10× the
cost of a euclidean centroid distance). CTR yaw rate estimation was
~0.05 ms (essentially free).

**Cut 1 — OBB SAT early-exit when centroid is large.**
`prediction/strategy_simulator.py`. Threshold
`half_circ_ego + half_circ_fellow + 3.0 m` (≈ 8.25 m for IAC
Dallaras). When centroid distance exceeds this, the conservative
lower bound `centroid_dist - half_circ_ego - half_circ_fellow` is
guaranteed ≥ 3 m — comfortably above the 0.5 m hard filter AND
above the boundary-case OBB clearance values (~3 m gaps in lateral
pass tests) where SAT precision actually matters. The bound is
always ≤ true OBB edge gap, so reporting it is conservative
(over-rejects on the boundary, never under-rejects). Selector
ranking is unaffected because clearance is consumed only by the
filter, not the tiebreak (verified: `min_clearance_m` consumers are
`strategy_selector.py:87` filter and `:114` soft fallback, plus
telemetry).

**Cut 2 — per-strategy break on overlap.**
`prediction/strategy_simulator.py`. Once OBB clearance hits 0 (full
overlap detected), the strategy is guaranteed to fail the hard
filter regardless of subsequent ticks. Break the integration loop
instead of spending the remaining ~20 OBB SAT calls.
`reachable_progress_at_horizon_m` is irrelevant for filtered-out
strategies, so the truncated value is fine.

**Cut 3 — throttle the `[Strategy]` telemetry print.**
`tactical_planner.py`. Pre-SD-29 the print fired every control
tick (~600 lines per 30 s sample); string formatting of per-strategy
dicts plus I/O was a measurable contributor to the `other` timing
bucket. Post-SD-29: print only when (a) the selector decision
changes (catches every transition), or (b) every 10th consecutive
tick of the same decision (statistical sampling). Per-tick numerics
still recorded on `state.strategy_min_clearances` etc; only the
human-readable line is throttled.

**Cut 4 — drop the dead `pc_diag["samples"]` allocation.**
`assessment/pass_geometry.py`. Verified via `grep -rn 'pc_diag'`
that no caller in the codebase reads `pc_diag["samples"]` — only
`min_clear_m`, `closest_t_s`, and `breach_count` are consumed.
15 tuples × 600 ticks × 30 samples = 270k pointless allocations
per campaign. Removed the init line and the dict entry.

**Diagnostics.** `StrategyOutcome` gained `obb_calls`, `obb_skips`,
and `early_exit_tick` fields, surfaced on the throttled `[Strategy]`
log line as `obb_calls=N obb_skips=M early_exits=K`. Lets us see at
a glance how often each cut fires in production. SD-29 smoke on S2
showed mean obb_calls=3.8, obb_skips=58.5, early_exits=1.5 per
planner-tick — i.e., ~94% of OBB ticks short-circuit via cut 1, and
~1.5 of 4 strategies hit overlap and break early per tick on
average.

**Files.**

- `src/scenic/domains/racing/prediction/strategy_simulator.py`
  (cuts 1, 2, instrumentation)
- `src/scenic/domains/racing/tactical_planner.py` (cut 3)
- `src/scenic/domains/racing/assessment/pass_geometry.py` (cut 4)

Smoke results (single sample on S2): planner section dropped from
15.0 ms → 4.4 ms p50 (−71%). Sum-of-section means dropped from
36.95 ms → 28.37 ms (−23%). All 151 racing unit tests + 11 unit-bank
cases pass with no test changes needed.

**Live verification — REGRESSED, reverted (`2c2eb8ab`).** A partial
30-sample S2 CE campaign at `--seed 42` (run dir
`verifai_20260429_144214/`, stopped at sample 5) showed:

- **Sample 1 (seed 42)**: collision **AND** off-track,
  `bbox_gap_min=0.00 m`, **147 commit_pass_left** / 0 right / 0
  success / 37 abort. The same scenario in attempt-1 baseline went
  0/299/1/0 (cleanly picked pass_right). The selector flipped the
  side decision because the conservative bound at marginal ticks
  reported `pass_left ≈ 0.86 m` (above the 0.5 m filter) while
  `pass_right` SAT-evaluated to 0.00 m — exactly the inverse of
  reality.
- **Sample 6 (user observation, not in summary.csv)**: ego "spin
  due to crazy control commands". Same root cause class — the
  selector commits to a pass that the actual physics can't resolve.
- Samples 2–5 looked fine on the behavioral metrics, so the
  regression isn't uniform. It's the *marginal* close-call cases
  where the bound's ~0.4 m looseness vs true SAT decides which side
  of the filter the strategy lands on.

The wall-clock cut alone (planner 15 → 4 ms p50) wasn't worth that
behavioral cost. Reverted in one step. Acceptance criteria from
`docs/falsify_results.md` (collisions ≤ 6 / 30, etc.) were violated
already in the first sample.

**Lesson for future cycles.** The "always-≤ true SAT" property is
necessary but not sufficient for behavior preservation. A bound
that's monotonic with true SAT can still flip *relative ranking*
between two strategies whose true SATs are close — the selector
filter is on absolute value but selection among survivors is on
relative ranking via progress, and the bound tightness can
re-rank when both strategies sit near the threshold. Any future
"compute fewer SATs" idea has to either (a) only fire when SAT is
guaranteed safe by a *much* larger margin than the filter
threshold, or (b) preserve the ranking *between* strategies, not
just the safe / unsafe classification of each in isolation.

#### Part 2: wall-clock budget analysis

Question the SD-29 cuts surfaced: even after the 4–5 ms Scenic-side
cut, the simulator still ran ~2× wall-time per sim-time. Where does
the rest of the wall budget go?

Methodology: created a one-off no-control idle scenario
(`examples/racing/calibration/no_control_idle.scenic`) and a parser
script (`src/scenic/domains/racing/benchmarks/analyze_tick_timing.py`).
The idle scenario stripped Scenic's behavior body to just `wait` plus
a `[TickTime]` print, leaving the cosim+VEOS+IPC plumbing in place.
Both files were diagnostic-only and have been removed; the findings
are recorded here.

**Per sim-step wall budget (10 ms sim time target):**

| Layer | Wall ms / sim step | Notes |
|---|---:|---|
| Idle (no Scenic logic) | 12.3 ms | 1.23× real-time = ~23% baseline overhead |
| Full SD-29 smoke | 22.0 ms | 2.20× real-time |
| Difference (Scenic adds) | +9.7 ms / sim step | what the Scenic stack costs |

**Per control-tick breakdown (50 ms sim time target = 5 sim steps):**

| Component | Per control tick |
|---|---:|
| VEOS sim (5 sim steps × ~12 ms) | ~61 ms |
| Scenic compute (`tick_ms`) | ~28 ms |
| IPC + glue | ~22 ms |
| **Total wall** | **~110 ms** |

**The earlier "bottleneck is upstream of Scenic" diagnosis was
wrong.** That conclusion came from quoting `wall_step − tick_ms = 87 ms`
as "outside Scenic", but ~61 ms of those 87 ms is legitimate VEOS
sim cost (5 sim steps must run for the simulator to advance 50 ms).
True non-Scenic, non-VEOS overhead is ~22 ms / control tick.

**Headroom for future cycles.** To hit real-time wall pacing
(50 ms / control tick) we'd need to cut ~60 ms total:

1. **Scenic side: 28 ms today.** SD-29 already cut planner.
   Lateral OSQP MPC is now the largest single bucket (~7-10 ms).
   Tactical planner fast-paths and behavior-body dispatch overhead
   are also non-trivial.
2. **IPC + glue: ~22 ms.** Worth instrumenting
   `SyncStepBridge.advance()` separately — TCP localhost roundtrips
   shouldn't take this long. Possible contributors: MAPort batching
   semantics, ControlDesk readbacks per step, JSON
   (de)serialization. Could be a meaningful single-digit-ms win.
3. **VEOS overhead: ~11 ms above the real-time floor.** The idle
   showed VEOS is 23% slow on its own. To recover this we'd need
   either a faster machine or a "VEOS as fast as possible" pacing
   knob rather than the soft-real-time cosim mode currently in use.

**Bimodal idle distribution.** The idle scenario showed
`wall_step p50=5 ms, p90=46 ms` — bimodal, not a single Gaussian.
~80% of sim ticks are light VEOS work (~5 ms wall), ~20% are
heavier (~46 ms wall) — likely ASM bookkeeping, MAPort syncs, or
readback-heavy ticks. This bimodality is hidden in the full stack
because the larger Scenic compute swamps the per-tick variance, but
it persists underneath.

**Why the doc lives here, not in code.** The diagnostic scripts
(`no_control_idle.scenic`, `analyze_tick_timing.py`) were one-off
investigation tools — not infrastructure worth maintaining. The
findings above are what's load-bearing for future cycles. If a
future cycle needs to redo the analysis, the methodology is small
enough to recreate from this entry.

### SD-30 — Selector clearance tiebreak + 1.0 m hard filter

S2 falsifier sample 1 collided despite a passing prediction: the
0.5 m clearance bar admitted close-call `pass_left` choices when a
safer `pass_right` was available. SD-30 raised
`strategy_min_clearance_m` from 0.5 → 1.0 (~half a Dallara width)
and added a secondary tiebreak inside the `survivors` rank-1 group
where, when both pass strategies pass the filter, the one with
greater predicted clearance wins. Clearance is now both a hard bar
and a tiebreak signal.

**Files.** `src/scenic/domains/racing/planner/strategy_selector.py`,
`src/scenic/domains/racing/tactical_planner.py` (config default).

**Live verification.** Sample 1 fixed; broader campaign showed three
*new* failure modes (samples 6, 8, 9), each a distinct mechanism —
the SD-31/32 plan addresses all three.

### SD-31 — Curvature-clipped pass cap

S2 sample 6 went off-track and "flew" through Laguna Seca's
turn-19 hairpin (κ ≈ 0.143, r ≈ 7 m) because the COMMIT_PASS speed
cap was set to opp+commit_speed_margin without respecting the
curvature ahead. SD-31 clips the commit cap by `sqrt(a_y_max / κ)`
so ego enters tight curves at a tire-grip-feasible speed regardless
of the strategy layer's preferred margin. New config field
`commit_max_lateral_accel_mps2 = 8.0` (~0.82 g, conservative for
IAC slicks).

**Files.** `src/scenic/domains/racing/tactical_planner.py`
(`_curvature_clip` helper + new field), `behaviors.scenic`
(thread `curvature_ahead_max` to `tactical_planner_step_v1`).

### SD-32A — Honor safety caps quickly via slew-rate bump

The planner cap dropped to ~7 m/s on hairpin entry (SD-31) but the
slew-rate limiter on `effective_target_speed` (~7-12 m/s²) took
~0.85 s to converge — by then ego was at the apex. SD-32A bumps
`slew_down_ms` to 15 m/s² (~1.5 g, brake-limited) when the binding
cap is `tactical` or `curvature` AND the cap is asking for
deceleration. Other slew rates (CTE, cruise) unchanged — those
target smoothness, not safety.

**Files.** `src/scenic/domains/racing/behaviors.scenic` (slew block
near line 1707).

### SD-32B — Closing-rate gap-feasibility guard

Sample 8 collided longitudinally because the strategy simulator
reported `pass_right ≈ 1-3 m` clearance for a pass that physically
couldn't complete in the available longitudinal gap. SD-32B adds a
pre-integration early-return in `simulate_strategy()` for `pass_*`:
if `closing_speed × lane_change_tau × 1.2 > longitudinal_gap`, the
pass is clamped to `min_clearance_m=0.0` with reason
`gap_too_short_for_lane_change`. Original cut used absolute ego
speed (`v_ego × τ`) which over-blocked legitimate passes; the
**refined** version uses closing rate (`max(0, v_ego - v_opp) × τ`)
so non-closing fellows don't trip the guard.

**Files.**
`src/scenic/domains/racing/prediction/strategy_simulator.py`.

### SD-32C — Track-frame lateral pass-side guard

Sample 9 collided from a `pass_left` commit into a fellow ALREADY
on the LEFT TTL — geometric nonsense the simulator's lane-change
blend asymmetry didn't catch. SD-32C adds a sanity guard: if
`fellow_lat_track > +1.0 m` (substantially left of optimal),
`pass_left` is clamped to 0; mirror for `pass_right`. Original cut
used `OpponentSituation.lateral_m` which is in the **ego heading
frame** (sign flips when ego yaws). **Refined** version uses a new
`_signed_cross_track_at_s()` helper that projects fellow xy onto
the optimal polyline's tangent — true track-frame lateral, sign-
stable across ego heading changes.

**Files.**
`src/scenic/domains/racing/prediction/strategy_simulator.py`
(helper + guard).

### SD-33/34 — REVERTED

Two coordinated structural fixes attempted then reverted:

- **SD-33** (selector closing-flag demote of `stay_optimal`):
  forced selector to drop `stay_optimal` when `closing_flag=True`
  AND no `pass_*` survived the hard filter. Intended to break F14-
  style "selector picks stay_optimal because it has the best
  progress, even though we're closing on a wall" failures.
- **SD-34** (side-polyline curvature drivability guard): blocked
  passes when the side TTL has a tighter curve ahead than the
  pass speed could handle. Intended to prevent off-tracks at the
  Corkscrew when ego commits to right TTL.

Live verification of the combined cut showed visualization
regressions: the un-gated SD-33 forced extra `pass_right` commits
on sample 1 (94 vs ~29 baseline), driving ego onto the right TTL
through the Corkscrew → off-track. Reverted at `faa54ccc`. The
underlying problems both still need fixes, but as separate plans
with tighter gating conditions.

### SD-35 — Risk-metric overhaul (longer lookahead, blocker-aware, no-escape gate)

F14's active-blocker run surfaced a structural defect in
`emergency_risk_01`: ego ran straight into the blocker at 14 m/s
closing rate while the recorded risk plateaued around 0.5–0.6.
The thresholds (`abort_risk_01=0.55`, `emergency_risk_enter_01
=0.85`) sat *above* the metric's saturation floor in the actively-
blocked regime, so neither gate ever fired. Three causes:

1. **TTC normalization horizon (4.0 s) too long for the linear
   ramp.** `(4 - ttc) / 4` only reads 0.93 at sub-second TTC — no
   headroom above the abort threshold. Replaced with exponential
   `1 - exp(-2.0 / max(ttc, 0.1))`. Crosses 0.50 at ttc ~3 s
   (abort), 0.75 at ttc ~1.5 s (emergency-stable). Smooth ramp,
   no plateau.
2. **`flyby_d` damping suppresses risk against active blockers.**
   The damping factor reduced gap+TTC pressures by 0.32–0.88 when
   `pred_lat` was small. Intent was "ignore parallel fly-bys" but
   actively-mirroring blockers keep `pred_lat` small for the
   *wrong* reason. Added precondition: when
   `closing_speed_mps > 5.0`, return `flyby_d = 1.0` (no damping).
3. **No "no-escape" gate.** When all corridors are closed AND
   `gap_ok=0` AND `closing_flag=1`, the situation is committed-
   into-a-wall but the channel-max sometimes scored only ~0.6.
   Added a final hard floor: in that condition, force
   `risk = max(risk, 0.95)`.

Threshold harmonization (matched to the new exponential ramp):

- `pass_safe_risk_max`: 0.48 → 0.45 (don't commit if any TTC < 3.5 s)
- `abort_risk_01`: 0.55 → 0.50 (abort committed pass at TTC ~3 s)
- `emergency_risk_enter_01`: 0.85 → 0.75 (brake floor at TTC ~1.5 s)
- `emergency_risk_exit_01`: 0.55 → 0.50 (preserve 0.25 hysteresis)

**Files.**
`src/scenic/domains/racing/assessment/race_situation.py`
(`_compute_emergency_risk`, `_longitudinal_opening_dampen`),
`src/scenic/domains/racing/tactical_planner.py` (config),
`src/scenic/domains/racing/safety/stability_guard.py` (config).

**Live verification.** F14 risk now reaches 0.50 at the right
moment (~1.5 s before contact) and crosses the abort threshold;
SD-36 is needed for the abort to actually brake.

### SD-36 — ABORT_PASS brake authority

After SD-35 the planner FSM correctly transitioned to ABORT_PASS,
but ego still ran into the blocker because **`final_brake = 0`
throughout the abort window**. Root causes (both load-bearing):

1. **`_abort_result` returned `None` for `target_speed_cap`.**
   With no cap, `effective_target_speed` fell back to the racing-
   line target, MPC saw positive speed error, throttle stayed at
   1.0. New: cap = `max(3.0, opp_speed - abort_speed_margin_mps)`,
   so MPC sees a clear *negative* speed error and produces brake.
   New config field `abort_speed_margin_mps = 5.0` for tunability.
2. **Stability guard's brake floor wasn't reaching the actuator.**
   `stability_guard_enabled` defaults to `False` in
   `FollowRacingLineMPCBehavior` and **no production scenario
   enabled it** — the guard was dead code. F14 now opts in
   explicitly. Additionally added a panic-brake bypass in
   `behaviors.scenic` so when `emergency_stable_mode=True`, the
   guard's brake floor (0.30/0.45/0.60) reaches the actuator
   without being clipped by the upstream `brake_cap` (0.25).

**Files.**
`src/scenic/domains/racing/tactical_planner.py` (`_abort_result`,
new config), `src/scenic/domains/racing/behaviors.scenic`
(panic-brake bypass after `stability_guard_step`),
`examples/racing/f_shared/F14_fellow_ahead_active_blocker.scenic`
(`stability_guard_enabled=True`).

**Architectural note.** The `stability_guard_enabled=False` default
on the production behavior means the safety layer is opt-in per
scenario. This should probably be reversed (guard auto-enables when
`tactical_planner_enabled=True`) but that change risks regressing
S2 / F-bank scenarios that have implicit assumptions; deferred to
a later plan.

### SD-37 — Abort cooldown (with restorative gap growth)

F14 ego escaped the first commit but immediately tried the *other*
side, getting trapped in a left → abort → right → abort →
contact oscillation. SD-37 adds a cooldown after ABORT_PASS exits:
for `abort_cooldown_s` seconds, the strategy authority's
`pass_left` / `pass_right` selections are downgraded to
`follow_fellow`. New state field `last_abort_exit_s` stamps the
cooldown start at all 4 ABORT exit return paths.

**SD-37b refinement.** First cut used `abort_cooldown_s = 1.5` and
`cap = opp + 0.3` during cooldown FOLLOW. Live verification showed
the gap *shrank* during cooldown (10 → 8 m) because ego crept
forward at ~opp speed. Tightened to:

- `abort_cooldown_s`: 1.5 → 3.0 (more time for blocker mirror to
  settle)
- `cooldown_follow_speed_offset_mps`: new field, default `-1.5`,
  so during cooldown the cap is `opp - 1.5` (ego falls behind at
  1.5 m/s, gap GROWS by ~4.5 m over the 3 s window)
- `strategy_min_clearance_m`: 1.0 → 2.0 (one car-width margin
  required for a commit; rejects the marginal `1.13 m` clearances
  that were re-triggering commits right after cooldown expired)

**Files.** `src/scenic/domains/racing/tactical_planner.py`
(config + state + cooldown gate in `_strategy_to_planner_output`).

### SD-38 — Throttle/brake mutual exclusion + idle-creep brake floor

User noticed simultaneous throttle + brake outputs (e.g.
`final_throttle=0.20, final_brake=0.12`). Root cause: the
stability guard's `reapproach_hold` mode applied a throttle cap
AND a brake floor independently, with no mutual exclusion. The
existing `BRAKE_THROTTLE_EXCLUSION_THRESHOLD = 0.05` check at
`behaviors.scenic:2387` ran *before* the guard, so the guard's
combined output bypassed it.

Three changes:

1. **In-guard mutex.** When `reapproach_hold` floors brake,
   also zero throttle. Guard's outputs are now internally
   coherent.
2. **Defense-in-depth post-guard mutex.** After
   `stability_guard_step` and the SD-36 panic-brake bypass, run a
   final exclusion: if both `final_throttle > 0.05` and
   `final_brake > 0.05`, brake wins (throttle = 0). Catches *any*
   future source of conflict.
3. **Idle-creep brake floor at near-stop speeds.** When
   `guard_active` AND `current_speed < 2.0 m/s`, force
   `final_brake ≥ 0.20`. Counters dSPACE plant idle propulsion
   that would otherwise cause low-speed creep into a stopped
   fellow.

**Grade caveat (documented as known limitation).** The 0.20 floor
is a flat-road approximation. Real idle physics is *torque*, not
constant speed — uphill the engine drag dominates and ego
decelerates without brake; downhill gravity assists and the floor
may be insufficient. The longitudinal MPC at
`mpc_longitudinal.py:430` already does grade-aware
throttle/brake conversion (gravity compensation), but the SD-38
floor is grade-blind. Future improvement when the controller is
exercised on Laguna Seca's Corkscrew (~6% grade): make the floor
grade-aware via `floor = 0.20 + max(0, -grade_pct * 0.05)`.

**Files.** `src/scenic/domains/racing/safety/stability_guard.py`
(in-guard mutex), `src/scenic/domains/racing/behaviors.scenic`
(post-guard mutex + idle-creep floor).

### SD-39 — Less-aggressive commit + abort-exit TTL retention

Two coordinated fixes for the "ego accelerates into blocker
during commit, then swerves into the still-overlapping blocker
at abort exit" pattern:

1. **Lower `commit_speed_margin_mps`**: 16.0 → 5.0 (rev1) → 2.0
   (rev2). Default 16 made commits *accelerative races* — ego
   accelerated from `opp_speed` up to `opp_speed + 16 m/s` during
   the commit window, gaining closing momentum that the abort
   layer then had to fight. Each tightening helped some but the
   structural cause (MPC overshoot, see SD-40) means even tight
   margins can't fully fix it via this knob.
2. **Abort-exit TTL retention.** When ABORT_PASS exits and bbox
   is still overlapping (`overlap_hazard_raw=True`), the planner
   used to snap TTL back to "optimal" — causing ego to steer hard
   left/right toward optimal and *swipe into the still-touching
   fellow*. New `_abort_exit_ttl()` helper keeps the commit-side
   TTL while overlapping; reverts to optimal once bbox-clear.
   Applied at all 4 ABORT exit return paths.

Also tuned F14 specifically: ego `target_speed: 60 → 20 m/s`. With
the racing-line target lower, the slew limiter on
`effective_target_speed` has a much smaller range to traverse
during tactical commits (20 → 14 cap = 0.4 s slew vs 60 → 14 =
3 s slew). Reduces slew-induced overshoot from ~7 m/s to ~3 m/s.

**Files.** `src/scenic/domains/racing/tactical_planner.py`
(commit margin, `_abort_exit_ttl` helper, applied at 4 sites),
`examples/racing/f_shared/F14_fellow_ahead_active_blocker.scenic`
(target_speed).

### SD-40 — Brain-leg cohesion: hard-ceiling clamp after slew

F14 still showed contact even after all the above tunes. The user
identified the structural framing: **the planner is the brain, the
MPC is the leg; the brain's hard limits should be non-negotiable.**
Current behavior: planner sets `tactical_speed_cap = 12.3` (FOLLOW
mode), MPC ends up tracking 14.5+ because the slew limiter on
`effective_target_speed` (`behaviors.scenic:1711`) takes 1+ seconds
to drag the MPC's tracked target down to the planner cap. During
that lag, the leg accelerates while the brain is screaming "follow!".

SD-40 adds a hard-ceiling clamp *after* the slew:

```python
if _p3cap is not None:
    effective_target_speed = min(effective_target_speed, float(_p3cap))
```

Slew is preserved for **smoothness** (gradual changes still smooth,
upward changes still rate-limited), but the planner's tactical cap
is enforced as an immediate ceiling. The MPC's tracked target can
never exceed what the brain currently forbids.

**Files.** `src/scenic/domains/racing/behaviors.scenic` (one block
just after the existing slew at line 1711).

**Open issue.** Live verification of the post-SD-40 run still shows
contact. The brain-leg disconnect is structurally addressed at the
target-speed level, but other failure modes remain (likely strategy
selector picking wrong mode, or non-determinism in scene generation
producing harder F14 instances). A structural follow-up is planned
to investigate end-to-end.

### F14 + S3 + FellowActiveBlockBehavior — adversarial blocker test

New negative-passing-test scenario. Fellow has full ego visibility
and uses it adversarially: each control tick, projects ego xy onto
its TTL, takes the resulting track-frame lateral as its target `d`,
clips to `±max_lat_offset_m`, and slews `d` toward target with
bounded lateral velocity. Speed tracks ego speed plus an offset
(default `-5 mph`), clamped. Asymmetric information by design — ego
can't read blocker's internal target.

Movement physically plausible: rate-limited via the existing
`_swerve_oc_slew_d_toward()` primitive (3 m/s lateral cap → ≤0.03 m
per 10 ms tick → no teleporting).

**Files.**
`src/scenic/domains/racing/fellow/commands.py`
(`compute_active_block_plant_command`),
`src/scenic/domains/racing/behaviors.scenic`
(`FellowActiveBlockBehavior`),
`src/scenic/domains/racing/fellow/__init__.py` (export),
`examples/racing/f_shared/F14_fellow_ahead_active_blocker.scenic`
(F-bank regression scenario), `src/scenic/domains/racing/benchmarks/f_scenario_bank.py`
(register F14),
`examples/racing/falsifiable/S3_blocker_falsify.scenic`
(falsifiable variant for VerifAI campaigns).

### SD-37 logging cleanup (Phase 1)

While debugging F14, the log volume made traces hard to read. Three
consolidations:

1. **`[EvalEvent]` and `[EvalEventDiag]` collapsed into one.** The
   diag line was paired 1:1 with EvalEvent and only added
   `ego_speed_mps`, `opp_center_dist_m`, `rel_speed_mps`,
   `rel_longitudinal_m`, `overlap_state`, `seg`, `ahead_hint` —
   folded into EvalEvent's fields. Saves ~156 lines per F14 run.
2. **`[EvalContact]` deleted.** Listed in `_RECORD_TAGS` for
   documentation but never read by `_records_extract`. Strict
   dead code.
3. **Legacy `[Phase2]` (failure-path) and `[Tactical]` (TTL switch
   + periodic mode dump) emissions deleted.** Both already covered
   by `[Assessment]`, `[Phase0Event]`, and `[CtrlTrace]`.

**Files.** `src/scenic/domains/racing/behaviors.scenic` (4 sites),
`src/scenic/domains/racing/benchmarks/metrics.py` (`_RECORD_TAGS`).
Backward-compatible — no downstream parser breakage.

### SD-41 — Brain-leg architectural refactor (IAC-standard reference trajectory)

After SD-30..40 stacked nine band-aids (curvature clip, gap guard,
side guard, risk overhaul, abort cap, abort cooldown, mutex layers,
commit margin tune, hard-ceiling clamp) trying to enforce planner
intent at the actuator, the user crystallized the structural framing:
*"the brain says one thing, the leg does another."* Each fix
addressed a specific symptom; the underlying disease was that the
planner emitted a sparse goal (`mode, ttl, cap, reason`) and the MPC
had to reconstruct intent through a chain of caps, slew limiters,
throttle ramps, and brake clips. Every accumulated layer was a
potential disconnect.

**Architectural target** (per IAC-standard literature — TUM, KAIST,
F1Tenth):

```
PLANNER ──► PlannerReference (7-col, N-horizon) ──► SAFETY SUPERVISOR ──► MPC
                                                          │                 │
                                                          ▼                 ▼
                                                   reference swap        pure tracker
                                                   (e.g. safe-stop)      (no override)
```

Planner emits a **dense reference trajectory** per tick: the
`PlannerReference` dataclass with `s_m / x_m / y_m / psi_rad /
kappa_radpm / vx_mps / ax_mps2 / traj_id / t_planner_stamp_s / mode /
decision_reason / is_safe_stop / binding_cap_source / ttl_key`. MPC
consumes the trajectory directly with no override authority over
strategic intent. The stability guard sits in a third channel that
*can* swap the reference (e.g., to a safe-stop trajectory) but never
modifies MPC outputs.

Sources: TUM Autonomous Motorsport (J. Field Robotics 2023, arXiv
2205.15979); KAIST IAC team (arXiv 2303.09463);
TUMFTM/global_racetrajectory_optimization (7-column trajectory
schema); Hierarchical Motion Planning for Autonomous Racing
(arXiv 2003.04882).

The refactor was staged so each step was independently committable
and gated on F-bank verification. Stages D and E shipped in modified
form because of two discovered constraints:

#### Stage A — `PlannerReference` dataclass + planner package

New module `src/scenic/domains/racing/planner/`. Defines the
7-column dataclass plus `is_stale()` and `horizon_length()` helpers.
Pure type definition — no behavioral effect.

#### Stage B — Planner emits dense reference; smart commit cap

Two pieces:

1. **`_smart_commit_cap()`** in `tactical_planner.py`: when fellow
   speed is below a stationary threshold (~3 m/s), the
   `COMMIT_PASS_*` cap returns the racing-line target instead of
   `max(3.0, opp+commit_margin) = 3.0`. F9's brake-for-no-reason
   during a stationary-blocker pass is fixed at the planner level.
   Above the threshold, behavior is identical (F2/F14 commit caps
   unchanged).

2. **`build_reference()`** in `tactical_planner.py`: composes all
   caps (cte / curvature / global / scripted / tactical) into
   `vx_mps[0]`, integrates `s_m` over the MPC horizon, looks up
   `(x,y) / psi / kappa` along the chosen TTL polyline, derives
   `ax_mps2`. Returns a fully-populated `PlannerReference`.
   Behaviors.scenic stores the result on `self._planner_reference`
   each tick. A new `[PlannerRef]` log line emits per tick with
   `vx0` (planner-composed cap, no slew) alongside `eff` (post-slew
   effective) for diff'ing the brain-leg gap.

   F9 verified: COMMIT cap rises from 3.0 to 44.0; ego completes
   pass with no contact.

#### Stage C — Longitudinal MPC consumes the reference; SD-40 clamp deleted

`v_ref_profile` is sourced from `PlannerReference.vx_mps` directly
when the tactical planner is on. The planner already composed all
caps with no slew or hard-ceiling band-aid, so the MPC tracks the
brain's intent at the same tick it's set. The per-step curvature
reduction still runs on top, so apex constraints still apply.

The SD-40 hard-ceiling clamp on `effective_target_speed` is deleted.
It was forcing `eff <= planner_cap` in one tick to defeat the slew
limiter; with `v_ref_profile` now sourced from the reference, the
slew limiter no longer affects what the MPC tracks in tactical mode.

The MPC fallback PID also reads `v_ref_profile[0]` instead of the
legacy `effective_target_speed` so brain intent is honored even on
solver failure.

The slew limiter, `_speed_caps` dict, and `effective_target_speed`
remain (they still feed the legacy non-tactical fallback path used
by F2-style scenarios). In tactical mode they are telemetry-only —
the `[PlannerRef]` log's `eff` field now exposes the brain-leg gap
so we can verify it is no longer reaching the actuators.

F14 trace confirms the gap (e.g. `ABORT_PASS` at t=3.45s with
`vx0=3.0` vs `eff=8.02` — 5 m/s of slew-induced drift); MPC tracks
`vx0` directly via `v_ref_profile`.

#### Stage D — DEFERRED (lateral MPC consumes reference)

Plan called for `mpc_lateral.py` to take `psi_ref / kappa_ref / v_ref`
from `PlannerReference` instead of computing them internally via
`ReferenceBuilder`. Skipped after Stage C analysis:

- `v_ref` is already flowing through `PlannerReference` to lateral
  MPC (via the `v_ref_profile` kwarg from Stage C — same array).
- The remaining piece (`psi_ref / kappa_ref / x_m / y_m`) is currently
  produced by `ReferenceBuilder` using ~900 lines of spline fitting +
  curvature smoothing; replacing with `build_reference`'s naive
  forward-difference values would be a quality regression for lateral
  tracking.
- The architectural value of Stage D is *smooth lateral merges
  during* `COMMIT_PASS_*` (per-step blend of optimal→side TTL using
  `strategy_lane_change_tau_s`), which is a feature, not plumbing.
  Plain plumbing without the merge feature trades quality for purity.

Disposition: revisit Stage D only if/when planner-shaped lateral
merges become a desired feature; otherwise delete from the plan.

#### Stage E — Safety supervisor (skeleton)

Planned: pre-MPC reference-swap channel where the guard, on
predicted collision, replaces `PlannerReference.vx_mps` with a
linear ramp-to-zero, MPC tracks the ramp naturally → structured
deceleration with no post-MPC clipping required.

Two implementation attempts both regressed F14: the safe-stop ramp's
tail-of-horizon zeros caused the MPC to brake more aggressively than
the planner's `ABORT_PASS` already wanted, perturbing ego's
trajectory enough to trip a pre-existing pit_mode false positive in
the segment classifier (SD-12a).

Bisection of three F14-only back-to-back runs revealed that **F14
has plant-level non-determinism**: 1 cm cte divergence at t=3.1s
amplifies through closed-loop chaos to 85 / 65 / 0 contacts across
three identical-code, identical-init runs. Python is bitwise
deterministic on identical inputs; the noise originates in the
dSPACE plant readback (likely floating-point ordering, IPC timing,
or tire/suspension hysteresis in VEOS).

Stage E ships as a **skeleton**:

- **Helpers shipped** in `safety/stability_guard.py`:
  `should_swap_for_emergency()` (trigger logic mirroring the
  existing `emergency_trigger`) and `swap_reference_for_emergency()`
  (returns a `min(planner_vx, ramp_to_zero)` profile preserving
  lateral state).
- **Auto-enable rule** in `behaviors.scenic`: when
  `_tactical_planner_enabled = True` and the guard isn't explicitly
  set, auto-enable it. Override via `--param
  stability_guard_auto_enable False`. F9 was unset → it now gains
  the guard (verified firing in the new run).
- **Pre-MPC swap call NOT wired**. The post-MPC SD-36 panic-brake
  bypass continues to provide emergency authority and is preserved.

Disposition: when we want a smoother safe-stop profile that doesn't
perturb ego's trajectory (e.g. constant `min(planner_vx, current_v *
0.7)` instead of a ramp-to-zero), the helpers are ready to wire.

#### Stage F — Cleanup (minimal)

Only one truly dead block was removed: the `"tactical"` branch of
SD-32A's safety-binding slew bump in `behaviors.scenic`. After
Stage C wired `v_ref_profile` straight from the reference, the slew
limiter no longer affects what the MPC tracks in tactical mode — it
only mutates the telemetry-only `effective_target_speed` scalar. The
SD-32A bump's "tactical" check was doing pointless work each tick.
The "curvature" check stays load-bearing for non-tactical scenarios.

Other plan-listed deletions (SD-36 panic-brake bypass, SD-38
post-guard mutex, CTE-driven throttle override) are NOT applied
because they all assumed Stage E's pre-MPC reference-swap was wired.
With Stage E shipped as a skeleton, those band-aids continue to
provide load-bearing emergency authority and stay in place.

#### Net architectural state after SD-41

| Concern | Pre-SD-41 | Post-SD-41 |
|---|---|---|
| Planner output | `(mode, ttl, cap, reason)` tuple | Dense `PlannerReference` (7-col, N-horizon) |
| MPC v_ref source | `effective_target_speed` after `min()` chain + slew + SD-40 clamp | `PlannerReference.vx_mps` directly |
| Cap composition site | `behaviors.scenic` (after planner emits scalar cap) | `tactical_planner.build_reference()` (planner owns) |
| Brain-leg cap delivery latency | Up to 1+ second slew lag | Same tick (no slew, no clamp) |
| F9 stationary-blocker brake | Capped at 3 m/s during pass | Racing-line speed during pass |
| Safety guard channel | Post-MPC command clip only | Helpers exist for pre-MPC swap (not wired); post-MPC clip retained |
| Lateral profile | TTL waypoints via `ReferenceBuilder` (unchanged) | TTL waypoints via `ReferenceBuilder` (Stage D deferred) |
| Telemetry | `_speed_caps` dict | `[PlannerRef]` log + `binding_cap_source` field |

**Files.**
- New: `src/scenic/domains/racing/planner/__init__.py`,
  `src/scenic/domains/racing/planner/planner_reference.py`.
- Modified: `src/scenic/domains/racing/tactical_planner.py`
  (`build_reference()`, `_smart_commit_cap()`),
  `src/scenic/domains/racing/behaviors.scenic` (consumer wiring,
  auto-enable, SD-32A "tactical" branch removal),
  `src/scenic/domains/racing/safety/stability_guard.py`
  (supervisor helpers).
