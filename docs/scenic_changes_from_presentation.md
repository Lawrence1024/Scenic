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

**Stages landed.**

- **SD-25a — strategy_selector.py:95 tiebreak.** Replaced the
  single-key `min(tied, key=TIEBREAK_RANK)` with a tuple-keyed
  comparison: `(TIEBREAK_RANK, -min_clearance_m)`. For pass_left vs
  pass_right at rank 1, the higher-clearance side wins via the
  negated-clearance secondary. For all other comparisons (different
  ranks), the rank dominates the tuple — behaviour unchanged.
  Symmetric: works for fellow-on-left and fellow-on-right scenarios.
- **SD-25d — offline regression bank.** Added
  `src/scenic/domains/racing/benchmarks/sd25_selector_unit_bank.py`
  with hand-crafted `StrategyOutcome` cases asserting the tiebreak
  picks higher clearance AND that all pre-SD-25 behaviours are
  preserved. No simulator, no Scenic compile; single-command
  runner. Verified: 11/11 pass after both A and B landed.
- **SD-25b — closing-flag gate for stay_optimal.** Added kwarg
  `closing_on_current_line: bool = False` to `select_strategy`.
  When True (set by tactical_planner from
  `assessment_closing_flag`), `stay_optimal` is excluded from the
  primary survivor set. The hard filter then forces escalation —
  to a pass strategy if any side has ≥2.5 m clearance, else
  soft-fallback to `follow_fellow`. Last-resort `stay_optimal` at
  the bottom of the fallback ladder is unchanged (the SD-4
  emergency brake is the no-good-option safety net).

**Stage REVERTED.**

- **SD-25c — abort_speed_margin_mps speed cap.** Originally landed
  in commit `0fc64781`: `_abort_result` returned
  `cap = opp_speed_mps + abort_speed_margin_mps` (default 2.0 m/s)
  instead of `None`. Reverted in commit `ae7b7f87` after a 30-sample
  re-run surfaced a worse failure mode.

  **What went wrong.** The 30-sample CE re-run at `--seed 42`
  showed multiple severe off-track events that didn't exist in
  the baseline:
    | Sample | Off-track? | track_clearance_m | Notes |
    |---|---|---|---|
    | #003 | TRUE | -0.43 m | mild |
    | #004 | TRUE | **-8.77 m** | severe |
    | #010 | TRUE | **-20.4 m** | catastrophic, also collision |

  Sample #10 traces show the regression mechanism precisely.
  At t=9.80 s, abort fired (`commit_invalidated_hazard`) mid-corner.
  The `[Planner]` line at that tick reads
  `target_speed_cap=10.78 = opp_speed (8.78) + abort_speed_margin_mps (2.0)`.
  Ego state at t=10.00 s: speed=26.09 m/s, gear=3, segment "main
  curve", CTE=-0.54 m. The MPC was now asked to brake from 26 →
  11 m/s **while** still tracking the LEFT TTL (per SD-2d's
  keep-commit-side-during-side-by-side) **through a corner**. The
  combination is dynamically infeasible — ego understeers, builds
  cross-track error from -0.54 m → -8 m, and exits the corner
  ~20 m off the track surface.

  **Why the original intent was right but the implementation wasn't.**
  Sample #4 of the baseline run rear-ended fellow 1.15 s after a
  correctly-detected abort because ego coasted at target_speed
  during recovery. The intent — don't keep accelerating into the
  fellow during abort — is correct. The implementation — instant
  cap from racing speed to fellow_speed+2 m/s — is too aggressive
  for mid-corner aborts. Two design directions for a future cycle:
    1. Apply the cap only when ego is on the OPTIMAL TTL during
       abort recovery, not when keeping the commit-side TTL. The
       commit-side path is already the side-by-side recovery
       trajectory; layering a hard speed cut on top breaks the
       MPC.
    2. Use a gentle deceleration profile (rate-limited cap)
       rather than an instant target. Gives the MPC time to
       follow a curve at decreasing speed.

  Both require more design work than a one-line patch. Deferred.
  Sample #4's original rear-end re-emerges as a known issue — a
  single occurrence in 30 samples, vs. multiple severe off-track
  events the speed cap caused. Net safety win to revert.

**The falsifier did its job.** SD-25 is the first cycle where the
falsification framework surfaced a regression that a unit bank
couldn't have caught. The 11/11-pass selector unit bank doesn't
exercise abort recovery dynamics; only a real simulation through
the cosim bridge can. Reading the per-sample log for the
worst-rho violation pinpointed the regression in ~30 minutes —
the framework's value is exactly that.

**End-state files (post-revert).**

- `src/scenic/domains/racing/planner/strategy_selector.py` — SD-25a tiebreak + SD-25b closing-flag kwarg.
- `src/scenic/domains/racing/tactical_planner.py` — SD-25b passes `closing_on_current_line` to selector. The SD-25c `abort_speed_margin_mps` field is removed; `_abort_result` returns `cap=None` again.
- `src/scenic/domains/racing/benchmarks/sd25_selector_unit_bank.py` — 11/11 pass.

**Verification (user-side).** Re-run the same 30-sample CE
campaign at `--seed 42`:

```powershell
python src/scenic/domains/racing/benchmarks/verifai_runner.py `
    examples/racing/falsifiable/S1_falsify.scenic `
    --sampler ce --monitor safety --count 30 --seed 42 --time 3000 `
    --quiet *>sd25_postrevert.log
```

Expected: collision count drops from 13/30 baseline → ~3/30
(SD-25a fixes the 10 dominant pass_left bias cases; SD-25b fixes
the 2 stay_optimal rear-ends; sample #4's abort coast remains as
a known issue pending a smarter abort cap design).
