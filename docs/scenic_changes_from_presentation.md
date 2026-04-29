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
