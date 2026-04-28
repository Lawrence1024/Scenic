# Code-change list surfaced while preparing the Scenic core review presentation

A running ledger of code warts and design questions that came up while
writing the talk. None of these are blockers for delivering the talk;
they're follow-up work where rehearsing the explanation forced an
honest look at the current implementation.

Ordered roughly by how badly they bite. Each item: the current
behavior, why it's wrong (or worth questioning), and the rough fix
shape.

---

## 1. mainTrack / pitTrack don't use XODR road width

**Current:** `create_track_regions()` in
`src/scenic/domains/racing/segments/track_regions.py:275-352` builds
the regions by taking the **centerline** of each `_mainRacingRoads`
road and **buffering it by ±6 m** (default `mainTrackBuffer`). Same
shape for `pitTrack` with ±1.5 m.

**Why it's wrong:** XODR encodes per-station lane widths; each `Road`
in Scenic's parsed Network has a `polygon` attribute representing the
actual drivable area. We discard that and use a constant-radius
symmetric buffer instead. So our current `mainTrack` treats the LGS
track as if it were 12 m wide everywhere — which is roughly right on
straights but wrong at corner apexes where the racing surface
narrows. It's also symmetric whereas real tracks are not.

**Fix shape:** replace
```
buffer(road.centerline, mainTrackBuffer)
```
with
```
union of (road.polygon for road in _mainRacingRoads)
```
or use the road's `drivable_polygon` if exposed by the driving
domain. Verify visually against race_common's `outside.csv` /
`inside.csv` geofences (the comparison is already partially in
place, see `track_regions.py:290-292`).

**Why we hadn't done it:** historical accident. The first version
worked off TTL CSV centerlines (no width), and we just buffered. When
we migrated to XODR-native (Phase B.5), we kept the buffer logic.

---

## 2. ttlRegion buffer is too large for "place on TTL" semantics

**Current:** `ttlRegion(file)` in
`src/scenic/domains/racing/model.scenic:106-111` and
`create_ttl_region_from_file()` at
`segments/track_regions.py:243-272` buffer the TTL CSV centerline by
**±6 m** (same `mainTrackBuffer` default).

**Why it's wrong:** the per-vehicle TTL placement default
(`position: new Point on ttlRegion(self.ttlFileName)` on
`RacingCar`) was meant to make `new RacingCar` land **on** the racing
line, not in a 12 m-wide envelope around it. With ±6 m, sampling is
effectively over the whole track surface — same as `mainTrack`.

**Fix shape:** distinguish two intents:
- "On the TTL polygon" (driving anywhere within the racing-line
  buffer) — keep something like ±1.5 m so the car is recognizably
  *near* the line.
- "On the TTL polyline" — sample directly on the centerline (zero
  buffer). This composes with `frenetOffset` (Ask 1 in the deck)
  if/when Scenic core absorbs Frenet.

Pick one as the default; ship a parameter for the other.

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

**Current:** `RacingTrack` (`segments/tracks.py:183-852`) is a
Python class wrapping Scenic's `Network`. It's exposed as `_track`
in `model.scenic`, but `_track` is a Python object, not a Scenic
`Region`, so:
```scenic
new RacingCar on _track          # doesn't work today
```
fails. Users must use the derived regions (`mainTrack`,
`pitTrack`, `ttlRegion(file)`) instead.

**Fix shape:** either
- Make `RacingTrack` Region-compatible (subclass `Region` or
  expose `track.region` returning the union of all main + pit
  road polygons), OR
- Add a top-level `raceTrack` alias that resolves to
  `mainTrack ∪ pitTrack`.

~5 LOC either way.

---

## 6. BoundsCheck off-track measurement is unreliable (TWO compounding issues)

**Symptom:** the 50-sample CE campaign at
`results/verifai_20260428_052048/` flags 50/50 samples as off-track,
but the dSPACE viewer shows ego visually within the track surface
across the whole campaign. The off-track signal is broken.

**Issue A -- frame mismatch.** Every `[BoundsCheck]` line carries a
`residual_mag` between two estimates of ego position:
- `pos=(...)` -- what the simulator passed to `compute_bounds_distance`,
  derived from the dSPACE actor's reported position (in whatever
  frame the simulator stores)
- `xodr_from_gps=(...)` -- what GPS readback would give, projected
  into the LGS_v1 XODR frame via `geoReference`

In the 50-sample run, this residual is **constant ~10 m across the
entire lap**, with `pos` consistently +9 m east and +5 m north of
`xodr_from_gps`. Constant magnitude + constant direction is the
signature of a frame-transform offset, not noise. So the ego
position passed to BoundsCheck is offset from the true position by
~10 m, which is enough to push samples spuriously across the inner
geofence.

**Issue B -- `d_in = 0` triggers OUT, but cutting inside an inner
geofence on a racetrack is normal.** `bounds_check.py:173` defines:
```
in_track = (point inside outer polygon) AND (point outside inner polygon)
```
That sets `in_track=False` whenever ego touches the INNER boundary --
i.e., when ego cuts an apex or rides the inside curb. Even a clean
late-apex line on a corner gets flagged. In the 50-sample run, ego
trips this gate with `d_out ≈ 10 m` (lots of margin to the OUTER
edge, where "off the track" actually means).

**Net effect:** 50/50 off-track is a measurement artifact. The
collision metric (29/50) and left/right asymmetry (28 left-only /
0 right-only) are unaffected -- those use OBB intersection in the
same coordinate frame as ego/fellow positions, so any frame offset
cancels. The slides have been updated to drop the off-track claim
while keeping the collision findings.

**Fix shape (two pieces):**
1. **Frame calibration.** Confirm which frame the simulator's actor
   `position` is reported in (RD-frame? Final.xodr-frame?
   LGS_v1-frame?). Apply the right transform before calling
   `compute_bounds_distance`. The user's memory note already
   documents `LGS_v1 ↔ RD = (-6.101, -50.761)` -- the residual we
   see (~+9, +5) doesn't match that exactly, suggesting yet another
   frame in the chain. This is a calibration audit task.
2. **Semantic fix on off-track.** Either:
   - Only flag OUT when `d_out` collapses (ego left via the OUTER
     edge); leave `d_in=0` as a separate diagnostic, OR
   - Regenerate `track_inside.csv` to follow the actual physical
     inside boundary of the track surface (typically the inside
     curb / pit wall), not the racing-line "do not cut here" line
     race_common may currently encode.

Until either piece lands, the `track_clearance_m` field in
`SampleMetrics` and the `off_track` boolean both produce
unreliable readings, and `monitors.track_clearance` /
`monitors.safety_min` should not be trusted for falsification on
this map. The `monitors.collision_robustness` (driven by
`bbox_gap_m_min`) remains valid.

**How we caught it:** comparing visualization (ego visibly on
track) against the parsed log (50/50 off-track) during slide
review. Without the visual sanity check, we'd have published the
100% off-track number as a real finding.

---

## 7. verifai_runner should print sample-progress to the terminal even with `--quiet *>file`

**Symptom:** running the falsifier with the recommended PowerShell
invocation
```
python verifai_runner.py ... --quiet *>run.log
```
buries every `[VerifaiRunner] === sample N/M ===` line inside the
log file. There is no terminal output at all during the ~80 min
run, so the user can't tell which sample is currently running
without tail-ing the file.

**Fix shape:** route the `[VerifaiRunner] sample-progress` print
calls through `sys.__stdout__` (the original, pre-redirect stdout)
instead of plain `print()`. PowerShell's `*>` only redirects
`sys.stdout` / `sys.stderr` -- writes to `sys.__stdout__` bypass it
and reach the terminal directly.

Concretely, in `src/scenic/domains/racing/benchmarks/verifai_runner.py:main()`
at the per-iteration progress lines, do:
```python
sys.__stdout__.write(f"[VerifaiRunner] === sample {i+1}/{count} ===\n")
sys.__stdout__.flush()
```
instead of `print(...)`.

Pure housekeeping; no functional change. ~5-line edit.

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
