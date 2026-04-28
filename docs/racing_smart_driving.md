# Racing — smart driving on one opponent (SD-* cycle)

**Status:** SD-3 structural redesign landed (`milestone-smart-drive-redesign` to be tagged after F-bank validation).

## Pre-SD baseline (from full_stack_20260427_031327)

| Scenario | Collision | Off-track | Commits attempted | Note |
|---|---|---|---|---|
| F1 (fellow behind) | False | False | 0 | correct — no overtake needed |
| **F2 (slow ahead, optimal)** | False | False | **0** | **primary SD target** |
| F3L (slow ahead, left TTL) | False | False | 81 | works today; SD must not regress |
| F3R (slow ahead, right TTL) | False | False | 53 | works today; SD must not regress |
| F4 (sudden stop) | False | False | 0 | emergency-brake path; SD must NOT trigger commits here |
| F5 (swerve out) | False | False | 19 | works |
| F6 (left occupied) | False | False | 117 | works |
| F7 (right occupied) | False | False | 92 | works |
| **F8 (corner-entry+ahead)** | **True** | False | 259 | secondary SD target — collision @ t=7.85s, 12 m OOB |
| F9 (stationary obstacle) | False | False | 56 | works |

## Cycles

### SD-2 (knob-twisting cycle, 2026-04-26 → 2026-04-27)

A series of small reactive fixes attacking the F2_tactical contact: corridor hysteresis,
asymmetric corridor opening, pass_safe re-routing, abort TTL hold, realistic overtake
tuning, SETUP gap gate, matched-speed FOLLOW unblock. Each fix exposed the next
failure mode. Final SD-2g run still ended in contact — the planner waited at matched
speed in FOLLOW for 13 sec then rear-ended fellow.

Lesson recorded: this was structurally the wrong approach. The planner was making
overtake decisions reactively without ever asking "given the upcoming TTL geometry
and our speed differential, can this pass actually complete safely?".

### SD-3 (structural redesign cycle, 2026-04-27)

Rebuilt the overtake decision around three structural invariants:

1. **Δv-derived initiation distance** (SD-3b). Setup/commit gap thresholds are
   now `clamp(floor, ceiling, slope·Δv + intercept)` — high Δv means swerve out
   far (gap closes fast), low Δv means wait closer (lateral shift will be ready
   before alongside). Replaces the SD-2e/2f constants.

2. **Geometric look-ahead** (SD-3a + SD-3c). New `pass_window_check(side, …)`
   helper walks ego on the candidate side TTL and opp on optimal forward in
   arc-length, samples every 0.25 s, returns False if min-clearance over the
   2.5 s window drops below `min_lat_clearance_m=1.6`. Wired into SETUP entry
   so the F2_tactical "right TTL converges into corner" failure is now
   detected and rejected before initiating any lateral shift.

3. **HOLD-on-pass-side phase** (SD-3d). After COMMIT relation-flip-behind, ego
   no longer instantly switches to optimal TTL (the bug that caused most contact
   events). Instead, ego enters `HOLD_PASS_{LEFT,RIGHT}` and stays on the side
   TTL until BOTH (a) `delta_s_behind ≥ hold_release_long_m(Δv)` AND
   (b) `pass_window_check("merge_back", …)` returns OK. Real geometric exit
   condition, not a constant `K·Δv`.

#### State machine

```
FREE_RUN ─→ FOLLOW ──→ SETUP_PASS_{L,R} ──→ COMMIT_PASS_{L,R}
                ↑       │                    │
                │       ↓ (look-ahead fail)  ↓ (relation flips behind
                │       │                    │  AND still side-by-side)
                │       │                    ↓
                │       │              HOLD_PASS_{L,R}    ← NEW (SD-3d)
                │       │                    │
                │       ↓                    ↓ (merge-back safe)
                └──── ABORT_PASS ←───────────┴──────→ FREE_RUN
```

#### Δv-derived formulas

`Δv = max(0.5, ego_speed_mps - opp_speed_mps)` (floored).

| Gate | Formula | Δv=2 | Δv=5 | Δv=15 |
|---|---|---|---|---|
| SETUP entry max gap | `clamp(12, 42, 1.4·Δv + 14)` | 16.8 m | 21.0 m | 35.0 m |
| COMMIT entry max gap | `clamp(8, 30, 0.9·Δv + 8)` | 9.8 m | 12.5 m | 21.5 m |
| HOLD release long gap | `6.4 + 0.3·Δv` | 7.0 m | 7.9 m | 10.9 m |

Anchor at observed F2 successful pass (Δv ≈ 5 m/s).

#### Pass-window look-ahead

`assessment/pass_geometry.py:pass_window_check(side, …)`. Defaults:
- `pass_duration_s = 2.5` (observed F2 successful pass alongside window 2-3 s)
- `sample_dt_s = 0.25` (10 samples)
- `min_lat_clearance_m = 1.6` (IAC half-width 0.96 m × 2 + 0.5 m buffer)

Three modes: `"left"` / `"right"` / `"merge_back"` (used by HOLD exit gate).

#### Speed caps by phase

| Phase | Cap | Notes |
|---|---|---|
| FREE_RUN | none | drive optimal at target_speed |
| FOLLOW | `opp + 2.5` | gentle slipstream approach |
| SETUP | `max(3.0, opp + 4.5)` | enables real closing rate |
| COMMIT | `max(3.0, opp + 8.0)` | bounded fast pass; SD-2e |
| HOLD | `max(ego_at_entry, opp + 1.5)` | freeze gain, no merge-cut acceleration |
| ABORT | none | brake-for-distance authority |

#### Files

- `src/scenic/domains/racing/assessment/pass_geometry.py` — NEW `pass_window_check`
- `src/scenic/domains/racing/tactical_planner.py` — Δv helpers + HOLD branch + look-ahead wiring
- `src/scenic/domains/racing/behaviors.scenic` — threads polylines + ego/opp s into planner
- `src/scenic/domains/racing/mpc/testing/test_pass_geometry.py` — 5 unit tests
- `src/scenic/domains/racing/mpc/testing/test_tactical_planner.py` — 4 new SD-3 tests

#### Verification

```powershell
# F2_tactical end-to-end (PowerShell, Tee-Object for streaming):
scenic examples/racing/calibration/F2_tactical.scenic --2d `
    --model scenic.simulators.dspace.racing_model --simulate --count 1 --time 3000 `
    *>F2_tactical.log
```

```bash
# Convert UTF-16 → UTF-8, then grep:
python -c "open('F2_tactical_utf8.log','w',encoding='utf-8').write(
    open('F2_tactical.log','rb').read().decode('utf-16-le','replace'))"
grep -c "pass_window_unsafe" F2_tactical_utf8.log    # ≥1 means look-ahead firing
grep "Tactical.*HOLD_PASS" F2_tactical_utf8.log       # HOLD entered
grep "decision_reason=hold_release_merge_safe"        # clean release
grep -c "decision_reason=contact_recovery_hold"       # MUST be 0
```

#### Acceptance gates

- F2_tactical: collision=0, ≥1 `pass_success` event, ≥1 HOLD entry
- F3L/F3R: commit counts within ±20% of pre-redesign baseline
- F4: collision=0, no pass attempts (look-ahead should reject)
- F1/F5/F6/F7/F8/F9: collision unchanged from baseline

### SD-10 (surgical restructure + perf cycle, 2026-04-27)

After 9+ SD-N patch cycles, F-bank evidence flagged structural problems:
F7 ttl ping-pong + 287× pit_mode_guard, F9 false predicted_collision parking ego at v=0,
plus a runtime regression (~140ms tick_ms vs ~10ms budget). Six surgical stages, each
independently revertible:

- **SD-10a** Dead-code purge: surfaced 2 hardcoded hysteresis literals as
  `setup_entry_persistence_cycles` / `follow_pressure_threshold_cycles` config fields.
- **SD-10c** F7 fix: pit_mode hysteresis (3-tick consecutive) + `setup_min_dwell_s`
  prevents transient pit_mode flicker from cancelling SETUP within 0.75s.
  F-bank: 287× pit_mode_guard → 0; ttl ping-pong gone.
- **SD-10d** F9 fix: stationary-fellow PathPredict bypass when
  `opp_speed ≤ stationary_opp_speed_mps AND |lateral_m| > stationary_overlap_relief_lateral_m`.
  F-bank: 292× false predicted_collision → 0; ego no longer parks at v=0.
- **SD-10b** Unify SETUP entry counters: `pass_intent_candidate_count` +
  `setup_commit_candidate_count` → single `opening_confidence_count`. Cuts SETUP
  arming latency from 4 ticks to `max(pass_intent_entry_cycles, setup_commit_entry_cycles)`
  (=2). Required moving the `setup_max_hold_s` timeout check upstream of `commit_active`
  so the faster arming can't preempt it.
- **SD-10e** Time-headway adaptive FOLLOW cap: 3-band hysteresis around
  `target_gap = max(follow_tight_gap_m, ego_speed * follow_time_headway_s)`.
  Per published racing literature; replaces fixed `opp + 2.5` cap that ping-ponged
  near the gap boundary.

#### Performance cycle (parallel to behavior fixes)

- **SD-10g** Per-tick wall-clock instrumentation: `[TickTime]` log line.
- **SD-10h** Cache Shapely LineString in `pass_geometry._xy_at_arclength` keyed on
  `id(waypoints)`. Per-call: 1.875 ms uncached → 0.02 ms cached (94×).
- **SD-10i / SD-10l** Per-section `[TickBreakdown]` instrumentation (segmap, assess_opp,
  predict, assess_race, planner, lon, lat, other). Found the ttl_switch lag was
  ~500 ms `build_waypoint_segment_map_from_ttl` rebuild, not Shapely or OSQP.
- **SD-10j** Cache LineString + lap-length in `situation_assessment._arc_length_project_xy`
  / `polyline_lap_length_m`. Hybrid cache key `(id, n, first_xy, last_xy)` — `id` alone
  is unsafe because Python recycles ids after GC.
- **SD-10o** Warm-start delays: when `_ensure_cosim_started` detects an existing IPC
  client pid alive, `_setupRun` uses shortened "warm" pauses
  (`pre_download_delay_warm_s`, `post_modeldesk_download_delay_warm_s`,
  `post_connect_settle_warm_s`). Cold start: 30 s of fixed sleeps; warm start: 2 s.
  Saves ~28 s per warm scenic invocation.

Reverted (failed approaches kept in history for the lessons):
- **SD-10k** Pre-warm Shapely caches at TTL preload — no measurable effect on the
  ttl_switch lag, confirming Shapely was not the bottleneck. Pointed at SD-10l
  instrumentation to find the real culprit.
- **SD-10m / SD-10n** Pre-build segment maps per TTL — user noted the racing track
  and segments should be XODR-derived (not TTL-keyed). Cleaner approach deferred
  to a future cycle (segment lookup keyed on road/arc-length rather than per-polyline
  waypoint index).

#### F-bank result (full_stack_20260427_230918, all 10 scenarios)

| Scenario | Collision | Note |
|---|---|---|
| F1 | False | correct (fellow behind) |
| F2 | False | overtake successful (1 pass_success) |
| F3L | False | overtake successful, no hesitation |
| F3R | False | overtake successful |
| F4 | True | sudden-stop rear-end (known limitation; no SETUP attempt) |
| F5 | False | correct (stayed behind) |
| F6 | False | overtake successful |
| F7 | False | clean FOLLOW, no ttl ping-pong |
| F8 | False | corner-conservative (correct, by design) |
| F9 | False | cautious crawl past stationary fellow — see below |

#### Known limitations after SD-10

- **F4 sudden-stop rear-end**: ego brakes via SD-4 EMERGENCY_STABLE but stopping
  distance from cruise speed exceeds available gap. No structural fix — the
  scenario is "fellow stops without warning at 9m gap"; physically tight.
- **F9 stationary-obstacle cautious cruise**: a chicken-and-egg deadlock when
  fellow is stationary AND classified as `ahead=1` (forward in ego's heading frame)
  AND laterally on/near the racing line. Sequence:
  1. FOLLOW cap = `max(3.0, opp_speed + follow_speed_margin_mps)` = `max(3.0, 0+2.5)` = 3.0 m/s
  2. ego_speed stays low → Δv ≈ ego_speed → `setup_gap_max(Δv) ≈ 1.4·Δv + 14` ≈ 14 m
  3. actual longitudinal_m to stationary fellow ≈ 38 m at start
  4. `setup_too_far_follow` gate fires → stay in FOLLOW → cap stays at 3.0 m/s
  Ego does eventually pass (lateral clearance 5-6 m, no contact), just slowly. SD-10d
  fixed the EMERGENCY_STABLE part; the FOLLOW-cap part is a separate structural issue,
  candidate for a future "static-obstacle FREE_RUN bypass" patch.

#### Performance bottom line

| Metric | Pre-SD-10 | Post-SD-10 |
|---|---|---|
| Steady-state tick_ms (p50) | 23.5 ms | 10–12 ms |
| Steady-state tick_ms (mean) | 31 ms | 12–14 ms |
| ttl_switch lag (peak tick_ms) | 530–600 ms | unchanged (segmap rebuild — see SD-10m revert) |
| Cold-start delay | ~38 s | ~38 s (cold path unchanged) |
| Warm-start delay | ~38 s | ~10 s (SD-10o) |

### SD-11 (trajectory-prediction-as-primary-authority restructure, 2026-04-27)

**Motivation:** The SD-10 cycle shipped clean F-bank results but F9 surfaced a
deep architectural gap that no patch fixed: ego cruised at 3 m/s for ~10 s
past a stationary fellow at lat=−5.5 m, even though every tick logged
`predicted_collision=0`. Root cause:

- `path_collision_predicted` (1.5 s horizon) was wired only as a **brake-trigger
  gate** — when snapshot heuristics wanted to brake, prediction could VETO.
- **No code path consulted prediction to GREEN-LIGHT speed.** The planner picked
  FOLLOW vs FREE_RUN purely from `sit.distance_m < relevance_dist_m=95 m AND
  sit.ahead`. Result: prediction was half-wired — could withhold a brake, but
  couldn't unblock acceleration when the geometry was clear.

**User directive:** "Predict where the fellow is going to go and where we are
going to go. Only when the prediction is about to collide in a reasonable
amount of future distance, do we chicken out and abort. Surprises handled by
emergency braking. Look at what we have and think through how to restructure —
instead of adding patches left and right."

**Design — two-key safety:** SD-11 makes a 10 s trajectory-rollout prediction
the **primary** decision authority for the FOLLOW-vs-FREE_RUN-vs-SETUP entry
choice. SD-4's existing 1.5 s `path_collision_predicted` stays untouched as
the **independent emergency-brake** layer. Strategy commits, SD-4 vetoes.

**Stages (all in commits e6e5345f → ffae91fb):**

- **SD-11a** Multi-step `FellowPredictor.trajectory(horizon_s, sample_dt_s)`
  returning a list of `(t, x, y, s)` tuples. Reuses the existing recency-weighted
  CV velocity estimator (no new estimator). Pure additive method.
- **SD-11b** Ego-strategy trajectory simulator
  (`prediction/strategy_simulator.py`). For each candidate
  (`stay_optimal` / `follow_fellow` / `pass_left` / `pass_right`), simulates ego
  forward over the horizon and reports `(reachable_progress, reachable_speed,
  min_clearance, completed)`. Strategy speed profiles are simple piecewise
  ramps; pass_* uses a 3-phase profile (lane-change → alongside → merge-back).
  Reuses the cached `_xy_at_arclength` from `pass_geometry`.
- **SD-11c** Pure-function strategy selector
  (`planner/strategy_selector.py`). Filter by `min_clearance >= 2.5 m`,
  rank by `reachable_progress`, tiebreak `stay_optimal > pass_* > follow_fellow`.
  Soft-fallback chain: hard-filter empty → try `follow_fellow` at 1.5 m → else
  `stay_optimal` last-resort (SD-4 catches it).
- **SD-11d** Dual-planner shim, telemetry-only. Strategy pipeline runs every
  tick and logs `[Strategy] selected=... reason=... clearances={...} progress={...}`.
  No behavior change with `use_strategy_authority=False` (default).
- **SD-11e** Strategy authority. When `use_strategy_authority=True`, a new
  branch fires immediately after the no_opponent guard:
  `stay_optimal → FREE_RUN`, `follow_fellow → FOLLOW (cap=opp+0.3, NO 3.0 m/s
  floor — was the F9 deadlock cause)`, `pass_* → SETUP_PASS_{side}` with full
  lifecycle seeding. Hysteresis (`strategy_commit_cycles=2`) requires the same
  selection for N consecutive ticks before honoring. Authority **never preempts
  mid-flight execution** — only fires when `state.mode in (FREE_RUN, FOLLOW)`.
- **SD-11e fix** Threaded the `use_strategy_authority` Scenic param into
  `TacticalPlannerConfig` (the original commit forgot the param reader, so
  the flag was a no-op until this fix landed in `behaviors.scenic`).
- **SD-11f** F-bank validation. F9.scenic updated to enable
  `tactical_planner_enabled=True` and `prediction_enabled=True` so the F9
  deadlock and its SD-11 fix are reproducible by running this file directly.

**Default horizon: 10 s with sample_dt_s=0.5** — chosen to cover a full
overtake (approach ≤ 4 s + lateral shift ~1.5 s + side-by-side ~3 s + merge
~1.5 s ≈ 10 s, matching F1 DRS overtake duration). At ego_speed=30 m/s this is
300 m forward — well within the cached 3500-pt polyline. Compute: 4 strategies
× 20 samples × 2 polyline calls × ~50 µs cached `_xy_at_arclength` ≈ **16 ms/tick**
within the 50 ms control budget. Beyond ~12 s, prediction noise (linear
extrapolation of recency-weighted velocity) starts to dominate signal.

**State machine preserved as executor:** the 6 states
(FREE_RUN/FOLLOW/SETUP_PASS_*/COMMIT_PASS_*/HOLD_PASS_*/ABORT_PASS) all stay.
The strategy authority owns the **entry decision**; the existing lifecycle
owns **execution**. All SD-2..SD-10 safety latches (setup_min_dwell, hold_max_s,
abort_until_s, contact_recovery, predicted_collision_gate) preserved unchanged.

**F2_tactical validation (commit ffae91fb, real run):**

- Startup confirmed: `[FollowRacingLineMPCBehavior ego] SD-11e strategy authority ENABLED`
- 880 `decision_reason=strategy_*` lines (878 `strategy_stay_optimal` + 2 `strategy_pass_right`)
- 36 `pass_right` + 42 `pass_left` strategy selections → only **2 honored** as
  authority firings (hysteresis correctly filtered single-tick blips)
- **One full overtake completed** at t=6.70 s — `pass_success=1`,
  `pass_success_free_run`, returned to FREE_RUN at full speed
- **Zero hard-brake ticks** (`brake>0.3`) in the entire 30 s run
- Max speed 37.47 m/s, min after 5 s = 12.08 m/s — no scared braking

**How to opt in:**

```
scenic <scenario> --2d --model scenic.simulators.dspace.racing_model
    --simulate --count 1 --time 3000
    --param use_strategy_authority True
    *>run.log
```

Default still off (`use_strategy_authority=False`) for safe rollback. Snapshot
path stays alive and tested for regression net.

**Tests:** 41 new across `test_fellow_predictor_trajectory.py` (8),
`test_strategy_simulator.py` (11), `test_strategy_selector.py` (10),
`test_tactical_planner_sd11d.py` (5), `test_tactical_planner_strategy.py` (7).
Total racing suite: **177 pass** (was 136 pre-SD-11).

**Reuse map (no rebuilding):**

| Primitive | File | SD-11 reuse |
|---|---|---|
| `_xy_at_arclength` (cached LineString) | `assessment/pass_geometry.py:65` | Both ego and fellow walks in strategy_simulator |
| `_recency_weighted_velocity_xy` | `prediction/fellow_predictor.py:50` | CV velocity for trajectory extrapolation |
| `_apply_predicted_collision_gate` | `tactical_planner.py:79` | UNCHANGED — SD-4 emergency layer keeps firing |
| `path_collision_predicted` (1.5s horizon) | `assessment/pass_geometry.py:194` | UNCHANGED — independent emergency brake |
| Existing 6-state machine | `tactical_planner.py` | EXECUTOR — strategy seeds it, lifecycle counters carry plan across ticks |

**Known limitations / deferred:**

- **State-machine simplification** (~450 LOC of internal phase 3–9 chain
  becomes deletable once strategy authority is the norm) — deferred to
  SD-11g pending a clean F-bank baseline with the flag default-on.
- **F-bank scenarios other than F2/F9** still need to verify SD-11 doesn't
  regress (F3L/F3R/F6 overtakes, F4 sudden-stop emergency brake, F5/F7/F8
  conservative behavior).
- **Default flip** — `use_strategy_authority` stays `False` until full
  F-bank validation completes.
