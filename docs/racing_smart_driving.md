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
