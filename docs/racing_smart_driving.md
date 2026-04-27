# Racing — smart driving on one opponent (SD-* cycle)

**Status:** kicking off. Built on `milestone-cleanup-cc-complete` (`72e79f66`).
Populated incrementally as SD-1 through SD-3 land.

## Pre-SD baseline (from full_stack_20260427_031327)

| Scenario | Collision | Off-track | Commits attempted | Note |
|---|---|---|---|---|
| F1 (fellow behind) | False | False | 0 | correct — no overtake needed |
| **F2 (slow ahead, optimal)** | False | False | **0** | **primary SD-2a target** |
| F3L (slow ahead, left TTL) | False | False | 81 | works today; SD must not regress |
| F3R (slow ahead, right TTL) | False | False | 53 | works today; SD must not regress |
| F4 (sudden stop) | False | False | 0 | emergency-brake path; SD must NOT trigger commits here |
| F5 (swerve out) | False | False | 19 | works |
| F6 (left occupied) | False | False | 117 | works |
| F7 (right occupied) | False | False | 92 | works |
| **F8 (corner-entry+ahead)** | **True** | False | 259 | secondary SD target — collision @ t=7.85s, 12 m OOB |
| F9 (stationary obstacle) | False | False | 56 | works |

The SD cycle must (a) bring F2's commit count above zero and complete the
overtake, (b) reduce F8's collision rate, (c) preserve F3L/F3R/F9 within ±20%
commits, (d) keep F4 collision-free.

## Goal

Make ego attempt — and complete — overtakes on a single slower opponent
across the F-bank scenarios. The blocker as of `milestone-cleanup-cc-complete`
is that F2_tactical's tactical planner sits in FOLLOW indefinitely (48
transitions, 0 COMMIT_PASS_*) when the slow fellow is on ego's racing line.
F3L/F3R/F9 (opponent on alternate TTL) already overtake successfully —
historical phase11_runner showed 81 / 54 / 56 commit_pass counts.

## Root cause (3 coupled gates)

Per the staged plan in `~/.claude/plans/structured-knitting-hopcroft.md`:

1. `assessment/race_situation.py` `_compute_corridor_open_flags` —
   when `|pred_lat_m| < 1.5`, both `left_open` and `right_open` are forced
   False. Then in the planner at `tactical_planner.py:758-759`, both-False
   forces `pass_safe = False`. F2 opponent is on optimal line (lat ≈ 0),
   so this fires every tick.
2. Even if corridor flags were both True symmetrically, the planner gate at
   `tactical_planner.py:381` (`asymmetric_opening = left XOR right`) yields
   False. The fix needs to open EXACTLY ONE side.
3. `tactical_planner.py:735` checks raw `sit.collision_risk_01 <= 0.48`.
   Closing 51 m/s on a 35 m gap → ttc = 0.7 s → risk ≈ 0.83. Even with
   corridors fixed, raw collision risk vetoes pass_safe. Damping needs to
   apply to the centered-opponent case too.

## Plan (sketch)

- **SD-1**: regression baseline on F3L/F3R/F9 + F2_tactical. No code change.
- **SD-2a**: Loosen `_compute_corridor_open_flags` for centered slow
  opponents — distance + closing-speed bucketed asymmetric opening
  (default right-side bias). ~25 LOC.
- **SD-2b** (conditional): Add "obvious overtake" damping in
  `_longitudinal_opening_dampen`. ~10 LOC.
- **SD-2c** (conditional): Route `pass_safe` through dampened
  `assessment_emergency_risk_01` instead of raw `sit.collision_risk_01`. ~3 LOC.
- **SD-3**: Validate across F-bank. F2_tactical commit count ≥ 5;
  F3L/F3R/F9 within ±20% of SD-1 baseline; F4 collision == 0.

Tag `milestone-smart-drive-one-opponent` on validation pass.

## Verification

```
python src/scenic/domains/racing/benchmarks/commit_pass_runner.py 2>&1 | tee /tmp/sd_commit.log
scenic examples/racing/calibration/F2_tactical.scenic --2d \
    --model scenic.simulators.dspace.racing_model --simulate --count 1 --time 12000 \
    *>F2_tactical.log
grep -c "Tactical.*mode=COMMIT_PASS" F2_tactical.log     # ≥ 5 means success
grep -c "Tactical.*mode=SETUP_PASS" F2_tactical.log      # ≥ 1 means SETUP arming
grep -c "CorridorAsym" F2_tactical.log                   # > 50 means new branch firing
```
