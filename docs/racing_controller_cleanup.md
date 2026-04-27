# Racing controller cleanup (RC-* cycle, 2026-04-26)

This doc summarizes the racing-controller refactor cycle landed during the
2026-04-26 working session. The starting point was a codebase where
"control logic was added left and right with different philosophies, sometimes
contradictory." Symptoms: F0 long-run spin-out at ~t=92s, F2 with multiple
20+ second off-track windows ending in 180-degree spins, brake/throttle chatter
unsettling the car at every corner.

The cleanup does **not** change the controller architecture (still MPC-lateral
+ MPC-longitudinal) but resolves the contradictions, removes dead code, and
brings tuning into line with race_common's IAC dSPACE reference. The previous
F0 spin signature is gone, F2_tactical completes a full lap with all OOB
events recoverable, and the codebase is now legible enough to attempt
planner-level work safely.

## Commit timeline

Tagged: `milestone-rc-cleanup` (after RC-W) and `milestone-rc-cleanup-elev`
(after RC-Z; full-lap-clean state). Use `git revert <sha>` to back out any
single stage.

| Commit | Stage | Net change |
|---|---|---|
| `ca010adc` | RC-1 | `[CtrlTrace]` per-tick telemetry — read-only |
| `a97f931c` | RC-4a | Delete `mpc/speed_profile.py` (340 LOC dead) + `mpc/temp_config` (36 KB stale YAML) |
| `4958944b` | RC-4b | Sync `MPCConfig.__init__` defaults to `vehicle_mpc.yaml` hot values |
| `1363a856` | RC-5 | Default-on `prediction_enabled` / `assessment_enabled` / `commit_abort_enabled`; fix assessment gate at `behaviors.scenic:901` |
| `5c96fd3a` | RC-6 | Stability-guard TTL rate-limit bypassed during COMMIT_PASS_*/ABORT_PASS; explicit speed-cap precedence dict |
| `5a2ff58d` | RC-W | MPC tracking weights de-tuned to race_common parity (`w_ey` 10→5, `w_epsi` 5→1, `wT_ey` 15→10, `w_ey_high_curv` 22→5, `w_epsi_high_curv` 12→1) |
| `e23392fd` | RC-7a | Segment-type telemetry: `seg=<id>/<type>` and `seg_ahead=<id>/<type>` in `[CtrlTrace]` |
| `8d25ac41` | RC-7b | Speed-reference slew_up clamped to 0 when current or upcoming segment is `curve` (race-line "carry momentum through" pattern) |
| `44c83603` | RC-Z | Backfill real elevation into `ttl_*_xodr.csv` from race_common's `ttl_17.csv` (54.86 m vertical span vs prior ~0.06 m flat) |

## Why each stage was needed

### RC-1 — telemetry baseline
We were debugging blind. A single consolidated `[CtrlTrace]` per tick
(after stability guard, before action emission) surfaced bugs that prior
logs hid — most notably that `brake_mpc=0.000` THROUGHOUT the F0 t=85-95s
spin (refuting the brake-cap-tuning hypothesis we were chasing).

### RC-4 — dead code + default sync
`mpc/speed_profile.py` was a previous decomposition attempt that drifted to
dead code with independently-tuned thresholds. `mpc/temp_config` was a
36 KB stale YAML with old parameter names never loaded. `MPCConfig.__init__`
defaults had drifted significantly from YAML hot values (e.g.
`w_ey_high_curv` default was 8.0 while YAML had 22.0); when YAML loads
correctly the defaults are inert, but any code path falling back to defaults
silently used under-tuned weights.

### RC-5 — wire smart features
`prediction_enabled` / `assessment_enabled` / `commit_abort_enabled` were
all default False, meaning the smart-features stack we'd already built
never fired. Also fixed a gate-coupling bug at `behaviors.scenic:901`
where the assessment block didn't run when only tactical was enabled.

### RC-6 — stop controllers fighting
Stability-guard rate-limited TTL switches at 0.75s intervals with no
awareness of planner state. Planner could pick "switch to left for
overtake" mid-pass; guard would block; MPC kept stale racing-line
reference. Now guard bypasses during COMMIT_PASS_*/ABORT_PASS — planner
intent wins over chatter-protection.

### RC-W — weight de-tune
Our MPC tracking weights were 2-12× more aggressive than race_common's
IAC dSPACE `mpc_lateral` config. This caused MPC to over-correct CTE
excursions, saturating steer at ±0.282 (full lock) on every CTE blip —
that saturation was the root cause of the spin cascades. Detuning to
race_common parity dropped saturated-steer ticks from hundreds to ~2
out of 2400. Lap time impact ~5% slower; tradeoff is worth it.

### RC-7 — segment-aware speed planner
The OpenDRIVE-derived segment_map already produces names like
"main straight" / "main curve" / "pit straight" / "pit curve" — but
nothing in the behavior loop was reading them. RC-7a surfaces them in
telemetry; RC-7b uses them to clamp speed-reference slew_up to 0
whenever a curve is current or within 25 wp ahead. This implements the
race-line "brake before, carry momentum through, throttle on exit"
pattern by preventing throttle from slamming to 1.0 between two close
curves (a chicane). Visual confirmation: 2-wheels-off-ground bouncing
at the F2_tactical t=104-108 chicane is gone after RC-7b.

### RC-Z — restore real elevation
`ttl_*_xodr.csv` had near-flat z column (range ~0.06m across full lap),
so the longitudinal MPC's grade-compensation infrastructure
(`gravity_force = mass * g * sin(grade)` at `mpc_longitudinal.py:399`)
was effectively a no-op. Backfilled real elevation from race_common's
`ttl_17.csv` (54.86 m vertical span — Corkscrew alone drops ~18 m).
File format unchanged (still 3-column `x, y, z`); only z values updated.

## What this is NOT

- **NOT a controller swap**: still MPC-lateral + MPC-longitudinal. race_common
  uses Pure Pursuit as primary lateral; we considered switching but RC-W's
  weight de-tune was sufficient to address the saturation issue without
  the multi-day port cost.
- **NOT a Pure Pursuit port**: file under "future work if MPC tuning
  approach hits its ceiling" (see `feedback_brake_cap_dead_end.md`).
- **NOT planner-level work**: the tactical state machine still doesn't
  attempt overtakes (planner sits in FOLLOW indefinitely instead of
  entering SETUP_PASS_*/COMMIT_PASS_*); separate research effort.
- **NOT a tuning sweep**: only the 5 weights identified as drifted vs
  race_common were touched. MPC horizon, vehicle physics, max_steer,
  brake/throttle authority — all unchanged.

## Lessons learned (preserved in memory)

`feedback_brake_cap_dead_end.md` captures the most important meta-lesson:
**brake-cap tuning is the wrong knob for racing MPC issues**. Multiple
iterations during this session (RC-3, RC-3.1, RC-3.2, RC-FC v1) tried to
fix the spin by tuning brake authority and adding friction-circle limiters
at the action-emission layer. All four created worse failure modes than
they fixed (mid-corner brake slams, recovery-traps when the cap clipped
needed brake authority while ego was already off-line). Root cause was
structural: longitudinal MPC and lateral MPC compute commands independently
without coordinating around the friction circle. No knob at the
merge/cap layer can fix this without breaking the recovery path. The
right fix turned out to be RC-W (lower the tracking weights so MPC asks
for less aggressive commands in the first place).

If F0 spinout returns or a similar issue surfaces:
1. **Don't** tune brake cap.
2. **Don't** add action-emission-layer friction-circle limiters.
3. Look at planner-level: lateral_accel default vs race_common (currently
   8.0 vs race_common 15.0), MPC tracking weights, steer-rate limits.
4. Or implement a proper Pure Pursuit lateral controller as race_common
   does.

## Validation snapshot (post-RC-Z, full lap on F2_tactical, 150s sim)

- 3000 `[CtrlTrace]` ticks (full coverage)
- Mean speed 26.1 m/s, max 60.0 m/s (target_speed reached on straights)
- 683 OOB events across 15 short windows (all recoverable; rate 4.55/s,
  improved from pre-RC-W's ~9/s)
- Saturated steer ticks 23 (down from hundreds pre-RC-W)
- `curve_hold` engaged 26.9% of run as designed
- Full lap completed; user-reported visual confirmation: looks fine

## Where to next (planner-level work, separate cycle)

- Tactical planner tuning so it actually attempts SETUP_PASS_* /
  COMMIT_PASS_* with slower opponents (currently sits in FOLLOW
  indefinitely) — this is the **SD-* cycle** documented in
  [`docs/racing_smart_driving.md`](racing_smart_driving.md).
- Consider raising `max_lateral_acceleration` from our 8.0 to race_common's
  15.0 now that the weight de-tune means MPC won't over-react
- Optionally: import LON_VEL (per-waypoint optimal velocity) from
  race_common's ttl_17 as a richer speed-target source than constant
  `target_speed=60` — RC-V if pursued
- Pure Pursuit port if MPC tuning hits a ceiling we can't get past

## Cleanup cycle (CC-*) — post-RC

Following the RC cycle, the codebase had accumulated 869 references to
`_phase\d+_*` symbols whose numeric tag conveyed nothing semantic, plus
significant legacy (6 dead phase scenario directories on the LagunaSeca
map, 18 phase-numbered plan docs, `coordinate_transform.py`, etc.).
The CC-* cycle (2026-04-26) cleaned this up:

| Stage | Commit | What |
|---|---|---|
| CC-1 | `96e0870d` | `docs/cleanup_inventory.md` — full deletion+rename map |
| CC-2 | `fabcfcd3` | Delete 6 phase scenario dirs, 4 standalone .scenic, 7 LagunaSeca map files, 12 TTL backups, `coordinate_transform.py`, `Laguna_Seca_transform.json`, `LAGUNA_SECA_SEGMENTS` constant |
| CC-3a | `44bf6852` | `_phase7_*` → `_prediction_*`, file `phase7_runner.py` → `prediction_runner.py` |
| CC-3b | `0f55560d` | `_phase8_*` → `_assessment_*`, file → `assessment_runner.py` |
| CC-3c | `2c094aec` | `_phase10_*` → `_guard_*`, file → `guard_runner.py` |
| CC-3d | `de8ee80c` | `_phase11_*` → `_commit_*`, file → `commit_pass_runner.py`, ~30 test names renamed |
| CC-3e | `85a6f4db` + `4e1d789c` | Phase 0/1/2/3/4/5/6/9/12 → baseline/scripted/opponent/tactical/shield/segment/orchestration/hazard/seg; remaining runner files renamed |
| CC-4 | `8788afa0` | Doc updates: deleted 18 phase-*.md plan docs, rewrote `plans/README.md`, added `docs/racing_smart_driving.md` stub, header pointer in `racing/README.md` |
| CC-5 fix | `72e79f66` | De-double `commit_commit_*` → `commit_*`, `guard_guard_*` → `guard_*` (sed artifact from CC-3) |

**Tag**: `milestone-cleanup-cc-complete` at `72e79f66`.

### CC-5 validation result (full F-bank, `full_stack_20260427_031327`)

| Scenario | Collision | Off-track | Commits | vs historical |
|---|---|---|---|---|
| F1 | False | False | 0 | ✓ (correct, fellow behind) |
| F2 | False | False | 0 | ← smart-driving target for SD-* |
| F3L | False | False | **81** | ✓ matches historical 81 |
| F3R | False | False | **53** | ✓ matches historical 54 (±1) |
| F4 | False | False | 0 | ✓ (emergency brake response) |
| F5 | False | False | 19 | overtake attempts |
| F6 | False | False | 117 | |
| F7 | False | False | 92 | |
| F8 | **True** | False | 259 | known-difficult corner-entry; pre-existing collision (12 m OOB at t=7.85s) |
| F9 | False | False | **56** | ✓ matches historical 56 |

Bit-for-bit reproducibility against the pre-CC baseline confirms the cleanup
introduced no behavioral regressions. F2 and F8 are the targets for the
SD-* cycle (centered slow opponent, corner-entry collision).

After CC-5 the SD-* cycle starts (see
[`docs/racing_smart_driving.md`](racing_smart_driving.md)).
