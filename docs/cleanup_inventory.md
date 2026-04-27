# Cleanup cycle (CC-*) inventory — change log of Part A

This document is the change log for the CC-1 through CC-5 cleanup work landing
on top of the `milestone-rc-cleanup-elev` baseline. Reviewers can scan this
file to see the full scope of deletions and renames before any individual
commit lands.

## Why now

The codebase has accumulated:
- **869 references** to `_phase\d+_*` symbols whose numeric tag conveys
  nothing semantic.
- **Six phase-numbered example directories** (phase0_benchmark through
  phase5_segments, ~36 `.scenic` files) all on the dead `LagunaSeca.xodr`
  map — broken since the B5/B6 frame migration to `LGS_v1.xodr` and
  never migrated forward.
- **Four standalone legacy `.scenic` files** at `examples/racing/` root,
  also on `LagunaSeca.xodr`.
- **Two debug-scenario directories** (fellow_smoke, fellow_placement_debug)
  superseded by F-bank.
- **Eighteen phase-numbered plan docs** in `src/scenic/domains/racing/plans/`
  documenting work from a prior development era.
- **Dead source files** (`coordinate_transform.py`,
  `Laguna_Seca_transform.json`) confirmed unused since RC-cycle exploration.
- **Twelve TTL backup files** (`*_og.csv`, `*_racecommon.csv`,
  `*_full_racecommon.csv`) — all preserved in git history if ever needed.

Per user direction: **F-Bank in `examples/racing/f_shared/` is the canonical
test set going forward.** Everything else listed below is legacy and slated
for deletion or renaming.

---

## CC-2 — Files to delete

### Example scenarios on legacy LagunaSeca.xodr map (all phase dirs + standalones)

| Path | Files | Why dead |
|---|---|---|
| `examples/racing/phase0_benchmark/` | 7 `.scenic` | All `param map = LagunaSeca.xodr`; never migrated to LGS_v1. F-bank F0 supersedes the baseline-ego-alone case. |
| `examples/racing/phase1_planner/` | 3 `.scenic` | Scripted-TTL-schedule scenarios; the `ttl_schedule` plumbing is a niche Phase-1 feature rarely used. F-bank F2/F3L/F3R cover the dynamic case via tactical_planner. |
| `examples/racing/phase2_assessment/` | 1 `.scenic` | Same map issue. Assessment is now exercised by F1-F12 (assessment_enabled=True is RC-5 default). |
| `examples/racing/phase3_tactical/` | 7 `.scenic` | Same map issue. Tactical planner exercised by `examples/racing/calibration/F2_tactical.scenic` + F3L/F3R via commit_pass_runner.py. |
| `examples/racing/phase4_pass_shield/` | 7 `.scenic` | Same map issue. |
| `examples/racing/phase5_segments/` | 11 `.scenic` | Same map issue. Segment-aware logic is exercised by F8/F10/F11/F12 (corner-entry F-bank scenarios). |
| `examples/racing/art_behavior.scenic` | 1 `.scenic` | LagunaSeca map; experimental ART integration superseded by current ART-on-cosim setup. |
| `examples/racing/art_fellow_combined.scenic` | 1 `.scenic` | Same. |
| `examples/racing/ego_mpc_behavior.scenic` | 1 `.scenic` | LagunaSeca map; standalone MPC demo, redundant with F0. |
| `examples/racing/fellow_control.scenic` | 1 `.scenic` | LagunaSeca map; standalone fellow demo, redundant with F1. |
| `examples/racing/fellow_smoke/` | 7 `.scenic` + README | Smoke-test scenarios, superseded by F1-F7. |
| `examples/racing/fellow_placement_debug/` | 9 `.scenic` + README | Debug scenarios for B6 work; B6 closed at milestone-rc-cleanup-elev. |

**Total: ~50 `.scenic` files + 4 README files removed.**

### Source files

| Path | Why dead |
|---|---|
| `src/scenic/simulators/dspace/geometry/coordinate_transform.py` | `_coordinate_transform = None` per simulator.py:656; never instantiated. Identified as dead in original RC plan Phase C. |
| `assets/maps/dSPACE/Laguna_Seca_transform.json` | Only consumer was `coordinate_transform.py`. |
| `assets/maps/dSPACE/LagunaSeca.xodr` | Replaced by `LGS_v1.xodr` in B5 (frame migration). All F-bank uses LGS_v1. |
| `assets/maps/dSPACE/LagunaSeca_old.xodr` | Even older variant, fully obsolete. |
| `assets/maps/dSPACE/LagunaSeca_MainTrack_FromTTL.xodr` | Auto-generated from old TTLs; superseded by LGS_v1's MathWorks-vendored geometry. |
| `assets/maps/dSPACE/LagunaSeca.snet` | dSPACE format paired with LagunaSeca.xodr; same vintage. |
| `assets/maps/dSPACE/Laguna_Seca.rd` | dSPACE road format paired with LagunaSeca.xodr. |

### Constants to delete (within preserved files)

| Symbol | File | Replacement |
|---|---|---|
| `LAGUNA_SECA_SEGMENTS` | `src/scenic/domains/racing/segments/segment_map.py:61-70` | Was used only when `use_conventional_laguna=True` and not curvature segments. XODR-derived curve/straight (`_build_curve_straight_segments`) is the live path; this constant is dead. |
| `ROUTE_ORIGINS` | `src/scenic/simulators/dspace/geometry/route_projection.py:6-11` | Calibration constants for OLD LagunaSeca.xodr ModelDesk routes. With LGS_v1 + RC-Z elevation, route projection uses TTL-CSV-derived road index. |
| `ROAD_START_POSITIONS` | same file:13-18 | Same. |
| `ROUTE_TRANSITION_POINTS` | same file:36-40 | Same. |
| `ROUTE_CORKSCREW_OFFSETS` | same file:46-53 | Same. |
| `ROUTE_ROAD_SEQUENCES` | same file:24-27 | Same. |

These constants are deleted in CC-2 alongside the file deletions; they're
within `route_projection.py` so the file stays but the constants are pruned.

### TTL backup files

All 12 are preserved in git history for revert if ever needed.

| File | Why dead |
|---|---|
| `assets/ttls/LS_ENU_TTL_CSV/ttl_optimal_xodr_og.csv` | OLD `_og` racing line backup; live file is `ttl_optimal_xodr.csv` (Path C). |
| `..._left_xodr_og.csv`, `..._right_xodr_og.csv`, `..._pit_xodr_og.csv` | Same. |
| `..._optimal_xodr_racecommon.csv` (and 3 others) | race_common 20-col vendored TTL backup; live file inherits race_common's elevation via RC-Z but not the racing-line geometry. |
| `..._optimal_xodr_full_racecommon.csv` (and 3 others) | Same. |

### Plan docs to delete

`src/scenic/domains/racing/plans/` — 18 phase-numbered plan docs:
- `phase-0-baseline-and-visibility.md`
- `phase-1-planner-mpc-integration.md`
- `phase-2-situation-assessment.md`
- `phase-3-smart-follow-and-stable-ttl.md`
- `phase-4-pass-commit-abort-and-shield.md`
- `phase-5-segment-aware-tactics.md`
- `phase-6-architecture-skeleton-and-observability.md`
- `phase-6-12-master-rollout.md`
- `phase-7-fellow-next-step-prediction.md`
- `phase-8-situation-assessment-and-dynamic-gap.md`
- `phase-9-tactical-planner-v1.md`
- `phase-10-stability-guard-and-emergency-policy.md`
- `phase-11-pass-commit-and-abort.md`
- `phase-12-segment-aware-tactical-intelligence.md`
- `comprehensive-planner-validation-runner.md`
- `deferred-scope.md`
- `fellow-placement-debug-matrix-and-metrics.md`
- `success-definition.md`

These document the historical Phase 0-12 development cycle. The features they
describe are still in the code but the planning context is no longer relevant.
Keeping `README.md` from the plans/ directory.

---

## CC-3 — Phase symbol renames

Strategy: drop the `_phase\d+_` prefix entirely. Each phase already has a
descriptive concept; the numeric prefix is dead weight.

### Rename map (full)

Listed in CC-3 sub-commit order so reviewers can correlate with each
sub-commit's diff:

#### CC-3a — Phase 7 → prediction (~30 references)

| Old | New |
|---|---|
| `_phase7_requested` | `_prediction_requested` |
| `_phase7_fellow_predictor` | `_fellow_predictor` |
| `phase7_runner.py` | `prediction_runner.py` |
| `[Phase7]` log strings (if any) | `[Prediction]` |

#### CC-3b — Phase 8 → assessment (~80 references)

| Old | New |
|---|---|
| `_phase8_assessment_state` | `_assessment_state` |
| `_phase8_gap_ok` | `_assessment_gap_ok` |
| `_phase8_overlap_flag` | `_assessment_overlap_flag` |
| `_phase8_closing_flag` | `_assessment_closing_flag` |
| `_phase8_emergency_risk_01` | `_assessment_emergency_risk_01` |
| `[Assessment]` log strings | unchanged (already descriptive) |

#### CC-3c — Phase 10 → guard (~40 references)

| Old | New |
|---|---|
| `_phase10_guard_state` | `_guard_state` |
| `_phase10_guard_active` | `_guard_active` |
| `_phase10_guard_reason` | `_guard_reason` |
| `_phase10_steer_limited` | `_guard_steer_limited` |
| `_phase10_brake_limited` | `_guard_brake_limited` |
| `_phase10_ttl_switch_blocked` | `_guard_ttl_switch_blocked` |
| `_phase10_emergency_stable_mode` | `_guard_emergency_stable_mode` |
| `phase10_ttl_switch_blocked` (log/CSV) | `guard_ttl_switch_blocked` |
| `phase10_runner.py` | `guard_runner.py` |
| `[Phase10*]` log strings | `[Guard]` |
| Test names like `_phase10_guard_*` (in `test_*.py`) | `_guard_*` |

#### CC-3d — Phase 11 → commit (~120 references)

| Old | New |
|---|---|
| `_phase11_commit_pass_left_count` | `commit_pass_left_count` |
| `_phase11_commit_pass_right_count` | `commit_pass_right_count` |
| `_phase11_abort_pass_count` | `abort_pass_count` |
| `_phase11_pass_success_count` | `pass_success_count` |
| Other `_phase11_*` (~30 test names + symbols) | drop `_phase11_` prefix |
| `phase11_runner.py` | `commit_pass_runner.py` |
| `phase11_*` columns in `phase_run_common.py` CSV | drop `phase11_` prefix |
| `[Phase11Commit]`, `[Commit]` log strings | `[Commit]` |

**CSV column rename note**: any external tooling reading the per-scenario
CSVs from `benchmarks/results/` will break on the column rename. Mitigation:
the new column names are clearly readable without the `phase11_` prefix
(e.g., `commit_pass_left_count` is self-explanatory). Document the rename
in CC-4's README update.

#### CC-3e — Remaining (Phase 0, 1, 2, 3, 9, 12) (~450 references)

| Old prefix | New |
|---|---|
| `_phase0_last_ttl_label`, `_phase0_runner` | `_last_ttl_label`, `baseline_runner` |
| `[Phase0Event]` (log) | `[Event]` |
| `_phase1_speed_cap` | `_scripted_speed_cap` |
| `_phase1_active_ttl` | `_scripted_ttl_active` |
| `_phase1_ttl_cache` | `_scripted_ttl_cache` |
| `_phase2_overlap_state` | `_opponent_overlap_state` |
| `_phase3_speed_cap` | `_tactical_speed_cap` |
| `[Phase3Tactical]` (log) | `[Tactical]` |
| `_phase9_hazard_brake_floor` | `_hazard_brake_floor` |
| `_phase12_seg_corner_entry_count` etc. | `seg_corner_entry_count` etc. |
| `phase12_runner.py` | `segment_aware_runner.py` |

### Renaming protocol (each sub-commit)

1. `git grep -l '_phaseN_'` to enumerate files touched.
2. `sed -i 's/_phaseN_FOO/_DESCRIPTIVE_FOO/g'` per file (or per-symbol via
   editor for higher safety).
3. Parse-test all F-bank scenarios.
4. Quick run F0 to confirm runtime behavior unchanged.
5. Commit with full rename map in commit message.

After each sub-commit, `git grep '_phaseN_' src/` should return zero
references to the renamed prefix.

---

## CC-4 — Documentation updates

| File | Action |
|---|---|
| `docs/frames.md` | Add note clarifying that Phase A/B inside the doc refer to the FRAME-CALIBRATION cycle (B5/B6), unrelated to the deleted code-side Phase 0-12. |
| `docs/racing_controller_cleanup.md` | Add new section "Cleanup cycle (CC-*) post-RC" referencing this inventory doc. |
| `docs/racing_smart_driving.md` | NEW — empty stub, populated during Part B (SD-* stages). |
| `src/scenic/domains/racing/README.md` | Major rewrite: replace Phase 0-12 references with descriptive section names matching the new symbol prefixes; include F-bank inventory (F0-F12 with one-line summary). |
| `src/scenic/domains/racing/segments/README.md` | Minor update for renamed `_segment_aware_*` symbols. |
| `src/scenic/domains/racing/plans/README.md` | Update to point to `docs/racing_controller_cleanup.md` and `docs/racing_smart_driving.md`; note that historical phase-*.md docs were removed in this cycle. |

---

## CC-5 — Validation (DONE 2026-04-26)

Run command (Windows-PowerShell-friendly for live output):
```powershell
python src/scenic/domains/racing/benchmarks/full_stack_runner.py 2>&1 | Tee-Object -FilePath full_stack.log
```

Result captured at `src/scenic/domains/racing/benchmarks/results/full_stack_20260427_031327/`:
F-bank reproduces historical commits within ±1: F3L=81 (hist 81), F3R=53 (hist 54),
F9=56 (hist 56). F8 collision is pre-existing (corner-entry edge case; SD-* target).
No regressions on collision/off-track for F1-F7, F9.

Tagged `milestone-cleanup-cc-complete` at commit `72e79f66`.

---

## What this cleanup is NOT doing

- **NOT deleting any F-bank scenario** — F0-F12 are the canonical test set.
- **NOT touching `examples/racing/calibration/`** — F2_tactical and the
  ego measurement scenario are kept; they're recent and active.
- **NOT migrating phase scenarios to LGS_v1.xodr** — they're being deleted.
  If any phase scenario demonstrates unique coverage not in F-bank, port
  manually as a one-off (out of scope here).
- **NOT renaming CC-* itself or RC-* commit prefixes** — those are this
  refactor's identifiers; they don't refer to dead code.
- **NOT renaming Phase A/B inside docs/frames.md** — frames-cycle phases
  are unrelated to control-cycle phases. We add a clarifying note instead.
