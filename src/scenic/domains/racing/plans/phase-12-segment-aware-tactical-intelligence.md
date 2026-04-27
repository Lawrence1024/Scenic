# Phase 12: Segment-Aware Tactical Intelligence

## Prerequisites (handoff from Phase 11)

Phase 11 delivers explicit commit/abort pass lifecycle control with safety guardrails.
Phase 12 improves decision timing by incorporating segment context directly into pass
acceptance and rejection logic.

This phase **adds** track-context intelligence; it should preserve or improve the
safety envelope from prior phases.

## Current Status

**Planned** (segment-conditioned tactical decisions and comparative validation).

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed phase guidance: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

## Goal

Make commit/setup decisions context-sensitive to segment type so behavior is more
aggressive where appropriate (straights) and more conservative where risk grows
(corner entry/body).

## What to Build

- Feed segment type into tactical decision logic:
  - `straight`
  - `corner_entry`
  - `corner_body`
  - `corner_exit`
- Add segment-conditioned modifiers for:
  - setup-pass acceptance thresholds
  - commit gating thresholds
  - reject/defer reasons at risky segments
- Preserve guard and abort behavior as hard constraints.

## Why It Matters

With simple fellow scripts, most residual race quality comes from ego timing quality:
where and when ego chooses to commit, defer, or abort.

## Success Criteria

Comparative outcomes should show:

- on straights: more valid setup/commit chains when corridor is open.
- on corner entry: fewer late poor commits and fewer timing-driven aborts.
- in disruptive corner-adjacent cases (`F5` near corner entry): no aggressive late
  commits into instability.

Safety and quality:

- maintain no-regression safety profile relative to Phase 11.
- improve tactical timing indicators in straight-vs-corner paired scenarios.

## Required Telemetry (Phase 12)

- `segment_type`
- `segment_modifier`
- `segment_accepted_or_rejected_pass_reason`
- optional comparison diagnostics:
  - commit rate by segment type
  - abort rate by segment type
  - pass success by segment type

## Benchmark / Scenario Guidance

Use matched scenario variants:

- `F2` on straight vs corner entry
- `F6` on straight vs corner entry
- `F7` on straight vs corner entry
- `F5` near corner entry

Runner guidance:

```bash
python -m scenic.domains.racing.benchmarks.segment_aware_runner --time 1000
```

Comparative review should explicitly report:

- commits gained on straights,
- bad commits reduced on corner entry,
- aborts reduced when timing quality improves.

## Implementation (code)

Primary targets:

- planner segment hooks in `src/scenic/domains/racing/tactical_planner.py`
- segment extraction and propagation from existing racing modules:
  - `src/scenic/domains/racing/segments/`
  - `src/scenic/domains/racing/assessment/`
  - `src/scenic/domains/racing/behaviors.scenic`
- benchmark parser additions for segment-conditioned metrics

## Exit Checklist

- [ ] Segment context is available at tactical decision time.
- [ ] Segment modifiers are applied and logged for setup/commit/reject paths.
- [ ] Straight vs corner-entry comparisons show expected directional improvements.
- [ ] Safety is not regressed relative to Phase 11 baseline.
- [ ] Run artifacts include segment-specific KPI summaries and caveats.

## Program Completion Handoff

Phase 12 closes the Phase 6-12 roadmap.

Follow-on work should use this endpoint as the new baseline for:

- multi-opponent extension (future scope),
- pitlane integration redesign (future scope),
- longer-horizon tactical prediction beyond next-step estimates.
