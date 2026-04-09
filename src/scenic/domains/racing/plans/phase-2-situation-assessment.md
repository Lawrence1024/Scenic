# Phase 2: Situation Assessment

## Goal

Build a stable opponent-state interpreter that converts raw geometry into race-state features for planning.

## What to Build

For one opponent, compute:

- ahead/behind relation
- relative progress `Δs`
- relative lateral relation
- closing speed
- overlap state:
  - clear behind
  - closing behind
  - partial overlap
  - side-by-side
  - clear ahead
- short-horizon collision risk
- segment context:
  - straight
  - corner entry
  - corner body
  - corner exit

This phase provides inputs only; it does not yet decide or execute overtakes.

## Why It Matters

Planner decisions should be based on race-state semantics, not only raw positions.

## Success Criteria

Labeled scenario snapshots classify correctly and stably, including:

- opponent 10 m ahead and slower
- ego overlapping on right
- corner entry with opponent ahead
- side-by-side on straight

No frame-to-frame flicker in simple/static cases.

## Exit Checklist

- [ ] Interpreter outputs all required features each cycle.
- [ ] Snapshot test set exists with expected labels.
- [ ] Classification is stable under small measurement noise.
- [ ] Outputs are ready for planner consumption in next phase.
