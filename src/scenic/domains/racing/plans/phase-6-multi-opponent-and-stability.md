# Phase 6: Multi-Opponent Robustness and Long-Run Stability

## Goal

Generalize planner behavior from single-opponent demos to robust multi-car race scenarios.

## What to Build

Add:

- nearest-threat selection
- front-threat vs side-threat handling
- logic to avoid switching into occupied adjacent corridors
- long-run stability checks over many laps

This phase turns planner behavior from a controlled demo into race-usable logic.

## Why It Matters

Real race traffic includes multiple dynamic threats and long-duration interactions.

## Success Criteria

In multi-car scenarios:

- ego completes laps without planner instability
- threat selection remains sensible
- no left/right switches into occupied adjacent corridors
- acceptable lap-time degradation in traffic
- stable behavior over long runs

## Exit Checklist

- [ ] Multi-opponent threat model is integrated and prioritized correctly.
- [ ] Corridor occupancy constraints are enforced.
- [ ] Long-run stability benchmark passes.
- [ ] Traffic performance remains acceptable against target baseline.
