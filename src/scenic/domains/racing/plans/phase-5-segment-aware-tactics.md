# Phase 5: Segment-Aware Tactics

## Goal

Make tactical choices context-sensitive to track segments so the planner is race-aware, not only obstacle-aware.

## What to Build

Use segment map and track structure to adapt tactics:

- on straights, allow more aggressive pass setup
- at corner entry, prefer inside only when overlap is established early
- in corner body, avoid fresh side-switch commitments
- on corner exit, prioritize completion and traction

Focus on improving decision quality while maintaining Phase 4 safety.

## Why It Matters

Without segment awareness, planner may be safe but strategically weak.

## Success Criteria

Compared to Phase 4 on mixed scenarios:

- same or better safety outcomes
- improved overtake timing
- fewer poor pass attempts into corners
- improved average lap time in traffic
- fewer aborts caused by late/weak commitments

## Exit Checklist

- [ ] Segment context is part of tactical decision inputs.
- [ ] Decision policy differs by segment type as intended.
- [ ] Mixed-scenario comparison against Phase 4 is documented.
- [ ] Safety is not regressed while tactical quality improves.
