# Phase 1: Planner-MPC Integration

## Goal

Create the integration point where a planner can choose active TTL every control cycle and feed ego MPC.

## What to Build

- Refactor ego control flow so planner output selects active reference each cycle.
- Extract reusable "one MPC step with selected reference" logic from current ego line-follow behavior.
- Add a top-level planner behavior/node that outputs:
  - `active_ttl ∈ {optimal, left, right}`
  - target speed cap
- Wire planner output into existing ego MPC.
- Keep selection scripted/manual in this phase (no tactical intelligence yet).

## Why It Matters

This is the core architectural unlock that enables all later smart-driving logic.

## Success Criteria

Manual switch tests pass at race pace on main loop:

- optimal -> left
- left -> right
- right -> optimal

For each switch:

- no crash during switch
- no oscillation/chatter
- no immediate off-track event
- MPC remains tracking after switch

## Exit Checklist

- [ ] Planner output contract is defined and used by ego control path.
- [ ] Active TTL can be switched during run without controller reset failures.
- [ ] Three manual-switch tests pass with stable tracking.
- [ ] Integration works on main-track lap behavior.
