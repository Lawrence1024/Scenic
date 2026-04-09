# Deferred Scope

The following items are explicitly out of scope for the current planner implementation:

- redesigning pit route detection
- pit entry commitment logic
- pit exit merge planner
- pitlane tactical planning

Current rule:

- keep existing pit handling as-is
- if code indicates pit mode, planner yields to current pit behavior

These items can be added later as a separate top-level route mode once the main-track smart planner is stable.
