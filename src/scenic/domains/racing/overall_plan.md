The revised project goal is:

**Build an opponent-aware ego planner that chooses among `optimal / left / right` TTLs and feeds the existing ego MPC, while letting the current pit handling remain as-is.**

That means:

* we do **not** redesign pit routing yet
* we do **not** solve pit entry/exit logic yet
* if existing code says the car is in pit mode, the new planner simply yields to current behavior
* all early success criteria are measured on **Lap/main-track behavior**

---

# Revised phased plan

## Phase 0 — Freeze the baseline and add visibility

### What to build

Before changing behavior, add logging and a repeatable scenario bank.

Add logs for:

* current TTL
* planner mode
* ego `s`, speed
* opponent relative `Δs`, relative speed
* line-switch events
* collision / off-track / near-miss events
* lap time

Create a small benchmark scenario set:

* no opponent
* slower opponent on optimal
* slower opponent on left
* slower opponent on right
* opponent weaving lightly
* opponent just ahead into a corner
* opponent side-by-side start

### Why this phase matters

If you skip this, every later change becomes subjective.

### Success criteria

You can run the same scenario set repeatedly and automatically produce:

* lap completion status
* lap time
* number of TTL switches
* minimum opponent distance
* collision yes/no
* off-track yes/no

**Success =** every benchmark scenario runs to completion and produces a metrics report automatically.

---

## Phase 1 — Create the planner-to-MPC integration point

### What to build

Refactor the current ego control flow so the planner can choose the active TTL **every cycle**.

Do not make it smart yet. Just make the plumbing real.

Concretely:

* extract the core “one MPC step using a selected reference” logic from your current ego line-following behavior
* create a top-level planner behavior/node that outputs:

  * `active_ttl ∈ {optimal, left, right}`
  * target speed cap
* connect that to the existing ego MPC

At the end of this phase, the new planner can switch TTLs based on a scripted command schedule, not intelligence.

### Why this phase matters

This is the actual architectural unlock. Without this, all “smart driving” stays as comments and flags.

### Success criteria

Run three manual-switch tests:

* optimal → left
* left → right
* right → optimal

Measure:

* no crash during switch
* no oscillation/chatter
* no immediate off-track
* MPC continues to track after the switch

**Success =** commanded TTL switches work reliably at race speed on the main loop.

---

## Phase 2 — Build the situation assessment layer

### What to build

Implement an ego-centric opponent-state interpreter.

For one opponent, compute:

* opponent ahead / behind
* relative progress `Δs`
* relative lateral relation
* closing speed
* overlap state:

  * clear behind
  * closing behind
  * partial overlap
  * side-by-side
  * clear ahead
* simple short-horizon collision risk
* current segment type:

  * straight
  * corner entry
  * corner body
  * corner exit

This phase still does **not** commit to overtaking. It just gives the planner the right inputs.

### Why this phase matters

The planner should not reason from raw positions only. It needs race-state features.

### Success criteria

Create labeled scenario snapshots and verify the interpreter outputs the correct race relation.

Examples:

* “opponent 10 m ahead, slower”
* “ego overlapping on right”
* “corner entry, opponent ahead”
* “side-by-side in straight”

**Success =** the interpreter classifies benchmark snapshots correctly and stably, with no frame-to-frame flicker in simple cases.

---

## Phase 3 — Smart follow mode and stable TTL choice

### What to build

Now add the first real tactical layer, but keep it conservative.

Planner modes:

* `FREE_RUN`
* `FOLLOW`
* `SETUP_LEFT`
* `SETUP_RIGHT`

Behavior:

* if no relevant opponent: stay on `optimal`
* if opponent blocks progress and no pass is safe yet: `FOLLOW`
* if a side looks promising: shift to `SETUP_LEFT` or `SETUP_RIGHT`
* add hysteresis so the planner does not bounce left/right every second

Important: in this phase, a “setup” is mostly positioning. It is not yet a hard overtake commit.

### Why this phase matters

This is the first real “smart driving” phase:
the ego should stop blindly driving into the opponent and start positioning intelligently.

### Success criteria

In the blocked-opponent scenarios:

* ego does **not** rear-end the opponent
* ego either follows safely or repositions to a better TTL
* unnecessary TTL switching stays low
* planner remains stable over multiple laps

Suggested measurable targets:

* 0 collisions in benchmark follow scenarios
* minimum gap always above chosen safety threshold
* TTL switches per lap remain bounded, not oscillatory
* in free-run, planner spends most time on `optimal`

**Success =** ego behaves conservatively and intelligently around a slower opponent, without yet needing full overtakes.

---

## Phase 4 — Add pass commit, abort, and safety shield

### What to build

Now expand the tactical modes to:

* `COMMIT_PASS_LEFT`
* `COMMIT_PASS_RIGHT`
* `ABORT_PASS`
* `EMERGENCY_AVOID`

Add a safety shield beneath them:

* if corridor collapses, abort
* if overlap becomes unsafe, freeze/abort
* if opponent closes too quickly, brake or tuck in
* if boundary margin gets too small, abort

This is the phase where overtaking becomes a real behavior, not just positioning.

### Why this phase matters

This is the core of “race smartly.”

### Success criteria

In dedicated overtake scenarios:

* ego completes safe passes when the corridor is genuinely available
* ego aborts when a pass becomes unsafe
* ego does not continue a doomed pass just because it already started

Suggested measurable targets:

* high pass success rate in “clear opportunity” scenarios
* high abort success rate in “closing corridor” scenarios
* 0 collisions in abort-test scenarios
* 0 boundary violations caused by pass attempts

**Success =** ego can both pass and bail out intelligently.

---

## Phase 5 — Make the planner race-aware, not just obstacle-aware

### What to build

Use the segment map and track structure to make tactical choices context-sensitive.

Examples:

* on straights, allow more aggressive pass setup
* at corner entry, prefer inside only when overlap is established early enough
* in corner body, avoid fresh side-switch commitments
* on corner exit, prioritize completion and traction

This phase should improve *decision quality*, not just safety.

### Why this phase matters

Without segment awareness, the planner may be safe but still race badly.

### Success criteria

Compare against Phase 4 on mixed scenarios:

* same or better safety
* improved overtake timing
* fewer bad pass attempts into corners
* improved average lap time in traffic
* fewer aborts caused by late, poor commitments

**Success =** planner becomes strategically smarter, not just reactive.

---

## Phase 6 — Multi-opponent robustness and long-run stability

### What to build

Generalize the logic beyond one opponent.

Add:

* nearest-threat selection
* front threat vs side threat handling
* “do not switch into another car”
* long-run consistency over many laps

This is where the planner stops being a demo and starts becoming usable in race scenarios.

### Success criteria

In multi-car scenarios:

* ego still completes laps without planner instability
* threat selection remains sensible
* no left/right switching into adjacent occupied corridors
* lap-time degradation in traffic remains acceptable
* planner remains stable over long runs

**Success =** behavior still works when the environment is not clean and simple.

---

# What is explicitly deferred

For now, these are **not** part of the core implementation target:

* redesigning pit route detection
* pit entry commitment logic
* pit exit merge planner
* pitlane tactical planning

We simply keep the existing pit handling.

Later, once the smart main-track planner works, pit can be added as a separate top-level route mode.

---

# The shortest practical roadmap

If you want the most efficient build order, do it like this:

1. **Phase 0**: metrics + benchmark scenarios
2. **Phase 1**: planner can command the active TTL into MPC
3. **Phase 2**: opponent-state interpreter
4. **Phase 3**: safe follow + stable setup-left/setup-right
5. **Phase 4**: commit/abort overtaking + safety shield
6. **Phase 5**: segment-aware tactical intelligence
7. **Phase 6**: multi-opponent robustness

That ordering gives you usable behavior early and avoids premature pit complexity.

---

# What “project success” should mean at this stage

I would define project success in three levels:

### Minimum success

* ego no longer blindly drives the nominal line into a slower opponent
* ego can choose left/right/optimal intelligently
* ego can follow safely and reposition without chatter

### Strong success

* ego can complete and abort overtakes safely
* decisions are stable and segment-aware
* performance in traffic is clearly better than nominal-line baseline

### Full smart-driving success

* ego handles multi-car traffic robustly over long runs
* tactics improve both safety and race performance
* planner is mature enough that pit integration becomes the next logical step rather than a rewrite

---

If you want, the next thing I can do is turn this into a **very concrete engineering checklist** mapped to likely files/components in your current racing library, so each phase becomes “edit these files, add these tests, log these metrics.”
