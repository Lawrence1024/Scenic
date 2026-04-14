## Fellow behavior primitives we can rely on

From your current stack, the fellow is basically limited to these behaviors:

1. **TTL cruise**

   * follow `optimal`
   * follow `left`
   * follow `right`

2. **Sudden stop**

   * drive normally on its TTL
   * then command speed to zero

3. **Out-of-control swerve**

   * drive on TTL
   * then swerve right
   * then swerve left
   * then stop

So the fellow is **not** a rich racing agent. It is more like a scripted traffic obstacle with a few controlled disturbance patterns.

That means the scenario design should stop assuming:

* intelligent blocking
* tactical defense
* adaptive lane changes
* strategic racecraft from the fellow

Instead, every test should be based on one of these simple fellow scripts.

---

# Revised roadmap from Phase 6 onward

I’ll make this very concrete:

* what to implement
* what exact fellow script to use
* what ego should do
* what logs to verify
* what counts as done

---

# Standard test setup conventions

To keep the tests repeatable, use a standard way of defining scenarios.

For each test, define:

* **ego start TTL**: usually `optimal`
* **fellow start TTL**: `optimal`, `left`, or `right`
* **ego speed**
* **fellow speed**
* **initial longitudinal gap**
* **segment type**: straight, corner entry, corner body, corner exit
* **fellow script**:

  * `TTL_CRUISE`
  * `TTL_SUDDEN_STOP`
  * `TTL_SWERVE_OUT_OF_CONTROL`

And for logs, keep a standard contract from Phase 6 onward:

* current planner state
* active TTL
* target speed cap
* ego speed
* fellow speed
* ego progress
* fellow progress
* actual gap
* safe gap
* left/right/optimal open flags
* prediction outputs when available
* guard activation flags
* decision reason string
* collision / off-track / forced-stop flags

---

# Shared scenario bank using only real fellow capabilities

These are the reusable scenarios for all later phases.

## F0 — Ego alone

No fellow.

Use:

* ego on `optimal`
* standard race speed
* straight + mixed lap

Purpose:

* baseline
* prove planner does not interfere when there is no opponent

---

## F1 — Fellow behind, same TTL, cruising

Use:

* ego on `optimal`
* fellow on `optimal`
* fellow starts behind by a large gap
* fellow speed equal or slightly lower than ego
* fellow script: `TTL_CRUISE`

Purpose:

* verify ego stays in fast-lap mode
* verify no unnecessary caution because fellow is behind

---

## F2 — Fellow ahead, same TTL, slower, cruising

Use:

* ego on `optimal`
* fellow on `optimal`
* fellow ahead by moderate gap
* fellow speed clearly lower than ego
* fellow script: `TTL_CRUISE`

Purpose:

* verify follow behavior
* verify speed reduction when no pass yet committed

---

## F3L — Fellow ahead on left TTL, slower, cruising

Use:

* ego on `optimal`
* fellow on `left`
* fellow ahead by moderate gap
* fellow speed lower than ego
* fellow script: `TTL_CRUISE`

Purpose:

* verify planner reasons about corridor occupancy
* intended clean route should usually bias away from `left`

---

## F3R — Fellow ahead on right TTL, slower, cruising

Use:

* ego on `optimal`
* fellow on `right`
* fellow ahead by moderate gap
* fellow speed lower than ego
* fellow script: `TTL_CRUISE`

Purpose:

* symmetric test of F3L

---

## F4 — Fellow ahead, same TTL, sudden stop

Use:

* ego on `optimal`
* fellow on `optimal`
* fellow ahead by moderate gap
* fellow speed initially similar or slightly slower
* after a fixed interval, fellow script transitions to `TTL_SUDDEN_STOP`

Purpose:

* verify prediction and emergency stability
* verify ego does not panic-swerve

---

## F5 — Fellow ahead, same TTL, swerve right then left then stop

Use:

* ego on `optimal`
* fellow on `optimal`
* fellow ahead by moderate gap
* fellow script: `TTL_SWERVE_OUT_OF_CONTROL`

Purpose:

* verify prediction usefulness
* verify anti-chaotic control
* verify commit vs abort when corridor changes quickly

---

## F6 — Fellow ahead on left TTL, cruises there steadily

Use:

* ego on `optimal`
* fellow on `left`
* fellow ahead
* fellow script: `TTL_CRUISE`

Purpose:

* deterministic “left occupied” scenario

---

## F7 — Fellow ahead on right TTL, cruises there steadily

Use:

* ego on `optimal`
* fellow on `right`
* fellow ahead
* fellow script: `TTL_CRUISE`

Purpose:

* deterministic “right occupied” scenario

---

## F8 — Corner entry with fellow ahead on optimal

Use:

* ego on `optimal`
* fellow on `optimal`
* fellow ahead
* both entering a corner
* fellow script: `TTL_CRUISE`

Purpose:

* later segment-aware logic
* ego should not force bad late passes

---

# Phase 6 — Restructure skeleton + observability

## What is being implemented

This phase is about **code structure**, not intelligence.

You introduce the new layers in code:

* state extraction
* prediction shell
* assessment shell
* planner shell
* guard shell
* top-level ego orchestrator

Even if some of them are still simple, they must exist and be called.

## Scenarios to run

Run:

* F0
* F1
* F2

These are enough because the goal is only to prove the new architecture is alive without breaking baseline driving.

## What to verify

You should see, on every control cycle:

* state layer log
* planner layer log
* guard layer log
* executor call log

And the vehicle should still complete the scenario without regression.

## Done criteria

Phase 6 is done when:

1. The new modules are actually used by ego at runtime.
2. The old monolithic path is no longer the only active path.
3. F0 completes without crash or off-track.
4. F1 completes without ego behaving strangely because fellow is behind.
5. F2 completes without architecture-induced regression.
6. Logs contain a per-cycle planner state and active TTL, even if still simple.

### Minimal benchmark

For each run, confirm:

* `planner_state` present in log
* `active_ttl` present in log
* `decision_reason` present in log
* no collision
* no off-track caused by restructure

---

# Phase 7 — Fellow next-step prediction

## What is being implemented

Now add a real predictor for the fellow.

At minimum:

* next-step position estimate
* next-step heading or progress estimate

This should work from the fellow’s pose history only.

## Scenarios to run

Run:

* F2
* F4
* F5
* F6
* F7

Why these:

* F2 tests simple constant cruise on `optimal`
* F6/F7 test simple constant cruise on non-optimal TTLs
* F4 tests stop onset
* F5 tests the most dynamic fellow motion you actually have

## What to verify

New logs must include:

* `fellow_pred_x`, `fellow_pred_y`
* `fellow_pred_s`
* `prediction_error_next_step`

## Done criteria

Phase 7 is done when:

### In F2, F6, F7

* the next-step prediction is consistently close to actual next-step fellow motion
* error stays small and stable

### In F4

* when the fellow begins its stop event, prediction reflects the speed drop better than simply assuming continued cruise

### In F5

* the predictor reacts to the right-left maneuver and gives a meaningfully better forecast than “fellow stays where it is”

### Minimal benchmark

Produce a summary report with:

* average next-step position error for each scenario
* maximum next-step position error for each scenario

**Done** means:

* prediction runs every cycle
* errors are bounded
* predictor is measurably better than a zero-motion or hold-last-pose baseline, especially in F5

---

# Phase 8 — Situation assessment + dynamic safe gap

## What is being implemented

Build the assessment layer that turns current and predicted state into tactical facts.

Outputs should include:

* fellow ahead / behind
* closing / not closing
* overlap / no overlap
* dynamic safe following gap
* `optimal_open`, `left_open`, `right_open`

## Scenarios to run

Run:

* F1
* F2
* F6
* F7
* F4

These cover:

* behind
* ahead
* left occupied
* right occupied
* emergency closure

## What to verify

New logs must include:

* `fellow_relation`
* `closing_flag`
* `safe_gap`
* `actual_gap`
* `gap_ok`
* `optimal_open`
* `left_open`
* `right_open`

## Done criteria

Phase 8 is done when:

### In F1

* relation is logged as `behind` for almost the whole run
* planner inputs do not falsely treat fellow as blocking

### In F2

* relation is logged as `ahead`
* `safe_gap` increases with ego speed
* if ego closes too much, `gap_ok` becomes false

### In F6

* `left_open` is false or strongly penalized compared with the other corridors

### In F7

* `right_open` is false or strongly penalized compared with the other corridors

### In F4

* when the stop begins, the assessment reflects rapidly degrading safety margin

### Minimal benchmark

For each scenario, define an expected label and verify the logs match it.

Examples:

* F1 → behind
* F2 → ahead, follow candidate
* F6 → left occupied
* F7 → right occupied
* F4 → emergency risk rises after stop onset

**Done** means the assessment logs match the scenario truth stably, without wild flicker.

---

# Phase 9 — Tactical planner v1: free-run, follow, setup

## What is being implemented

Add the first real tactical planner states:

* `FREE_RUN`
* `FOLLOW`
* `SETUP_LEFT`
* `SETUP_RIGHT`

Still no hard pass commit yet.

Behavior expectation:

* if fellow is behind → free-run
* if fellow is ahead and blocking → follow
* if one side looks promising → enter setup for that side

## Scenarios to run

Run:

* F0
* F1
* F2
* F6
* F7

These are ideal because the fellow is still simple and deterministic.

## What to verify

New logs must include:

* `planner_state`
* `chosen_ttl`
* `decision_reason`
* `target_speed_cap`

## Done criteria

Phase 9 is done when:

### In F0

* ego remains in `FREE_RUN`
* ego mostly stays on `optimal`

### In F1

* ego remains in `FREE_RUN`
* ego does not slow down unnecessarily because the fellow is behind

### In F2

* ego enters `FOLLOW`
* ego reduces target speed when the fellow is ahead and no commitment exists
* ego does not rear-end the fellow

### In F6

* ego should not choose `left` as its setup side while left is occupied by the fellow
* planner should either stay `FOLLOW` or bias to `SETUP_RIGHT`

### In F7

* symmetric to F6: avoid `right`, prefer `SETUP_LEFT` or remain `FOLLOW`

### Minimal benchmark

Look at decision logs over time.

Examples of acceptable sequences:

* F2: `FREE_RUN -> FOLLOW`
* F6: `FOLLOW -> SETUP_RIGHT`
* F7: `FOLLOW -> SETUP_LEFT`

**Done** means the planner picks sensible setup states from the fellow’s actual simple occupancy pattern and does not chatter.

---

# Phase 10 — Stability guard / anti-swerve emergency policy

## What is being implemented

Add the safety/stability guard that prevents dangerous control combinations.

This is where you explicitly block:

* hard brake + hard swerve combinations
* violent repeated TTL switches
* steering snap caused by sudden decision changes

## Scenarios to run

Run:

* F4
* F5
* F2 with very small initial gap
* F6 and F7 with aggressive ego closing speed

These are the cases where the guard matters.

## What to verify

Logs must include:

* `guard_active`
* `guard_reason`
* `steer_limited`
* `brake_limited`
* `ttl_switch_blocked`
* `emergency_stable_mode`

## Done criteria

Phase 10 is done when:

### In F4

* ego responds to the fellow’s stop without a panic lateral jump
* if braking becomes strong, steering aggressiveness is reduced

### In F5

* ego does not mirror the fellow’s crazy right-left motion with equally crazy commands
* guard prevents unstable oscillations

### In tight-gap F2

* ego stabilizes into controlled following instead of unstable brake-steer conflict

### Minimal benchmark

Measure:

* maximum steering rate
* maximum brake command
* number of TTL changes per second
* number of guard activations

**Done** means:

* these values stay within chosen limits
* the logs show guard interventions for the right scenarios
* no crash is caused by chaotic ego reaction

---

# Phase 11 — Pass commit / abort logic

## What is being implemented

Now add true overtaking states:

* `COMMIT_PASS_LEFT`
* `COMMIT_PASS_RIGHT`
* `ABORT_PASS`

Since the fellow is simple, this phase should be tested mostly against:

* a fellow cruising on one TTL
* a fellow suddenly stopping
* a fellow swerving out of control

Not against imagined intelligent defense.

## Scenarios to run

Run:

* F2
* F6
* F7
* F4
* F5

## What to verify

Logs must include:

* `commit_trigger`
* `abort_trigger`
* `pass_success`
* `abort_success`
* `post_event_state`

## Done criteria

Phase 11 is done when:

### In F2

* ego can eventually commit to a pass if one corridor remains safely open
* decision log should clearly show why that corridor was chosen

### In F6

* ego should strongly prefer passing away from the occupied left side
* a successful sequence might be:
  `FOLLOW -> SETUP_RIGHT -> COMMIT_PASS_RIGHT -> FREE_RUN`

### In F7

* symmetric:
  `FOLLOW -> SETUP_LEFT -> COMMIT_PASS_LEFT -> FREE_RUN`

### In F4

* if the fellow suddenly stops and a safe pass route is not viable, ego should not force a commit
* if a previously planned route becomes bad, ego should log `ABORT_PASS` or stay in controlled follow/emergency mode

### In F5

* if the fellow’s swerve invalidates the chosen route, ego must abort cleanly instead of continuing into instability

### Minimal benchmark

For repeated runs:

* count successful clean bypasses in F6/F7
* count safe aborts in F4/F5
* count crashes

**Done** means:

* planner logs show the correct decision chain
* successful bypasses happen in the deterministic cruise cases
* safe aborts happen in stop/swerve disruption cases
* no crash occurs because ego kept a bad commit alive too long

---

# Phase 12 — Segment-aware tactical intelligence

## What is being implemented

Now use segment type to improve decision timing.

This matters even more now because the fellow is simple. Since the fellow is not tactically clever, a lot of the remaining intelligence should come from ego understanding **where on the track** the interaction happens.

## Scenarios to run

Run these versions of existing tests on specific segment types:

* F2 on straight
* F2 on corner entry
* F6 on straight
* F6 on corner entry
* F7 on straight
* F7 on corner entry
* F5 near corner entry

## What to verify

Logs must include:

* `segment_type`
* `segment_modifier`
* `segment_accepted_or_rejected_pass_reason`

## Done criteria

Phase 12 is done when:

### On straights

* ego is more willing to set up and commit when the corridor is open

### On corner entry

* ego becomes more conservative
* late, bad pass attempts are reduced

### In F5 near corner entry

* ego should not combine a fellow’s unpredictable swerve with an aggressive late pass attempt

### Comparative benchmark

Compare the same scenario before and after segment-aware logic.

Examples:

* F6 straight vs F6 corner entry
* F7 straight vs F7 corner entry

You should see:

* more good commits on straights
* fewer bad commits into corners
* fewer aborts caused by bad timing

**Done** means the logs show the segment context actively influencing decisions and the outcomes improve.

---

# What success looks like now, given the limited fellow

Because the fellow is simple, the benchmarks should not ask:

* “Did ego outsmart an intelligent defender?”

They should ask:

* “Did ego correctly interpret a simple scripted obstacle-like racing agent and make a stable, explainable decision?”

So the most important evidence is:

1. **Correct understanding of fellow behavior**

   * cruise on a chosen TTL
   * stop event
   * right-left-stop event

2. **Correct planner interpretation**

   * behind → free-run
   * ahead and blocking → follow
   * occupied side avoided
   * open side used
   * route invalidation triggers abort or stable fallback

3. **Correct physical outcome**

   * no rear-end
   * no chaotic swerve
   * successful pass in deterministic cruise cases
   * safe abort in disruptive cases

---

# Best scenario-to-phase mapping, revised

If you want the tightest version:

## Phase 6

* F0, F1, F2

## Phase 7

* F2, F4, F5, F6, F7

## Phase 8

* F1, F2, F4, F6, F7

## Phase 9

* F0, F1, F2, F6, F7

## Phase 10

* F4, F5, tight-gap F2, aggressive-closing F6/F7

## Phase 11

* F2, F4, F5, F6, F7

## Phase 12

* F2/F6/F7 on straight vs corner entry, plus F5 near corner entry

---

# My strongest recommendation for your test discipline

Because the fellow is simple, every scenario should be described in one line like this:

* **Fellow script**
* **Fellow TTL**
* **Fellow speed**
* **Initial gap**
* **Track segment type**

For example:

* `TTL_CRUISE, left, 130 mph, 35 m gap, straight`
* `TTL_SUDDEN_STOP, optimal, 150 mph then stop at t=8 s, 30 m gap, straight`
* `TTL_SWERVE_OUT_OF_CONTROL, optimal, 140 mph, 40 m gap, corner entry`

That way, every benchmark is grounded in something the fellow can actually execute.

---