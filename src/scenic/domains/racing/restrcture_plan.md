# Ego Smart Driving Restructure Planner

## Purpose

This document proposes a major restructuring of the current racing stack so that smarter ego driving logic can be added cleanly over time. The immediate goal is not just to add one more behavior, but to reorganize the code into clear layers so that future logic such as prediction, tactical passing, safer following, and emergency handling can be implemented without continuing to grow one giant behavior file.

This planner document is deliberately opinionated. It describes what the stack should become, what the first new capabilities should be, how ego is expected to behave around one fellow car, and how to separate responsibilities across files and layers.

Pit lane logic is intentionally deferred for now. Existing pit handling should continue to work as-is. The main focus of this document is ego behavior on the main racing loop with one fellow car.

---

## Main Problems in the Current Stack

The current racing stack already has many strong pieces: track geometry, TTL abstractions, waypoints, segments, MPC, and several behavior wrappers. The problem is not lack of capability. The problem is that too much logic is packed into the same places.

The most important structural problem is that `FollowRacingLineMPCBehavior` currently does too many jobs at once. It is simultaneously acting as controller setup code, waypoint progression logic, segment-aware logic, speed management, safety handling, and final control output logic. It is therefore very difficult to add more advanced tactical behavior without making that file even harder to reason about.

A second problem is that the current decision-level behaviors are mostly wrappers that set intent and then delegate back into the same executor. This means the codebase has the vocabulary for strategy, but not yet a clean planning layer that owns decision-making independently from low-level control.

A third problem is that the current stack does not have a proper explicit prediction layer. Decisions are still mostly reactive to the fellow’s current pose instead of to a short-horizon estimate of what the fellow will do next.

A fourth problem is that emergency behavior can produce combinations of hard braking and hard swerving that are physically dangerous and tactically poor. The stack needs a dedicated safety policy for smoothness and stability, not just emergency reactions.

Finally, the current following behavior uses a target gap idea, but the intended safety principle should be time-based and braking-based rather than tied to one fixed constant distance.

---

## Design Goals

The restructure should aim for the following properties.

First, each file or module should belong to one primary layer. A file should not try to be world model, planner, controller, and safety system at the same time.

Second, the ego stack should become planner-first rather than wrapper-first. That does not mean replacing the MPC. It means giving ego a real tactical planning layer that sits above the MPC executor.

Third, the system should support richer logic without requiring giant edits to one behavior file. New logic should be added by expanding a planner, a predictor, or a policy module, not by mixing more conditionals into the executor.

Fourth, decisions should be made from predicted short-horizon state, not just from current pose.

Fifth, follow and pass behavior should match a clear racing intent. If the fellow is behind, ego should mostly run for fast lap time. If the fellow is ahead, ego should follow intelligently, wait for openings, reduce target speed when no clear route is available, and only overtake when the conditions are favorable.

Sixth, emergency handling must forbid wild combinations of aggressive braking and strong swerve. The planner should prefer stable, controlled reactions.

Seventh, following safety should depend on speed and braking time, not one fixed gap. A faster car should require more space to follow safely.

---

## Core Behavioral Intent for Ego

The intended ego behavior can be summarized in a few simple principles.

If the fellow car is behind ego and not creating immediate pressure, ego should simply drive for best progress and best lap time. In practice this means staying on the best TTL for speed unless there is a tactical reason to do otherwise.

If the fellow car is ahead of ego, ego should become opportunity-seeking rather than blindly aggressive. Ego should continue advancing but should not force an impossible route. If there is no clear passable route ahead, the target speed should be reduced so that ego follows with discipline instead of charging into the fellow.

If an opening appears, ego may decide to overtake. That decision should not be based on one frame or one geometric coincidence. It should be based on a short-horizon prediction of both cars and on a safety-aware estimate of whether the selected TTL remains viable long enough to complete or safely abandon the move.

If an emergency occurs, ego must not perform dangerous combined actions such as hard braking plus hard swerving. The system should explicitly prevent these combinations. The default emergency philosophy should be: maintain stability first, preserve controllability second, then trade speed for safety in a bounded way.

If ego is following the fellow, the following distance should depend on current speed and estimated braking time rather than on one fixed constant. The higher the speed, the larger the required safe gap.

---

## Proposed Layered Architecture

The target architecture should separate the stack into clear layers.

### Layer 1: World and Track Data

This layer owns static geometry and racing references:

* track regions
  n- TTL definitions
* waypoints
* segment maps
* ring topology and progress helpers

This layer should answer questions such as where the optimal, left, and right TTLs are, how progress along them is measured, and what type of segment the current location belongs to.

This layer should not decide behavior.

### Layer 2: State Extraction and Normalization

This layer reads simulator state and converts it into a clean racing-state representation for ego and fellow. It should extract:

* ego pose, heading, speed, gear
* fellow pose, heading, speed
* nearest progress along track
* relation to active TTL and reference lines
* current segment classification
* available route candidates

This layer should not make tactical decisions. It should just provide a clean, normalized state.

### Layer 3: Prediction

This is the first new major capability.

This layer should estimate short-horizon future state for ego and fellow. At minimum, only using the fellow’s observed pose over time, it should estimate where the fellow is likely to be at the next timestep and optionally over a small horizon.

The first predictor does not need to be complicated. A simple constant-velocity or constant-turn-rate estimate is enough for the first version. The important point is that planning should stop being purely reactive.

Prediction output should include:

* predicted fellow position next step
* predicted fellow heading next step
* predicted fellow progress on track next step
* predicted ego progress under each candidate TTL over a short horizon
* simple occupancy or corridor confidence for left, right, and optimal

This layer should not decide what ego does. It should provide future estimates the planner can reason over.

### Layer 4: Situation Assessment

This layer converts raw current state and predicted state into tactical facts.

It should answer questions such as:

* is the fellow ahead or behind
* are we closing or being dropped
* are we currently following
* are we partially overlapping
* which TTLs look blocked over the next step or short horizon
* which TTLs look open
* is the current situation a free-run, follow, pass setup, or emergency-style condition
* how large should the safe following gap be at the current speed

This layer should remain descriptive rather than prescriptive. It tells the planner what the situation is.

### Layer 5: Tactical Planner

This is the real decision owner.

Given the assessed situation, the planner should decide:

* whether ego should free-run, follow, prepare to pass, commit to pass, abort, or handle emergency conservatively
* which TTL should be active: optimal, left, or right
* what target speed or speed cap should be used
* whether ego should remain patient because no passable route exists
* whether a previously promising line is no longer safe and should be abandoned

This is the layer where the new smart driving logic should primarily live.

### Layer 6: Safety and Comfort Policy

This layer constrains tactical outputs so the vehicle remains stable and realistic.

It should apply limits such as:

* no aggressive swerving beyond allowed lateral command rate
* no combined full braking and hard steering in the same step unless absolutely required
* bounded TTL switching frequency
* bounded steering slew
* bounded deceleration based on current dynamics and confidence
* conservative fallback if planner confidence is low

This layer should be able to override or soften the tactical plan if the tactical plan is too aggressive.

### Layer 7: Reference Selection and Control Interface

This layer translates planner output into a reference the existing executor and MPC can consume.

For now, the main outputs are:

* selected TTL or reference family
* target speed
* optional scaling factors or constraints

This layer should be the only place where planner decisions are translated into controller inputs.

### Layer 8: Low-Level Control Execution

This layer contains the MPC and actuation output.

Its job is to track the selected reference cleanly, not to make race strategy decisions. It should remain focused on:

* reference building
* lateral MPC
* longitudinal MPC
* gear handling if still kept here
* final steer/throttle/brake actions

The long-term direction is that the executor should become simpler because higher-level planning and safety logic are moved out of it.

---

## Prediction: The First New Capability

The first important new capability should be a prediction module.

The immediate practical requirement is modest: only looking at the opponent’s pose over time, estimate where the fellow will be at the next timestep. This is already enough to improve tactical decisions substantially.

A good first version should maintain a short rolling history of the fellow state. From the most recent samples, it can estimate:

* velocity vector
* heading rate
* progress rate along track
* lateral drift with respect to the nearest TTL

Then it can predict the next state using a simple motion model.

The predictor does not need to solve full trajectory forecasting. For the first implementation, a one-step-ahead estimate is enough. After that, it can be extended to a short horizon of perhaps a few control steps.

The planner should use this information to ask questions like:

* if ego stays on the current TTL, where will the fellow be next step
* if ego shifts to left or right, will that route still be open next step
* is the fellow moving toward the line ego wanted to take
* is the gap opening or shrinking

Prediction should therefore be treated as a dedicated module, not as a few ad hoc lines mixed into the behavior file.

---

## Intended Tactical Logic

### Case A: Fellow Behind Ego

If the fellow is behind ego and not an immediate threat, ego should prioritize fastest progress and stable lap time. In practice this means:

* use the best available racing TTL, usually optimal
* do not slow down unnecessarily
* do not make defensive or evasive choices unless the fellow becomes tactically relevant

This is the clean free-run case.

### Case B: Fellow Ahead of Ego, No Clear Pass

If the fellow is ahead and there is no clear passable route, ego should follow rather than force an unsafe or low-probability move.

In this mode, ego should:

* maintain a speed-dependent safe following gap
* reduce target speed if needed so the gap remains stable
* continue monitoring candidate TTLs for openings
* avoid excessive left-right switching

This is the disciplined follow case.

### Case C: Fellow Ahead, Opening Appears

If the fellow is ahead and a candidate route becomes passable according to the predictor and situation assessor, ego may decide to overtake.

This should happen only when:

* the candidate TTL remains open with reasonable confidence over the short horizon
* switching to that TTL does not require extreme steering or unstable control
* the selected route offers real progress advantage rather than cosmetic movement

Once an opening is judged valid, the planner may commit to the overtake.

### Case D: Emergency or Rapidly Degrading Situation

If the situation degrades quickly, the planner should preserve stability first.

This means:

* avoid large simultaneous steering and braking spikes
* prefer a bounded, stable response
* if the chosen pass route collapses, back out in a controlled way
* prioritize maintaining controllability and avoiding oscillations

Emergency behavior should look composed, not panicked.

---

## Safe Following Distance Policy

The safe following distance should not be a fixed constant. It should be derived from speed and braking time.

A natural first model is time headway based:

safe_gap = minimum_distance + speed * desired_time_headway

This is already much better than one fixed constant. It means that at higher speed, ego naturally demands more space.

A stronger version should include braking estimates:

safe_gap = base_margin + reaction_distance + braking_distance_difference

Conceptually, this means ego should think in terms of how long and how far it would need to slow down safely if the fellow brakes or the route closes.

For the first implementation, a speed-dependent time-headway model is good enough. It can later be extended with explicit deceleration assumptions.

The planner should use this dynamic safe gap when deciding whether to stay in follow mode, reduce target speed, or begin preparing for a pass.

---

## Anti-Swerve and Smooth Emergency Policy

One explicit design goal is to prevent dangerous behavior that combines hard braking with hard swerving.

The stack should therefore include a safety policy with rules such as:

* if braking demand is above a strong threshold, cap lateral aggressiveness
* if steering demand is large, avoid also commanding full braking in the same step unless collision risk is extreme
* limit steering slew rate so the vehicle does not snap violently between decisions
* require tactical TTL switches to remain stable for at least a minimum dwell time before allowing another change
* when planner confidence is low, prefer follow or mild speed reduction over abrupt maneuvers

This is not only for realism. It also makes the planner more debuggable, because unstable command combinations often hide the real tactical error underneath.

---

## Proposed Tactical State Machine

The new planner should be state-based.

A reasonable first tactical state machine is:

* `FREE_RUN`
* `FOLLOW`
* `SETUP_PASS_LEFT`
* `SETUP_PASS_RIGHT`
* `COMMIT_PASS_LEFT`
* `COMMIT_PASS_RIGHT`
* `ABORT_PASS`
* `EMERGENCY_STABLE`

The purpose of these states is clarity. They should mean exactly what they say.

`FREE_RUN` means the fellow is not materially constraining ego and ego is driving for best lap time.

`FOLLOW` means the fellow is ahead and no sufficiently safe opening exists. Ego should hold discipline and keep a dynamic safe gap.

`SETUP_PASS_LEFT` and `SETUP_PASS_RIGHT` mean a candidate line looks promising but the pass is not yet fully committed. Ego may bias its speed and TTL choice to position for the move.

`COMMIT_PASS_LEFT` and `COMMIT_PASS_RIGHT` mean the planner believes the route is sufficiently open and is actively executing the move.

`ABORT_PASS` means the previously chosen line is no longer acceptable and ego should return to a safe controlled state without panic.

`EMERGENCY_STABLE` means the priority is to remain stable and non-chaotic rather than to optimize lap time or continue racing aggressively.

---

## Proposed File and Layer Restructure

The goal is not just to add modules, but to assign each module a clear role.

One possible target structure is the following.

### `racing/world/`

Owns static racing world abstractions.

Possible contents:

* track adapters
* TTL registry
* waypoint access helpers
* segment access helpers
* progress/ring helpers

### `racing/state/`

Owns normalized runtime racing state.

Possible contents:

* ego state extraction
* fellow state extraction
* combined race state object
* helper conversions from simulator objects into planner state

### `racing/prediction/`

Owns future-state estimation.

Possible contents:

* motion history buffer
* simple one-step predictor
* short-horizon predictor
* candidate TTL corridor occupancy estimates

### `racing/assessment/`

Owns interpretation of the current and predicted state.

Possible contents:

* relative relation classification
* gap assessment
* open-route assessment
* dynamic safe-gap calculation
* emergency severity estimate

### `racing/planner/`

Owns tactical decisions.

Possible contents:

* tactical state machine
* TTL decision policy
* follow / setup / commit / abort logic
* planner outputs and decision records

### `racing/safety/`

Owns smoothness and stability rules.

Possible contents:

* steering slew guard
* brake-steer coupling policy
* TTL switch hysteresis
* bounded emergency response policy

### `racing/reference/`

Owns translation of planner decisions into controller-ready references.

Possible contents:

* active TTL selector
* target speed shaping
* controller command preparation

### `racing/control/`

Owns low-level execution.

Possible contents:

* MPC wrappers
* gear policy if retained here
* actuator output glue

### `racing/behaviors/`

Owns Scenic behavior shells only.

Possible contents:

* top-level ego behavior shell
* top-level fellow behavior shell
* minimal wrappers that call into planner and executor modules

The critical idea is that the giant logic currently inside `behaviors.scenic` should be pushed downward into dedicated Python modules. Scenic behaviors should increasingly become shells that coordinate modules rather than dense algorithm containers themselves.

---

## Immediate Restructure Principle for Existing Code

The current code should not be rewritten all at once. Instead, it should be extracted around responsibilities.

The first large extraction should be from the current ego executor behavior. The logic inside it should be split into separate helpers for:

* initialization and startup synchronization
* waypoint progression and segment lookup
* gear management
* prediction-aware planning inputs
* safety filtering
* MPC invocation
* final control emission

The planner-specific decisions should not live in that executor. The executor should eventually accept a planner output and simply drive the chosen reference.

Similarly, wrapper behaviors such as follow mode or lane selection should gradually stop containing direct tactical logic. They should either disappear into the new planner layer or become thin interfaces over it.

---

## First Concrete New Modules to Add

If only a few modules are added first, the most valuable starting set is:

### `prediction/fellow_predictor.py`

Maintains short history of fellow pose and estimates next-step state.

### `assessment/race_situation.py`

Uses current and predicted state to classify ahead/behind, gap quality, line openness, and follow safety.

### `planner/tactical_planner.py`

Owns the tactical state machine and chooses one of `optimal`, `left`, or `right` plus target speed policy.

### `safety/stability_guard.py`

Filters planner outputs to avoid dangerous combined steer/brake reactions and excessive switching.

### `behaviors/ego_main.py`

A new top-level ego behavior shell that orchestrates:

* extract state
* run prediction
* assess situation
* run planner
* apply safety guard
* call low-level executor

This would already create a much cleaner architecture without forcing a full rewrite on day one.

---

## Planner Inputs and Outputs

The planner should have a clean interface.

### Inputs

* current ego state
* current fellow state
* predicted ego short-horizon estimate if useful
* predicted fellow next-step or short-horizon estimate
* current segment type
* current active TTL
* dynamic safe gap estimate
* current control confidence or stability flags

### Outputs

* tactical state
* selected TTL
* target speed or target speed cap
* follow gap policy
* planner confidence
* whether the move is free-run, follow, setup, commit, abort, or emergency-stable

The planner should not output raw steering or raw brake commands.

---

## What Should Stay Out of the Planner

The planner should not own:

* direct MPC equations
* low-level actuator output
* simulator-specific unit conversion
* waypoint ring bookkeeping details
* giant Scenic control loops

The planner should work with clean state and produce clean tactical outputs.

---

## Discussion Questions for the Next Iteration

The next discussion should probably focus on these design choices.

First, how much of the current `FollowRacingLineMPCBehavior` should remain inside Scenic versus be extracted into Python modules.

Second, what exact motion model should be used for the first fellow predictor. A one-step constant-velocity model is probably enough initially, but we should confirm the expected control timestep and what signals are available reliably.

Third, what exact formula should define the dynamic safe following gap. A time-headway model is the easiest first version, but we should decide what values make sense for the current race speeds.

Fourth, how conservative the anti-swerve safety policy should be. We need it strong enough to prevent dangerous behavior but not so strong that the car becomes unable to react.

Fifth, whether the first tactical planner should reason only in terms of `optimal`, `left`, and `right`, or whether it should also score confidence over each line based on predicted occupancy over more than one step.

---

## Recommended Immediate Next Step

The best immediate next step is not to code everything at once. It is to lock the target architecture and then decide the first extraction boundary.

The strongest candidate is:

1. create a dedicated prediction module for the fellow
2. create a race-situation assessment module
3. create a top-level planner shell that chooses `optimal`, `left`, or `right` plus target speed
4. keep the existing MPC executor for now, but call it from a cleaner wrapper

That gives the stack a real planning layer without breaking the entire control system at once.

---

## Final Summary

This project should be treated as a major architecture improvement rather than as a small behavior tweak. The key shift is from a mostly executor-centric stack toward a layered stack with clear ownership for state, prediction, assessment, planning, safety, reference selection, and control.

The first major capability to add should be prediction of the fellow’s next position from pose history. That prediction should drive smarter tactical decisions.

The desired ego behavior is clear. If the fellow is behind, ego runs for fastest time. If the fellow is ahead, ego becomes disciplined: follow safely when no route is open, reduce speed when necessary, and overtake only when a predicted opening actually exists.

Safety should be dynamic and speed-dependent, and emergency behavior must explicitly avoid unstable combinations of heavy braking and violent swerving.

The result should be a cleaner codebase where each file belongs to one layer, and where more advanced logic can be added without continuing to grow one giant control behavior.
