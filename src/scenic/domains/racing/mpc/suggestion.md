Most viable fix: **stop driving “waypoint-to-waypoint” at corners** and instead give the MPC a **continuous, curvature-aware reference** *plus* a simple **speed gate**. This solves “didn’t turn in time” without rewriting your whole controller.

## The solution (minimal changes, maximum payoff)

### 1) Fit a spline centerline and compute curvature lookahead

* Convert your global waypoints into an **arc-length parameterized spline** (cubic B-spline / cubic Hermite).
* At runtime, project the car onto the spline to get (s_0).
* Sample a reference trajectory over the MPC horizon: (s_k = s_0 + v \cdot k\Delta t).
* For each (s_k), compute:

  * reference position ((x_r, y_r))
  * reference heading (\psi_r)
  * **curvature (\kappa_r)** (from spline derivatives)

Why this works: the MPC **sees the left turn early** because curvature is defined continuously, not revealed “one waypoint late”.

### 2) Use curvature-adaptive sampling density (only in tight turns)

Even if your map points are 5 m apart, you resample densely near high curvature:

* If (|\kappa| < 0.01) (R > 100 m): sample every **1.0–2.0 m**
* If (|\kappa| \ge 0.01): sample every **0.25–0.5 m**

This removes polygon-corner artifacts and prevents late turn-in.

### 3) Fix waypoint advancement: progress-based, not radius-based

Replace “advance if within radius 3 m” with:

**Advance based on arc-length progress**

* Maintain (s_0) = projected arc-length on spline
* Your “current index” is whatever spline segment contains (s_0)
* No more skipping ahead because you got “close” to a future point while laterally off.

This directly removes the log symptom where global CTE is large but MPC’s internal (e_y) becomes tiny.

### 4) Add a simple speed gate from curvature (one line of logic)

Before solving MPC (or as an MPC constraint), cap speed using:
[
v_{\max}(s) = \sqrt{\frac{a_{y,\max}}{|\kappa(s)| + \epsilon}}
]
Choose (a_{y,\max}) based on your sim/tires (start conservative):

* indoor sim / modest grip: **6–8 m/s²**
* race grip sim: **8–12 m/s²**

Then set commanded speed:
[
v_{\text{ref}} = \min(v_{\text{desired}}, \min_{s\in [s_0, s_0+L]} v_{\max}(s))
]

This ensures you *never* enter a sharp left at 10 m/s if the car can’t physically make it.

### 5) Keep steering slew limit, but make it curvature-aware (optional, easy win)

Slew limiting is good for stability, but in corners it can be too restrictive.

Use:

* on straights: normal slew limit
* in corners ((|\kappa|) above threshold): allow **2×** faster steering rate

This prevents “MPC realizes late but actuator won’t move fast enough”.

---

## Why this is the *most viable* option

* **Doesn’t require a new MPC model**
* **Doesn’t require retuning everything**
* Fixes the exact failure chain:

  * curvature revealed too late ✅
  * waypoint skipping masks lateral error ✅
  * entering corners too fast ✅
  * steering can’t ramp fast enough ✅

---

## Implementation checklist (what to do first)

1. Implement spline + projection → get (s_0)
2. Replace waypoint-advance logic with “use (s_0)” as progress
3. Resample reference along horizon from spline
4. Add curvature-based (v_{\max}) gate

If you paste your current waypoint selection / advancement code (the “within radius” part) and how you build the MPC reference vector, I can rewrite it into the spline/progress version in a way that drops into your code with minimal disruption.
