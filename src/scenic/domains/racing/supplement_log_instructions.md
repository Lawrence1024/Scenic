Here’s what I’d log for Todo2 so we can validate it *quickly* from a short run, plus what to do with A–E and the old logs.

## Logs to add for validating Todo2 (minimal but sufficient)

Todo2 is “CTE deadzone near-line only.” The failure modes are:

* deadzone triggers when it shouldn’t (suppresses correction while still off-line)
* deadzone doesn’t trigger at all (no benefit)
* deadzone interacts with curvature/association gates and causes plan jitter

So log these **per control tick** (or at least every N=5–10 ticks), and also print them whenever deadzone state changes:

### 1) Deadzone decision + thresholds (must-have)

* `deadzone_applied` (bool)
* `dz_cte_m` (value used this tick)
* `cte_used_for_control` (after deadzone; e.g., 0 if inside zone)
* `cte_raw` (before deadzone)
* `deadzone_reason` (single token is enough):
  `CTE_SMALL`, `MATCH_GOOD`, `CURV_LOW`, or `BLOCKED_CTE_BIG/MATCH_BAD/CURV_HIGH`

**What I’ll look for:** deadzone applies mostly when `|cte_raw|` is tiny, and never when `|cte_raw|` is > ~0.3–0.5m.

### 2) Association quality (must-have)

* `match_dist_m`
* `gate_accept` (if you still have gating)
* `segment_id` (or `s_ref` if you use it)

**What I’ll look for:** deadzone should almost always require `match_dist_m` below your “good association” threshold.

### 3) Curvature regime (recommended)

* `curv_ahead_max` (or whatever you use)
* `curv_regime` (LOW/MID/HIGH) if you have it
* `kappa_ref_at_proj` (optional)

**What I’ll look for:** deadzone should be rare/disabled in high curvature.

### 4) Steering pipeline (recommended, very helpful)

* `steer_mpc_raw`
* `steer_after_caps`
* `steer_after_lpf` (final)
* `steer_rate` (optional)

**What I’ll look for:** straights should have less tiny steer noise after Todo2, without changing curve entry.

### 5) A “tripwire” warning (strongly recommended)

Print a warning if this ever happens:

* `deadzone_applied == true` **AND** `abs(cte_raw) > 0.5m`
  or
* `deadzone_applied == true` **AND** `match_dist_m > 1.5m`

If it triggers even once, Todo2 is unsafe / miswired.

---

## How to interpret “Todo2 is good” from a short log (10–30s)

Todo2 is doing the right thing if:

* Deadzone applies frequently on straights when `|CTE| < ~0.1–0.2m`
* Deadzone almost never applies in curve segments or when `|CTE| > ~0.3m`
* Steering on straights shows fewer tiny oscillations (raw MPC may still oscillate, but final steering should calm down)
* No increase in peak |CTE| near curve entry (should stay similar to your “healthy” run)


If you add the **must-have logs (sections 1–2)** plus the **tripwire**, that’s enough for me to validate Todo2 confidently from a short snippet.
