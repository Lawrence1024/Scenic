I read the new long `run.log` (front + the `[STEER_CAL]` block).

### The good news

You **did add the `[STEER_CAL]` lines** successfully — I see **221** of them, with fields:
`v, yaw_rps, delta_cmd_rad, L, kappa_meas, kappa_pred, curv_err, kappa_ratio`.

### The blocker

In this entire log, **`yaw_rps` is always `0.0000`** (every single `[STEER_CAL]` line). As a result:

* `kappa_meas` is always **0**
* `curv_err` becomes just `-kappa_pred`

So this log **cannot validate the steering-unit mapping via curvature** yet — it only tells us your *predicted* curvature, not what the vehicle actually did.

That almost certainly means your yaw-rate signal source is either:

* not wired / always zero in the channel you’re reading, or
* being rounded to 0 due to formatting/units, or
* you’re accidentally logging the wrong field.

Also note: some older uploads in this chat have expired on my side. Your current file is fine, but if you want me to compare against older baselines, you’ll need to re-upload those logs.

---

## What to add to make this a fair “already running” analysis (no special experiment)

You don’t need wheel radians. You just need **one reliable measured yaw rate** (or an equivalent you can compute).

### Option 1 (best): compute yaw rate yourself from heading (no extra sim signals needed)

Most stacks already have ego heading/yaw angle (ψ). Log:

* `psi_rad` (heading)
* `dt`

Then compute and log:

* `yaw_rps_est = wrapToPi(psi_now - psi_prev) / dt`

This avoids the “yaw rate channel stuck at 0” problem entirely.

**Log line (per tick or every N ticks):**

```
[STEER_CAL] v=... psi=... yaw_rps_est=... delta_cmd_rad=... L=... kappa_meas=... kappa_pred=... curv_err=...
```

Where:

* `kappa_meas = yaw_rps_est / max(v, 0.5)`

### Option 2: compute curvature from trajectory geometry (also robust)

If you already log position `(x,y)`:

* compute curvature from 3 consecutive points (or use heading derivative again).
  This is slightly noisier but works even if heading is noisy.

---

## You also need one IO log to resolve the “240 unit” question cleanly

Right now, I **still don’t see any log line showing the actual number you send to dSPACE**.

Add:

* `cmd_value_sent` (the exact float written to `Const_steering_cmd/Value`)
* `theta_sw_deg_sent` (same as cmd_value_sent if you believe it’s wheel-deg)
* `steer_norm`
* `delta_cmd_rad`
* `R_used`

**Example:**

```
[STEER_IO] u_norm=... delta_cmd_rad=... steer_norm=... theta_sw_deg_sent=... R=...
```

With this, we can verify:

* `theta_sw_deg_sent ≈ steer_norm * 240`
* and `steer_norm ≈ delta_cmd_rad / 0.2816`

---

## What “correct” will look like once yaw is real

On steady cornering (ignoring transient moments):

* `sign(kappa_meas) == sign(kappa_pred)` almost always
* `kappa_meas` should be the same order as `kappa_pred` (not 10× off)
* `curv_err = kappa_meas - kappa_pred` should be centered near 0 with some bias/noise

If instead you see something like `kappa_meas ≈ 15× kappa_pred`, that would indicate you’re mixing steering wheel vs road wheel units.

---

## Concrete checklist for your next run (10 seconds is enough)

Add these logs:

1. **Heading-based yaw rate**

* `psi_rad`, `yaw_rps_est`

2. **IO command value**

* `cmd_value_sent` to `Const_steering_cmd/Value`

3. **Existing control**

* `delta_cmd_rad`, `delta_max`, `steer_norm`

That’s it. Then I can definitively tell you whether the 240 scaling is:

* steering wheel degrees (very likely), or
* something else.

If you paste a snippet of your code where you fetch “yaw rate” (the thing currently printing 0.0000), I can also tell you what likely signal/name is wrong and how to replace it with the heading-derivative method.
