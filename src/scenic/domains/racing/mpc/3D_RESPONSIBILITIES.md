# Task D — 3D responsibilities (split so PolylineRegion flattening doesn’t mislead)

Because Scenic’s **PolylineRegion flattens Z**, responsibilities should be split cleanly:

## Split

* **MPC / vehicle dynamics**  
  Should get **Z / elevation** from TTL, road surface, or racing model inputs (not from a flattened polyline). Use elevation where the controller or dynamics actually need it (e.g. grade, 3D pose).

* **Gates / regions / projections**  
  Can stay **2D (XY)**. Then do **not** feed them into any logic that assumes “3D geometry” (e.g. arc-length in 3D, elevation along path). Keep gates and projections in XY and document that they are 2D.

## If you truly need a 3D polyline

If something (e.g. elevation profile, 3D arc-length, or query-by-height) requires a **3D polyline**:

* **Stop using PolylineRegion for that purpose.**  
* Use your **own 3D representation** (e.g. a small class or a numpy array of (x, y, z) or (s, z)) and query it directly.  
* Build that representation from the same source as the TTL (e.g. OpenDRIVE, centerline + elevation) so it stays consistent.

This keeps “2D for gating/projection” and “3D where needed” clearly separated and avoids wrong assumptions from flattened geometry.
