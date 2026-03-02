# TTL Analysis: Section 1 vs Driving View, and Segment 43

This document explains why the **boundary graph** can show the TTL on the outer boundary (e.g. in “section 1”) while the **driving visualization** shows the car in the middle of the road, and how this relates to **segment 43** and CTE.

## 0. The transformation is correct

**You’re right:** we *are* driving in XODR. Scenic uses the map and the behavior TTL in the **XODR** frame. The pipeline is consistent:

- **TTL and map:** Loaded and used in XODR (e.g. `ttl_main_road.csv` or `ttl_racing_line_xodr.csv` in `LS_ENU_TTL_CSV` with offset (0,0)).
- **Placement:** When we place the ego, we take Scenic’s (XODR) position, apply **XODR → RD** with `apply_coordinate_transform`, then project to (s, t) on the RD road. So the car is placed correctly in the sim’s RD world.
- **Readback:** ControlDesk gives position in **RD**. We apply **RD → XODR** with `apply_inverse_coordinate_transform` and store that in Scenic. So Scenic’s state (position, etc.) is always in XODR.
- **MPC:** Uses XODR waypoints and XODR position (from readback). CTE and control are computed in one frame (XODR), so tracking is consistent.

So the transformation is **sorted out for driving**: the car follows the XODR path, and we convert consistently so that “on path in XODR” corresponds to “car in the right place in RD.” If the car looks like it’s driving fine in the RD view, that’s expected — the transform is doing its job.

The “discrepancy” below is **not** a transformation bug. It’s that we’re comparing **two different definitions of the track** (XODR boundaries vs RD lane geometry), so the same path can look “on the edge” in one and “more centered” in the other.

---

## 1. Why the graph shows “on the boundary” but the driving view shows “middle”

### Two different representations of the track

- **Boundary graph** (`visualize_ttl_boundaries.py`): Everything is in **XODR**.
  - Black lines = inner/outer track boundaries from the **XODR** file.
  - Blue line = TTL from the CSV (also XODR).
  - So the graph answers: “Where does the TTL sit relative to the **XODR** track edges?” In that frame, the TTL really does sit on or very close to the outer boundary in section 1 (e.g. ~0.1 m away). The graph is correct for XODR.

- **Driving visualization** (dSPACE / ControlDesk): The sim draws the **RD** road — lane edges, center, etc. are defined by the **RD** geometry, not by the XODR boundaries. So “middle of the road” there means “middle of the **RD** lane.”

The transformation aligns XODR and RD (e.g. same centerline / reference), but the **lane width and edge positions** can still differ between the two formats. So:

- In **XODR** (our graph): TTL is 0.1 m from the outer XODR boundary → “on the edge.”
- The **same** TTL, transformed to RD, sits at one physical world position. In the **RD** view, the lane might be drawn wider or with a different reference, so that same position looks “more centered” in the RD lane. So you see “driving in the middle” in RD even though in XODR the path is on the outer boundary.

### Summary

- **Transformation:** Correct. Behavior and tracking are in XODR; placement and readback convert XODR ↔ RD consistently; the car drives correctly in RD.
- **Section 1:** In **XODR**, the TTL genuinely sits near the outer boundary (racing line generator put it there). The graph is correct.
- **RD view:** You’re comparing the car to the **RD** lane. XODR boundaries and RD lane edges are not the same geometry, so “on the outer XODR boundary” can correspond to “looks more centered in the RD lane.” No bug — just two different definitions of where the “edges” and “middle” are.

## 2. Segment 43 and CTE

### Why a "big error" in segment 43 when the car looks in the middle?

**CTE is not "distance to the middle of the road."** It is **distance to the TTL (racing line)**. The MPC minimizes distance to the **reference path** (the TTL), not to the lane center. So: **"Driving in the middle of the road"** = car centered between the two lane edges (what you see). **"Big CTE in segment 43"** = car far from the TTL in that stretch. Those are two different references. In segment 43 the **racing line** may be offset from the lane center (e.g. inside at apex, outside on approach). If the car stays in the visual "middle" but the TTL is 1–2 m to one side, CTE will be large even though the drive looks fine. So the most likely explanation: **in segment 43 the TTL is offset from the lane center**, and the car is following the lane center (or a comfortable line) rather than the TTL.

### Could a “bad” TTL cause high CTE in segment 43?

Yes. The racing line generator does **not** clamp the TTL to the XODR inner/outer boundaries; it only offsets from the centerline (with a max deviation). So in some segments the TTL can:

- Sit very close to one boundary (e.g. outer) and leave little margin.
- Be **offset from the lane center** (by design, for a racing line), so "middle of the road" driving gives large CTE.
- Have locally high curvature or sharp transitions that are hard for the MPC to track.
- Disagree with the actual drivable lane (e.g. if XODR and RD geometry differ in that region).

If in segment 43 the TTL is poor in that sense, the controller will try to follow it and any small error (or delay) can show up as **elevated CTE** there, even if the rest of the lap looks good.

### “Visualization looks fine when driving through”

That can still be consistent with a CTE issue in segment 43:

1. **Brief spike:** CTE might be high for a short time (or a few waypoints) in segment 43; the overall “look” of the drive can still be fine.
2. **Different reference in the sim:** The driving view might be showing the RD reference path or lane center; the MPC might be tracking the XODR TTL. If they differ in segment 43 (e.g. due to transform or geometry), you see “fine” visually but CTE is computed against the XODR TTL and can be high.
3. **Segment 43 is a small part of the lap:** So the majority of the lap can look good while segment 43 still has a local TTL or tracking issue.

So: “visualization looks fine” does **not** rule out a TTL-related CTE problem in segment 43. It’s worth checking TTL quality there (distance to inner/outer boundary, curvature, smoothness, and **offset from lane center**).

## 3. What to do next

1. **Confirm frames:** Treat the graph as “truth in XODR”; treat the driving view as “truth in RD.” They can differ; no need to force them to match unless you align both to the same frame (e.g. transform TTL to RD and plot vs RD boundaries).
2. **Analyze TTL in segment 43:** Use `analyze_ttl_quality.py` (in this folder) to get per-point distance to inner/outer boundary and curvature. Use your run logs to see which waypoint indices correspond to segment 43 (e.g. from “segment 43” log lines and the waypoint index at that time), then inspect those indices in the TTL quality output. Look for:
   - Very small distance to one boundary (TTL almost on the line).
   - High or discontinuous curvature.
3. **Optional:** If segment 43 TTL is bad, consider re-generating the racing line with boundary clamping or with a tighter max offset in that region, or adjust the generator so the TTL stays farther from the boundaries in critical sections.
