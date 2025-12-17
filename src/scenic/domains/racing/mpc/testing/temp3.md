Yes — I pulled these from your **LagunaSeca.xodr** by summing **lane width polynomials for lanes with `type="driving"`** inside each road’s laneSections.

## Road widths (driving lanes only)

### The Corkscrew1

* **Driving width (min / mean / max):** **12.00 m / 13.48 m / 14.00 m**
* **Road length in this XODR:** 2484.58 m

### Andretti Hairpin1_3

* **Driving width (min / mean / max):** **11.00 m / 11.85 m / 12.00 m**
* **Road length in this XODR:** 988.05 m

If you instead include *all* lanes (including `type="none"` borders), the “total paved structure” can be wider in parts — but the numbers above are the cleanest “road drivable width” interpretation.

---

## How to interpret “centerline” in XODR (so it matches RD center-of-road)

In OpenDRIVE:

* The **planView** defines the **reference line** (world `(x,y)`, plus heading).
* Lanes are in the road’s local **Frenet frame** `(s, t)`:

  * `s` goes forward along the reference line
  * `t` is lateral, **positive to the left**, negative to the right
* `laneOffset(s)` shifts where lane 0 (and the lane stack) sits relative to the reference line.

At any `s`, define:

* `L(s)` = sum of widths of all **driving** lanes on the **left** side
* `R(s)` = sum of widths of all **driving** lanes on the **right** side
* `O(s)` = `laneOffset(s)`

Then the **center of the drivable road surface** is at:
[
t_{\text{center}}(s) ;=; O(s) ;+; \frac{L(s)-R(s)}{2}
]

Meaning: to turn the XODR reference line into a “middle of the road” centerline, you laterally shift it by `t_center(s)` along the local normal.

For your two roads (driving lanes only):

* **The Corkscrew1:** `t_center(s)` is **exactly 0** everywhere → its reference line is already centered (after laneOffset is applied).
* **Andretti Hairpin1_3:** `t_center(s)` ranges about **0 → 1 m** (mean ≈ **0.73 m**) → the “true middle” is up to ~1 m **left** of the reference line (relative to the road heading).

---

## Tiny Python function (no utils) to get width + “road-middle” offset from XODR

Copy/paste:

```python
import xml.etree.ElementTree as ET
from bisect import bisect_right

def _poly(a, b, c, d, u):
    return a + b*u + c*u*u + d*u*u*u

def _piecewise_poly_at(records, s):
    # records: list of (s0, a,b,c,d) sorted by s0
    if not records:
        return 0.0
    s0s = [r[0] for r in records]
    i = bisect_right(s0s, s) - 1
    if i < 0:
        i = 0
    s0, a,b,c,d = records[i]
    return _poly(a,b,c,d, s - s0)

def road_width_and_center_t(xodr_path: str, road_name: str, s: float):
    """
    Returns (driving_width_m, t_center_m) at distance s along the road.

    driving_width_m: sum of lane widths where lane@type == "driving" (left + right)
    t_center_m: lateral offset (meters) from reference line to the midpoint of the driving surface
                (+ left, - right), including laneOffset(s).
    """
    root = ET.parse(xodr_path).getroot()

    road = None
    for r in root.findall(".//road"):
        if (r.get("name") or "") == road_name:
            road = r
            break
    if road is None:
        raise ValueError(f'Road "{road_name}" not found.')

    length = float(road.get("length", "0"))
    s = max(0.0, min(length, s))

    # laneOffset(s)
    lane_offsets = []
    for lo in road.findall("./lanes/laneOffset"):
        lane_offsets.append((
            float(lo.get("s", "0")),
            float(lo.get("a", "0")),
            float(lo.get("b", "0")),
            float(lo.get("c", "0")),
            float(lo.get("d", "0")),
        ))
    lane_offsets.sort(key=lambda x: x[0])
    O = _piecewise_poly_at(lane_offsets, s)

    # choose laneSection by s
    sections = [(float(ls.get("s", "0")), ls) for ls in road.findall("./lanes/laneSection")]
    sections.sort(key=lambda x: x[0])
    s_starts = [x[0] for x in sections]
    idx = bisect_right(s_starts, s) - 1
    if idx < 0:
        idx = 0
    s0, ls = sections[idx]
    ds = s - s0

    # lane width at ds using width@a,b,c,d with sOffset
    def lane_width_at(lane_elem, ds):
        wrecs = []
        for w in lane_elem.findall("width"):
            wrecs.append((
                float(w.get("sOffset", "0")),
                float(w.get("a", "0")),
                float(w.get("b", "0")),
                float(w.get("c", "0")),
                float(w.get("d", "0")),
            ))
        wrecs.sort(key=lambda x: x[0])
        if not wrecs:
            return 0.0
        w_s0s = [x[0] for x in wrecs]
        j = bisect_right(w_s0s, ds) - 1
        if j < 0:
            j = 0
        sOff, a,b,c,d = wrecs[j]
        return _poly(a,b,c,d, ds - sOff)

    def side_sum(side_elem):
        if side_elem is None:
            return 0.0
        total = 0.0
        for lane in side_elem.findall("lane"):
            if lane.get("id") == "0":
                continue
            if lane.get("type") != "driving":
                continue
            total += lane_width_at(lane, ds)
        return total

    L = side_sum(ls.find("left"))
    R = side_sum(ls.find("right"))
    driving_width = L + R

    t_center = O + (L - R) / 2.0
    return driving_width, t_center
```

Example usage:

```python
xodr = "LagunaSeca.xodr"
print(road_width_and_center_t(xodr, "The Corkscrew1", 100.0))
print(road_width_and_center_t(xodr, "Andretti Hairpin1_3", 100.0))
```

If you want, I can also extract a **polyline centerline** (world XY samples) for each road by combining `planView` geometry + this `t_center(s)` shift, so you can compare it directly against the RD center lane.
