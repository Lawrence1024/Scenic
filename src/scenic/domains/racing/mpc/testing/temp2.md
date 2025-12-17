Here’s a **small, copy-pasteable** function that:

1. takes a **reference polyline** (e.g., from your `.rd` sampling),
2. reads lane widths from the **`.xodr`** for a given road,
3. computes the **pavement midline** by offsetting the reference line by
   [
   \text{offset}(s)=\frac{W_\text{left}(s)-W_\text{right}(s)}{2}
   ]
   (using OpenDRIVE conventions: **+t is left**, lane IDs **>0 left**, **<0 right**).

```python
import math
import xml.etree.ElementTree as ET

def make_midline_from_xodr(reference_xy, xodr_path, road_name, lane_filter=None):
    """
    reference_xy: [(x,y), ...] sampled along the road reference line (e.g. from .rd)
    xodr_path: path to .xodr
    road_name: <road name="..."> to read lane widths from
    lane_filter: optional function(lane_id:int, lane_type:str)->bool to include/exclude lanes
                 default: include driving + shoulder + restricted + entry/exit/parking/border as long as width exists
    returns: [(x_mid, y_mid), ...] midline points in world coords
    """

    # ---------- helpers ----------
    def _attr(node, k, default=None):
        v = node.get(k)
        return default if v is None else v

    def _find_road(root):
        for r in root.findall(".//road"):
            if _attr(r, "name", "") == road_name:
                return r
        raise ValueError(f"road name not found in xodr: {road_name}")

    def _lane_ok(lane_id, lane_type):
        if lane_filter is not None:
            return bool(lane_filter(lane_id, lane_type))
        # Reasonable default: keep most lanes that might contribute to paved width
        return lane_type in {
            "driving", "shoulder", "restricted", "entry", "exit", "onRamp", "offRamp",
            "parking", "border", "biking"
        }

    def _width_at_s_in_lane(lane_node, s_rel):
        """Pick the last <width sOffset<=s_rel> and evaluate cubic a+b*ds+c*ds^2+d*ds^3."""
        widths = lane_node.findall("./width")
        if not widths:
            return 0.0
        # choose record with max sOffset <= s_rel
        best = None
        best_s0 = -1e18
        for w in widths:
            s0 = float(_attr(w, "sOffset", "0.0"))
            if s0 <= s_rel and s0 >= best_s0:
                best = w
                best_s0 = s0
        if best is None:
            # if all sOffset > s_rel, take the first
            best = widths[0]
            best_s0 = float(_attr(best, "sOffset", "0.0"))

        ds = max(0.0, s_rel - best_s0)
        a = float(_attr(best, "a", "0.0"))
        b = float(_attr(best, "b", "0.0"))
        c = float(_attr(best, "c", "0.0"))
        d = float(_attr(best, "d", "0.0"))
        wv = a + b*ds + c*ds*ds + d*ds*ds*ds
        return max(0.0, wv)

    def _build_lane_sections(road):
        lane_sections = []
        lanes_parent = road.find("./lanes")
        if lanes_parent is None:
            return lane_sections
        for ls in lanes_parent.findall("./laneSection"):
            s0 = float(_attr(ls, "s", "0.0"))
            lane_sections.append((s0, ls))
        lane_sections.sort(key=lambda x: x[0])
        return lane_sections

    def _widths_at_s(lane_sections, s):
        """Return (W_left, W_right) at road-s by summing lane widths on each side."""
        if not lane_sections:
            return (0.0, 0.0)

        # choose active laneSection: last with s0 <= s
        idx = 0
        for i in range(len(lane_sections)):
            if lane_sections[i][0] <= s:
                idx = i
            else:
                break
        s0, ls = lane_sections[idx]
        s_rel = max(0.0, s - s0)

        W_left = 0.0
        W_right = 0.0

        # left side lanes (positive ids)
        left = ls.find("./left")
        if left is not None:
            for lane in left.findall("./lane"):
                lane_id = int(_attr(lane, "id", "0"))
                lane_type = _attr(lane, "type", "")
                if lane_id > 0 and _lane_ok(lane_id, lane_type):
                    W_left += _width_at_s_in_lane(lane, s_rel)

        # right side lanes (negative ids)
        right = ls.find("./right")
        if right is not None:
            for lane in right.findall("./lane"):
                lane_id = int(_attr(lane, "id", "0"))
                lane_type = _attr(lane, "type", "")
                if lane_id < 0 and _lane_ok(lane_id, lane_type):
                    W_right += _width_at_s_in_lane(lane, s_rel)

        return (W_left, W_right)

    def _arc_lengths(xy):
        """cumulative s along polyline"""
        s = [0.0]
        for i in range(1, len(xy)):
            dx = xy[i][0] - xy[i-1][0]
            dy = xy[i][1] - xy[i-1][1]
            s.append(s[-1] + math.hypot(dx, dy))
        return s

    def _headings(xy):
        """heading along polyline (radians). last point uses previous segment."""
        if len(xy) < 2:
            return [0.0]*len(xy)
        hdg = []
        for i in range(len(xy)):
            if i < len(xy) - 1:
                dx = xy[i+1][0] - xy[i][0]
                dy = xy[i+1][1] - xy[i][1]
            else:
                dx = xy[i][0] - xy[i-1][0]
                dy = xy[i][1] - xy[i-1][1]
            hdg.append(math.atan2(dy, dx))
        return hdg

    # ---------- parse xodr widths ----------
    root = ET.parse(xodr_path).getroot()
    road = _find_road(root)
    lane_sections = _build_lane_sections(road)

    # ---------- offset reference line to pavement midline ----------
    s_list = _arc_lengths(reference_xy)
    hdg_list = _headings(reference_xy)

    mid = []
    for (x, y), s, hdg in zip(reference_xy, s_list, hdg_list):
        W_left, W_right = _widths_at_s(lane_sections, s)
        offset = 0.5 * (W_left - W_right)  # + is left in OpenDRIVE
        nx = -math.sin(hdg)  # unit normal pointing left
        ny =  math.cos(hdg)
        mid.append((x + offset*nx, y + offset*ny))

    return mid
```

### How to use it (with your `.rd` sampler)

Assuming you already have your `.rd` reference line points:

```python
rd_ref = sample_rd_reference_line("Laguna_Seca.rd", "The Corkscrew1", ds=0.5)
midline = make_midline_from_xodr(rd_ref, "LagunaSeca.xodr", "The Corkscrew1")
```

### Notes (so you interpret it correctly)

* This **does not change coordinate frames**. It assumes your `.rd` sampled points are already in the same world frame as XODR (which is true for your files).
* The function computes the **midline of total lane widths** (left + right). If a road has lanes only on one side (common in your xodr), this will shift the reference line toward the interior by about half that side’s width.
* If you want to **only count `type="driving"` lanes** (exclude shoulders, etc.), pass a `lane_filter`.
