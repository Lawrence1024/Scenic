#!/usr/bin/env python3
"""
Pitlane TTL utilities:
- Label/copy temp_pitlane_ttl.csv as "begin pitlane" (temp_pitlane_begin_ttl.csv).
- Build "end pitlane" TTL from st_to_xodr_results.txt (temp_pitlane_end_ttl.csv).
- Run merge analysis for both begin and end pitlane TTLs against the existing racing TTL.
"""
import csv
import re
import shutil
from pathlib import Path

# Script lives at src/scenic/simulators/dspace/create_new_ttl/; repo root is 5 levels up
CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = CREATE_NEW_TTL.parent.parent.parent.parent.parent
TEMP_TXT = CREATE_NEW_TTL / "temp.txt"
ST_TO_XODR = CREATE_NEW_TTL / "st_to_xodr_results.txt"
EXISTING_TTL = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_main_road.csv"
TEMP_PITLANE_TTL = CREATE_NEW_TTL / "temp_pitlane_ttl.csv"
OUT_BEGIN_TTL = CREATE_NEW_TTL / "temp_pitlane_begin_ttl.csv"
OUT_END_TTL = CREATE_NEW_TTL / "temp_pitlane_end_ttl.csv"
COMBINED_TTL = CREATE_NEW_TTL / "ttl_pitlane_main_pitlane.csv"
# Closed loop: Pit Lane -> Corkscrew -> Pit Lane (in assets next to main road TTL)
TTLS_FOLDER = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"
PIT_CORKSCREW_LOOP_TTL = TTLS_FOLDER / "ttl_pitlane.csv"


def parse_result_line(line: str):
    """Parse a line like '     900.0 |      0.000 |     -105.337052 |     -478.107182 | ...'
    Return (s, xodr_x, xodr_y, xodr_z) or None.
    """
    line = line.strip()
    if not line or line.startswith("-") or line.startswith("=") or ("s " in line and "t " in line and "XODR" in line):
        return None
    parts = [p.strip() for p in re.split(r"\|", line)]
    if len(parts) < 5:
        return None
    try:
        s = float(parts[0])
        xodr_x = float(parts[2])
        xodr_y = float(parts[3])
        xodr_z = float(parts[4])
        return (s, xodr_x, xodr_y, xodr_z)
    except (ValueError, IndexError):
        return None


def load_temp_txt(path: Path):
    """Load (s, x, y, z) from temp.txt."""
    rows = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            r = parse_result_line(line)
            if r is not None:
                rows.append(r)
    return rows


def load_st_to_xodr(path: Path):
    """Load (s, x, y, z) from st_to_xodr_results.txt."""
    rows = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            r = parse_result_line(line)
            if r is not None:
                rows.append(r)
    return rows


def write_ttl_csv(path: Path, points, comment=None):
    """Write (x,y,z) CSV with header. points: list of (x,y,z) or (s,x,y,z)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        for row in points:
            if len(row) >= 4:
                w.writerow([f"{row[1]:.9f}", f"{row[2]:.9f}", f"{row[3]:.9f}"])
            else:
                w.writerow([f"{row[0]:.9f}", f"{row[1]:.9f}", f"{row[2]:.9f}"])
    print(f"Wrote {len(points)} points to {path}")


def load_existing_ttl(path: Path):
    """Load existing TTL CSV as list of (x,y,z)."""
    points = []
    with path.open("r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if row and not row[0].strip().startswith("#"):
                if row[0].lower() == "x" and len(row) >= 2:
                    continue
                if len(row) >= 2:
                    try:
                        x, y = float(row[0]), float(row[1])
                        z = float(row[2]) if len(row) >= 3 else 0.0
                        points.append((x, y, z))
                    except ValueError:
                        continue
    return points


def load_ttl_csv(path: Path):
    """Load TTL CSV as list of (x,y,z)."""
    points = []
    with path.open("r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if row and not row[0].strip().startswith("#"):
                if row[0].lower() == "x" and len(row) >= 2:
                    continue
                if len(row) >= 2:
                    try:
                        x, y = float(row[0]), float(row[1])
                        z = float(row[2]) if len(row) >= 3 else 0.0
                        points.append((x, y, z))
                    except ValueError:
                        continue
    return points


def dist2(p, q):
    return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2


def find_overlap_ranges(existing_pts, temp_pts, threshold_m=15.0):
    """Existing TTL indices where min distance to temp TTL < threshold_m. Return contiguous ranges."""
    n_ex = len(existing_pts)
    threshold_sq = threshold_m * threshold_m
    overlap_indices = []
    for j in range(n_ex):
        ex_pt = (existing_pts[j][0], existing_pts[j][1])
        d2_min = min(dist2(ex_pt, (t[0], t[1])) for t in temp_pts)
        if d2_min <= threshold_sq:
            overlap_indices.append(j)
    if not overlap_indices:
        return []
    overlap_indices.sort()
    ranges = []
    start = overlap_indices[0]
    end = start
    for j in overlap_indices[1:]:
        if j == end + 1 or (end == n_ex - 1 and j == 0):
            end = j
        else:
            ranges.append((start, end))
            start = j
            end = j
    ranges.append((start, end))
    if len(ranges) > 1 and ranges[-1][1] < ranges[-1][0]:
        ranges = [ranges[-1]] + ranges[:-1]
    return ranges


def nearest_index(pts, target_xy):
    """Return (index, distance_m) of point in pts nearest to target_xy."""
    d2_min, j_min = float("inf"), -1
    for j, p in enumerate(pts):
        d2 = dist2((p[0], p[1]), target_xy)
        if d2 < d2_min:
            d2_min, j_min = d2, j
    return j_min, d2_min ** 0.5


def run_merge_analysis(label: str, temp_pts: list, existing_pts: list):
    """Print merge points and overlap ranges for one pitlane TTL."""
    n_temp = len(temp_pts)
    j_start, d_start = nearest_index(existing_pts, (temp_pts[0][0], temp_pts[0][1]))
    j_end, d_end = nearest_index(existing_pts, (temp_pts[-1][0], temp_pts[-1][1]))
    print(f"\n  Merge points: temp[0]   -> existing index {j_start}  ({d_start:.2f} m)")
    print(f"               temp[{n_temp - 1}] -> existing index {j_end}  ({d_end:.2f} m)")
    ranges = find_overlap_ranges(existing_pts, temp_pts, threshold_m=15.0)
    print(f"  Overlap ranges (existing TTL, within 15 m):")
    if not ranges:
        print("    (none)")
    else:
        for i, (a, b) in enumerate(ranges):
            if a <= b:
                print(f"    Range {i + 1}: index {a} to {b} (inclusive) — {b - a + 1} points")
            else:
                print(f"    Range {i + 1}: index {a} to {b} (wrapped) — {len(existing_pts) - a + b + 1} points")


def blend_segment(end_pts: list, start_pts: list, n_blend: int) -> list:
    """
    Cross-fade between the last n_blend points of end_pts and the first n_blend points of start_pts.
    Returns n_blend interpolated points (x,y,z). Uses linear interpolation in (x,y,z).
    """
    if n_blend <= 0 or len(end_pts) < n_blend or len(start_pts) < n_blend:
        return []
    out = []
    for k in range(n_blend):
        t = (k + 1) / (n_blend + 1)
        a = end_pts[-n_blend + k]
        b = start_pts[k]
        pt = (
            (1 - t) * a[0] + t * b[0],
            (1 - t) * a[1] + t * b[1],
            (1 - t) * a[2] + t * b[2],
        )
        out.append(pt)
    return out


def build_combined_ttl(begin_pts: list, existing_pts: list, out_path: Path, blend_n: int = 6) -> bool:
    """
    Build TTL: start of pitlane -> main loop (existing TTL) -> back into pitlane.
    At each junction, blend over blend_n points to smooth the transition.
    """
    if not begin_pts or not existing_pts:
        return False
    n_ex = len(existing_pts)
    j_pit_entry, d_pe = nearest_index(existing_pts, (begin_pts[0][0], begin_pts[0][1]))
    j_pit_exit, d_px = nearest_index(existing_pts, (begin_pts[-1][0], begin_pts[-1][1]))
    # Main segment: from (pit exit + 1) to pit entry (inclusive)
    i_start = (j_pit_exit + 1) % n_ex
    i_end = j_pit_entry
    if i_start <= i_end:
        main_slice = list(range(i_start, i_end + 1))
    else:
        main_slice = list(range(i_start, n_ex)) + list(range(0, i_end + 1))
    n_main = len(main_slice)
    if n_main < blend_n or len(begin_pts) < blend_n:
        blend_n = min(3, n_main, len(begin_pts))
    if blend_n <= 0:
        blend_n = 0

    combined = []
    main_pts = [existing_pts[i] for i in main_slice]

    # 1) Pitlane -> main: pitlane (trim last blend_n) + blend + main (trim first blend_n)
    combined.extend(begin_pts[:-blend_n] if blend_n else begin_pts)
    if blend_n:
        blend1 = blend_segment(begin_pts, main_pts, blend_n)
        combined.extend(blend1)
    combined.extend(main_pts[blend_n:])  # full main segment from rejoin to pit entry

    # 2) Main -> pitlane: use pitlane path only (no blend, no main road)
    combined.extend(begin_pts)

    write_ttl_csv(out_path, combined)
    print(f"\n[Combined TTL] Pit entry existing index {j_pit_entry} ({d_pe:.2f} m), pit exit {j_pit_exit} ({d_px:.2f} m)")
    print(f"  Main segment: existing indices {i_start}..{i_end} ({len(main_slice)} points)")
    print(f"  Blending: {blend_n} points at pitlane->main only; main->pitlane uses pitlane path only")
    print(f"  Total: {len(combined)} points -> {out_path.name}")
    return True


def build_pitlane_corkscrew_loop(
    pitlane_pts: list, existing_pts: list, out_path: Path, blend_n: int = 6
) -> bool:
    """
    Closed loop: Pit Lane -> Corkscrew -> Pit Lane.
    Merge pitlane into existing TTL at overlap (rejoin), follow existing to pit entry (overlap),
    then smooth back to loop start (pitlane start). No coordinate transform; uses overlap only.
    """
    if not pitlane_pts or not existing_pts:
        return False
    n_ex = len(existing_pts)
    j_pit_entry, d_pe = nearest_index(existing_pts, (pitlane_pts[0][0], pitlane_pts[0][1]))
    j_pit_exit, d_px = nearest_index(existing_pts, (pitlane_pts[-1][0], pitlane_pts[-1][1]))
    i_start = (j_pit_exit + 1) % n_ex
    i_end = j_pit_entry
    if i_start <= i_end:
        main_slice = list(range(i_start, i_end + 1))
    else:
        main_slice = list(range(i_start, n_ex)) + list(range(0, i_end + 1))
    main_pts = [existing_pts[i] for i in main_slice]
    n_main = len(main_pts)
    blend_n = min(blend_n, len(pitlane_pts) - 1, n_main - 1) if blend_n else 0
    if blend_n < 0:
        blend_n = 0

    loop = []
    # 1) Pit Lane (trim last blend_n) + blend onto main at rejoin
    loop.extend(pitlane_pts[:-blend_n] if blend_n else pitlane_pts)
    if blend_n:
        loop.extend(blend_segment(pitlane_pts, main_pts, blend_n))
    # 2) Main (Corkscrew segment) from rejoin to pit entry
    loop.extend(main_pts[blend_n:])
    # 3) Close loop: main end is near pitlane start. Smooth the curve if gap is noticeable.
    dist_close = dist2((main_pts[-1][0], main_pts[-1][1]), (pitlane_pts[0][0], pitlane_pts[0][1])) ** 0.5
    if dist_close > 2.0 and blend_n >= 2:
        close_n = min(4, blend_n)
        close_blend = blend_segment(main_pts, pitlane_pts, close_n)
        if close_blend:
            loop = loop[:-close_n] + close_blend
    write_ttl_csv(out_path, loop)
    print(f"\n[Pit Lane -> Corkscrew -> Pit Lane] Closed loop: pit entry idx {j_pit_entry} ({d_pe:.2f} m), pit exit {j_pit_exit} ({d_px:.2f} m)")
    print(f"  Main segment: existing indices {i_start}..{i_end} ({len(main_slice)} points)")
    print(f"  Blending: {blend_n} pts at pit->main; close gap {dist_close:.2f} m")
    print(f"  Total: {len(loop)} points -> {out_path.name}")
    return True


def main():
    print("=" * 60)
    print("Pitlane TTL: begin + end + merge analysis")
    print("=" * 60)

    # 1. Label temp_pitlane_ttl.csv as "begin pitlane"
    if TEMP_PITLANE_TTL.exists():
        shutil.copy2(TEMP_PITLANE_TTL, OUT_BEGIN_TTL)
        print(f"\n[Begin pitlane] Copied {TEMP_PITLANE_TTL.name} -> {OUT_BEGIN_TTL.name} (labeled as begin pitlane)")
    else:
        print(f"\n[Begin pitlane] {TEMP_PITLANE_TTL.name} not found; skip. Run combine_and_compare_ttl once with temp.txt + st_to_xodr 0-959 to create it.")

    # 2. Build "end pitlane" TTL from current st_to_xodr_results.txt
    print(f"\n[End pitlane] Loading {ST_TO_XODR.name}...")
    end_rows = load_st_to_xodr(ST_TO_XODR)
    if not end_rows:
        print(f"  No data in {ST_TO_XODR.name}; cannot create end pitlane TTL.")
    else:
        s_min, s_max = min(r[0] for r in end_rows), max(r[0] for r in end_rows)
        print(f"  Got {len(end_rows)} rows; s range {s_min:.0f} .. {s_max:.0f}")
        end_pts = [(r[1], r[2], r[3]) for r in sorted(end_rows, key=lambda x: x[0])]
        write_ttl_csv(OUT_END_TTL, end_pts)

    # 3. Load existing racing TTL
    print(f"\n[Existing TTL] Loading {EXISTING_TTL.relative_to(REPO_ROOT)}...")
    existing_pts = load_existing_ttl(EXISTING_TTL)
    print(f"  {len(existing_pts)} points")

    # 4. Merge analysis for begin pitlane
    begin_pts = None
    print("\n--- Begin pitlane: merge analysis ---")
    if OUT_BEGIN_TTL.exists():
        begin_pts = load_ttl_csv(OUT_BEGIN_TTL)
        print(f"  Begin pitlane TTL: {len(begin_pts)} points ({OUT_BEGIN_TTL.name})")
        run_merge_analysis("begin", begin_pts, existing_pts)
    else:
        print("  (begin TTL file not found)")

    # 5. Build combined TTL (pitlane -> main -> pitlane)
    if begin_pts is not None:
        build_combined_ttl(begin_pts, existing_pts, COMBINED_TTL)
        # 6. Build closed loop: Pit Lane -> Corkscrew -> Pit Lane (merge via overlap)
        build_pitlane_corkscrew_loop(begin_pts, existing_pts, PIT_CORKSCREW_LOOP_TTL)

    # 7. Merge analysis for end pitlane
    print("\n--- End pitlane: merge analysis ---")
    if end_rows:
        print(f"  End pitlane TTL: {len(end_pts)} points ({OUT_END_TTL.name})")
        run_merge_analysis("end", end_pts, existing_pts)
    else:
        print("  (end TTL not built)")

    print("\nDone.")


if __name__ == "__main__":
    main()
