import os
import sys
import math
import csv
import statistics
import time
from typing import List, Tuple, Dict

# Reuse the same geometry code path as the dSPACE simulator
from scenic.simulators.dspace.geometry.xodr_parser import build_xodr_sec_points


def read_ttl_xy(csv_path: str, max_rows: int = None) -> List[Tuple[float, float]]:
    """Read ENU x,y from TTL CSV. Skips the first metadata line."""
    points: List[Tuple[float, float]] = []
    with open(csv_path, newline="") as f:
        reader = csv.reader(f)
        # Skip first line (metadata)
        try:
            next(reader)
        except StopIteration:
            return points
        for i, row in enumerate(reader):
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                points.append((x, y))
            except Exception:
                # Skip malformed lines; print only first few issues
                if i < 5:
                    print(f"[WARN] Malformed row at {i+2}: {row}")
            if max_rows is not None and len(points) >= max_rows:
                break
    return points


def _all_road_segments(road_index: Dict) -> List[List[Tuple[float, float, float]]]:
    roads_obj = road_index.get("roads", {})
    segments = []
    for road_name, road_data in roads_obj.items():
        sec_list = road_data.get("sec_points", [])
        for pts in sec_list:
            if pts and len(pts) >= 2:
                segments.append(pts)
    return segments


def project_with_distance(road_index: Dict, pos: Tuple[float, float]) -> Tuple[float, float, float]:
    """Project (x,y) to nearest road ref segment using same approach as projection.py, but return distance too.
    Returns (s, t, euclidean_distance)."""
    px, py = float(pos[0]), float(pos[1])
    segments = _all_road_segments(road_index)
    if not segments:
        return 0.0, 0.0, float("inf")
    best = None  # (dist2, s_proj, t_signed)
    for pts in segments:
        for i in range(len(pts) - 1):
            x0, y0, s0 = pts[i]
            x1, y1, s1 = pts[i + 1]
            vx, vy = x1 - x0, y1 - y0
            seg_len2 = vx * vx + vy * vy
            if seg_len2 <= 1e-12:
                continue
            wx, wy = px - x0, py - y0
            u = (wx * vx + wy * vy) / seg_len2
            u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
            qx = x0 + u * vx
            qy = y0 + u * vy
            dx, dy = px - qx, py - qy
            dist2 = dx * dx + dy * dy
            seg_len = math.sqrt(seg_len2)
            # left normal
            nx_left, ny_left = (-vy / seg_len, vx / seg_len)
            raw_t = dx * nx_left + dy * ny_left
            t_signed = raw_t * 0.3  # use same scaling as projection.py
            s_proj = s0 + u * (s1 - s0)
            if best is None or dist2 < best[0]:
                best = (dist2, s_proj, t_signed)
    if best is None:
        return 0.0, 0.0, float("inf")
    return float(best[1]), float(best[2]), math.sqrt(best[0])


def centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    n = len(points)
    return sx / n, sy / n


def transform_points(points: List[Tuple[float, float]], transform: Dict) -> List[Tuple[float, float]]:
    """Apply a 2D similarity transform (rotation, optional mirror, translation).
    Transform dict fields:
      - rot_deg: rotation angle in degrees, applied around origin
      - mirror_x: bool, if True flip x after rotation
      - mirror_y: bool, if True flip y after rotation
      - translate: (dx, dy) tuple
    """
    rot_deg = float(transform.get("rot_deg", 0.0))
    mirror_x = bool(transform.get("mirror_x", False))
    mirror_y = bool(transform.get("mirror_y", False))
    dx, dy = transform.get("translate", (0.0, 0.0))
    th = math.radians(rot_deg)
    c, s = math.cos(th), math.sin(th)
    out: List[Tuple[float, float]] = []
    for (x, y) in points:
        xr = c * x - s * y
        yr = s * x + c * y
        if mirror_x:
            xr = -xr
        if mirror_y:
            yr = -yr
        out.append((xr + dx, yr + dy))
    return out


def evaluate_alignment(road_index: Dict, ttl_points: List[Tuple[float, float]], label: str, max_eval: int = 1000) -> Dict:
    """Project transformed TTL points onto road and compute distance stats."""
    dists: List[float] = []
    step = max(1, len(ttl_points) // max_eval)
    for i in range(0, len(ttl_points), step):
        s, t, d = project_with_distance(road_index, ttl_points[i])
        dists.append(d)
    if not dists:
        return {"label": label, "count": 0}
    dists_sorted = sorted(dists)
    pct_2m = 100.0 * sum(1 for d in dists if d <= 2.0) / len(dists)
    pct_5m = 100.0 * sum(1 for d in dists if d <= 5.0) / len(dists)
    summary = {
        "label": label,
        "count": len(dists),
        "mean": statistics.mean(dists),
        "median": statistics.median(dists),
        "p90": dists_sorted[int(0.90 * (len(dists_sorted) - 1))],
        "p95": dists_sorted[int(0.95 * (len(dists_sorted) - 1))],
        "p99": dists_sorted[int(0.99 * (len(dists_sorted) - 1))],
        "pct_le_2m": pct_2m,
        "pct_le_5m": pct_5m,
    }
    return summary


def main():
    # Inputs (adjust paths if needed)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    default_xodr = os.path.join(repo_root, "assets", "maps", "dSPACE", "LagunaSeca.xodr")
    ttl_dir = os.path.join(repo_root, "assets", "ttls", "LS_ENU_TTL_CSV")
    ttl_candidates = [
        "ttl_3g_v1r.csv",
        "ttl_9g_l1p35_r1p35_v1r.csv",
        "ttl_16_l1p25_r1p25_v1r.csv",
        "ttl27_v5.csv",
    ]
    # Allow narrowing to a single file via env
    only = os.environ.get("TTL_ONLY")
    if only:
        ttl_candidates = [only]

    xodr_path = sys.argv[1] if len(sys.argv) > 1 else default_xodr
    if not os.path.exists(xodr_path):
        print(f"[ERROR] XODR not found: {xodr_path}")
        sys.exit(1)
    print(f"[INFO] Using XODR: {xodr_path}")

    # Build road index from XODR
    print("[STEP] Building road index from XODR (independent roads)...")
    road_index = build_xodr_sec_points(xodr_path, step=2.0)
    seg_count = sum(len(v.get("sec_points", [])) for v in road_index.get("roads", {}).values())
    pt_count = sum(len(pts) for v in road_index.get("roads", {}).values() for pts in v.get("sec_points", []))
    print(f"[INFO] Road index: roads={len(road_index.get('roads', {}))}, segments={seg_count}, points={pt_count}")

    # Prepare transforms to try (rotation/mirror + centroid alignment)
    base_roads_points = []
    for v in road_index.get("roads", {}).values():
        for pts in v.get("sec_points", []):
            base_roads_points.extend((x, y) for (x, y, s) in pts)
    road_cx, road_cy = centroid(base_roads_points) if base_roads_points else (0.0, 0.0)
    print(f"[INFO] Road centroid approx: ({road_cx:.3f}, {road_cy:.3f})")

    candidate_rots = [0.0, 90.0, -90.0, 180.0]
    candidate_mirrors = [(False, False), (True, False), (False, True)]

    for csv_name in ttl_candidates:
        ttl_start = time.time()
        quick_mode = bool(os.environ.get("TTL_QUICK"))
        max_seconds = float(os.environ.get("TTL_MAX_SECONDS", "20" if quick_mode else "120"))
        csv_path = os.path.join(ttl_dir, csv_name)
        if not os.path.exists(csv_path):
            print(f"[SKIP] Missing TTL: {csv_path}")
            continue
        print(f"\n[TTL] Evaluating: {csv_name}")
        ttl_pts = read_ttl_xy(csv_path)
        print(f"[INFO] Loaded TTL points: {len(ttl_pts)}")
        if not ttl_pts:
            continue
        ttl_cx, ttl_cy = centroid(ttl_pts)
        print(f"[INFO] TTL centroid: ({ttl_cx:.3f}, {ttl_cy:.3f})")

        best = None  # (score_median, summary, transform)
        tried = 0
        for rot in candidate_rots:
            for mx, my in candidate_mirrors:
                if (time.time() - ttl_start) > max_seconds:
                    print(f"[TIMEOUT] Base sweep exceeded {max_seconds:.1f}s for {csv_name}. Stopping base sweep.")
                    break
                # First, rotate/mirror around origin, then translate to align centroids
                rough = transform_points(ttl_pts, {"rot_deg": rot, "mirror_x": mx, "mirror_y": my, "translate": (0.0, 0.0)})
                rcx, rcy = centroid(rough)
                dx, dy = (road_cx - rcx, road_cy - rcy)
                transformed = transform_points(ttl_pts, {"rot_deg": rot, "mirror_x": mx, "mirror_y": my, "translate": (dx, dy)})
                summary = evaluate_alignment(road_index, transformed, label=f"rot={rot}, mx={mx}, my={my}, centroid-align")
                tried += 1

                print(f"[TRY] {summary['label']}: n={summary.get('count',0)}, "
                      f"mean={summary.get('mean',float('nan')):.2f} m, "
                      f"median={summary.get('median',float('nan')):.2f} m, "
                      f"p90={summary.get('p90',float('nan')):.2f} m, p95={summary.get('p95',float('nan')):.2f} m, "
                      f"<=2m={summary.get('pct_le_2m',0.0):.1f}% <=5m={summary.get('pct_le_5m',0.0):.1f}%")
                sys.stdout.flush()

                score = summary.get("median", float("inf"))
                if best is None or score < best[0]:
                    best = (score, summary, {"rot_deg": rot, "mirror_x": mx, "mirror_y": my, "translate": (dx, dy)})
            if (time.time() - ttl_start) > max_seconds:
                break

        if best:
            print(f"[BEST] {csv_name}: {best[1]} | transform={best[2]}")

            # Refinement sweep around best rotation and local translation
            b_rot = best[2]["rot_deg"]
            b_mx = best[2]["mirror_x"]
            b_my = best[2]["mirror_y"]
            # Start with centroid alignment translation as base
            rough = transform_points(ttl_pts, {"rot_deg": b_rot, "mirror_x": b_mx, "mirror_y": b_my, "translate": (0.0, 0.0)})
            rcx, rcy = centroid(rough)
            base_dx, base_dy = (road_cx - rcx, road_cy - rcy)
            print(f"[REFINE] Base centroid dx,dy=({base_dx:.3f},{base_dy:.3f}) around rot={b_rot}, mx={b_mx}, my={b_my}")

            refine_best = None
            quick = bool(os.environ.get("TTL_QUICK"))
            # Rotation sweep
            if quick:
                rot_step = 5.0
                rot_min = b_rot - 10.0
                rot_max = b_rot + 10.0
            else:
                rot_step = 1.0
                rot_min = b_rot - 15.0
                rot_max = b_rot + 15.0
            rot_values = []
            rv = rot_min
            while rv <= rot_max + 1e-6:
                rot_values.append(round(rv, 6))
                rv += rot_step

            # Translation sweep
            if quick:
                trans_range = range(-30, 31, 15)
            else:
                trans_range = range(-50, 51, 10)

            for rot in rot_values:
                if (time.time() - ttl_start) > max_seconds:
                    print(f"[TIMEOUT] Refinement exceeded {max_seconds:.1f}s for {csv_name}. Aborting refinement.")
                    break
                # Recompute centroid alignment per rotation
                rough = transform_points(ttl_pts, {"rot_deg": rot, "mirror_x": b_mx, "mirror_y": b_my, "translate": (0.0, 0.0)})
                rcx2, rcy2 = centroid(rough)
                dx0, dy0 = (road_cx - rcx2, road_cy - rcy2)
                for dx_add in trans_range:
                    for dy_add in trans_range:
                        if (time.time() - ttl_start) > max_seconds:
                            break
                        dx = dx0 + dx_add
                        dy = dy0 + dy_add
                        transformed = transform_points(ttl_pts, {"rot_deg": rot, "mirror_x": b_mx, "mirror_y": b_my, "translate": (dx, dy)})
                        summary = evaluate_alignment(road_index, transformed, label=f"refine rot={rot}, dx={dx}, dy={dy}")
                        score = summary.get("median", float("inf"))
                        if refine_best is None or score < refine_best[0]:
                            refine_best = (score, summary, {"rot_deg": rot, "mirror_x": b_mx, "mirror_y": b_my, "translate": (dx, dy)})
                    if (time.time() - ttl_start) > max_seconds:
                        break

            if refine_best:
                print(f"[REFINE BEST] {csv_name}: {refine_best[1]} | transform={refine_best[2]}")
        print(f"[DONE] Tried {tried} transforms for {csv_name}")
        elapsed = time.time() - ttl_start
        print(f"[TTL] Elapsed {elapsed:.1f}s for {csv_name} (limit {max_seconds:.1f}s)")


if __name__ == "__main__":
    main()


