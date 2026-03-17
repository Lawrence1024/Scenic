"""Evaluate the ModelDesk placement pipeline: (x,y) -> (s,t).

Replicates the same process used at runtime when placing vehicles in ModelDesk:
1. Build the road index from the TTL folder (ttl_main_road.csv + ttl_pitlane.csv),
   same as placement does when ttlFolder is set.
2. Project coordinates via project_world_to_st_route_specific with route Lap or Pit.

Feed in coordinates from ttl_optimal_xodr.csv (Lap) and ttl_pit_xodr.csv (Pit) to
see the (s, t) values the pipeline would produce. Use this to evaluate the pipeline
without running ModelDesk.

Usage (from repo root):
  python -m src.scenic.domains.racing.ttl_processing.test_ttl_st_values
  python -m src.scenic.domains.racing.ttl_processing.test_ttl_st_values --sanity-check
  python -m src.scenic.domains.racing.ttl_processing.test_ttl_st_values --sample 100
  python -m src.scenic.domains.racing.ttl_processing.test_ttl_st_values --ttl-folder path/to/ttl_folder
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Ensure repo src is on path when run as script
_REPO_ROOT = Path(__file__).resolve().parents[5]
_SRC = _REPO_ROOT / "src"
if _SRC.exists() and str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from scenic.simulators.dspace.geometry.projection import project_world_to_st
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.ttl.road_index import build_road_index_from_ttl


DEFAULT_TTL_FOLDER = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"
DEFAULT_OPTIMAL = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_optimal_xodr.csv"
DEFAULT_PIT = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_pit_xodr.csv"


def load_ttl_csv(path: Path) -> List[Tuple[float, float, float]]:
    """Load TTL CSV with header x,y,z; return list of (x, y, z)."""
    pts: List[Tuple[float, float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        first = next(r, None)
        if first and len(first) >= 2 and first[0].strip().lower() == "x":
            pass  # skip header
        elif first and len(first) >= 2:
            try:
                x, y = float(first[0]), float(first[1])
                z = float(first[2]) if len(first) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                pass
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x, y = float(row[0]), float(row[1])
                z = float(row[2]) if len(row) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                continue
    return pts


def sanity_check_t(road_index: Dict[str, Any], sec_points: List[Tuple[float, float, float]], offset_m: float = 2.0) -> None:
    """Prove that t is computed correctly: project a point offset from the centerline; expect t ≈ ±offset_m."""
    if not sec_points or len(sec_points) < 2:
        return
    # Use first segment; left normal = (-vy/len, vx/len)
    x0, y0, s0 = sec_points[0]
    x1, y1, s1 = sec_points[1]
    vx, vy = x1 - x0, y1 - y0
    seg_len = math.hypot(vx, vy)
    if seg_len < 1e-9:
        return
    nx_left = -vy / seg_len
    ny_left = vx / seg_len
    # Point 2m to the left of (x0,y0)
    px = x0 + offset_m * nx_left
    py = y0 + offset_m * ny_left
    s_val, t_val = project_world_to_st(road_index, (px, py))
    print(f"  Sanity check: point {offset_m} m left of first vertex -> s={s_val:.4f}, t={t_val:.4f} (expect t ~ +{offset_m})")
    # Point 2m to the right (negative normal)
    px_right = x0 - offset_m * nx_left
    py_right = y0 - offset_m * ny_left
    s_r, t_r = project_world_to_st(road_index, (px_right, py_right))
    print(f"  Sanity check: point {offset_m} m right of first vertex -> s={s_r:.4f}, t={t_r:.4f} (expect t ~ -{offset_m})")


def run(
    ttl_folder: Path,
    optimal_path: Path,
    pit_path: Path,
    sample_step: int = 1,
    output_csv: Optional[Path] = None,
    sanity_check: bool = False,
) -> None:
    """Replicate the placement pipeline: build TTL road index, project waypoints with route Lap/Pit."""
    # Same as placement when ttlFolder is set
    road_index = build_road_index_from_ttl(str(ttl_folder))
    if not road_index:
        print("[ERROR] Could not build road index from TTL folder (need ttl_main_road.csv and ttl_pitlane.csv).")
        return
    print(f"[Pipeline] Using TTL folder: {ttl_folder}")

    if sanity_check:
        main_sec = road_index.get("roads", {}).get("MainTrack_TTL", {}).get("sec_points", [[]])[0]
        if main_sec:
            sanity_check_t(road_index, main_sec, offset_m=2.0)

    results: List[Dict[str, Any]] = []

    for label, path, route_pref in [
        ("optimal (Lap/R2)", optimal_path, "Lap"),
        ("pit (Pit/R1)", pit_path, "Pit"),
    ]:
        if not path.exists():
            print(f"\n[{label}] File not found: {path}")
            continue
        pts = load_ttl_csv(path)
        if len(pts) < 2:
            print(f"\n[{label}] Not enough points in {path}")
            continue
        print(f"\n--- {path.name} (route {route_pref}) ---")
        print(f"  Points: {len(pts)}")

        for i in range(0, len(pts), sample_step):
            x, y = float(pts[i][0]), float(pts[i][1])
            s_val, t_val = project_world_to_st_route_specific(
                road_index, (x, y), route_preference=route_pref
            )
            results.append(
                {
                    "file": path.name,
                    "route": route_pref,
                    "index": i,
                    "x": x,
                    "y": y,
                    "s": s_val,
                    "t": t_val,
                }
            )

        sample_indices = [0, len(pts) - 1]
        if len(pts) > 2:
            sample_indices = [0, len(pts) // 2, len(pts) - 1]
        print("  Sample (index, x, y, s, t):")
        for idx in sample_indices:
            x, y = float(pts[idx][0]), float(pts[idx][1])
            s_val, t_val = project_world_to_st_route_specific(
                road_index, (x, y), route_preference=route_pref
            )
            print(f"    [{idx}] x={x:.4f}, y={y:.4f} -> s={s_val:.4f}, t={t_val:.6f}")

    if output_csv and results:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        with open(output_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(
                f,
                fieldnames=["file", "route", "index", "x", "y", "s", "t"],
            )
            w.writeheader()
            w.writerows(results)
        print(f"\nWrote full (s,t) table to {output_csv}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate the placement pipeline: (x,y) -> (s,t) using same TTL index and route-specific projection."
    )
    parser.add_argument(
        "--ttl-folder",
        type=Path,
        default=DEFAULT_TTL_FOLDER,
        help="TTL folder (must contain ttl_main_road.csv and ttl_pitlane.csv); same as scenario ttlFolder",
    )
    parser.add_argument(
        "--optimal",
        type=Path,
        default=DEFAULT_OPTIMAL,
        help="CSV of (x,y) waypoints to project as Lap route",
    )
    parser.add_argument(
        "--pit",
        type=Path,
        default=DEFAULT_PIT,
        help="CSV of (x,y) waypoints to project as Pit route",
    )
    parser.add_argument(
        "--sample",
        type=int,
        default=1,
        metavar="N",
        help="Report every Nth point (default 1 = all)",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=None,
        help="Write full (index,x,y,s,t) table to this CSV",
    )
    parser.add_argument(
        "--sanity-check",
        action="store_true",
        help="Project offset points (2m left/right) to verify t is computed correctly (expect t ~ +/-2)",
    )
    args = parser.parse_args()
    run(
        ttl_folder=args.ttl_folder,
        optimal_path=args.optimal,
        pit_path=args.pit,
        sample_step=args.sample,
        output_csv=args.output_csv,
        sanity_check=args.sanity_check,
    )


if __name__ == "__main__":
    main()
