#!/usr/bin/env python3
"""
Generate a pitlane TTL that runs in parallel with the existing racing TTL.

High‑level behavior:
- Load the existing racing TTL (main loop) in XODR coordinates.
- Build the racing track from the full LagunaSeca.xodr map.
- Extract a continuous polyline for the pit path:
  main→pit connector, pit lane, pit→main connector.
- Find the closest points on the racing TTL to the pit entry/exit junctions.
- Replace the arc of the racing TTL between entry and exit with the pit path,
  resampled to the same number of points along that arc (preserving loop length
  and sampling density).
- Save the result as a new TTL CSV for the pitlane.

Default IO:
- Input XODR: assets/maps/dSPACE/LagunaSeca.xodr
- Input main TTL: assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv
- Output pit TTL (tooling copy): create_new_ttl/ttl_pitlane_xodr.csv
- Output pit TTL (runtime asset): assets/ttls/LS_ENU_TTL_CSV/ttl_pitlane_xodr.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np

# Add Scenic src to path so we can import the racing domain
REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(SRC_ROOT))

from scenic.domains.racing.tracks import createRacingTrack  # type: ignore
from scenic.domains.racing.segments.segment_map import (  # type: ignore
    _get_road_centerline,
)


@dataclass
class Polyline:
    points: List[Tuple[float, float]]

    @property
    def length(self) -> float:
        if len(self.points) < 2:
            return 0.0
        acc = 0.0
        for i in range(1, len(self.points)):
            dx = self.points[i][0] - self.points[i - 1][0]
            dy = self.points[i][1] - self.points[i - 1][1]
            acc += math.hypot(dx, dy)
        return acc

    def resample(self, n_points: int) -> "Polyline":
        """Resample polyline to exactly n_points, equally spaced in arc length."""
        if n_points <= 1 or len(self.points) < 2:
            return Polyline(self.points[:])

        # Build cumulative arc length
        s_cum = [0.0]
        for i in range(1, len(self.points)):
            dx = self.points[i][0] - self.points[i - 1][0]
            dy = self.points[i][1] - self.points[i - 1][1]
            s_cum.append(s_cum[-1] + math.hypot(dx, dy))
        total = s_cum[-1]
        if total <= 0.0:
            return Polyline(self.points[:])

        def interp_at(s: float) -> Tuple[float, float]:
            if s <= 0.0:
                return self.points[0]
            if s >= total:
                return self.points[-1]
            # Find segment containing s
            for i in range(1, len(s_cum)):
                if s_cum[i] >= s:
                    s0, s1 = s_cum[i - 1], s_cum[i]
                    t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
                    x0, y0 = self.points[i - 1]
                    x1, y1 = self.points[i]
                    return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            return self.points[-1]

        target_s = np.linspace(0.0, total, n_points)
        out = [interp_at(float(s)) for s in target_s]
        return Polyline(out)


def _load_ttl_csv(path: Path) -> np.ndarray:
    """Load TTL CSV (x,y,z), returning an (N,3) array of floats."""
    pts: List[Tuple[float, float, float]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        # Expect header x,y,z but tolerate anything with >= 2 columns
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                z = float(row[2]) if len(row) >= 3 else 0.0
                pts.append((x, y, z))
            except ValueError:
                continue
    if not pts:
        raise RuntimeError(f"TTL file {path} has no valid points")
    return np.asarray(pts, dtype=float)


def _write_ttl_csv(path: Path, points: np.ndarray) -> None:
    """Write TTL CSV (x,y,z) from an (N,3) array."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        for x, y, z in points:
            writer.writerow([f"{x:.9f}", f"{y:.9f}", f"{z:.9f}"])


def _road_polyline(road) -> Polyline:
    """Extract centerline polyline (x,y) for a single OpenDRIVE road."""
    cl = _get_road_centerline(road)
    if cl is None:
        return Polyline([])
    ls = getattr(cl, "lineString", None)
    if ls is None or not getattr(ls, "coords", None):
        return Polyline([])
    pts = [(float(x), float(y)) for (x, y, *_) in ls.coords]
    return Polyline(pts)


def _best_pit_sequence(pit_roads: Sequence[object]) -> Polyline:
    """Given pit‑related roads (pit lane + junction links), build best continuous path.

    We try all permutations and orientations of the given roads and choose the
    ordering which minimizes the sum of distances between consecutive segments.
    This is robust for the simple Laguna Seca pit structure (2 connectors + pit).
    """
    from itertools import permutations, product

    # Precompute polylines for each road
    polys = [_road_polyline(r) for r in pit_roads]
    if any(len(p.points) < 2 for p in polys):
        raise RuntimeError("One or more pit roads have no valid centerline")

    best_cost = float("inf")
    best_seq: List[Tuple[int, bool]] | None = None

    for order in permutations(range(len(pit_roads))):
        for flips in product([False, True], repeat=len(pit_roads)):
            cost = 0.0
            ok = True
            prev_end = None
            for idx_pos, road_idx in enumerate(order):
                poly = polys[road_idx]
                pts = poly.points[::-1] if flips[road_idx] else poly.points
                if idx_pos == 0:
                    prev_end = pts[-1]
                    continue
                if prev_end is None:
                    ok = False
                    break
                start = pts[0]
                dx = start[0] - prev_end[0]
                dy = start[1] - prev_end[1]
                step_cost = math.hypot(dx, dy)
                cost += step_cost
                prev_end = pts[-1]
            if not ok:
                continue
            if cost < best_cost:
                best_cost = cost
                best_seq = [(road_idx, flips[road_idx]) for road_idx in order]

    if best_seq is None:
        raise RuntimeError("Could not find a consistent pitlane sequence")

    merged: List[Tuple[float, float]] = []
    for road_idx, flip in best_seq:
        pts = polys[road_idx].points
        if flip:
            pts = list(reversed(pts))
        if not merged:
            merged.extend(pts)
        else:
            # Avoid duplicating the joining point
            if merged[-1] == pts[0]:
                merged.extend(pts[1:])
            else:
                merged.extend(pts)
    return Polyline(merged)


def _nearest_index(points: np.ndarray, target: Tuple[float, float]) -> int:
    """Index of point in `points` closest to target (x,y)."""
    dx = points[:, 0] - target[0]
    dy = points[:, 1] - target[1]
    d2 = dx * dx + dy * dy
    return int(np.argmin(d2))


def _forward_index_range(n: int, start: int, end: int) -> List[int]:
    """Return indices from start to end moving forward modulo n, inclusive."""
    idxs: List[int] = []
    i = start
    while True:
        idxs.append(i)
        if i == end:
            break
        i = (i + 1) % n
    return idxs


def build_pitlane_ttl(
    xodr_path: Path,
    main_ttl_path: Path,
) -> np.ndarray:
    """Construct a pitlane TTL parallel to the given main racing TTL."""
    if not xodr_path.exists():
        raise FileNotFoundError(f"XODR not found: {xodr_path}")
    if not main_ttl_path.exists():
        raise FileNotFoundError(f"Main TTL not found: {main_ttl_path}")

    print(f"[INFO] Loading main TTL from {main_ttl_path}")
    main_pts = _load_ttl_csv(main_ttl_path)  # (N,3)
    n_main = main_pts.shape[0]
    print(f"[INFO]   {n_main} points")

    print(f"[INFO] Building RacingTrack from {xodr_path}")
    track = createRacingTrack(
        str(xodr_path),
        direction="counterclockwise",
        pitLaneRoadName="pit",
    )

    pit_roads_all = list(getattr(track, "_pitRoads", None) or [])
    if not pit_roads_all:
        raise RuntimeError("RacingTrack has no _pitRoads; pit lane not detected")

    conn_set = set(getattr(track.network, "connectingRoads", []) or [])
    pit_connectors = [r for r in pit_roads_all if r in conn_set]
    pit_lane_roads = [r for r in pit_roads_all if r not in conn_set]

    if len(pit_connectors) < 2:
        raise RuntimeError(
            f"Expected at least 2 pit connectors, found {len(pit_connectors)}"
        )
    print(
        f"[INFO] Pit roads: {len(pit_lane_roads)} pit lane road(s), "
        f"{len(pit_connectors)} connector road(s)"
    )

    # Build continuous pit path polyline (connectors + pit lane roads)
    pit_path_roads: List[object] = list(pit_connectors) + list(pit_lane_roads)
    pit_poly = _best_pit_sequence(pit_path_roads)
    if len(pit_poly.points) < 2:
        raise RuntimeError("Pit path polyline has fewer than 2 points")

    entry_xy = pit_poly.points[0]
    exit_xy = pit_poly.points[-1]
    print(
        f"[INFO] Pit entry approx at ({entry_xy[0]:.2f}, {entry_xy[1]:.2f}), "
        f"exit approx at ({exit_xy[0]:.2f}, {exit_xy[1]:.2f})"
    )

    # Find nearest points on main TTL to pit entry/exit
    idx_entry = _nearest_index(main_pts, entry_xy)
    idx_exit = _nearest_index(main_pts, exit_xy)
    print(f"[INFO] Nearest main TTL index to pit entry: {idx_entry}")
    print(f"[INFO] Nearest main TTL index to pit exit:  {idx_exit}")

    # Indices along the lap from entry to exit (following main TTL direction)
    replace_idx = _forward_index_range(n_main, idx_entry, idx_exit)
    m_replace = len(replace_idx)
    print(f"[INFO] Replacing {m_replace} point(s) between entry and exit along main TTL")

    # Resample pit path to same number of points
    pit_resampled = pit_poly.resample(m_replace)

    # Prepare new TTL: start from main TTL, then overwrite the pit section
    pit_ttl = np.array(main_pts, copy=True)

    z_start = float(main_pts[replace_idx[0], 2])
    z_end = float(main_pts[replace_idx[-1], 2])
    if m_replace > 1:
        zs = np.linspace(z_start, z_end, m_replace)
    else:
        zs = np.array([z_start], dtype=float)

    for k, idx in enumerate(replace_idx):
        x, y = pit_resampled.points[k]
        pit_ttl[idx, 0] = x
        pit_ttl[idx, 1] = y
        pit_ttl[idx, 2] = zs[k]

    return pit_ttl


def main() -> int:
    default_xodr = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    default_main_ttl = (
        REPO_ROOT
        / "assets"
        / "ttls"
        / "LS_ENU_TTL_CSV"
        / "ttl_racing_line_xodr.csv"
    )
    default_out_tool = REPO_ROOT / "create_new_ttl" / "ttl_pitlane_xodr.csv"
    default_out_assets = (
        REPO_ROOT
        / "assets"
        / "ttls"
        / "LS_ENU_TTL_CSV"
        / "ttl_pitlane_xodr.csv"
    )

    parser = argparse.ArgumentParser(
        description=(
            "Generate a pitlane TTL parallel to the existing racing TTL. "
            "Outside the pit section, the TTL matches the main racing TTL; "
            "between pit entry and exit it follows the pit lane."
        )
    )
    parser.add_argument(
        "--xodr",
        type=Path,
        default=default_xodr,
        help=f"Source OpenDRIVE map (default: {default_xodr})",
    )
    parser.add_argument(
        "--main-ttl",
        type=Path,
        default=default_main_ttl,
        help=f"Main racing TTL CSV (default: {default_main_ttl})",
    )
    parser.add_argument(
        "--output-tool",
        type=Path,
        default=default_out_tool,
        help=f"Output pit TTL CSV (tooling copy, default: {default_out_tool})",
    )
    parser.add_argument(
        "--output-assets",
        type=Path,
        default=default_out_assets,
        help=(
            "Output pit TTL CSV (assets runtime copy, "
            f"default: {default_out_assets})"
        ),
    )
    args = parser.parse_args()

    try:
        pit_ttl = build_pitlane_ttl(args.xodr, args.main_ttl)
    except Exception as exc:
        print(f"[ERROR] Failed to build pitlane TTL: {exc}")
        return 1

    print(f"[INFO] Writing pitlane TTL to {args.output_tool}")
    _write_ttl_csv(args.output_tool, pit_ttl)
    print(f"[INFO] Writing pitlane TTL to {args.output_assets}")
    _write_ttl_csv(args.output_assets, pit_ttl)
    print("[INFO] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

