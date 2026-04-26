"""Verify that XODR-derived mainTrack/pitTrack regions look sane on LGS_v1.xodr.

We're about to switch ``create_track_regions`` to prefer the OpenDRIVE-native path over
the empirical-TTL-CSV path, but the OpenDRIVE path has been dormant in F-bank runs (TTL
path always wins when ``ttl_folder`` is set). Before flipping the default we want to
sanity-check that:

1. ``createRacingTrack(LGS_v1.xodr)`` correctly identifies main + pit roads.
2. ``build_track_regions_from_opendrive`` produces non-empty mainTrack / pitTrack
   regions whose extents match race_common's ground-truth track polygon.
3. The regions are wide enough that the racing line (loaded from ``ttl_optimal_xodr.csv``)
   stays inside mainTrack everywhere.

Read-only diagnostic. Run from repo root:
    python tools/frames/verify_xodr_native_regions.py
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
XODR = REPO / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"
OPTIMAL_TTL = REPO / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_optimal_xodr.csv"
INSIDE_CSV = REPO / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "track_inside.csv"
OUTSIDE_CSV = REPO / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "track_outside.csv"


def load_xy(p: Path) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    if not p.is_file():
        return out
    with open(p, newline="") as f:
        r = csv.reader(f)
        next(r, None)  # header
        for row in r:
            try:
                out.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return out


def main() -> int:
    import sys
    sys.path.insert(0, str(REPO / "src"))
    from scenic.domains.racing.segments.tracks import createRacingTrack
    from scenic.domains.racing.segments.track_regions import (
        build_track_regions_from_opendrive,
    )

    print(f"XODR: {XODR}")
    print()
    print("Step 1: createRacingTrack from LGS_v1.xodr ...")
    track = createRacingTrack(str(XODR))
    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])
    print(f"  main racing roads: {[(getattr(r, 'id', '?'), getattr(r, 'name', '?')) for r in main_roads]}")
    print(f"  pit roads:         {[(getattr(r, 'id', '?'), getattr(r, 'name', '?')) for r in pit_roads]}")
    print()

    print("Step 2: build_track_regions_from_opendrive ...")
    main_region, pit_region = build_track_regions_from_opendrive(track)
    print(f"  mainTrack: {type(main_region).__name__ if main_region else 'None'}")
    print(f"  pitTrack:  {type(pit_region).__name__ if pit_region else 'None'}")
    if main_region is None:
        print("  ERROR: mainTrack empty; OpenDRIVE path not viable")
        return 1

    # Region extents
    main_shapely = None
    pit_shapely = None
    try:
        main_shapely = main_region.polygons   # MultiPolygon (Shapely)
        pit_shapely = pit_region.polygons if pit_region is not None else None
        bbox_m = main_shapely.bounds
        print(f"  mainTrack bbox: x in [{bbox_m[0]:+.1f}, {bbox_m[2]:+.1f}], "
              f"y in [{bbox_m[1]:+.1f}, {bbox_m[3]:+.1f}]")
        print(f"  mainTrack area: {main_shapely.area:.0f} m^2")
        if pit_shapely is not None:
            bbox_p = pit_shapely.bounds
            print(f"  pitTrack  bbox: x in [{bbox_p[0]:+.1f}, {bbox_p[2]:+.1f}], "
                  f"y in [{bbox_p[1]:+.1f}, {bbox_p[3]:+.1f}]")
            print(f"  pitTrack  area: {pit_shapely.area:.0f} m^2")
    except Exception as e:
        print(f"  (couldn't read polygon directly: {e})")
    print()

    # Step 3: confirm racing line is inside mainTrack
    print("Step 3: confirm ttl_optimal_xodr.csv waypoints are inside mainTrack ...")
    pts = load_xy(OPTIMAL_TTL)
    if not pts:
        print(f"  ERROR: {OPTIMAL_TTL} not loadable")
        return 1
    print(f"  loaded {len(pts)} optimal-TTL waypoints")
    if main_shapely is None:
        print("  (skipping containment check; no shapely polygon)")
    else:
        from shapely.geometry import Point
        n_inside = 0
        n_outside = 0
        worst_outside = 0.0
        worst_pt = None
        for x, y in pts:
            p = Point(x, y)
            if main_shapely.contains(p):
                n_inside += 1
            else:
                d = float(p.distance(main_shapely))
                n_outside += 1
                if d > worst_outside:
                    worst_outside = d
                    worst_pt = (x, y)
        print(f"  inside mainTrack: {n_inside} ({100*n_inside/len(pts):.1f}%)")
        print(f"  outside:          {n_outside} ({100*n_outside/len(pts):.1f}%)")
        if worst_pt:
            print(f"  worst-outside waypoint: ({worst_pt[0]:.2f}, {worst_pt[1]:.2f}) "
                  f"distance to mainTrack edge: {worst_outside:.2f}m")
    print()

    # Step 4: compare to race_common boundaries
    print("Step 4: compare mainTrack extents to race_common geofence ...")
    inside_pts = load_xy(INSIDE_CSV)
    outside_pts = load_xy(OUTSIDE_CSV)
    if inside_pts and outside_pts and main_shapely is not None:
        from shapely.geometry import Point
        # Sample some race_common boundary points and check if they're near mainTrack edge.
        # If mainTrack is a 12m wide buffer around centerline, and race_common track is ~12m
        # wide, the mainTrack edge should be CLOSE to race_common's outside.csv (within ~1m).
        n_check = min(50, len(outside_pts))
        step = max(1, len(outside_pts) // n_check)
        dists = []
        for i in range(0, len(outside_pts), step):
            x, y = outside_pts[i]
            d = float(Point(x, y).distance(main_shapely.boundary))
            dists.append(d)
        if dists:
            print(f"  race_common outside.csv samples (n={len(dists)}): "
                  f"distance to mainTrack boundary  min={min(dists):.2f}m  "
                  f"max={max(dists):.2f}m  mean={sum(dists)/len(dists):.2f}m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
