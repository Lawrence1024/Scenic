#!/usr/bin/env python3
"""
Visualize mainTrack and pitTrack regions (buffered centerlines with width).

Track regions are the same as used by the racing model for placement
(new RacingCar on mainTrack / on pitTrack): main = 6 m each side of main
centerline, pit = 3.25 m each side of pit centerline, with overlap rule
(main wins over pit, so Corkscrew etc. are main only).

Usage (from repo root):
    python -m scenic.domains.racing.segments.visualize_track_regions --ttl-folder PATH [-o FILE]
    python -m scenic.domains.racing.segments.visualize_track_regions --map PATH [-o FILE]
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from scenic.core.geometry import plotPolygon
from scenic.domains.racing.segments.track_regions import (
    create_track_regions,
    MAIN_TRACK_BUFFER_M,
    PIT_TRACK_BUFFER_M,
)


def _find_repo_root() -> Path:
    """Find repo root (directory containing 'src' and 'assets')."""
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent


def main():
    parser = argparse.ArgumentParser(
        description="Visualize mainTrack and pitTrack regions (polygons with width)"
    )
    parser.add_argument(
        "--ttl-folder",
        type=str,
        default=None,
        help="Folder with ttl_main_road.csv and ttl_pitlane.csv (TTL centerlines)",
    )
    parser.add_argument(
        "--map",
        type=str,
        default=None,
        help="Path to OpenDRIVE .xodr (used if --ttl-folder not set)",
    )
    parser.add_argument(
        "--main-buffer",
        type=float,
        default=MAIN_TRACK_BUFFER_M,
        help=f"Buffer (m) each side of main centerline (default {MAIN_TRACK_BUFFER_M})",
    )
    parser.add_argument(
        "--pit-buffer",
        type=float,
        default=PIT_TRACK_BUFFER_M,
        help=f"Buffer (m) each side of pit centerline (default {PIT_TRACK_BUFFER_M})",
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default=None,
        help="Output image path (default: show interactively)",
    )
    args = parser.parse_args()

    repo_root = _find_repo_root()

    # Resolve paths
    ttl_folder = None
    if args.ttl_folder:
        ttl_folder = Path(args.ttl_folder)
        if not ttl_folder.is_absolute():
            ttl_folder = repo_root / ttl_folder
        if not ttl_folder.exists():
            print(f"TTL folder not found: {ttl_folder}", file=sys.stderr)
            sys.exit(1)

    map_file = None
    if args.map:
        map_file = Path(args.map)
        if not map_file.is_absolute():
            map_file = repo_root / map_file
        if not map_file.exists():
            print(f"Map file not found: {map_file}", file=sys.stderr)
            sys.exit(1)

    if ttl_folder is None and map_file is None:
        # Default: TTL folder
        ttl_folder = repo_root / "assets" / "ttls" / "LS_ENU_TTL_CSV"
        if not ttl_folder.exists():
            print("Neither --ttl-folder nor --map given and default TTL folder not found.", file=sys.stderr)
            sys.exit(1)
        print(f"Using default TTL folder: {ttl_folder}")

    # Build track regions (same logic as racing model)
    main_track, pit_track, _ = create_track_regions(
        map_file=str(map_file) if map_file else None,
        ttl_folder=str(ttl_folder) if ttl_folder else None,
        main_buffer_m=args.main_buffer,
        pit_buffer_m=args.pit_buffer,
        direction="counterclockwise",
        pitLaneRoadName="pit",
    )

    # Plot
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))

    from shapely.geometry import Polygon, MultiPolygon

    legend_handles = []

    def _plot_polygon_with_holes(ax, poly, facecolor, edgecolor):
        """Plot polygon (exterior + interiors as holes so inner boundary is white)."""
        if not poly.exterior or len(poly.exterior.coords) < 3:
            return
        x, y = poly.exterior.xy
        ax.fill(x, y, color=facecolor, alpha=0.35)
        for interior in poly.interiors:
            xi, yi = interior.xy
            ax.fill(xi, yi, color="white", edgecolor=edgecolor)

    # mainTrack: blue fill + boundary; interiors (infield) drawn white
    main_polygons = getattr(main_track, "polygons", None)
    if main_polygons is not None and not getattr(main_polygons, "is_empty", True):
        try:
            geoms = main_polygons.geoms if isinstance(main_polygons, MultiPolygon) else [main_polygons]
            for poly in geoms:
                if isinstance(poly, Polygon) and not poly.is_empty:
                    _plot_polygon_with_holes(ax, poly, "tab:blue", "tab:blue")
            plotPolygon(main_polygons, ax, style="b-", linewidth=1.5)
            legend_handles.append(Patch(facecolor="tab:blue", alpha=0.35, edgecolor="tab:blue", label=f"mainTrack (±{args.main_buffer} m)"))
        except Exception as e:
            print(f"Could not plot mainTrack: {e}", file=sys.stderr)
    else:
        print("mainTrack has no polygon geometry to plot.", file=sys.stderr)

    # pitTrack: green fill + boundary; interiors drawn white
    pit_polygons = getattr(pit_track, "polygons", None)
    if pit_polygons is not None and not getattr(pit_polygons, "is_empty", True):
        try:
            geoms = pit_polygons.geoms if isinstance(pit_polygons, MultiPolygon) else [pit_polygons]
            for poly in geoms:
                if isinstance(poly, Polygon) and not poly.is_empty:
                    _plot_polygon_with_holes(ax, poly, "tab:green", "tab:green")
            plotPolygon(pit_polygons, ax, style="g-", linewidth=1.5)
            legend_handles.append(Patch(facecolor="tab:green", alpha=0.35, edgecolor="tab:green", label=f"pitTrack (±{args.pit_buffer} m)"))
        except Exception as e:
            print(f"Could not plot pitTrack: {e}", file=sys.stderr)
    else:
        print("pitTrack has no polygon geometry to plot (may be empty after overlap removal).", file=sys.stderr)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    source = "TTL" if ttl_folder else "OpenDRIVE"
    ax.set_title(f"Track regions ({source}): mainTrack (blue), pitTrack (green); overlap = main")
    if legend_handles:
        ax.legend(handles=legend_handles, loc="upper right")

    if args.output:
        out = Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"Saved: {out}")
    plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
