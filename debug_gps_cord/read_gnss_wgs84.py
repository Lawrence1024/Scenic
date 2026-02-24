#!/usr/bin/env python3
"""
Read GNSS coordinates in WGS84 from dSPACE ControlDesk.

Reads longitude (deg), latitude (deg), and heading (deg) from the variable paths
documented in debug_gps_cord/README.md:
- Ego: Environment/Road/PlantModel/GPS_POSITION/GPS_CALC (single values)
- Fellows: VesiInterface/Vehicle Sensors/ground_truth/GPS_POSITION/GPS_CALC (array [0], [1], ...)

Usage:
  cd debug_gps_cord
  python read_gnss_wgs84.py [--samples N] [--interval SEC]
"""

import sys
import os
import time
import argparse
from pathlib import Path

# Fix encoding for Windows console
if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

# Add Scenic src to path
scenic_path = Path(__file__).resolve().parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

# --- Variable paths (WGS84) from dSPACE ---
# Ego: single values from Environment/Road/PlantModel
BASE_EGO = "Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC"
PATH_EGO_LON = f"{BASE_EGO}/Longitude_deg"
PATH_EGO_LAT = f"{BASE_EGO}/Latitude_deg"
PATH_EGO_HEADING = f"{BASE_EGO}/Heading_deg"

# Fellows: arrays under VesiInterface/Vehicle Sensors/ground_truth (index 0, 1, ...)
BASE_FELLOW = "Platform()://ASM_Traffic/Model Root/VesiInterface/Vehicle Sensors/ground_truth/GPS_POSITION/GPS_CALC"


def get_ego_gnss(cd):
    """Read ego vehicle GNSS (WGS84): longitude_deg, latitude_deg, heading_deg."""
    try:
        lon = float(cd.get_var(PATH_EGO_LON))
        lat = float(cd.get_var(PATH_EGO_LAT))
        heading = float(cd.get_var(PATH_EGO_HEADING))
        return {"longitude_deg": lon, "latitude_deg": lat, "heading_deg": heading}
    except Exception as e:
        print(f"[Ego GNSS] Error: {e}")
        return None


def get_fellow_gnss(cd, fellow_index=0):
    """Read one fellow vehicle GNSS (WGS84) by array index."""
    try:
        path_lon = f"{BASE_FELLOW}/Longitude_deg[{fellow_index}]"
        path_lat = f"{BASE_FELLOW}/Latitude_deg[{fellow_index}]"
        path_heading = f"{BASE_FELLOW}/Heading_deg[{fellow_index}]"
        lon = float(cd.get_var(path_lon))
        lat = float(cd.get_var(path_lat))
        heading = float(cd.get_var(path_heading))
        return {"longitude_deg": lon, "latitude_deg": lat, "heading_deg": heading}
    except Exception as e:
        print(f"[Fellow {fellow_index} GNSS] Error: {e}")
        return None


def read_all_gnss(cd, num_fellows=1):
    """Read ego and up to num_fellows fellow GNSS values."""
    out = {"ego": None, "fellows": []}
    out["ego"] = get_ego_gnss(cd)
    for i in range(num_fellows):
        row = get_fellow_gnss(cd, i)
        out["fellows"].append(row)
    return out


def print_gnss(gnss, label="GNSS"):
    """Print a single GNSS dict (WGS84)."""
    if gnss is None:
        print(f"   {label}: (read failed)")
        return
    print(f"   {label}: lon={gnss['longitude_deg']:.8f}° lat={gnss['latitude_deg']:.8f}° heading={gnss['heading_deg']:.3f}°")


def main():
    parser = argparse.ArgumentParser(description="Read GNSS (WGS84) from dSPACE ControlDesk.")
    parser.add_argument("--samples", type=int, default=1, help="Number of read samples (default 1)")
    parser.add_argument("--interval", type=float, default=0.5, help="Seconds between samples (default 0.5)")
    parser.add_argument("--fellows", type=int, default=1, help="Number of fellow indices to read (default 1)")
    args = parser.parse_args()

    print("=" * 60)
    print("GNSS (WGS84) readback from ControlDesk")
    print("=" * 60)

    try:
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
    except ImportError as e:
        print(f"[ERROR] Could not import ControlDesk connection: {e}")
        print("  Run from Scenic repo root and ensure 'src' is on path.")
        return 1

    try:
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2",
        ).connect()
        print("[OK] Connected to ControlDesk\n")
    except Exception as e:
        print(f"[ERROR] Could not connect to ControlDesk: {e}")
        return 1

    for sample in range(args.samples):
        if args.samples > 1:
            print(f"--- Sample {sample + 1}/{args.samples} ---")
        data = read_all_gnss(cd, num_fellows=args.fellows)
        print_gnss(data["ego"], "Ego (WGS84)")
        for i, f in enumerate(data["fellows"]):
            print_gnss(f, f"Fellow[{i}] (WGS84)")
        if args.samples > 1 and sample < args.samples - 1:
            time.sleep(args.interval)

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
