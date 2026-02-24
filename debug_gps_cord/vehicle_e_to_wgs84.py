#!/usr/bin/env python3
"""
Find conversion between Vehicle Coordinate System E (dSPACE) and WGS84 using one
known corresponding point.

Correspondence (same physical point):
  Vehicle E (meters):  x = -20.001929315,  y = -49.881083992,  z = 3.63857835596
  WGS84 (degrees):     lon = -121.75605141, lat = 36.58691208,  heading = 217.027

We assume Vehicle_CoorSys_E is a local tangent-plane frame (ENU: x=East, y=North, z=Up)
with origin at (lon0, lat0, h0). One point fixes (lon0, lat0); h0 is set so that
at the calibration point altitude = z_E (or 0 if not needed).

Usage:
  python vehicle_e_to_wgs84.py              # derive and save transform, run round-trip check
  python vehicle_e_to_wgs84.py --validate    # load saved transform and validate only
"""

import json
import math
import argparse
from pathlib import Path
from typing import Tuple

# WGS84 ellipsoid (semi-major axis, meters)
R_EARTH_M = 6378137.0

# --- Calibration point (one point in both systems) ---
# Vehicle Coordinate System E (dSPACE DISP_Plant / Pos_*_Vehicle_CoorSys_E[m])
VEH_E_X_M = -20.001929315
VEH_E_Y_M = -49.881083992
VEH_E_Z_M = 3.63857835596

# WGS84 (GNSS from GPS_CALC)
WGS84_LON_DEG = -121.75605141
WGS84_LAT_DEG = 36.58691208
WGS84_HEADING_DEG = 217.027  # for reference; not used in position conversion


def derive_origin(
    x_e: float, y_e: float, z_e: float,
    lon_deg: float, lat_deg: float,
) -> Tuple[float, float, float]:
    """
    Compute local tangent-plane origin (lon0, lat0, h0) so that
    local (x_e, y_e, z_e) corresponds to WGS84 (lon_deg, lat_deg, alt).
    Assumes ENU: x = East, y = North, z = Up.
    We set altitude reference h0 = 0 so that alt = z_e at the calibration point.
    """
    lat_rad = math.radians(lat_deg)
    cos_lat = math.cos(lat_rad)
    # Meters to degrees: d_lon = x_e / (R * cos(lat)), d_lat = y_e / R
    deg_per_m_lon = (180.0 / math.pi) / (R_EARTH_M * cos_lat)
    deg_per_m_lat = (180.0 / math.pi) / R_EARTH_M

    lon0 = lon_deg - x_e * deg_per_m_lon
    lat0 = lat_deg - y_e * deg_per_m_lat
    h0 = 0.0  # so that at (x_e, y_e, z_e) altitude = z_e
    return (lon0, lat0, h0)


def local_to_wgs84(
    lon0: float, lat0: float, h0: float,
    x: float, y: float, z: float,
) -> Tuple[float, float, float]:
    """Convert Vehicle E (local ENU) (x, y, z) in meters to WGS84 (lon_deg, lat_deg, alt_m)."""
    lat0_rad = math.radians(lat0)
    cos_lat0 = math.cos(lat0_rad)
    deg_per_m_lon = (180.0 / math.pi) / (R_EARTH_M * cos_lat0)
    deg_per_m_lat = (180.0 / math.pi) / R_EARTH_M

    lon_deg = lon0 + x * deg_per_m_lon
    lat_deg = lat0 + y * deg_per_m_lat
    alt_m = h0 + z
    return (lon_deg, lat_deg, alt_m)


def wgs84_to_local(
    lon0: float, lat0: float, h0: float,
    lon_deg: float, lat_deg: float, alt_m: float,
) -> Tuple[float, float, float]:
    """Convert WGS84 (lon_deg, lat_deg, alt_m) to Vehicle E (local ENU) (x, y, z) in meters."""
    lat0_rad = math.radians(lat0)
    cos_lat0 = math.cos(lat0_rad)
    m_per_deg_lon = (R_EARTH_M * cos_lat0) * (math.pi / 180.0)
    m_per_deg_lat = R_EARTH_M * (math.pi / 180.0)

    x = (lon_deg - lon0) * m_per_deg_lon
    y = (lat_deg - lat0) * m_per_deg_lat
    z = alt_m - h0
    return (x, y, z)


def build_transform_dict(lon0: float, lat0: float, h0: float) -> dict:
    """Build a JSON-serializable transform description."""
    return {
        "type": "vehicle_e_to_wgs84",
        "description": "Local tangent-plane (ENU) origin for Vehicle Coordinate System E",
        "lon0_deg": lon0,
        "lat0_deg": lat0,
        "h0_m": h0,
        "R_earth_m": R_EARTH_M,
        "convention": "ENU: x=East, y=North, z=Up (meters)",
        "calibration_point_vehicle_e": [VEH_E_X_M, VEH_E_Y_M, VEH_E_Z_M],
        "calibration_point_wgs84": [WGS84_LON_DEG, WGS84_LAT_DEG],
    }


def load_transform(path: str) -> Tuple[float, float, float]:
    """Load transform from JSON; return (lon0, lat0, h0)."""
    with open(path, "r") as f:
        d = json.load(f)
    return (d["lon0_deg"], d["lat0_deg"], d["h0_m"])


def save_transform(path: str, lon0: float, lat0: float, h0: float) -> None:
    """Save transform to JSON."""
    d = build_transform_dict(lon0, lat0, h0)
    with open(path, "w") as f:
        json.dump(d, f, indent=2)
    print(f"Saved transform to {path}")


def main():
    parser = argparse.ArgumentParser(description="Derive Vehicle E <-> WGS84 conversion from one point.")
    parser.add_argument("--validate", action="store_true", help="Only validate using saved transform")
    parser.add_argument("--out", default=None, help="Output JSON path (default: same dir as script)")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    out_path = args.out or str(script_dir / "vehicle_e_wgs84_transform.json")

    if args.validate:
        lon0, lat0, h0 = load_transform(out_path)
        print(f"Loaded origin: lon0={lon0:.8f}° lat0={lat0:.8f}° h0={h0:.3f}m")
    else:
        lon0, lat0, h0 = derive_origin(
            VEH_E_X_M, VEH_E_Y_M, VEH_E_Z_M,
            WGS84_LON_DEG, WGS84_LAT_DEG,
        )
        print("Derived local tangent-plane origin (Vehicle E origin = this WGS84 point):")
        print(f"  lon0 = {lon0:.10f}°")
        print(f"  lat0 = {lat0:.10f}°")
        print(f"  h0   = {h0:.6f} m")
        save_transform(out_path, lon0, lat0, h0)

    # Round-trip: Vehicle E -> WGS84 -> Vehicle E
    lon1, lat1, alt1 = local_to_wgs84(lon0, lat0, h0, VEH_E_X_M, VEH_E_Y_M, VEH_E_Z_M)
    x2, y2, z2 = wgs84_to_local(lon0, lat0, h0, lon1, lat1, alt1)

    print("\nRound-trip check (Vehicle E -> WGS84 -> Vehicle E):")
    print(f"  Original Vehicle E:  ({VEH_E_X_M:.6f}, {VEH_E_Y_M:.6f}, {VEH_E_Z_M:.6f}) m")
    print(f"  -> WGS84:             ({lon1:.8f}°, {lat1:.8f}°, {alt1:.6f} m)")
    print(f"  -> back to Vehicle E: ({x2:.6f}, {y2:.6f}, {z2:.6f}) m")
    err = math.sqrt((x2 - VEH_E_X_M)**2 + (y2 - VEH_E_Y_M)**2 + (z2 - VEH_E_Z_M)**2)
    print(f"  Position error:       {err:.6e} m")
    lon_err = abs(lon1 - WGS84_LON_DEG)
    lat_err = abs(lat1 - WGS84_LAT_DEG)
    print(f"  WGS84 vs expected:    d_lon={lon_err:.2e}° d_lat={lat_err:.2e}°")
    if err < 1e-6 and lon_err < 1e-8 and lat_err < 1e-8:
        print("  [OK] Conversion consistent.")
    else:
        print("  [WARN] Small numerical differences (expected with floating point).")

    return 0


if __name__ == "__main__":
    exit(main())
