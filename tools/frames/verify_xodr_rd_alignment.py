"""Verify the alignment between Scenic XODR-xy and dSPACE RD-xy via GPS round-trip.

This is the Phase A.1 step. It quantifies whether the new map LGS_v1.xodr's xy frame
is identical to dSPACE's RD frame (single-source assumption, like the OLD map) or
offset/rotated (requiring a GPS bridge in placement).

Procedure (per sample):
  1. Read ego RD-xy from dSPACE: Pos_x_Vehicle_CoorSys_E, Pos_y_Vehicle_CoorSys_E.
  2. Read ego GPS from dSPACE:    GPS_CALC/Longitude_deg, GPS_CALC/Latitude_deg.
  3. Convert dSPACE GPS -> expected XODR-xy via pyproj.Transformer using the XODR's
     <geoReference> proj string.
  4. residual_m = ||RD-xy - expected XODR-xy||.

If residual_m is small (< 0.5 m) AND consistent across multiple samples, then
XODR-xy ~= RD-xy and we can keep the single-source assumption for placement.
If residual_m is large or non-uniform, the placement path must compose:
  Scenic xy -> GPS (via XODR geoReference) -> RD (via calibrated affine).

This script samples once on invocation by default. Use --samples N --interval S to
collect a time series while ego is driving.

Requires:
  - dSPACE simulation running with ego placed (after warmup)
  - MAPort connection available (the same path Scenic uses)
  - pyproj installed (already a transitive dependency)

Run from repo root, with dSPACE up:
  python tools/frames/verify_xodr_rd_alignment.py
  python tools/frames/verify_xodr_rd_alignment.py --samples 30 --interval 1.0
  python tools/frames/verify_xodr_rd_alignment.py --xodr assets/maps/dSPACE/LGS_v1.xodr
"""
from __future__ import annotations

import argparse
import math
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XODR = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"


def _add_repo_src_to_path() -> None:
    """Make the Scenic package importable when this script is run from repo root."""
    src = REPO_ROOT / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


def read_xodr_proj_string(xodr_path: Path) -> str:
    """Extract the <geoReference> proj string from an XODR file."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    header = root.find("header")
    if header is None:
        raise ValueError(f"{xodr_path}: no <header> element")
    geo = header.find("geoReference")
    if geo is None or not (geo.text or "").strip():
        raise ValueError(
            f"{xodr_path}: no <geoReference> CDATA in header. "
            "This XODR cannot be placed in GPS without a calibrated transform."
        )
    text = geo.text.strip()
    if "+proj=" not in text:
        raise ValueError(f"{xodr_path}: <geoReference> does not look like a proj string: {text!r}")
    return text


def make_gps_to_xodr_transformer(proj_string: str):
    """Return a pyproj.Transformer mapping GPS (lon, lat in WGS84) -> XODR-xy (m)."""
    import pyproj
    src = pyproj.CRS.from_proj4("+proj=longlat +datum=WGS84 +no_defs")
    dst = pyproj.CRS.from_proj4(proj_string)
    return pyproj.Transformer.from_crs(src, dst, always_xy=True)


def make_xodr_to_gps_transformer(proj_string: str):
    """Inverse: XODR-xy -> GPS (lon, lat)."""
    import pyproj
    src = pyproj.CRS.from_proj4(proj_string)
    dst = pyproj.CRS.from_proj4("+proj=longlat +datum=WGS84 +no_defs")
    return pyproj.Transformer.from_crs(src, dst, always_xy=True)


def connect_maport():
    """Connect to dSPACE via MAPort. Returns the maport handle or raises."""
    _add_repo_src_to_path()
    from scenic.simulators.dspace.maport import session as maport_session
    mp = maport_session.connect_and_prepare_maport(None, start_if_needed=False)
    if mp is None:
        raise RuntimeError(
            "MAPort connection failed. Is dSPACE running and the experiment loaded?"
        )
    return mp


def read_ego_rd_and_gps(mp) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    """Read (rd_x, rd_y, lon, lat). Any field None if read failed."""
    _add_repo_src_to_path()
    from scenic.simulators.dspace.controldesk.readback import (
        EGO_PATH_X, EGO_PATH_Y, EGO_GPS_LONGITUDE_DEG, EGO_GPS_LATITUDE_DEG,
    )
    try:
        rd_x = float(mp.get_var(EGO_PATH_X))
    except Exception:
        rd_x = None
    try:
        rd_y = float(mp.get_var(EGO_PATH_Y))
    except Exception:
        rd_y = None
    try:
        lon = float(mp.get_var(EGO_GPS_LONGITUDE_DEG))
    except Exception:
        lon = None
    try:
        lat = float(mp.get_var(EGO_GPS_LATITUDE_DEG))
    except Exception:
        lat = None
    return rd_x, rd_y, lon, lat


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xodr", default=str(DEFAULT_XODR), type=Path,
                    help="path to XODR (default: %(default)s)")
    ap.add_argument("--samples", default=1, type=int,
                    help="number of samples to collect (default: 1)")
    ap.add_argument("--interval", default=1.0, type=float,
                    help="seconds between samples when --samples > 1 (default: 1.0)")
    args = ap.parse_args()

    print(f"XODR: {args.xodr}")
    if not args.xodr.exists():
        print(f"  ERROR: XODR not found")
        return 1

    proj_string = read_xodr_proj_string(args.xodr)
    print(f"  geoReference: {proj_string}")

    try:
        gps_to_xodr = make_gps_to_xodr_transformer(proj_string)
    except Exception as e:
        print(f"  ERROR: failed to build pyproj transformer: {e}")
        return 1

    print()
    print("Connecting to dSPACE via MAPort...")
    try:
        mp = connect_maport()
    except Exception as e:
        print(f"  ERROR: {e}")
        return 1
    print("  connected")

    print()
    cols = "  ".join([
        "sample", "rd_x", "rd_y", "lon", "lat",
        "xodr_from_gps_x", "xodr_from_gps_y", "residual_m"
    ])
    print(cols)
    print("-" * len(cols))

    residuals: List[float] = []
    samples: List[Tuple[float, float, float, float, float, float, float]] = []
    for i in range(int(args.samples)):
        rd_x, rd_y, lon, lat = read_ego_rd_and_gps(mp)
        if None in (rd_x, rd_y, lon, lat):
            print(f"  sample #{i}: read failed (rd_x={rd_x}, rd_y={rd_y}, lon={lon}, lat={lat})")
            if i + 1 < args.samples:
                time.sleep(args.interval)
            continue

        x_from_gps, y_from_gps = gps_to_xodr.transform(lon, lat)
        dx = rd_x - x_from_gps
        dy = rd_y - y_from_gps
        res = math.hypot(dx, dy)
        residuals.append(res)
        samples.append((rd_x, rd_y, lon, lat, x_from_gps, y_from_gps, res))
        print(f"  #{i:>3}  {rd_x:>10.3f}  {rd_y:>10.3f}  {lon:>11.6f}  {lat:>10.6f}  "
              f"{x_from_gps:>10.3f}  {y_from_gps:>10.3f}  {res:>9.3f}")

        if i + 1 < args.samples:
            time.sleep(args.interval)

    print()
    if not residuals:
        print("ERROR: no successful samples; cannot evaluate alignment")
        return 1

    res_min = min(residuals)
    res_max = max(residuals)
    res_mean = sum(residuals) / len(residuals)
    print(f"Residual stats over {len(residuals)} samples: min={res_min:.3f} m, "
          f"max={res_max:.3f} m, mean={res_mean:.3f} m")

    # Per-component offsets (useful if it's a pure translation).
    if len(samples) >= 1:
        dxs = [s[0] - s[4] for s in samples]
        dys = [s[1] - s[5] for s in samples]
        dx_mean = sum(dxs) / len(dxs)
        dy_mean = sum(dys) / len(dys)
        dx_range = max(dxs) - min(dxs)
        dy_range = max(dys) - min(dys)
        print(f"Per-component offset (RD - XODR_from_GPS): "
              f"dx_mean={dx_mean:+.3f} m (range {dx_range:.3f}), "
              f"dy_mean={dy_mean:+.3f} m (range {dy_range:.3f})")

    print()
    if res_max < 0.5:
        print("CONCLUSION: XODR-xy ~= dSPACE RD-xy (single-source). No GPS bridge needed.")
        print("            Phase A can proceed without GPS->RD calibration.")
    elif (max(dxs) - min(dxs) < 0.5) and (max(dys) - min(dys) < 0.5):
        print("CONCLUSION: Pure translation between XODR and RD frames "
              f"(dx={dx_mean:+.3f}, dy={dy_mean:+.3f}).")
        print("            Calibrate GPS->RD as identity + this translation, save to")
        print("            assets/maps/dSPACE/LGS_v1_gps_rd_calibration.json.")
    else:
        print("CONCLUSION: XODR and RD frames differ non-uniformly across samples.")
        print("            Need a full affine GPS->RD calibration via "
              "domains/racing/gnss_transform.fit_transform_from_csv().")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
