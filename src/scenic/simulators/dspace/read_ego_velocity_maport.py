#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read ego velocity via MAPort (or ControlDesk) to verify the sim is moving the vehicle.

Ego velocity comes from DISP_Plant: v_x and v_y in km/h. Speed in m/s = norm(vx, vy) / 3.6.
Use this after reset_vehicle_to_drive to confirm the modification worked: if speed_m_s > 0
(and above a small threshold), the sim is applying throttle.

Run with ControlDesk (or VEOS with MAPort) connected and the experiment loaded.

Usage:
  python -m scenic.simulators.dspace.read_ego_velocity_maport [--threshold 0.5] [--maport]
  # From repo root:
  python src/scenic/simulators/dspace/read_ego_velocity_maport.py [--threshold 0.5]

Exit: 0 if speed_m_s >= threshold (vehicle moving), 1 otherwise or on read failure.
"""

import argparse
import math
import sys
from pathlib import Path


def _add_repo_to_path():
    """Ensure Scenic package is importable when script is run as __main__ from repo."""
    src = Path(__file__).resolve().parent.parent.parent.parent
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


def read_ego_velocity_maport():
    """Read ego vx, vy (km/h) via MAPort. Returns (vx_kmh, vy_kmh) or (None, None)."""
    try:
        from scenic.simulators.dspace.maport import session as maport_session
        from scenic.simulators.dspace.controldesk.readback import EGO_PATH_VX, EGO_PATH_VY
    except ImportError:
        _add_repo_to_path()
        from scenic.simulators.dspace.maport import session as maport_session
        from scenic.simulators.dspace.controldesk.readback import EGO_PATH_VX, EGO_PATH_VY

    mp = maport_session.connect_and_prepare_maport(None, start_if_needed=False)
    if mp is None:
        return None, None
    try:
        vx = float(mp.get_var(EGO_PATH_VX))
        vy = float(mp.get_var(EGO_PATH_VY))
        return vx, vy
    except Exception:
        return None, None


def read_ego_velocity_controldesk():
    """Read ego vx, vy (km/h) via ControlDesk COM. Returns (vx_kmh, vy_kmh) or (None, None)."""
    try:
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
        from scenic.simulators.dspace.controldesk.readback import EGO_PATH_VX, EGO_PATH_VY
    except ImportError:
        _add_repo_to_path()
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
        from scenic.simulators.dspace.controldesk.readback import EGO_PATH_VX, EGO_PATH_VY

    cd = ControlDeskApp().connect()
    cd.go_online()
    cd.start_measurement()
    try:
        vx = float(cd.get_var(EGO_PATH_VX))
        vy = float(cd.get_var(EGO_PATH_VY))
        return vx, vy
    except Exception:
        return None, None


def main():
    parser = argparse.ArgumentParser(
        description="Read ego velocity via MAPort; exit 0 if speed >= threshold (vehicle moving)."
    )
    parser.add_argument(
        "--threshold", "-t", type=float, default=0.5,
        help="Minimum speed (m/s) to consider vehicle moving (default: 0.5)"
    )
    parser.add_argument("--maport", action="store_true", help="Use MAPort only (default: try MAPort, then COM)")
    parser.add_argument("--com", action="store_true", help="Use ControlDesk COM only (skip MAPort)")
    args = parser.parse_args()

    use_maport = not args.com

    if use_maport:
        vx_kmh, vy_kmh = read_ego_velocity_maport()
        if vx_kmh is None and vy_kmh is None:
            print("MAPort read failed, trying ControlDesk COM...")
            vx_kmh, vy_kmh = read_ego_velocity_controldesk()
    else:
        vx_kmh, vy_kmh = read_ego_velocity_controldesk()

    if vx_kmh is None or vy_kmh is None:
        print("Failed to read ego velocity (DISP_Plant Velocities).", file=sys.stderr)
        sys.exit(1)

    vx_ms = vx_kmh / 3.6
    vy_ms = vy_kmh / 3.6
    speed_ms = math.sqrt(vx_ms * vx_ms + vy_ms * vy_ms)

    print("Ego velocity (MAPort/COM read from DISP_Plant):")
    print(f"  v_x_kmh: {vx_kmh:.2f}  v_y_kmh: {vy_kmh:.2f}")
    print(f"  speed_m_s: {speed_ms:.3f}")

    if speed_ms >= args.threshold:
        print(f"  -> Vehicle is moving (speed >= {args.threshold} m/s).")
        sys.exit(0)
    else:
        print(f"  -> Vehicle not moving (speed < {args.threshold} m/s).")
        sys.exit(1)


if __name__ == "__main__":
    main()
