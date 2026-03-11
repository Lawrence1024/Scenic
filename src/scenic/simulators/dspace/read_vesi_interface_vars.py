#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read the 16 VesiInterface variables that initialize_vesi_interface() writes.

Use this when the simulator is in External Control mode (ego not controlled by
Scenic) to capture the current values. You can then use this snapshot as the
baseline to restore when switching to External Control, so we do not overwrite
them with Manual Control values.

Run with ControlDesk (or VEOS with MAPort) connected and the experiment loaded.
Output: printed to console and saved to external_control_baseline.json in the
script directory (or path given by --output).

Usage:
  python -m scenic.simulators.dspace.read_vesi_interface_vars [--output path.json]
  # Or from repo root:
  python src/scenic/simulators/dspace/read_vesi_interface_vars.py [--output path.json]
"""

import argparse
import json
import sys
from pathlib import Path

# Paths written by initialize_vesi_interface() in controldesk/connection.py
# (same 16 variables so we know what to read for External Control baseline)
VESI_INTERFACE_PATHS = [
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value", "Sw_Activate_CLIF"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value", "Sw_Manual_VESI_Overwrite"),
    ("Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value", "Sw_RaceControl"),
    ("Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value", "Const_sys_state"),
    ("Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value", "Const_track_flag"),
    ("Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value", "Const_veh_flag"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_brake_cmd/Value", "Const_enable_brake_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_gear_cmd/Value", "Const_enable_gear_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_steering_cmd/Value", "Const_enable_steering_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_throttle_cmd/Value", "Const_enable_throttle_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value", "Const_throttle_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value", "Const_brake_cmd_front"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value", "Const_brake_cmd_rear"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value", "Const_steering_cmd"),
    ("Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value", "Const_gear_cmd"),
    ("Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value", "Pos_ClutchPedal"),
]


def _json_serialize(val):
    """Convert value to JSON-serializable form (list, float, int)."""
    if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
        return list(float(x) if isinstance(x, (int, float)) else x for x in val)
    if isinstance(val, (int, float)):
        return val
    return val


def _add_repo_to_path():
    """Ensure Scenic package is importable when script is run as __main__ from repo."""
    src = Path(__file__).resolve().parent.parent.parent.parent
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))


def read_via_controldesk():
    """Use ControlDesk COM to read variables. Returns (path -> value) dict or None on failure."""
    try:
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
    except ImportError:
        _add_repo_to_path()
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

    cd = ControlDeskApp().connect()
    cd.go_online()
    cd.start_measurement()
    out = {}
    for path, short_name in VESI_INTERFACE_PATHS:
        try:
            val = cd.get_var(path)
            out[path] = val
            print(f"  {short_name}: {val}")
        except Exception as e:
            print(f"  {short_name}: [READ ERROR] {e}")
            out[path] = None
    return out


def read_via_maport():
    """Use MAPort (XIL API) to read variables. Returns (path -> value) dict or None if unavailable."""
    try:
        from scenic.simulators.dspace.maport import session as maport_session
    except ImportError:
        _add_repo_to_path()
        from scenic.simulators.dspace.maport import session as maport_session

    try:
        mp = maport_session.connect_and_prepare_maport(None, start_if_needed=False)
        if mp is None:
            return None
        out = {}
        for path, short_name in VESI_INTERFACE_PATHS:
            try:
                val = mp.get_var(path)
                out[path] = val
                print(f"  {short_name}: {val}")
            except Exception as e:
                print(f"  {short_name}: [READ ERROR] {e}")
                out[path] = None
        return out
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description="Read 16 VesiInterface variables for External Control baseline.")
    parser.add_argument("--output", "-o", type=Path, default=None,
                        help="Output JSON path (default: external_control_baseline.json in script dir)")
    parser.add_argument("--maport", action="store_true", help="Prefer MAPort over ControlDesk")
    args = parser.parse_args()

    out_path = args.output
    if out_path is None:
        out_path = Path(__file__).resolve().parent / "external_control_baseline.json"

    print("Reading 16 VesiInterface variables (current values = External Control baseline)...")
    if args.maport:
        print("Using MAPort...")
        result = read_via_maport()
        if result is None:
            print("MAPort failed, trying ControlDesk...")
            result = read_via_controldesk()
    else:
        print("Using ControlDesk...")
        result = read_via_controldesk()

    if result is None:
        print("Failed to read variables.", file=sys.stderr)
        sys.exit(1)

    # Build JSON-serializable snapshot (path -> value)
    snapshot = {path: _json_serialize(val) for path, val in result.items()}
    with open(out_path, "w") as f:
        json.dump(snapshot, f, indent=2)
    print(f"\nSaved to {out_path}")


if __name__ == "__main__":
    main()
