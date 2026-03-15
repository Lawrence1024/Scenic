#!/usr/bin/env python3
"""Read VesiInterface variables from ControlDesk and compare to external_control_baseline.json.

When scenic_control=False we apply external_control_baseline.json. This script checks
if current ControlDesk values match that baseline (e.g. after a run or in External Control mode).

Usage (from repo root):
  PYTHONPATH=src python src/scenic/simulators/dspace/compare_vesi_baseline.py
"""
import json
import sys
from pathlib import Path

# Same 16 variables as read_vesi_interface_vars / session._apply_external_control_baseline
VESI_PATHS = [
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

BASELINE_PATH = Path(__file__).resolve().parent / "external_control_baseline.json"


def _norm_val(v):
    """Normalize for comparison: list -> first element if len 1 else list; scalar as-is."""
    if hasattr(v, "__iter__") and not isinstance(v, (str, bytes)):
        v = list(v)
        return v[0] if len(v) == 1 else v
    return v


def _values_match(baseline_val, current_val, tol=1e-5):
    b = _norm_val(baseline_val)
    c = _norm_val(current_val)
    if isinstance(b, (int, float)) and isinstance(c, (int, float)):
        return abs(float(b) - float(c)) <= tol
    if isinstance(b, (list, tuple)) and isinstance(c, (list, tuple)):
        if len(b) != len(c):
            return False
        return all(
            abs(float(x) - float(y)) <= tol
            for x, y in zip(b, c)
        )
    return b == c


def main():
    src = Path(__file__).resolve().parent.parent.parent.parent
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))

    if not BASELINE_PATH.is_file():
        print(f"Baseline not found: {BASELINE_PATH}")
        print("Run read_vesi_interface_vars.py once to create external_control_baseline.json")
        sys.exit(1)

    with open(BASELINE_PATH) as f:
        baseline = json.load(f)

    from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

    print("Connecting to ControlDesk...")
    cd = ControlDeskApp().connect()
    cd.go_online()
    cd.start_measurement()

    print(f"\nComparing to baseline: {BASELINE_PATH.name}\n")
    print(f"{'Variable':<35} {'Baseline (we set)':<24} {'Current (CD)':<24} {'Match'}")
    print("-" * 95)

    mismatches = 0
    for path, short_name in VESI_PATHS:
        expected = baseline.get(path)
        try:
            current = cd.get_var(path)
        except Exception as e:
            current = f"<READ ERR: {e}>"
            mismatches += 1
        match = _values_match(expected, current) if expected is not None and not isinstance(current, str) else "?"
        if match is False:
            mismatches += 1
        status = "OK" if match is True else ("MISMATCH" if match is False else "?")
        exp_str = str(expected) if expected is not None else "N/A"
        cur_str = str(current)[:22] + ".." if len(str(current)) > 24 else str(current)
        print(f"{short_name:<35} {exp_str:<24} {cur_str:<24} {status}")

    print("-" * 95)
    print(f"Mismatches: {mismatches}")
    return 0 if mismatches == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
