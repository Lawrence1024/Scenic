# -*- coding: utf-8 -*-
"""Read a single ControlDesk variable by path via COM.

Usage:
  python tools/read_cd_var.py "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Angle_SteeringWheel[deg]/Value"
"""

import sys


def main():
    if len(sys.argv) < 2:
        print("Usage: read_cd_var.py <variable_path>")
        return 2
    path = sys.argv[1]

    try:
        import pythoncom
        from win32com.client import Dispatch
        pythoncom.CoInitialize()
        app = Dispatch("ControlDeskNG.Application")
        exp = app.ActiveExperiment
        if not exp:
            print("No ActiveExperiment")
            return 1
        plats = exp.Platforms
        plat0 = plats[0]
        vdesc = plat0.ActiveVariableDescription
        variables = getattr(vdesc, 'Variables', None)
        if variables is None:
            print("Variables container is None; attempting direct index may fail...")
        try:
            val = vdesc.Variables[path].ValueConverted
            print(f"READ OK: {path} = {val}")
            return 0
        except Exception as e:
            print(f"READ FAIL via vdesc.Variables: {e}")
        # Try platform-level indirection if available
        try:
            val = plat0.ActiveVariableDescription.Variables[path].ValueConverted
            print(f"READ OK (plat chain): {path} = {val}")
            return 0
        except Exception as e:
            print(f"READ FAIL via plat chain: {e}")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 2


if __name__ == '__main__':
    sys.exit(main())



