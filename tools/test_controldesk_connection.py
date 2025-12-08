#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Quick test to see if ControlDesk connection works."""

import sys

print("Starting ControlDesk connection test...", flush=True)

try:
    import pythoncom
    from win32com.client import Dispatch
    
    print("Initializing COM...", flush=True)
    pythoncom.CoInitialize()
    
    print("Connecting to ControlDesk...", flush=True)
    app = Dispatch("ControlDeskNG.Application")
    print("✓ Connected to ControlDesk", flush=True)
    
    print("Checking for ActiveExperiment...", flush=True)
    exp = app.ActiveExperiment
    if exp is None:
        print("✗ No ActiveExperiment found - please open an experiment in ControlDesk", flush=True)
        sys.exit(1)
    
    print(f"✓ Active experiment: {exp.Name}", flush=True)
    
    print("Test completed successfully!", flush=True)
    
except Exception as e:
    print(f"✗ Error: {e}", flush=True)
    import traceback
    traceback.print_exc()
    sys.exit(1)



