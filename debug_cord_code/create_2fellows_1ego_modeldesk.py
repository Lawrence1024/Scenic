#!/usr/bin/env python3
"""
Create a ModelDesk scenario with 1 ego and 2 fellows (standalone, no Scenic/dSPACE run).

- Ego: configured via Maneuver API at (s_ego, t_ego).
- Fellow 1 and 2: same s value, different t values (side-by-side).

Usage: run with ModelDesk open and a project/experiment active that has a scenario
       with an ego maneuver (e.g. LagunaSeca_ExternalControl).
"""

import sys
import os
import time
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:
        pass
if sys.stderr.encoding != 'utf-8':
    try:
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

# Add Scenic src so we can use dspace legacy helpers
scenic_path = Path(__file__).resolve().parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch

try:
    from scenic.simulators.dspace.utils import legacy as dutils
except ImportError:
    # Fallback: try geometry utils
    sys.path.insert(0, str(scenic_path))
    from scenic.simulators.dspace.geometry.utils import (
        clear_collection,
        ensure_two_segments,
        configure_seg0_absolute_pose,
        configure_seg1_motion,
        make_endless_transition,
        activate_type,
        set_activity_constant,
    )
    class _dutils:
        clear_collection = clear_collection
        ensure_two_segments = ensure_two_segments
        configure_seg0_absolute_pose = configure_seg0_absolute_pose
        configure_seg1_motion = configure_seg1_motion
        make_endless_transition = make_endless_transition
        activate_type = activate_type
        set_activity_constant = set_activity_constant
    dutils = _dutils

# --- Configuration: (s, t) for ego and fellows ---
# Fellows share the same s, different t (e.g. left/right of centerline)
S_EGO = 400.0
T_EGO = 0.0

S_FELLOWS = 500.0   # same s for both
T_FELLOW_1 = -1.5   # left of centerline (m)
T_FELLOW_2 = 1.5    # right of centerline (m)

ROUTE = "R2"        # Lap route
SOURCE_SCENARIO = "LagunaSeca_ExternalControl"
NEW_SCENARIO_NAME = "Scenic_2fellows_1ego"


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("Connecting to ModelDesk...")
    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    if proj is None:
        raise RuntimeError("Open a ModelDesk project first.")
    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("Activate an experiment in ModelDesk.")
    print(f"  Project: {getattr(proj, 'Name', '?')}, Experiment: {getattr(exp, 'Name', '?')}")
    return app, proj, exp


def copy_scenario(app, exp, source_name, new_name):
    """Activate source scenario, save as new name, activate the copy."""
    if source_name:
        try:
            exp.ActivateTrafficScenario(source_name)
            print(f"  Activated source: {source_name}")
        except Exception as e:
            print(f"  Warning: could not activate '{source_name}': {e}")
    try:
        exp.TrafficScenario.SaveAs(new_name, True)
        print(f"  Saved copy as: {new_name}")
    except Exception as e:
        print(f"  Warning: SaveAs failed: {e}")
    try:
        exp.ActivateTrafficScenario(new_name)
    except Exception as e:
        print(f"  Warning: could not activate new scenario: {e}")
    pythoncom.PumpWaitingMessages()
    time.sleep(0.2)
    proj = app.ActiveProject
    exp = proj.ActiveExperiment
    ts = exp.TrafficScenario
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")
    return ts


def configure_ego(ts, s_val, t_val, route_name=ROUTE):
    """Set ego position (s, t) and route via Maneuver API."""
    print("Configuring ego...")
    mc = ts.Maneuver
    if mc.Count == 0:
        print("  Warning: No Maneuver in scenario; ego not configured.")
        return
    ego_maneuver = mc.Item(0)
    try:
        ego_maneuver.Enabled = True
    except Exception:
        pass
    seqs = ego_maneuver.Sequences
    if seqs.Count == 0:
        print("  Warning: No sequences in ego maneuver.")
        return
    seq = seqs.Item(0)

    # Route
    try:
        route_sel = seq.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0
        route_sel.Activate(route_name)
        print(f"  Ego route: {route_name}")
    except Exception as e:
        print(f"  Warning: set route: {e}")

    # Position: StartPosition (longitudinal)
    seq.StartPosition = float(s_val)
    seq.InitialLongitudinalVelocity = 0.0
    print(f"  Ego s={s_val}, t={t_val}")

    # Segment 0 start position (if available)
    segs = seq.Segments
    if segs.Count > 0:
        seg0 = segs.Item(0)
        if hasattr(seg0, 'StartPosition'):
            try:
                seg0.StartPosition = float(s_val)
            except Exception:
                pass
        # Lateral (t) on segment 0
        if abs(t_val) > 0.01:
            try:
                lat0 = seg0.Activity.LateralType
                dutils.activate_type(lat0, "Deviation")
                dep = getattr(lat0.ActiveElement, "DependencyType", None)
                if dep is not None:
                    dutils.activate_type(dep, "Absolute")
                dutils.set_activity_constant(lat0, t_val)
            except Exception as e:
                print(f"  Warning: ego lateral t: {e}")


def add_fellow(ts, name, s_val, t_val, route_name=ROUTE):
    """Add one fellow with given (s, t) and route."""
    print(f"  Adding {name} at s={s_val}, t={t_val}...")
    F = ts.Fellows.Add()
    try:
        F.Name = name
    except Exception:
        F.Name = f"Fellow_{ts.Fellows.Count}"
    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"    Warning: seg1 motion: {e}")
    try:
        dutils.make_endless_transition(segs)
    except Exception:
        pass
    try:
        route_sel = S1.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0
        route_sel.Activate(route_name)
    except Exception as e:
        print(f"    Warning: set route: {e}")


def main():
    print("=" * 60)
    print("Create ModelDesk scenario: 1 ego + 2 fellows (same s, different t)")
    print("=" * 60)

    app, proj, exp = connect_to_modeldesk()

    print("\nCopying scenario...")
    ts = copy_scenario(app, exp, SOURCE_SCENARIO, NEW_SCENARIO_NAME)

    print("\nClearing existing fellows...")
    dutils.clear_collection(ts.Fellows)
    print(f"  Fellows count: {ts.Fellows.Count}")

    configure_ego(ts, S_EGO, T_EGO, ROUTE)

    print("\nAdding fellows (same s, different t)...")
    add_fellow(ts, "Fellow_1", S_FELLOWS, T_FELLOW_1, ROUTE)
    add_fellow(ts, "Fellow_2", S_FELLOWS, T_FELLOW_2, ROUTE)

    print("\nSaving scenario...")
    try:
        ts.Save()
        print("  Saved.")
    except Exception as e:
        print(f"  Save error: {e}")
        return 1

    print("\nDone.")
    print(f"  Scenario: {getattr(ts, 'Name', NEW_SCENARIO_NAME)}")
    print(f"  Ego: s={S_EGO}, t={T_EGO}")
    print(f"  Fellow_1: s={S_FELLOWS}, t={T_FELLOW_1}")
    print(f"  Fellow_2: s={S_FELLOWS}, t={T_FELLOW_2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
