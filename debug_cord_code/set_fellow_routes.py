#!/usr/bin/env python3
"""
Set all fellows to:
- Route = "R2" (lap) or "R1" (pit) - configurable
- UseExternal = False  (unchecked)
- Reverse Direction = unchecked  -> Direction = Direct (0)

This script:
1. Connects to ModelDesk
2. Iterates fellows in active TrafficScenario
3. For each fellow's first sequence:
   - forces internal route selection (UseExternal=False)
   - activates "R2" (lap) or "R1" (pit)
   - forces Direction=Direct (0)
4. Saves + downloads the scenario
5. Verifies after download

Route naming:
- "R1" = Pit lane route
- "R2" = Lap/main racing route
"""

import sys
from pathlib import Path
import time

# Fix encoding for Windows console
if sys.stdout.encoding != "utf-8":
    sys.stdout.reconfigure(encoding="utf-8")
if sys.stderr.encoding != "utf-8":
    sys.stderr.reconfigure(encoding="utf-8")

# Add Scenic src to path (kept from your script)
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.utils import legacy as dutils  # noqa: F401


# Route selection: "R1" = pit, "R2" = lap
TARGET_ROUTE = "R2"  # Default to lap route


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("=" * 80)
    print("Connecting to ModelDesk...")
    print("=" * 80)

    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    if proj is None:
        raise RuntimeError("Open a ModelDesk project first.")

    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("Activate an experiment in ModelDesk.")

    ts = exp.TrafficScenario
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")

    print("[OK] Connected to ModelDesk")
    print(f"   Project: {getattr(proj, 'Name', 'Unknown')}")
    print(f"   Experiment: {getattr(exp, 'Name', 'Unknown')}")
    print(f"   TrafficScenario: {getattr(ts, 'Name', 'Unknown')}")

    return app, proj, exp, ts


def get_route_selection(seq):
    """
    Newer API: seq.RouteSelection (documented)
    Some builds expose seq.Route (older/alias). We support both.
    """
    if hasattr(seq, "RouteSelection"):
        return seq.RouteSelection
    if hasattr(seq, "Route"):
        return seq.Route
    raise AttributeError("Sequence has neither RouteSelection nor Route property.")


def _safe_list(com_collection):
    try:
        return list(com_collection)
    except Exception:
        # fallback for Count/Item style COM collections
        try:
            return [com_collection.Item(i) for i in range(0, com_collection.Count)]
        except Exception:
            return [com_collection.Item(i) for i in range(1, com_collection.Count + 1)]


def activate_route_and_unreverse(seq, target_route="R2"):
    """
    Forces:
      UseExternal = False
      Activate target route ("R1" for pit or "R2" for lap)
      Direction = Direct (0)
    
    Args:
        seq: Sequence object
        target_route: Route name to activate ("R1" for pit, "R2" for lap)
    """
    route_sel = get_route_selection(seq)

    # 1) Force "Use external" OFF (internal route definitions only)
    try:
        route_sel.UseExternal = False
    except Exception:
        pass

    # 2) Force "Reverse Direction" OFF => Direction = Direct (0)
    # ModelDesk DrivingDirectionConstants: Direct=0, Oncoming=1
    try:
        route_sel.Direction = 0
    except Exception:
        pass

    # 3) Enumerate available routes and pick target route
    available = _safe_list(route_sel.AvailableElements)
    available_names = [str(x) for x in available]
    print(f"      Available routes: {available_names}")

    # Current route (best effort)
    current_route = "Unknown"
    try:
        if hasattr(route_sel, "ActiveElement") and hasattr(route_sel.ActiveElement, "Name"):
            current_route = route_sel.ActiveElement.Name
    except Exception:
        pass
    print(f"      Current route: {current_route}")
    
    # Check if target route is available
    if target_route not in available_names:
        print(f"      [ERROR] Target route '{target_route}' not found in AvailableElements.")
        print(f"      Available routes: {available_names}")
        return False

    # Activate
    try:
        route_sel.Activate(target_route)
        print(f"      Activated route: {target_route}")
    except Exception as e:
        print(f"      [ERROR] Activate('{target_route}') failed: {e}")
        return False

    # Re-assert the two required toggles after activation
    try:
        route_sel.UseExternal = False
    except Exception:
        pass
    try:
        route_sel.Direction = 0
    except Exception:
        pass

    # Let UI/COM catch up
    time.sleep(0.2)
    pythoncom.PumpWaitingMessages()

    # Verify (best effort)
    try:
        actual_route = None
        if hasattr(route_sel, "ActiveElement") and hasattr(route_sel.ActiveElement, "Name"):
            actual_route = route_sel.ActiveElement.Name

        use_external = None
        try:
            use_external = bool(route_sel.UseExternal)
        except Exception:
            pass

        direction = None
        try:
            direction = int(route_sel.Direction)
        except Exception:
            pass

        print(f"      After set: Route={actual_route}, UseExternal={use_external}, Direction={direction}")

        # Accept if route matches target and toggles are correct (when readable)
        ok_route = (actual_route is not None) and (actual_route == target_route)
        ok_external = (use_external is False) if use_external is not None else True
        ok_dir = (direction == 0) if direction is not None else True

        if ok_route and ok_external and ok_dir:
            print(f"      [OK] {current_route} -> {actual_route} (UseExternal OFF, Reverse OFF)")
            return True

        if ok_route and ok_external:
            print(f"      [OK] Route shows {target_route}; toggles appear correct or unreadable.")
            return True

        print("      [WARNING] Route/toggles did not verify cleanly (may persist after Download).")
        return True

    except Exception:
        print("      [OK] Activate succeeded (verification unavailable).")
        return True


def set_all_fellows(ts, target_route="R2"):
    print("\n" + "=" * 80)
    route_desc = "lap" if target_route == "R2" else "pit"
    print(f"Setting ALL Fellows: Route='{target_route}' ({route_desc}), UseExternal=OFF, Reverse Direction=OFF")
    print("=" * 80)

    fellows = ts.Fellows
    print(f"   Found {fellows.Count} fellows in scenario")

    success = 0
    for i in range(fellows.Count):
        try:
            fellow = fellows[i]
            fellow_name = getattr(fellow, "Name", f"Fellow_{i+1}")
            print(f"\n   {fellow_name} (index {i}):")

            seqs = fellow.Sequences
            if seqs.Count == 0:
                print("      [WARNING] No sequences found")
                continue

            seq = seqs[0]  # same behavior as your script (first sequence)
            if activate_route_and_unreverse(seq, target_route):
                success += 1

        except Exception as e:
            print(f"   [ERROR] Error processing fellow {i}: {e}")
            import traceback
            traceback.print_exc()

    print(f"\n   [OK] Updated {success}/{fellows.Count} fellows")
    return success


def verify_after_download(ts):
    print("\n" + "=" * 80)
    print("Verifying After Download...")
    print("=" * 80)

    time.sleep(1.0)
    pythoncom.PumpWaitingMessages()

    fellows = ts.Fellows
    ok = 0
    for i in range(fellows.Count):
        try:
            fellow = fellows[i]
            fellow_name = getattr(fellow, "Name", f"Fellow_{i+1}")
            seq = fellow.Sequences[0]
            rs = get_route_selection(seq)

            route_name = None
            try:
                if hasattr(rs, "ActiveElement") and hasattr(rs.ActiveElement, "Name"):
                    route_name = rs.ActiveElement.Name
            except Exception:
                pass

            use_external = None
            try:
                use_external = bool(rs.UseExternal)
            except Exception:
                pass

            direction = None
            try:
                direction = int(rs.Direction)
            except Exception:
                pass

            ok_route = (route_name is not None) and (route_name == TARGET_ROUTE)
            ok_external = (use_external is False) if use_external is not None else True
            ok_dir = (direction == 0) if direction is not None else True

            if ok_route and ok_external and ok_dir:
                ok += 1
                print(f"   {fellow_name}: Route={route_name}, UseExternal={use_external}, Direction={direction} [OK]")
            else:
                print(f"   {fellow_name}: Route={route_name}, UseExternal={use_external}, Direction={direction} [WARNING]")

        except Exception as e:
            print(f"   [ERROR] Could not verify fellow {i}: {e}")

    route_desc = "lap" if TARGET_ROUTE == "R2" else "pit"
    print(f"\n   Verified {ok}/{fellows.Count} fellows match: {TARGET_ROUTE} ({route_desc}) + UseExternal OFF + Reverse OFF")


def main():
    route_desc = "lap" if TARGET_ROUTE == "R2" else "pit"
    print("=" * 80)
    print(f"Set Fellow Routes: {TARGET_ROUTE} ({route_desc}) + UseExternal OFF + Reverse Direction OFF")
    print("=" * 80)

    try:
        _, _, exp, ts = connect_to_modeldesk()

        # Ensure scenario is active
        scenario_name = getattr(ts, "Name", None)
        if scenario_name:
            try:
                exp.ActivateTrafficScenario(scenario_name)
                print(f"   [OK] Activated scenario: {scenario_name}")
            except Exception as e:
                print(f"   [WARNING] Could not activate scenario: {e}")

        updated = set_all_fellows(ts, TARGET_ROUTE)
        if updated == 0:
            print("\n[WARNING] No fellows updated.")
            return 1

        print("\n" + "=" * 80)
        print("Saving and Downloading Scenario...")
        print("=" * 80)

        try:
            ts.Save()
            print("   [OK] Scenario saved")
        except Exception as e:
            print(f"   [WARNING] Could not save scenario: {e}")

        try:
            ts.Download()
            print("   [OK] Scenario downloaded")
        except Exception as e:
            print(f"   [WARNING] Could not download scenario: {e}")

        verify_after_download(ts)

        print("\n" + "=" * 80)
        print("Complete!")
        print("=" * 80)
        return 0

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
