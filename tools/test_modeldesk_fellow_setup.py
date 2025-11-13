# -*- coding: utf-8 -*-
"""Test script to configure ModelDesk fellow with External control setup.

This script demonstrates how to configure a fellow vehicle in ModelDesk with:
- Segment 0: Position (constant) + Lane selection (constant)
- Segment 1: Velocity (Extern) + Lane selection (Extern)

This matches the configuration shown in the ModelDesk UI where the Type property
is set to "Extern" for both longitudinal and lateral controls in segment 1.

Usage:
  python tools/test_modeldesk_fellow_setup.py [--fellow-name F1] [--inspect]

Prerequisites:
  - ModelDesk application must be open
  - Active project and experiment must be loaded
  - Traffic scenario must exist
"""

import sys
import os
import argparse

# Fix Windows console encoding for Unicode characters
if sys.platform == 'win32':
    try:
        import io
        if hasattr(sys.stdout, 'buffer'):
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        if hasattr(sys.stderr, 'buffer'):
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
    except:
        pass  # Fallback to default encoding

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import pythoncom
from win32com.client import Dispatch

# Import helper functions from geometry utils
from scenic.simulators.dspace.geometry.utils import (
    ensure_two_segments,
    activate_type,
    set_activity_constant,
    make_endless_transition,
    _count_any
)


def log(level: str, msg: str):
    """Simple logging function."""
    print(f"[{level}] {msg}")


def info(msg: str):
    log("INFO", msg)


def warn(msg: str):
    log("WARN", msg)


def err(msg: str):
    log("ERROR", msg)


def connect_modeldesk():
    """Connect to ModelDesk COM application."""
    try:
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
            raise RuntimeError("Traffic scenario not found.")
        info("Connected to ModelDesk")
        return app, proj, exp, ts
    except Exception as e:
        err(f"Failed to connect to ModelDesk: {e}")
        raise


def inspect_type_property_path(seg, segment_name="Segment"):
    """Debug helper to find where the Type='Extern' property is located."""
    print(f"\n=== Inspecting {segment_name} Type Property Path ===")
    
    # Longitudinal
    try:
        lt = seg.Activity.LongitudinalType
        print(f"LongitudinalType: {lt}")
        elem = lt.ActiveElement
        print(f"  ActiveElement: {elem}")
        print(f"  ActiveElement type: {type(elem)}")
        
        # Get all attributes (non-private)
        attrs = [x for x in dir(elem) if not x.startswith('_')]
        print(f"  ActiveElement attributes: {attrs[:20]}...")  # First 20
        
        # Check for Type directly
        if hasattr(elem, "Type"):
            type_obj = elem.Type
            print(f"  ✓ Found Type: {type_obj}")
            print(f"    Type type: {type(type_obj)}")
            if hasattr(type_obj, "AvailableElements"):
                avail = list(type_obj.AvailableElements)
                print(f"    Available Types: {avail}")
            # Try to get current value
            try:
                if hasattr(type_obj, "ActiveElement"):
                    print(f"    Current Type: {type_obj.ActiveElement}")
                elif hasattr(type_obj, "Value"):
                    print(f"    Current Type Value: {type_obj.Value}")
            except:
                pass
        
        # Check for Properties
        if hasattr(elem, "Properties"):
            props = elem.Properties
            print(f"  ✓ Found Properties: {props}")
            props_attrs = [x for x in dir(props) if not x.startswith('_')]
            print(f"  Properties attributes: {props_attrs[:20]}...")
            if hasattr(props, "Type"):
                print(f"    Properties.Type: {props.Type}")
                if hasattr(props.Type, "AvailableElements"):
                    print(f"      Available: {list(props.Type.AvailableElements)}")
    except Exception as e:
        print(f"  Error inspecting longitudinal: {e}")
        import traceback
        traceback.print_exc()
    
    # Lateral (same structure)
    try:
        lat = seg.Activity.LateralType
        print(f"\nLateralType: {lat}")
        elem = lat.ActiveElement
        print(f"  ActiveElement: {elem}")
        
        if hasattr(elem, "Type"):
            type_obj = elem.Type
            print(f"  ✓ Found Type: {type_obj}")
            if hasattr(type_obj, "AvailableElements"):
                print(f"    Available Types: {list(type_obj.AvailableElements)}")
        if hasattr(elem, "Properties"):
            props = elem.Properties
            print(f"  ✓ Found Properties: {props}")
            if hasattr(props, "Type"):
                print(f"    Properties.Type: {props.Type}")
    except Exception as e:
        print(f"  Error inspecting lateral: {e}")


def configure_seg0_lane_selection(segs, s: float, lane_index: int):
    """Configure segment 0 with Position (longitudinal) and Lane selection (lateral).
    
    Args:
        segs: Segments collection (segs[0] will be configured)
        s: Initial position along road (meters)
        lane_index: Lane index (0 = center lane, typically)
    """
    info(f"Configuring Segment 0: Position={s}m, Lane={lane_index}")
    
    # Longitudinal: "Position" (absolute along reference line)
    lt0 = segs[0].Activity.LongitudinalType
    if not activate_type(lt0, "Position"):
        if not activate_type(lt0, "DistanceMeter"):
            raise RuntimeError("seg0.LongitudinalType 'Position' not available.")
    set_activity_constant(lt0, s)
    info("  [OK] Set longitudinal to Position (constant)")
    
    # Lateral: "Lane selection" with constant lane index
    lat0 = segs[0].Activity.LateralType
    if not activate_type(lat0, "Lane selection"):
        if not activate_type(lat0, "Lane"):
            warn("  Could not activate 'Lane selection', trying 'Deviation' as fallback")
            activate_type(lat0, "Deviation")
            # Fallback to deviation if lane selection not available
            dep = getattr(lat0.ActiveElement, "DependencyType", None)
            if dep is not None:
                activate_type(dep, "Absolute")
            set_activity_constant(lat0, lane_index * 3.5)  # Approximate: 3.5m per lane
            info("  [OK] Set lateral to Deviation (fallback)")
            return
    
    # Set lane index constant
    try:
        # Navigate to the constant value: ActiveElement -> SourceType -> ActiveElement -> Constant
        tgt = lat0.ActiveElement
        if hasattr(tgt, "SourceType"):
            tgt = tgt.SourceType
            if hasattr(tgt, "ActiveElement"):
                tgt = tgt.ActiveElement
                tgt.Constant = int(lane_index)
                info(f"  [OK] Set lateral to Lane selection (constant lane {lane_index})")
            else:
                # Try direct constant
                tgt.Constant = int(lane_index)
                info(f"  [OK] Set lateral to Lane selection (constant lane {lane_index})")
        else:
            # Try direct constant on ActiveElement
            lat0.ActiveElement.Constant = int(lane_index)
            info(f"  ✓ Set lateral to Lane selection (constant lane {lane_index})")
    except Exception as e:
        warn(f"  Could not set lane index constant: {e}")
        # Try alternative path
        try:
            lat0.ActiveElement.Constant = int(lane_index)
            info(f"  [OK] Set lateral constant (alternative path)")
        except Exception as e2:
            warn(f"  Alternative path also failed: {e2}")


def set_type_to_extern(obj, property_name="Type", context="unknown"):
    """Set Type property to 'Extern' on an object.
    
    Args:
        obj: The COM object that has a Type property
        property_name: Name of the property (usually "Type")
        context: Context string for logging
    
    Returns:
        True if successful, False otherwise
    """
    if not hasattr(obj, property_name):
        return False
    
    type_obj = getattr(obj, property_name)
    
    # Method 1: Activate if it's an activatable enum (most common for ModelDesk dropdowns)
    # This should be tried first as ModelDesk typically uses Activate for dropdown selections
    if hasattr(type_obj, "Activate"):
        # Try "Extern" first
        if activate_type(type_obj, "Extern"):
            info(f"  [OK] Activated {context} Type to 'Extern'")
            return True
        # Try "External" spelling
        elif activate_type(type_obj, "External"):
            info(f"  [OK] Activated {context} Type to 'External'")
            return True
    
    # Method 2: Check if it has AvailableElements and find the right one
    if hasattr(type_obj, "AvailableElements"):
        try:
            avail = list(type_obj.AvailableElements)
            avail_str = [str(x) for x in avail]
            info(f"  Available Types for {context}: {avail_str}")
            
            # Try different spellings - prioritize "Extern"
            for name in ["Extern", "External", "Extern.", "External.", "Extern "]:
                if name in avail_str:
                    if hasattr(type_obj, "Activate"):
                        type_obj.Activate(name)
                        info(f"  [OK] Activated {context} Type to '{name}'")
                        return True
                    elif hasattr(type_obj, "Value"):
                        type_obj.Value = name
                        info(f"  [OK] Set {context} Type Value to '{name}'")
                        return True
        except Exception as e:
            warn(f"  Error checking AvailableElements: {e}")
    
    # Method 3: Try setting via Value property
    if hasattr(type_obj, "Value"):
        try:
            type_obj.Value = "Extern"
            info(f"  [OK] Set {context} Type Value to 'Extern'")
            return True
        except:
            try:
                type_obj.Value = "External"
                info(f"  [OK] Set {context} Type Value to 'External'")
                return True
            except:
                pass
    
    # Method 4: Direct property assignment (last resort)
    try:
        setattr(obj, property_name, "Extern")
        info(f"  [OK] Set {context} Type to 'Extern' (direct assignment)")
        return True
    except:
        pass
    
    # Method 5: Try "External" spelling (last resort)
    try:
        setattr(obj, property_name, "External")
        info(f"  [OK] Set {context} Type to 'External' (direct assignment)")
        return True
    except:
        pass
    
    return False


def configure_seg1_external_control(segs):
    """Configure segment 1 with Type='Extern' for both longitudinal and lateral.
    
    This enables External Signals control via ControlDesk.
    
    Args:
        segs: Segments collection (segs[1] will be configured)
    """
    info("Configuring Segment 1: External control (Type='Extern')")
    
    seg1 = segs[1]
    
    # Longitudinal: Set to Velocity, then set Type='Extern'
    lt1 = seg1.Activity.LongitudinalType
    if not activate_type(lt1, "Velocity"):
        if not activate_type(lt1, "Speed"):
            raise RuntimeError("seg1.LongitudinalType 'Velocity' not available.")
    info("  [OK] Set longitudinal activity to Velocity")
    
    # Set Type to "Extern" for longitudinal
    # The Type property on ActiveElement is read-only (returns integer enum)
    # We need to activate SourceType to "Extern" to set the Type
    long_elem = lt1.ActiveElement
    success = False
    
    # Method 1: Activate SourceType to "Extern" (this is the correct way!)
    if hasattr(long_elem, "SourceType"):
        source_type = long_elem.SourceType
        if hasattr(source_type, "Activate"):
            if activate_type(source_type, "Extern"):
                info(f"  [OK] Activated SourceType to 'Extern'")
                success = True
            elif activate_type(source_type, "External"):
                info(f"  [OK] Activated SourceType to 'External'")
                success = True
    
    # If that doesn't work, try Type under Properties (some versions might have it there)
    if not success and hasattr(long_elem, "Properties"):
        props = long_elem.Properties
        if set_type_to_extern(props, "Type", "longitudinal (Properties)"):
            success = True
    
    # Try navigating deeper into Properties if it exists
    if not success and hasattr(long_elem, "Properties"):
        props = long_elem.Properties
        # Check if Properties has sub-properties
        for attr in ["LongitudinalProperties", "Properties", "Type"]:
            if hasattr(props, attr):
                sub_obj = getattr(props, attr)
                if set_type_to_extern(sub_obj, "Type", f"longitudinal (Properties.{attr})"):
                    success = True
                    break
    
    if not success:
        warn("  [WARN] Could not set longitudinal Type to 'Extern' - inspecting object structure...")
        # Debug: show what's available
        try:
            info(f"  ActiveElement type: {type(long_elem)}")
            info(f"  ActiveElement attributes: {[x for x in dir(long_elem) if not x.startswith('_')][:20]}")
            # Check Type property details
            if hasattr(long_elem, "Type"):
                type_val = long_elem.Type
                info(f"  Type property value: {type_val} (type: {type(type_val)})")
                # Check if we can set it directly
                try:
                    long_elem.Type = "Extern"
                    info(f"  [OK] Set Type directly to 'Extern'")
                    success = True
                except Exception as e:
                    info(f"  Cannot set Type directly: {e}")
                    # Try integer enum value if Type is an int
                    if isinstance(type_val, int):
                        info(f"  Type is integer enum - trying to find enum mapping...")
            # Try to find a method to set Type
            for method_name in ["SetType", "SetTypeValue", "ActivateType"]:
                if hasattr(long_elem, method_name):
                    info(f"  Found method: {method_name}")
                    try:
                        method = getattr(long_elem, method_name)
                        method("Extern")
                        info(f"  [OK] Called {method_name}('Extern')")
                        success = True
                        break
                    except Exception as e:
                        info(f"  Method {method_name} failed: {e}")
            # Try SourceType - it might be the selection object we need to activate
            if not success and hasattr(long_elem, "SourceType"):
                source_type = long_elem.SourceType
                info(f"  Checking SourceType: {type(source_type)}")
                # Try activating SourceType to "Extern" directly
                if hasattr(source_type, "Activate"):
                    if activate_type(source_type, "Extern"):
                        info(f"  [OK] Activated SourceType to 'Extern'")
                        success = True
                    elif activate_type(source_type, "External"):
                        info(f"  [OK] Activated SourceType to 'External'")
                        success = True
                # Or try SourceType.Type
                if not success and hasattr(source_type, "Type"):
                    if set_type_to_extern(source_type, "Type", "longitudinal (SourceType.Type)"):
                        success = True
            if hasattr(long_elem, "Properties"):
                props = long_elem.Properties
                info(f"  Properties type: {type(props)}")
                info(f"  Properties attributes: {[x for x in dir(props) if not x.startswith('_')][:20]}")
                if hasattr(props, "Type"):
                    type_obj = props.Type
                    info(f"  Properties.Type found! Type: {type(type_obj)}")
                    if hasattr(type_obj, "AvailableElements"):
                        avail = list(type_obj.AvailableElements)
                        info(f"  Available Type values: {avail}")
                    # Try to get current value
                    try:
                        if hasattr(type_obj, "ActiveElement"):
                            info(f"  Current Type value: {type_obj.ActiveElement}")
                        elif hasattr(type_obj, "Value"):
                            info(f"  Current Type value: {type_obj.Value}")
                    except:
                        pass
                else:
                    warn("  Properties.Type not found - checking for alternative property names...")
                    # Check for common alternatives
                    for alt_name in ["ControlType", "InputType", "SourceType", "Mode", "TypeEnum"]:
                        if hasattr(props, alt_name):
                            info(f"  Found alternative: Properties.{alt_name}")
            else:
                warn("  ActiveElement.Properties not found")
        except Exception as e:
            warn(f"  Error during inspection: {e}")
            import traceback
            traceback.print_exc()
    
    # Lateral: Set to Lane selection, then set Type='Extern'
    lat1 = seg1.Activity.LateralType
    if not activate_type(lat1, "Lane selection"):
        if not activate_type(lat1, "Lane"):
            warn("  Could not activate 'Lane selection', trying 'Deviation' as fallback")
            if not activate_type(lat1, "Deviation"):
                warn("  Could not activate 'Deviation', using 'Continue' as fallback")
                activate_type(lat1, "Continue")
                info("  [OK] Set lateral activity to Continue (fallback)")
                # Even with Continue, try to set SourceType to Extern if possible
                lat_elem = lat1.ActiveElement
                if hasattr(lat_elem, "SourceType"):
                    source_type = lat_elem.SourceType
                    if hasattr(source_type, "Activate"):
                        if activate_type(source_type, "Extern") or activate_type(source_type, "External"):
                            info("  [OK] Activated lateral SourceType to 'Extern' (even with Continue)")
                return  # Continue doesn't support external control well, but we tried
    
    info("  [OK] Set lateral activity to Lane selection")
    
    # Set Type to "Extern" for lateral
    # The Type property on ActiveElement is read-only (returns integer enum)
    # We need to activate SourceType to "Extern" to set the Type
    lat_elem = lat1.ActiveElement
    success = False
    
    # Method 1: Activate SourceType to "Extern" (this is the correct way!)
    if hasattr(lat_elem, "SourceType"):
        source_type = lat_elem.SourceType
        if hasattr(source_type, "Activate"):
            if activate_type(source_type, "Extern"):
                info(f"  [OK] Activated SourceType to 'Extern'")
                success = True
            elif activate_type(source_type, "External"):
                info(f"  [OK] Activated SourceType to 'External'")
                success = True
    
    # Method 2: Try accessing Type through the parent LateralType object (fallback)
    if not success and hasattr(lat1, "Type"):
        type_selection = lat1.Type
        if set_type_to_extern(lat1, "Type", "lateral (LateralType.Type)"):
            success = True
    
    # If that doesn't work, try Type under Properties (some versions might have it there)
    if not success and hasattr(lat_elem, "Properties"):
        props = lat_elem.Properties
        if set_type_to_extern(props, "Type", "lateral (Properties)"):
            success = True
    
    # Try navigating deeper into Properties if it exists
    if not success and hasattr(lat_elem, "Properties"):
        props = lat_elem.Properties
        # Check if Properties has sub-properties
        for attr in ["LateralProperties", "Properties", "Type"]:
            if hasattr(props, attr):
                sub_obj = getattr(props, attr)
                if set_type_to_extern(sub_obj, "Type", f"lateral (Properties.{attr})"):
                    success = True
                    break
    
    if not success:
        warn("  [WARN] Could not set lateral Type to 'Extern' - inspecting object structure...")
        # Debug: show what's available
        try:
            info(f"  ActiveElement type: {type(lat_elem)}")
            info(f"  ActiveElement attributes: {[x for x in dir(lat_elem) if not x.startswith('_')][:20]}")
            if hasattr(lat_elem, "Properties"):
                props = lat_elem.Properties
                info(f"  Properties type: {type(props)}")
                info(f"  Properties attributes: {[x for x in dir(props) if not x.startswith('_')][:20]}")
                if hasattr(props, "Type"):
                    type_obj = props.Type
                    info(f"  Properties.Type found! Type: {type(type_obj)}")
                    if hasattr(type_obj, "AvailableElements"):
                        avail = list(type_obj.AvailableElements)
                        info(f"  Available Type values: {avail}")
                    # Try to get current value
                    try:
                        if hasattr(type_obj, "ActiveElement"):
                            info(f"  Current Type value: {type_obj.ActiveElement}")
                        elif hasattr(type_obj, "Value"):
                            info(f"  Current Type value: {type_obj.Value}")
                    except:
                        pass
                else:
                    warn("  Properties.Type not found - checking for alternative property names...")
                    # Check for common alternatives
                    for alt_name in ["ControlType", "InputType", "SourceType", "Mode", "TypeEnum"]:
                        if hasattr(props, alt_name):
                            info(f"  Found alternative: Properties.{alt_name}")
            else:
                warn("  ActiveElement.Properties not found")
        except Exception as e:
            warn(f"  Error during inspection: {e}")
            import traceback
            traceback.print_exc()
    
    # Make segment endless
    try:
        make_endless_transition(segs)
        info("  [OK] Set segment transition to Endless")
    except Exception as e:
        warn(f"  Could not set endless transition: {e}")


def configure_fellow(ts, fellow_name="F1", s_position=50.0, lane_index=0, inspect_only=False):
    """Configure a fellow vehicle with the desired setup.
    
    Args:
        ts: TrafficScenario COM object
        fellow_name: Name of the fellow (e.g., "F1")
        s_position: Initial position along road (meters)
        lane_index: Initial lane index
        inspect_only: If True, only inspect without making changes
    """
    info(f"\n{'='*70}")
    info(f"Configuring Fellow: {fellow_name}")
    info(f"{'='*70}")
    
    # Get or create fellow
    try:
        fellow = ts.Fellows.Item(fellow_name)
        info(f"Found existing fellow: {fellow_name}")
    except:
        if inspect_only:
            err(f"Fellow '{fellow_name}' not found. Create it first or use a different name.")
            return
        fellow = ts.Fellows.Add()
        fellow.Name = fellow_name
        info(f"Created new fellow: {fellow_name}")
    
    # Get sequences
    sequences = fellow.Sequences
    if sequences.Count == 0:
        if inspect_only:
            err("No sequences found. Create at least one sequence first.")
            return
        seq = sequences.Add()
        info("Created new sequence")
    else:
        seq = sequences.Item(0)  # Use first sequence
        info(f"Using existing sequence (index 0)")
    
    # Ensure two segments
    segs = ensure_two_segments(seq)
    info(f"Ensured 2 segments exist (currently have {segs.Count})")
    
    if inspect_only:
        info("\n=== INSPECT MODE - No changes will be made ===")
        inspect_type_property_path(segs[0], "Segment 0")
        inspect_type_property_path(segs[1], "Segment 1")
        return
    
    # Configure Segment 0: Position + Lane selection (constant)
    try:
        configure_seg0_lane_selection(segs, s_position, lane_index)
    except Exception as e:
        err(f"Failed to configure Segment 0: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Configure Segment 1: Velocity (Extern) + Lane selection (Extern)
    try:
        configure_seg1_external_control(segs)
    except Exception as e:
        err(f"Failed to configure Segment 1: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Set route UseExternal if available
    try:
        route_sel = seq.Route
        if hasattr(route_sel, "UseExternal"):
            route_sel.UseExternal = True
            info("  [OK] Set Route.UseExternal = True")
    except Exception as e:
        warn(f"  Could not set Route.UseExternal: {e}")
    
    info(f"\n[OK] Configuration complete for {fellow_name}")
    info("  Remember to save and download the scenario in ModelDesk!")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Test ModelDesk fellow configuration with External control"
    )
    parser.add_argument(
        "--fellow-name",
        default="F1",
        help="Name of the fellow to configure (default: F1)"
    )
    parser.add_argument(
        "--position",
        type=float,
        default=50.0,
        help="Initial position along road in meters (default: 50.0)"
    )
    parser.add_argument(
        "--lane",
        type=int,
        default=0,
        help="Initial lane index (default: 0)"
    )
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="Only inspect the structure without making changes"
    )
    
    args = parser.parse_args()
    
    try:
        # Connect to ModelDesk
        app, proj, exp, ts = connect_modeldesk()
        
        # Configure fellow
        configure_fellow(
            ts,
            fellow_name=args.fellow_name,
            s_position=args.position,
            lane_index=args.lane,
            inspect_only=args.inspect
        )
        
        if not args.inspect:
            info("\n" + "="*70)
            info("Next steps:")
            info("1. Verify the configuration in ModelDesk UI")
            info("2. Check that Segment 1 shows Type='Extern' for both longitudinal and lateral")
            info("3. Save the scenario (ts.Save())")
            info("4. Download to VEOS (ts.Download())")
            info("="*70)
        
    except Exception as e:
        err(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

