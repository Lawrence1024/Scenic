#!/usr/bin/env python3
"""
Test script to verify we can access ModelDesk variables correctly.

This script tests accessing fellow vehicle configuration (routes, s, t values)
using the same methods as the main simulation code.
"""

import sys
import os
from pathlib import Path

# Fix encoding for Windows console
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
if sys.stderr.encoding != 'utf-8':
    sys.stderr.reconfigure(encoding='utf-8')

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.utils import legacy as dutils


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("="*80)
    print("Connecting to ModelDesk...")
    print("="*80)
    
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
    print(f"   Project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
    print(f"   Experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
    print(f"   TrafficScenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
    
    return app, proj, exp, ts


def test_fellow_access_method1(ts):
    """Test accessing fellows using the EXACT same path as set_activity_constant."""
    print("\n" + "="*80)
    print("Method 1: Reading values using same path as set_activity_constant()")
    print("="*80)
    
    try:
        fellows = ts.Fellows
        print(f"   Found {fellows.Count} fellows")
        
        # Try accessing by name (like the actual code does)
        for i in range(min(7, fellows.Count)):
            try:
                # Try by name first
                name = f"Fellow_{i+1}"
                try:
                    fellow = fellows.Item(name)
                except:
                    # Fall back to index (try 0-indexed first)
                    try:
                        fellow = fellows.Item(i)
                    except:
                        fellow = fellows.Item(i + 1)
                
                print(f"\n   Fellow: {fellow.Name if hasattr(fellow, 'Name') else name}")
                
                # Get sequences
                seqs = fellow.Sequences
                print(f"      Sequences count: {seqs.Count}")
                
                if seqs.Count > 0:
                    # Try 0-indexed first
                    try:
                        seq = seqs.Item(0)
                    except:
                        seq = seqs.Item(1)
                    
                    # Get route
                    try:
                        route = seq.Route
                        if hasattr(route, 'ActiveElement') and hasattr(route.ActiveElement, 'Name'):
                            route_name = route.ActiveElement.Name
                            print(f"      Route: {route_name}")
                        else:
                            print(f"      Route: (could not read name)")
                    except Exception as e:
                        print(f"      Route access error: {e}")
                    
                    # Get segments - use the same method as ensure_two_segments
                    segs = seq.Segments
                    print(f"      Segments count: {segs.Count}")
                    
                    if segs.Count > 0:
                        # Access segment 0 - try 0-indexed first
                        try:
                            seg0 = segs.Item(0)
                        except:
                            seg0 = segs.Item(1)
                        
                        # Read s value using EXACT same path as set_activity_constant
                        try:
                            lon_type = seg0.Activity.LongitudinalType
                            print(f"      LongitudinalType: {type(lon_type)}")
                            
                            # Follow the same path: ActiveElement -> SourceType -> ActiveElement -> Constant
                            ae = lon_type.ActiveElement
                            print(f"      ActiveElement: {type(ae)}")
                            
                            if hasattr(ae, 'SourceType'):
                                st = ae.SourceType
                                print(f"      SourceType: {type(st)}")
                                
                                if hasattr(st, 'ActiveElement'):
                                    st_ae = st.ActiveElement
                                    print(f"      SourceType.ActiveElement: {type(st_ae)}")
                                    
                                    if hasattr(st_ae, 'Constant'):
                                        s_val = st_ae.Constant
                                        print(f"      [OK] s (Longitudinal.Constant) = {s_val}")
                                    else:
                                        print(f"      [FAIL] SourceType.ActiveElement has no 'Constant' attribute")
                                        print(f"      Available: {[a for a in dir(st_ae) if not a.startswith('_')]}")
                            else:
                                # Maybe it's directly on ActiveElement?
                                if hasattr(ae, 'Constant'):
                                    s_val = ae.Constant
                                    print(f"      [OK] s (ActiveElement.Constant) = {s_val}")
                                else:
                                    print(f"      [FAIL] ActiveElement has no 'Constant' attribute")
                                    print(f"      Available: {[a for a in dir(ae) if not a.startswith('_')]}")
                                    
                        except Exception as e:
                            print(f"      Error reading s value: {e}")
                            import traceback
                            traceback.print_exc()
                        
                        # Read t value using EXACT same path
                        try:
                            lat_type = seg0.Activity.LateralType
                            print(f"      LateralType: {type(lat_type)}")
                            
                            # Follow the same path: ActiveElement -> SourceType -> ActiveElement -> Constant
                            ae = lat_type.ActiveElement
                            print(f"      ActiveElement: {type(ae)}")
                            
                            if hasattr(ae, 'SourceType'):
                                st = ae.SourceType
                                print(f"      SourceType: {type(st)}")
                                
                                if hasattr(st, 'ActiveElement'):
                                    st_ae = st.ActiveElement
                                    print(f"      SourceType.ActiveElement: {type(st_ae)}")
                                    
                                    if hasattr(st_ae, 'Constant'):
                                        t_val = st_ae.Constant
                                        print(f"      [OK] t (Lateral.Constant) = {t_val}")
                                    else:
                                        print(f"      [FAIL] SourceType.ActiveElement has no 'Constant' attribute")
                                        print(f"      Available: {[a for a in dir(st_ae) if not a.startswith('_')]}")
                            else:
                                # Maybe it's directly on ActiveElement?
                                if hasattr(ae, 'Constant'):
                                    t_val = ae.Constant
                                    print(f"      [OK] t (ActiveElement.Constant) = {t_val}")
                                else:
                                    print(f"      [FAIL] ActiveElement has no 'Constant' attribute")
                                    print(f"      Available: {[a for a in dir(ae) if not a.startswith('_')]}")
                                    
                        except Exception as e:
                            print(f"      Error reading t value: {e}")
                            import traceback
                            traceback.print_exc()
                
            except Exception as e:
                print(f"   Error accessing Fellow {i+1}: {e}")
                import traceback
                traceback.print_exc()
    
    except Exception as e:
        print(f"[ERROR] Error in method 1: {e}")
        import traceback
        traceback.print_exc()


def test_fellow_access_method2(ts):
    """Test accessing fellows using ensure_two_segments (returns list-like)."""
    print("\n" + "="*80)
    print("Method 2: Using ensure_two_segments() to get list-like segments")
    print("="*80)
    
    try:
        fellows = ts.Fellows
        print(f"   Found {fellows.Count} fellows")
        
        # Try to access by name (like the actual code does)
        for i in range(min(7, fellows.Count)):
            try:
                # Try by name first
                name = f"Fellow_{i+1}"
                try:
                    fellow = fellows.Item(name)
                except:
                    # Fall back to index
                    try:
                        fellow = fellows.Item(i)
                    except:
                        fellow = fellows.Item(i + 1)
                
                print(f"\n   Fellow: {fellow.Name if hasattr(fellow, 'Name') else name}")
                
                # Use the same helper functions as the actual code
                seqs = fellow.Sequences
                
                if seqs.Count > 0:
                    # Try 0-indexed first
                    try:
                        S1 = seqs.Item(0)
                    except:
                        S1 = seqs.Item(1)
                    
                    if S1:
                        # Use ensure_two_segments to get the segments collection
                        segs = dutils.ensure_two_segments(S1)
                        print(f"      Segments collection type: {type(segs)}")
                        print(f"      Segments count: {segs.Count if hasattr(segs, 'Count') else 'N/A'}")
                        
                        # Access segment 0 - try both list indexing and Item()
                        try:
                            seg0 = segs[0]  # List-like access
                        except:
                            try:
                                seg0 = segs.Item(0)  # COM Item access
                            except:
                                seg0 = segs.Item(1)
                        
                        print(f"      Segment 0 type: {type(seg0)}")
                        
                        # Read s, t using the same method as configure_seg0_absolute_pose
                        try:
                            # Read longitudinal (s)
                            lt0 = seg0.Activity.LongitudinalType
                            print(f"      LongitudinalType: {type(lt0)}")
                            
                            # Follow set_activity_constant path
                            ae = lt0.ActiveElement
                            if hasattr(ae, 'SourceType'):
                                st = ae.SourceType
                                if hasattr(st, 'ActiveElement'):
                                    st_ae = st.ActiveElement
                                    if hasattr(st_ae, 'Constant'):
                                        s_val = st_ae.Constant
                                        print(f"      [OK] s = {s_val}")
                                    else:
                                        print(f"      [FAIL] No Constant on SourceType.ActiveElement")
                            elif hasattr(ae, 'Constant'):
                                s_val = ae.Constant
                                print(f"      [OK] s = {s_val} (direct on ActiveElement)")
                            else:
                                print(f"      [FAIL] Could not find Constant")
                            
                            # Read lateral (t)
                            lat0 = seg0.Activity.LateralType
                            print(f"      LateralType: {type(lat0)}")
                            
                            # Follow set_activity_constant path
                            ae = lat0.ActiveElement
                            if hasattr(ae, 'SourceType'):
                                st = ae.SourceType
                                if hasattr(st, 'ActiveElement'):
                                    st_ae = st.ActiveElement
                                    if hasattr(st_ae, 'Constant'):
                                        t_val = st_ae.Constant
                                        print(f"      [OK] t = {t_val}")
                                    else:
                                        print(f"      [FAIL] No Constant on SourceType.ActiveElement")
                            elif hasattr(ae, 'Constant'):
                                t_val = ae.Constant
                                print(f"      [OK] t = {t_val} (direct on ActiveElement)")
                            else:
                                print(f"      [FAIL] Could not find Constant")
                                    
                        except Exception as e:
                            print(f"      Error reading segment properties: {e}")
                            import traceback
                            traceback.print_exc()
                
            except Exception as e:
                print(f"   Error accessing Fellow {i+1}: {e}")
                import traceback
                traceback.print_exc()
    
    except Exception as e:
        print(f"[ERROR] Error in method 2: {e}")
        import traceback
        traceback.print_exc()


def test_using_actual_placement_code(ts):
    """Test using the actual place_fellow code to see what it sets."""
    print("\n" + "="*80)
    print("Method 3: Inspecting what place_fellow() actually sets")
    print("="*80)
    
    try:
        from scenic.simulators.dspace.modeldesk.placement import place_fellow
        
        # Create a mock object with position
        class MockObj:
            def __init__(self, x, y, z):
                from scenic.core.vectors import Vector
                self.position = Vector(x, y, z)
                self.name = None
        
        # Test with first fellow's expected position
        mock_obj = MockObj(-101.919263, -457.524908, 0.0)
        
        # We need a mock sim object
        class MockSim:
            def __init__(self, ts):
                self.ts = ts
                self._coordinate_transform = None
                self._road_index = None
            
            def _detect_route_from_road_segment(self, obj):
                return "Lap"
            
            def _set_fellow_route_via_sequence(self, seq, obj):
                # Mock route setting
                pass
        
        mock_sim = MockSim(ts)
        
        # Try to place (this will show us what gets set)
        print("   Attempting to place fellow using place_fellow()...")
        try:
            result = place_fellow(mock_sim, mock_obj)
            if result:
                print(f"   [OK] Fellow placed: {result.Name if hasattr(result, 'Name') else 'Unknown'}")
            else:
                print("   [WARNING] place_fellow returned None")
        except Exception as e:
            print(f"   [ERROR] place_fellow failed: {e}")
            import traceback
            traceback.print_exc()
        
    except Exception as e:
        print(f"[ERROR] Error in method 3: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main test function."""
    print("="*80)
    print("ModelDesk Variable Access Test")
    print("="*80)
    print("\nThis script tests different methods to access ModelDesk fellow")
    print("configuration variables (routes, s, t values).")
    print("="*80)
    
    try:
        app, proj, exp, ts = connect_to_modeldesk()
    except Exception as e:
        print(f"\n[ERROR] Failed to connect: {e}")
        return 1
    
    # Test different access methods
    test_fellow_access_method1(ts)
    test_fellow_access_method2(ts)
    # test_using_actual_placement_code(ts)  # Commented out - might create new fellows
    
    print("\n" + "="*80)
    print("Test Complete")
    print("="*80)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

