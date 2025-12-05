"""Diagnostic script for ControlDesk connection issues.

This script tests the ControlDesk COM connection and provides detailed
information about what might be wrong.
"""

import sys
import traceback

def test_com_initialization():
    """Test if COM can be initialized."""
    print("\n" + "="*70)
    print("Test 1: COM Initialization")
    print("="*70)
    try:
        import pythoncom
        pythoncom.CoInitialize()
        print("✓ COM initialized successfully")
        return True
    except Exception as e:
        print(f"✗ COM initialization failed: {e}")
        traceback.print_exc()
        return False

def test_controldesk_progid(prog_id="ControlDeskNG.Application"):
    """Test if ControlDesk COM interface is available."""
    print("\n" + "="*70)
    print(f"Test 2: ControlDesk COM Interface ({prog_id})")
    print("="*70)
    try:
        from win32com.client import Dispatch
        print(f"  Attempting to connect to '{prog_id}'...")
        app = Dispatch(prog_id)
        print(f"✓ Connected to {prog_id}")
        print(f"  App object: {app}")
        return app
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        traceback.print_exc()
        return None

def test_alternative_progids():
    """Try alternative ControlDesk ProgIDs."""
    print("\n" + "="*70)
    print("Test 3: Trying Alternative ProgIDs")
    print("="*70)
    
    alternative_progids = [
        "ControlDeskNG.Application",
        "ControlDesk.Application",
        "dSPACE.ControlDesk.Application",
        "dSPACE.ControlDeskNG.Application",
    ]
    
    for prog_id in alternative_progids:
        print(f"\n  Trying: {prog_id}")
        app = test_controldesk_progid(prog_id)
        if app:
            return prog_id, app
    
    print("\n✗ None of the standard ProgIDs worked")
    return None, None

def list_available_com_objects():
    """List all available COM objects (may help identify correct ProgID)."""
    print("\n" + "="*70)
    print("Test 4: Searching for dSPACE COM Objects in Registry")
    print("="*70)
    
    try:
        import winreg
        
        # Search HKEY_CLASSES_ROOT for dSPACE related ProgIDs
        found_any = False
        with winreg.OpenKey(winreg.HKEY_CLASSES_ROOT, "") as root:
            i = 0
            while True:
                try:
                    key_name = winreg.EnumKey(root, i)
                    if "dspace" in key_name.lower() or "controldesk" in key_name.lower():
                        print(f"  Found: {key_name}")
                        found_any = True
                    i += 1
                except OSError:
                    break
        
        if not found_any:
            print("  ✗ No dSPACE or ControlDesk COM objects found in registry")
            print("  This suggests ControlDesk may not be installed or registered properly")
    except Exception as e:
        print(f"  Could not search registry: {e}")

def test_active_experiment(app):
    """Test if an active experiment is available."""
    print("\n" + "="*70)
    print("Test 5: Active Experiment")
    print("="*70)
    
    if not app:
        print("✗ Cannot test - no app connection")
        return False
    
    try:
        exp = app.ActiveExperiment
        if exp:
            print(f"✓ Active experiment found: {exp}")
            print(f"  Name: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
            return exp
        else:
            print("✗ No active experiment")
            print("  Solution: Open an experiment in ControlDesk")
            return None
    except Exception as e:
        print(f"✗ Could not access ActiveExperiment: {e}")
        traceback.print_exc()
        return None

def test_platform_management(app):
    """Test if platforms are available."""
    print("\n" + "="*70)
    print("Test 6: Platform Management")
    print("="*70)
    
    if not app:
        print("✗ Cannot test - no app connection")
        return False
    
    try:
        pm = app.PlatformManagement
        print(f"✓ PlatformManagement accessible: {pm}")
        
        platforms = pm.Platforms
        print(f"  Platforms collection: {platforms}")
        
        count = platforms.Count
        print(f"  Number of platforms: {count}")
        
        if count > 0:
            platform = platforms.Item(0)
            print(f"✓ Platform[0] accessible: {platform}")
            print(f"  Name: {platform.Name if hasattr(platform, 'Name') else 'Unknown'}")
            return True
        else:
            print("✗ No platforms available")
            print("  Solution: Load a platform/device in ControlDesk")
            return False
            
    except Exception as e:
        print(f"✗ Platform management error: {e}")
        traceback.print_exc()
        return False

def test_online_calibration(app):
    """Test if online calibration can be started."""
    print("\n" + "="*70)
    print("Test 7: Online Calibration")
    print("="*70)
    
    if not app:
        print("✗ Cannot test - no app connection")
        return False
    
    try:
        cm = app.CalibrationManagement
        print(f"✓ CalibrationManagement accessible: {cm}")
        
        print("  Attempting to start online calibration...")
        cm.StartOnlineCalibration()
        print("✓ Online calibration started successfully!")
        
        # Try to stop it again
        print("  Stopping online calibration...")
        cm.StopOnlineCalibration()
        print("✓ Online calibration stopped")
        
        return True
        
    except Exception as e:
        print(f"✗ Online calibration failed: {e}")
        print(f"  Error details: {type(e).__name__}")
        traceback.print_exc()
        
        if "No platform/device was able to start online calibration" in str(e):
            print("\n  This error means:")
            print("    - No platform/device is loaded, OR")
            print("    - The platform is not ready for online calibration")
            print("\n  Solutions:")
            print("    1. Make sure a platform is loaded in ControlDesk")
            print("    2. Check that the platform is connected and online")
            print("    3. Try loading the platform manually in ControlDesk first")
        
        return False

def main():
    """Run all diagnostic tests."""
    print("\n" + "="*70)
    print("ControlDesk Connection Diagnostic Tool")
    print("="*70)
    print("\nThis tool will test your ControlDesk connection and identify issues.")
    
    # Test 1: COM initialization
    if not test_com_initialization():
        print("\n" + "="*70)
        print("FATAL: COM initialization failed")
        print("="*70)
        return
    
    # Test 2 & 3: ControlDesk ProgID
    prog_id, app = test_alternative_progids()
    
    if not app:
        print("\n" + "="*70)
        print("FATAL: Could not connect to ControlDesk")
        print("="*70)
        print("\nPossible reasons:")
        print("  1. ControlDesk is not running")
        print("  2. ControlDesk is not installed")
        print("  3. COM registration is broken")
        print("\nSolutions:")
        print("  1. Start ControlDesk application")
        print("  2. If ControlDesk is running, try restarting it")
        print("  3. Re-install ControlDesk if needed")
        
        # Test 4: Search registry
        list_available_com_objects()
        return
    
    print(f"\n✓ Successfully connected using ProgID: {prog_id}")
    
    # Test 5: Active experiment
    exp = test_active_experiment(app)
    
    # Test 6: Platform management
    has_platform = test_platform_management(app)
    
    # Test 7: Online calibration (only if we have a platform)
    if has_platform:
        test_online_calibration(app)
    else:
        print("\n" + "="*70)
        print("SKIPPING: Online Calibration Test (no platform available)")
        print("="*70)
    
    # Final summary
    print("\n" + "="*70)
    print("DIAGNOSTIC SUMMARY")
    print("="*70)
    
    if app and exp and has_platform:
        print("✓ ControlDesk is properly configured")
        print(f"✓ Working ProgID: {prog_id}")
        print("\nYou should be able to run the Scenic simulation now.")
        print("\nIf you still get connection errors, the issue is likely with")
        print("online calibration. Make sure:")
        print("  - The platform is online and ready")
        print("  - No other application is using the platform")
        print("  - The experiment is properly configured")
    else:
        print("✗ ControlDesk connection has issues")
        print("\nRequired fixes:")
        if not app:
            print("  1. Start ControlDesk application")
        if not exp:
            print("  2. Load/open an experiment in ControlDesk")
        if not has_platform:
            print("  3. Load a platform/device in the experiment")
    
    print("="*70)

if __name__ == "__main__":
    main()
