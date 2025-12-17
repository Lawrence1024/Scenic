"""Test script to verify steering feedback reading from ControlDesk.

This script tests that the MPC can read steering feedback from ControlDesk
using the configured path.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from scenic.domains.racing.mpc.config import load_mpc_config
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

def test_steering_feedback():
    """Test reading steering feedback from ControlDesk."""
    print("="*70)
    print("MPC Steering Feedback Test")
    print("="*70)
    
    # Load MPC config
    print("\n[1] Loading MPC configuration...")
    try:
        config = load_mpc_config()
        print(f"    [OK] Config loaded")
        print(f"    Prediction horizon: {config.mpc_prediction_horizon}")
        print(f"    Control period: {config.ctrl_period}s")
    except Exception as e:
        print(f"    [FAIL] Could not load config: {e}")
        return False
    
    # Check steering path
    print("\n[2] Checking steering feedback path...")
    steer_path = config.controldesk_paths.get('steer_actual')
    if not steer_path:
        print("    [FAIL] steer_actual path not configured")
        return False
    print(f"    [OK] Path configured: {steer_path}")
    
    # Connect to ControlDesk
    print("\n[3] Connecting to ControlDesk...")
    try:
        cd = ControlDeskApp().connect()
        print("    [OK] Connected to ControlDesk")
    except Exception as e:
        print(f"    [FAIL] Could not connect: {e}")
        print("    Make sure ControlDesk is running and connected to simulator")
        return False
    
    # Test reading steering value
    print("\n[4] Testing steering feedback reading...")
    try:
        steer_val = cd.get_var(steer_path)
        steer_deg = float(steer_val)
        steer_rad = steer_deg * (3.14159 / 180.0)
        print(f"    [OK] Successfully read steering angle")
        print(f"    Value: {steer_deg:.3f} deg ({steer_rad:.4f} rad)")
        
        # Check if value is reasonable (should be in range ±20 deg typically)
        if abs(steer_deg) > 20.0:
            print(f"    [WARNING] Steering angle seems large: {steer_deg:.3f} deg")
            print(f"    Expected range: ±16.1 deg (max front wheel angle)")
        else:
            print(f"    [OK] Steering angle is within expected range")
            
    except Exception as e:
        print(f"    [FAIL] Could not read steering angle: {e}")
        print(f"    This might mean:")
        print(f"    1. Variable path is incorrect")
        print(f"    2. Variable doesn't exist in current ControlDesk setup")
        print(f"    3. Simulation is not running/initialized")
        return False
    
    # Test reading other state variables for comparison
    print("\n[5] Testing other state variables...")
    test_paths = {
        'pose_x': config.controldesk_paths.get('pose_x'),
        'pose_y': config.controldesk_paths.get('pose_y'),
        'yaw': config.controldesk_paths.get('yaw'),
        'speed': config.controldesk_paths.get('speed'),
    }
    
    for name, path in test_paths.items():
        if path:
            try:
                val = cd.get_var(path)
                print(f"    [OK] {name}: {val}")
            except Exception as e:
                print(f"    [WARN] {name}: Could not read ({e})")
    
    print("\n" + "="*70)
    print("Test Summary")
    print("="*70)
    print("[SUCCESS] Steering feedback path is working!")
    print("\nNext steps:")
    print("1. Run MPC scenario: scenic examples/racing/ego_mpc_behavior.scenic --simulate")
    print("2. Monitor logs for steering feedback values")
    print("3. Verify MPC is using actual steering angle in state")
    
    return True

if __name__ == '__main__':
    success = test_steering_feedback()
    sys.exit(0 if success else 1)

