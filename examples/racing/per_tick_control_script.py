#!/usr/bin/env python3
"""
Per-Tick Control Script for dSPACE VEOS

This script demonstrates the exact per-tick control loop from manual4.md,
assuming your existing infrastructure is set up:
- Docker containers running (VEOS, CTun, etc.)
- CTun connectivity established
- ModelDesk and ControlDesk applications open

Based on manual4.md requirements:
- 10ms timing (dt = 0.01) for per-tick control
- Variable paths: Environment.Vehicle.Fx.Driver.Throttle/Brake/SteeringWheelAngle
- External control flags enabled via ASM_Maneuver.py script
"""

import time
import sys
from win32com.client import Dispatch

def main():
    """Main per-tick control loop."""
    
    print("=== dSPACE Per-Tick Control Script ===")
    print("Assuming infrastructure is set up:")
    print("- Docker containers running (VEOS, CTun, etc.)")
    print("- CTun connectivity established")
    print("- ModelDesk and ControlDesk applications open")
    print()
    
    try:
        # Connect to ControlDesk (assumes CTun is already connected)
        print("[ControlDesk] Connecting to ControlDesk application...")
        cd = Dispatch("ControlDesk.Application")
        exp = cd.ActiveProject.ActiveExperiment
        
        print(f"[ControlDesk] Connected to experiment: {exp.Name}")
        
        # Get control variables for F2 (opponent1 with raceNumber=2)
        print("[ControlDesk] Getting control variables for F2...")
        throttle = exp.GetVariable("Environment.Vehicle.F2.Driver.Throttle")
        brake = exp.GetVariable("Environment.Vehicle.F2.Driver.Brake")
        steering = exp.GetVariable("Environment.Vehicle.F2.Driver.SteeringWheelAngle")
        
        print("[ControlDesk] ✅ Control variables obtained successfully")
        
        # Per-tick control loop (exactly as per manual4.md)
        dt = 0.01  # 10 ms (match sim)
        print(f"[PerTick] Starting per-tick control loop (dt={dt}s)")
        print("[PerTick] Press Ctrl+C to stop")
        
        t0 = time.time()
        tick_count = 0
        
        try:
            while True:
                t = time.time() - t0
                tick_count += 1
                
                # Example control logic (replace with your control algorithm)
                # Hold lane, sinusoid steer wiggle, accelerate gently
                throttle_value = 0.25  # 25% throttle
                brake_value = 0.0     # no brake
                steering_value = 2.0 * (0.5 * (1 if int(t) % 2 == 0 else -1))  # placeholder steer
                
                # Apply control inputs
                throttle.Value = throttle_value
                brake.Value = brake_value
                steering.Value = steering_value
                
                # Log every 100 ticks (1 second)
                if tick_count % 100 == 0:
                    print(f"[PerTick] Tick {tick_count}: throttle={throttle_value}, brake={brake_value}, steering={steering_value:.1f}°")
                
                # Maintain timing
                time.sleep(dt)
                
        except KeyboardInterrupt:
            print("\n[PerTick] Control loop stopped by user")
            
            # Safe-out (as per manual4.md)
            throttle.Value = 0.0
            brake.Value = 0.3     # light brake on exit
            steering.Value = 0.0
            
            print("[PerTick] Safe-out applied: throttle=0, brake=0.3, steering=0")
            
    except Exception as e:
        print(f"[Error] Failed to connect to ControlDesk: {e}")
        print("\nTroubleshooting:")
        print("1. Ensure ControlDesk application is open")
        print("2. Ensure CTun client is running: .\\bin\\ctun.exe client 127.0.0.1 --dest 10.6.0.2")
        print("3. Ensure VEOS is registered at 192.168.100.101")
        print("4. Ensure ASM_Traffic.sdf is assigned in ControlDesk")
        return 1
    
    print("[PerTick] Script completed successfully")
    return 0

if __name__ == "__main__":
    sys.exit(main())
