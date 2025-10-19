#!/usr/bin/env python3
"""
External Control Flag Setup Script for dSPACE VEOS

This script enables external control flags for fellow vehicles using
the ASM_Maneuver.py script as documented in manual4.md.

Usage:
    python enable_external_control.py [vehicle_numbers...]
    
Examples:
    python enable_external_control.py 2 3    # Enable F2 and F3
    python enable_external_control.py        # Enable all fellows
"""

import subprocess
import sys
import os

def enable_external_control(vehicle_numbers=None):
    """Enable external control flags for fellow vehicles.
    
    Args:
        vehicle_numbers: List of vehicle numbers to enable (e.g., [2, 3] for F2, F3)
                        If None, enables all fellow vehicles
    """
    
    print("=== dSPACE External Control Flag Setup ===")
    print("Based on manual4.md: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_X trackflag_4")
    print()
    
    # Default vehicle numbers if none specified
    if vehicle_numbers is None:
        vehicle_numbers = [2, 3, 4, 5, 6]  # Common fellow vehicle numbers
    
    success_count = 0
    
    for vehicle_num in vehicle_numbers:
        vehicle_flag = vehicle_num + 1  # F1 = vehicleflag_2, F2 = vehicleflag_3, etc.
        
        print(f"[ASM_Maneuver] Enabling external control for F{vehicle_num} (vehicleflag_{vehicle_flag})...")
        
        # Docker exec command as per manual4.md
        cmd = ['docker', 'exec', '-it', 'veos', 'python3', 
               '/home/dspace/scripts/ASM_Maneuver.py', 
               f'vehicleflag_{vehicle_flag}', 'trackflag_4']
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print(f"[ASM_Maneuver] ✅ Successfully enabled external control for F{vehicle_num}")
                success_count += 1
            else:
                print(f"[ASM_Maneuver] ❌ Failed for F{vehicle_num}: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            print(f"[ASM_Maneuver] ⏰ Timeout for F{vehicle_num}")
        except FileNotFoundError:
            print("[ASM_Maneuver] ❌ Docker not found - ensure Docker is running")
            break
        except Exception as e:
            print(f"[ASM_Maneuver] ❌ Error for F{vehicle_num}: {e}")
    
    print(f"\n[ASM_Maneuver] Summary: {success_count}/{len(vehicle_numbers)} vehicles enabled")
    
    if success_count > 0:
        print("\n✅ External control flags enabled successfully!")
        print("You can now use per-tick control via ControlDesk COM automation.")
    else:
        print("\n❌ No external control flags were enabled.")
        print("Troubleshooting:")
        print("1. Ensure Docker containers are running")
        print("2. Ensure VEOS container is named 'veos'")
        print("3. Check ASM_Maneuver.py script exists in VEOS container")

def main():
    """Main function."""
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        try:
            vehicle_numbers = [int(arg) for arg in sys.argv[1:]]
        except ValueError:
            print("Error: Vehicle numbers must be integers")
            print("Usage: python enable_external_control.py [vehicle_numbers...]")
            print("Example: python enable_external_control.py 2 3")
            return 1
    else:
        vehicle_numbers = None
    
    enable_external_control(vehicle_numbers)
    return 0

if __name__ == "__main__":
    sys.exit(main())
