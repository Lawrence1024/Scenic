# -*- coding: utf-8 -*-
"""
External Control Manager for dSPACE VEOS

This module handles external control flag management via ASM_Maneuver.py script.
External control flags are required for fellow vehicles to be controlled via ControlDesk.

Based on manual4.md: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4
"""

import subprocess
import os


class ExternalControlManager:
    """Manager for external control flags via ASM_Maneuver.py script."""
    
    @staticmethod
    def enableExternalControlViaScript(scene_objects):
        """Enable external control using ASM_Maneuver.py script.
        
        Based on manual4.md: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4
        
        Args:
            scene_objects: List of Scenic objects to enable external control for
        """
        try:
            # Method 1: Try Docker exec (preferred for containerized VEOS)
            if ExternalControlManager._tryDockerExec(scene_objects):
                return
            
            # Method 2: Try direct script execution (if running inside VEOS container)
            if os.path.exists('/home/dspace/scripts/ASM_Maneuver.py'):
                ExternalControlManager._runScriptDirectly(scene_objects)
            else:
                print("[ASM_Maneuver] Script not found - external control may need manual setup")
                print("[ASM_Maneuver] Try: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4")
                
        except Exception as e:
            print(f"[ASM_Maneuver] Error running script: {e}")
    
    @staticmethod
    def _tryDockerExec(scene_objects):
        """Try to run ASM_Maneuver.py via Docker exec."""
        try:
            # Enable external control for each fellow vehicle
            for scenic_obj in scene_objects:
                if hasattr(scenic_obj, 'raceNumber') and scenic_obj is not getattr(scene_objects[0], 'egoObject', None):
                    fellow_number = scenic_obj.raceNumber
                    vehicle_flag = fellow_number + 1  # F1 = vehicleflag_2, F2 = vehicleflag_3, etc.
                    
                    # Docker exec command as per manual4.md
                    cmd = ['docker', 'exec', '-it', 'veos', 'python3', 
                           '/home/dspace/scripts/ASM_Maneuver.py', 
                           f'vehicleflag_{vehicle_flag}', 'trackflag_4']
                    
                    try:
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            print(f"[ASM_Maneuver] [OK] Enabled external control for F{fellow_number}")
                        else:
                            print(f"[ASM_Maneuver] [FAIL] Failed for F{fellow_number}: {result.stderr}")
                            return False
                    except subprocess.TimeoutExpired:
                        print(f"[ASM_Maneuver] [TIMEOUT] Timeout for F{fellow_number}")
                        return False
                    except FileNotFoundError:
                        print("[ASM_Maneuver] Docker not found - trying direct script execution")
                        return False
                    except Exception as e:
                        err_msg = str(e).encode('ascii', 'replace').decode('ascii')
                        print(f"[ASM_Maneuver] Error for F{fellow_number}: {err_msg}")
                        return False
            
            return True
            
        except Exception as e:
            err_msg = str(e).encode('ascii', 'replace').decode('ascii')
            print(f"[ASM_Maneuver] Docker exec error: {err_msg}")
            return False
    
    @staticmethod
    def _runScriptDirectly(scene_objects):
        """Run ASM_Maneuver.py script directly (when inside VEOS container)."""
        try:
            for scenic_obj in scene_objects:
                if hasattr(scenic_obj, 'raceNumber') and scenic_obj is not getattr(scene_objects[0], 'egoObject', None):
                    fellow_number = scenic_obj.raceNumber
                    vehicle_flag = fellow_number + 1
                    
                    cmd = ['python3', '/home/dspace/scripts/ASM_Maneuver.py', 
                           f'vehicleflag_{vehicle_flag}', 'trackflag_4']
                    
                    try:
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            print(f"[ASM_Maneuver] [OK] Enabled external control for F{fellow_number}")
                        else:
                            print(f"[ASM_Maneuver] [FAIL] Failed for F{fellow_number}: {result.stderr}")
                    except Exception as e:
                        err_msg = str(e).encode('ascii', 'replace').decode('ascii')
                        print(f"[ASM_Maneuver] Error for F{fellow_number}: {err_msg}")
                        
        except Exception as e:
            err_msg = str(e).encode('ascii', 'replace').decode('ascii')
            print(f"[ASM_Maneuver] Direct script error: {err_msg}")
