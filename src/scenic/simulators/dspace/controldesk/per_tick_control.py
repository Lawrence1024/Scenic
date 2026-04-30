# -*- coding: utf-8 -*-
"""
External Control Manager for dSPACE VEOS

Sets the race-wide manual flags via ASM_Maneuver.py before maneuver start so
the plant lets cars move (green track flag, no-error vehicle flag, manual
override channel selected).

ASM_Maneuver.py accepts a list of independent commands; the integer after
each underscore is the VALUE to write, not a vehicle index. Each writes a
single race-wide scalar:
    manualflag      -> RaceControl/.../manual_mode             = 1.0
    trackflag_<N>   -> RaceControl/.../track_flag_manual[1,1]  = N
    vehicleflag_<N> -> RaceControl/.../veh_flag_manual[1,1]    = N

`manualflag` is required: without it manual_mode stays 0 and the manual
channel isn't selected as the source, so the flag values written are ignored
even though they're in the right slots. The three together are the same
race-go signals Scenic-mode applies via MAPort in simulator.py step 14.

Correct invocation (race-wide, regardless of vehicle count):
    docker exec veos python3 /home/dspace/scripts/ASM_Maneuver.py \\
        manualflag vehicleflag_0 trackflag_1
"""

import subprocess
import os


# Race-wide flag values written by ASM_Maneuver.py.
# trackflag_1 = green; vehicleflag_0 = no-error.
_TRACK_FLAG_GREEN = 1
_VEHICLE_FLAG_NO_ERROR = 0
_SCRIPT_PATH = '/home/dspace/scripts/ASM_Maneuver.py'
_SCRIPT_ARGS = [
    'manualflag',
    f'vehicleflag_{_VEHICLE_FLAG_NO_ERROR}',
    f'trackflag_{_TRACK_FLAG_GREEN}',
]


class ExternalControlManager:
    """Manager for race-wide manual flags via ASM_Maneuver.py."""

    @staticmethod
    def enableExternalControlViaScript(scene_objects=None):
        """Set race-wide track/vehicle flags so the plant allows movement.

        Args:
            scene_objects: Unused. Kept for backward-compatible signature; the
                ASM script writes a single race-wide scalar, so the call does
                not depend on vehicle count or identity.
        """
        try:
            if ExternalControlManager._tryDockerExec():
                return

            if os.path.exists(_SCRIPT_PATH):
                ExternalControlManager._runScriptDirectly()
            else:
                print("[ASM_Maneuver] Script not found - external control may need manual setup")
                print(f"[ASM_Maneuver] Try: docker exec -it veos python3 {_SCRIPT_PATH} {' '.join(_SCRIPT_ARGS)}")

        except Exception as e:
            print(f"[ASM_Maneuver] Error running script: {e}")

    @staticmethod
    def _tryDockerExec():
        """Run ASM_Maneuver.py via Docker exec for the whole race."""
        cmd = ['docker', 'exec', 'veos', 'python3', _SCRIPT_PATH, *_SCRIPT_ARGS]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print(f"[ASM_Maneuver] [OK] Race flags set "
                      f"(manual_mode=1, track={_TRACK_FLAG_GREEN} green, "
                      f"vehicle={_VEHICLE_FLAG_NO_ERROR} no-error).")
                return True
            print(f"[ASM_Maneuver] [FAIL] Race-flag set failed: {result.stderr.strip()}")
            return False
        except subprocess.TimeoutExpired:
            print("[ASM_Maneuver] [TIMEOUT] Race-flag set timed out")
            return False
        except FileNotFoundError:
            print("[ASM_Maneuver] Docker not found - trying direct script execution")
            return False
        except Exception as e:
            err_msg = str(e).encode('ascii', 'replace').decode('ascii')
            print(f"[ASM_Maneuver] Docker exec error: {err_msg}")
            return False

    @staticmethod
    def _runScriptDirectly():
        """Run ASM_Maneuver.py directly (when inside the VEOS container)."""
        cmd = ['python3', _SCRIPT_PATH, *_SCRIPT_ARGS]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print(f"[ASM_Maneuver] [OK] Race flags set "
                      f"(manual_mode=1, track={_TRACK_FLAG_GREEN} green, "
                      f"vehicle={_VEHICLE_FLAG_NO_ERROR} no-error).")
            else:
                print(f"[ASM_Maneuver] [FAIL] Race-flag set failed: {result.stderr.strip()}")
        except Exception as e:
            err_msg = str(e).encode('ascii', 'replace').decode('ascii')
            print(f"[ASM_Maneuver] Direct script error: {err_msg}")
