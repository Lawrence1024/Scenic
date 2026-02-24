"""I/O adapter for ControlDesk integration.

Provides functions to read vehicle state from ControlDesk and write
steering commands back to ControlDesk.
"""

from typing import Dict, Optional, Tuple
from scenic.simulators.dspace.simulator import DSpaceSimulation


def read_state_from_controldesk(sim: DSpaceSimulation, obj) -> Dict[str, float]:
    """Read vehicle state from ControlDesk.
    
    Args:
        sim: DSpaceSimulation instance
        obj: Vehicle object (ego or fellow)
        
    Returns:
        Dictionary with state:
            - 'x', 'y': position (meters)
            - 'yaw': heading (radians)
            - 'speed': speed (m/s)
            - 'yaw_rate': yaw rate (rad/s, optional)
            - 'steer_actual': actual steering angle (rad, optional)
    """
    if not sim._cd:
        return {}
    
    # Fast path: reuse per-step cache populated by DSpaceSimulation.getProperties
    cache_key = (sim.currentTime, id(obj))
    if hasattr(sim, "_state_cache") and cache_key in sim._state_cache:
        state = dict(sim._state_cache[cache_key])  # copy
        actor = getattr(obj, "dspaceActor", None)

        # Try to read actual steering only (optional) without re-reading full ego state
        if hasattr(sim, 'mpc_config') and sim.mpc_config:
            steer_path = sim.mpc_config.controldesk_paths.get('steer_actual')
            if steer_path:
                try:
                    steer_val = sim._cd.get_var(steer_path)
                    import math
                    steer_rad = math.radians(steer_val)
                    STEER_ACTUAL_SIGN = -1.0
                    state['steer_actual'] = STEER_ACTUAL_SIGN * steer_rad
                except Exception:
                    pass
        return state

    # Use existing readback infrastructure
    from scenic.simulators.dspace.controldesk.readback import read_ego_state, read_fellow_state
    from scenic.simulators.dspace.utils import legacy as dutils
    
    is_ego = (obj is sim.scene.egoObject)
    
    if is_ego:
        success = read_ego_state(sim, obj)
    else:
        success = read_fellow_state(sim, obj, dutils)
    
    if not success or not hasattr(obj, 'dspaceActor') or not obj.dspaceActor:
        return {}
    
    actor = obj.dspaceActor
    pos = actor.position
    vel = actor.linvel
    yaw = actor.heading
    
    state = {
        'x': float(pos.x),
        'y': float(pos.y),
        'yaw': float(yaw),
        'speed': float(vel.norm()),
    }
    
    # Try to read yaw_rate if available
    if hasattr(actor, 'angvel'):
        angvel = actor.angvel
        if hasattr(angvel, 'z'):
            state['yaw_rate'] = float(angvel.z)
    
    # Try to read actual steering if available
    # Check if config has steer_actual path configured
    if hasattr(sim, 'mpc_config') and sim.mpc_config:
        steer_path = sim.mpc_config.controldesk_paths.get('steer_actual')
        if steer_path:
            try:
                # Read steering angle from ControlDesk
                steer_val = sim._cd.get_var(steer_path)
                # Convert to radians (assuming degrees from ControlDesk)
                import math
                steer_rad = math.radians(steer_val)

                # IMPORTANT: MPC expects +delta=LEFT, -delta=RIGHT.
                # If ControlDesk reports the opposite sign, flip here.
                STEER_ACTUAL_SIGN = -1.0   # <-- use -1.0 only if left-turn readback is negative
                steer_rad = STEER_ACTUAL_SIGN * steer_rad
                state['steer_actual'] = steer_rad
                
                # Debug logging (first few reads only)
                if not hasattr(sim, '_steer_feedback_log_count'):
                    sim._steer_feedback_log_count = 0
                if sim._steer_feedback_log_count < 3:
                    print(f"[MPC Steering Feedback] Read: {steer_val:.3f} deg ({steer_rad:.4f} rad)")
                    sim._steer_feedback_log_count += 1
            except Exception as e:
                # Path might not exist or variable not available
                # Fall back to using previous control estimate
                if not hasattr(sim, '_steer_feedback_error_logged'):
                    print(f"[MPC Steering Feedback] Warning: Could not read steering feedback: {e}")
                    print(f"[MPC Steering Feedback] Falling back to previous control estimate")
                    sim._steer_feedback_error_logged = True
    
    return state


def write_steering_to_controldesk(sim: DSpaceSimulation, 
                                   obj, 
                                   steer_cmd_normalized: float):
    """Write steering command to ControlDesk.
    
    Args:
        sim: DSpaceSimulation instance
        obj: Vehicle object
        steer_cmd_normalized: Steering command in range [-1.0, 1.0]
    """
    if not sim._cd or not hasattr(sim, '_vehicle_controller'):
        return
    
    # Use existing VehicleController infrastructure
    # Store in _control_state, which will be applied by executeActions()
    if not hasattr(obj, '_control_state'):
        obj._control_state = {}
    
    obj._control_state['steering'] = steer_cmd_normalized

