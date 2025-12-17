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
    # TODO: Add ControlDesk path for steer_actual if available
    
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

