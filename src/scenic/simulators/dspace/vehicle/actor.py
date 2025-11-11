from scenic.core.vectors import Vector


class DSpaceVehicleActor:
    """Internal representation of a vehicle in the dSPACE simulator.
    
    Stores state mirrored from ControlDesk/plant for Scenic's object model.
    """
    def __init__(self, scenic_obj):
        self.scenic_obj = scenic_obj
        self.position = Vector(0, 0, 0)
        self.linvel = Vector(0, 0, 0)
        self.angvel = Vector(0, 0, 0)
        self.heading = 0.0
        # Physics model for kinematic fellows
        # (initialized in simulator when needed)
        self.physics = None
        # Decision tree / racing state
        self.speed_limit = None
        self.speed_type = None
        self.ttl_selection = None
        self.target_gap = None
        self.gap_type = None
        self.strategy_type = "cruise_control"
        self.scale_factor = 1.0
        self.powertrain_mode = "nominal"
        self.push2pass_active = False
        # Optional control params cache
        self._control_params = {}

    def set_control(self, control_dict):
        if not hasattr(self, '_control_params'):
            self._control_params = {}
        self._control_params.update(control_dict)


def ensure_actor(obj):
    """Ensure Scenic object has dspaceActor attached and seeded with pose."""
    if not hasattr(obj, 'dspaceActor') or obj.dspaceActor is None:
        obj.dspaceActor = DSpaceVehicleActor(obj)
        if hasattr(obj, 'position'):
            obj.dspaceActor.position = obj.position
        if hasattr(obj, 'heading'):
            obj.dspaceActor.heading = obj.heading
    return obj.dspaceActor


