from scenic.domains.driving.actions import Action

class SetVehicleControl(Action):
    """Placeholder for low-level control (wire to COM later if available)."""
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0):
        self.throttle, self.brake, self.steer = throttle, brake, steer
    def canBeTakenBy(self, agent):  # keep parallel with other adapters
        return hasattr(agent, "dspaceHandle") or True
    def applyTo(self, obj, sim):
        # TODO: call COM path to apply controls to Ego/Fellow if/when available
        pass
