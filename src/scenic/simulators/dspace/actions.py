from scenic.domains.driving.actions import Action


# Marker mixin to identify dSPACE-backed vehicle agents (mirrors CARLA pattern)
class _DSpaceVehicle:
    # Used to avoid importing Scenic classes from model.scenic in Python modules.
    # Action gating can check isinstance(agent, _DSpaceVehicle).
    pass

class SetVehicleControl(Action):
    """Set throttle, brake, and steering control for dSPACE vehicles.
    
    This action interfaces with the dSPACE simulator to control fellow vehicles
    dynamically during simulation.
    
    Args:
        throttle: Throttle input (0.0 to 1.0)
        brake: Brake input (0.0 to 1.0)
        steer: Steering angle (-1.0 to 1.0)
        velocity: Target velocity in m/s (optional)
    """
    
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0, velocity=None):
        self.throttle = max(0.0, min(1.0, throttle))
        self.brake = max(0.0, min(1.0, brake))
        self.steer = max(-1.0, min(1.0, steer))
        self.velocity = velocity
    
    def canBeTakenBy(self, agent):
        """Check if agent can take this action."""
        # Prefer explicit dSPACE vehicle marker if present; otherwise allow for now.
        try:
            return isinstance(agent, _DSpaceVehicle)
        except Exception:
            return True
    
    def applyTo(self, obj, sim):
        """Apply control inputs to the vehicle in dSPACE."""
        try:
            # Get vehicle name using raceNumber (dSPACE convention: F1, F2, etc.)
            if hasattr(obj, 'raceNumber'):
                vehicle_name = f"F{obj.raceNumber}"
            else:
                # Fallback for non-racing objects
                if obj is sim.scene.egoObject:
                    vehicle_name = "Ego"
                else:
                    vehicle_name = f"F{id(obj) % 100}"  # Generate F-number
            
            # Get dSPACE simulation instance
            dspace_sim = getattr(sim, 'sim', None)
            if not dspace_sim:
                print(f"Warning: dSPACE simulation instance not available for {vehicle_name}")
                return
            
            if not hasattr(dspace_sim, 'setVehicleControl'):
                print(f"Warning: setVehicleControl method not available for {vehicle_name}")
                return
            
            # Apply control inputs
            success = dspace_sim.setVehicleControl(
                vehicle_name=vehicle_name,
                throttle=self.throttle,
                brake=self.brake,
                steering=self.steer,
                velocity=self.velocity
            )
            
            if success:
                print(f"Applied control to {vehicle_name}: throttle={self.throttle}, brake={self.brake}, steer={self.steer}")
            else:
                print(f"Failed to apply control to {vehicle_name}")
                
        except Exception as e:
            print(f"Error applying vehicle control: {e}")

class SetThrottleAction(SetVehicleControl):
    """Set throttle control for dSPACE vehicles."""
    def __init__(self, throttle):
        super().__init__(throttle=throttle, brake=0.0, steer=0.0)

class SetBrakeAction(SetVehicleControl):
    """Set brake control for dSPACE vehicles."""
    def __init__(self, brake):
        super().__init__(throttle=0.0, brake=brake, steer=0.0)

class SetSteerAction(SetVehicleControl):
    """Set steering control for dSPACE vehicles."""
    def __init__(self, steer):
        super().__init__(throttle=0.0, brake=0.0, steer=steer)

class SetVelocityAction(SetVehicleControl):
    """Set target velocity for dSPACE vehicles."""
    def __init__(self, velocity):
        super().__init__(throttle=0.0, brake=0.0, steer=0.0, velocity=velocity)
