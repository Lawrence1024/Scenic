"""Vehicle physics model for kinematic control.

This module provides a simple physics simulation that converts control inputs
(throttle, brake, steering) into motion outputs (velocity, lateral deviation).
Used primarily for fellow vehicle control in external signal mode.
"""


class VehiclePhysicsState:
    """Simple physics model for fellow vehicle control.
    
    Converts throttle/brake/steering inputs into velocity/deviation outputs
    for ControlDesk external control signals. Uses basic kinematic equations
    with tunable parameters for realistic behavior.
    
    This model is designed for:
    - Fellow vehicles using external kinematic control
    - Real-time simulation with typical timesteps (0.01-0.1s)
    - Tunable parameters to match vehicle dynamics
    
    Attributes:
        velocity: Current longitudinal velocity in m/s
        deviation: Current lateral deviation from centerline in meters
        max_acceleration: Maximum acceleration in m/s²
        max_deceleration: Maximum braking deceleration in m/s²
        max_velocity: Maximum velocity limit in m/s
        min_velocity: Minimum velocity (typically 0) in m/s
        max_lateral_velocity: Maximum lateral velocity in m/s
        steering_sensitivity: Steering response in m/s per steering unit
    """
    
    def __init__(self, initial_velocity=0.0, initial_deviation=0.0):
        """Initialize physics state.
        
        Args:
            initial_velocity: Starting velocity in m/s (default: 0.0)
            initial_deviation: Starting lateral deviation from centerline in meters (default: 0.0)
        """
        # State variables
        self.velocity = initial_velocity  # m/s
        self.deviation = initial_deviation  # m (lateral offset from centerline)
        
        # Longitudinal dynamics parameters (tunable)
        # Increased from 10.0 to 20.0 m/s² for more realistic racing car acceleration
        # (0-100 km/h in ~1.4s, which is aggressive but realistic for racing cars)
        self.max_acceleration = 20.0  # m/s² (was 10.0)
        self.max_deceleration = 15.0  # m/s² (emergency braking)
        self.max_velocity = 100.0  # m/s (~360 km/h)
        self.min_velocity = 0.0  # m/s
        
        # Lateral dynamics parameters (tunable)
        self.max_lateral_velocity = 5.0  # m/s lateral movement
        self.steering_sensitivity = 2.0  # meters per second per steering unit
    
    def update(self, throttle, brake, steering, dt):
        """Update physics state based on control inputs.
        
        Uses simple Euler integration to update velocity and deviation based on
        control inputs. This is suitable for real-time simulation with small timesteps.
        
        Args:
            throttle: Throttle input in range [0.0, 1.0]
            brake: Brake input in range [0.0, 1.0]
            steering: Steering input in range [-1.0, 1.0] (negative=right, positive=left)
            dt: Time step in seconds (typically 0.01 to 0.1)
            
        Returns:
            Tuple[float, float]: (new_velocity, new_deviation) in m/s and meters
            
        Notes:
            - Braking takes priority over throttle when brake > 0.01
            - Steering effect scales with velocity (no steering when stopped)
            - All outputs are clamped to physical limits
        """
        # 1. Longitudinal dynamics (velocity)
        acceleration = throttle * self.max_acceleration - brake * self.max_deceleration
        
        # Integrate velocity using Euler method
        self.velocity += acceleration * dt
        
        # Clamp to physical limits
        self.velocity = max(self.min_velocity, min(self.max_velocity, self.velocity))
        
        # 2. Lateral dynamics (deviation from centerline)
        # Steering effect scales with velocity (can't steer when stopped)
        velocity_factor = min(self.velocity / 20.0, 1.0)  # Full effect at 20 m/s
        lateral_velocity = steering * self.steering_sensitivity * velocity_factor
        
        # Clamp lateral velocity
        lateral_velocity = max(-self.max_lateral_velocity, 
                              min(self.max_lateral_velocity, lateral_velocity))
        
        # Integrate deviation
        self.deviation += lateral_velocity * dt
        
        return self.velocity, self.deviation
    
    def reset(self, velocity=0.0, deviation=0.0):
        """Reset physics state to specified values.
        
        Args:
            velocity: New velocity in m/s (default: 0.0)
            deviation: New deviation in meters (default: 0.0)
        """
        self.velocity = velocity
        self.deviation = deviation
    
    def set_parameters(self, **kwargs):
        """Set physics parameters for tuning.
        
        Args:
            **kwargs: Parameter names and values to set. Valid parameters:
                - max_acceleration (m/s²)
                - max_deceleration (m/s²)
                - max_velocity (m/s)
                - min_velocity (m/s)
                - max_lateral_velocity (m/s)
                - steering_sensitivity (m/s per unit)
                
        Example:
            physics.set_parameters(max_acceleration=12.0, steering_sensitivity=2.5)
        """
        for param, value in kwargs.items():
            if hasattr(self, param):
                setattr(self, param, value)
            else:
                raise ValueError(f"Unknown physics parameter: {param}")

