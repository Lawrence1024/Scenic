"""Abstract interface to simulators supporting the racing domain."""

from scenic.core.simulators import Simulation, Simulator
from scenic.domains.driving.controllers import (
    PIDLateralController,
    PIDLongitudinalController,
)


class RacingSimulator(Simulator):
    """A `Simulator` supporting the racing domain.
    
    Racing simulators extend the driving domain with racing-specific features:
    - Closed-loop racing circuits with defined direction
    - Pit lanes separate from racing lanes
    - Track segments (main racing road vs pit lane road)
    - Racing-specific coordinate systems and route management
    
    Racing simulators should inherit from this class and implement:
    - `createSimulation()` - Create a RacingSimulation instance
    - Racing-specific object creation (RacingCar, FormulaCar, etc.)
    - Track segment identification and route assignment
    """

    pass


class RacingSimulation(Simulation):
    """A `Simulation` with a simulator supporting the racing domain.
    
    This subclass of `Simulation` provides racing-specific functionality:
    - Racing car controllers optimized for track performance
    - Track segment detection and route assignment
    - Racing-specific behaviors (racing line following, pit stops, etc.)
    - Performance monitoring and timing
    
    Racing simulations should inherit from this class and implement:
    - Racing-specific object creation and positioning
    - Track segment identification from OpenDRIVE maps
    - Route assignment based on track segments (pit lane vs main racing)
    """

    def getRacingControllers(self, agent):
        """Get longitudinal and lateral controllers optimized for racing.
        
        Racing controllers are tuned for higher performance than standard driving
        controllers, with more aggressive parameters suitable for track driving.
        
        Args:
            agent: The racing agent (RacingCar, FormulaCar, etc.)
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        if hasattr(agent, 'isFormulaCar') and agent.isFormulaCar:
            # Formula cars: High-performance controllers
            lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
            lat_controller = PIDLateralController(K_P=0.3, K_D=0.15, K_I=0.0, dt=dt)
        elif hasattr(agent, 'isRacingCar') and agent.isRacingCar:
            # Racing cars: Optimized for track performance
            lon_controller = PIDLongitudinalController(K_P=0.7, K_D=0.12, K_I=0.8, dt=dt)
            lat_controller = PIDLateralController(K_P=0.25, K_D=0.12, K_I=0.0, dt=dt)
        else:
            # Standard cars: Use driving domain defaults
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.2, K_D=0.1, K_I=0.0, dt=dt)
            
        return lon_controller, lat_controller

    def getRacingLineControllers(self, agent):
        """Get controllers optimized for following the racing line.
        
        Racing line controllers are tuned for smooth, high-speed cornering
        and optimal lap times rather than comfort or safety.
        
        Args:
            agent: The racing agent
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        if hasattr(agent, 'isFormulaCar') and agent.isFormulaCar:
            # Formula cars: Aggressive racing line following
            lon_controller = PIDLongitudinalController(K_P=0.9, K_D=0.2, K_I=1.0, dt=dt)
            lat_controller = PIDLateralController(K_P=0.4, K_D=0.2, K_I=0.0, dt=dt)
        elif hasattr(agent, 'isRacingCar') and agent.isRacingCar:
            # Racing cars: Optimized racing line following
            lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
            lat_controller = PIDLateralController(K_P=0.35, K_D=0.15, K_I=0.0, dt=dt)
        else:
            # Standard cars: Conservative racing line following
            lon_controller = PIDLongitudinalController(K_P=0.6, K_D=0.12, K_I=0.8, dt=dt)
            lat_controller = PIDLateralController(K_P=0.3, K_D=0.12, K_I=0.0, dt=dt)
            
        return lon_controller, lat_controller

    def getPitLaneControllers(self, agent):
        """Get controllers optimized for pit lane driving.
        
        Pit lane controllers are tuned for precise positioning and speed limits,
        prioritizing accuracy over speed for pit stop maneuvers.
        
        Args:
            agent: The racing agent
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        # All racing vehicles use similar pit lane controllers (precision over speed)
        lon_controller = PIDLongitudinalController(K_P=0.4, K_D=0.08, K_I=0.6, dt=dt)
        lat_controller = PIDLateralController(K_P=0.15, K_D=0.08, K_I=0.0, dt=dt)
        
        return lon_controller, lat_controller

    def getOvertakingControllers(self, agent):
        """Get controllers optimized for overtaking maneuvers.
        
        Overtaking controllers are tuned for quick lane changes and
        aggressive acceleration while maintaining control.
        
        Args:
            agent: The racing agent
            
        Returns:
            A pair of controllers for throttle and steering respectively.
        """
        dt = self.timestep
        
        if hasattr(agent, 'isFormulaCar') and agent.isFormulaCar:
            # Formula cars: Aggressive overtaking
            lon_controller = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=dt)
            lat_controller = PIDLateralController(K_P=0.12, K_D=0.4, K_I=0.0, dt=dt)
        elif hasattr(agent, 'isRacingCar') and agent.isRacingCar:
            # Racing cars: Optimized overtaking
            lon_controller = PIDLongitudinalController(K_P=0.7, K_D=0.12, K_I=0.8, dt=dt)
            lat_controller = PIDLateralController(K_P=0.1, K_D=0.35, K_I=0.0, dt=dt)
        else:
            # Standard cars: Conservative overtaking
            lon_controller = PIDLongitudinalController(K_P=0.5, K_D=0.1, K_I=0.7, dt=dt)
            lat_controller = PIDLateralController(K_P=0.08, K_D=0.3, K_I=0.0, dt=dt)
            
        return lon_controller, lat_controller

    def detectTrackSegment(self, position):
        """Detect which track segment a position belongs to.
        
        This method determines whether a position is on the main racing circuit
        or the pit lane based on distance calculations to track segments.
        
        Args:
            position: (x, y) world coordinates
            
        Returns:
            String indicating the track segment: 'mainRacing', 'pitLane', or None
        """
        # Default implementation - should be overridden by specific simulators
        return None

    def assignRoute(self, agent, track_segment):
        """Assign appropriate route based on track segment.
        
        This method maps track segments to simulator-specific routes.
        Different simulators may have different route naming conventions.
        
        Args:
            agent: The racing agent
            track_segment: Track segment identifier ('mainRacing' or 'pitLane')
            
        Returns:
            String indicating the route preference for the simulator
        """
        # Default mapping - can be overridden by specific simulators
        if track_segment == 'pitLane':
            return 'Pit'
        elif track_segment == 'mainRacing':
            return 'Lap'
        else:
            return None
