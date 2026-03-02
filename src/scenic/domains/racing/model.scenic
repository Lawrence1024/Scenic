"""Scenic world model for racing scenarios.

This model extends :doc:`scenic.domains.driving.model` with racing-specific
objects, regions, and constraints.

The map file must be specified using the ``map`` global parameter, and racing-specific
parameters can be set:

Example::

    param map = localPath('laguna_seca.xodr')
    param use2DMap = True
    param trackDirection = 'counterclockwise'  # or 'clockwise'
    param generateStartingGrid = True
    model scenic.domains.racing.model
"""

# Import everything from driving domain
from scenic.domains.driving.model import *
from scenic.domains.racing.tracks import RacingTrack, createRacingTrack
from scenic.domains.racing.behaviors import *
from scenic.domains.racing.actions import *
from scenic.core.regions import UnionRegion

## Racing-specific parameters

param trackDirection = 'counterclockwise'
param generateStartingGrid = True
param startingGridPositions = 20
param startingGridSpacing = 8.0  # meters between grid positions

# Track segment identification (optional)
param pitLaneRoadId = None  # e.g., "1545702203" for Laguna Seca
param pitLaneRoadName = "pit"  # Pattern to match pit lane name
param mainLineRoadId = None  # e.g., "2117817291" for Laguna Seca
# Junction assignment: OpenDRIVE road IDs of connecting roads for outer loop vs pit (e.g. (24, 34) and (25, 30))
param main_loop_connecting_road_ids = None  # e.g. (24, 34) for two junctions
param pit_connecting_road_ids = None  # e.g. (25, 30)

## Create racing track from the network

# Create track first so it's in scope for this module; then expose as param for scene.params['track'].
_track = createRacingTrack(
    globalParameters.map,
    direction=globalParameters.trackDirection,
    pitLaneRoadId=globalParameters.pitLaneRoadId,
    pitLaneRoadName=globalParameters.pitLaneRoadName,
    mainLineRoadId=globalParameters.mainLineRoadId,
    main_loop_connecting_road_ids=globalParameters.main_loop_connecting_road_ids,
    pit_connecting_road_ids=globalParameters.pit_connecting_road_ids,
    **globalParameters.map_options
)
param track = _track

# Replace the generic network with our racing track's network
network = _track.network

# Store road segment IDs in params for simulator access
param pitLaneRoadIds = [str(r.id) for r in _track._pitRoads] if getattr(_track, '_pitRoads', None) and _track._pitRoads else []
param mainRacingRoadIds = [str(r.id) for r in _track._mainRacingRoads] if _track._mainRacingRoads else []

## Racing-specific regions

## Racing regions (simplified per architecture):
#
# road          := entire drivable road surface
# mainRacingRoad, pitLaneRoad are mutually exclusive and their union == road

# Build pitLaneRoad region from all pit roads (primary + junction links)
_pit_roads = getattr(_track, '_pitRoads', None) or []
_all_pit_lanes = [lane for r in _pit_roads for lane in (r.lanes or [])]
pitLaneRoad: Region = (
    nowhere if not _all_pit_lanes
    else (UnionRegion(*_all_pit_lanes) if len(_all_pit_lanes) > 1 else _all_pit_lanes[0])
)

# Main racing road is the rest of the road excluding pitLaneRoad
mainRacingRoad: Region = road.difference(pitLaneRoad)

# Keep racingLine as the TTL default (can be overridden by actions)
racingLine: Region = _track.racingLine.region if hasattr(_track, 'racingLine') and _track.racingLine else mainRacingRoad

#: Start/finish line region
# TODO: Create actual start/finish line region from track data

## Starting grid

# Generate starting grid if requested
if globalParameters.generateStartingGrid:
    #: List of starting grid positions
    startingGrid = _track.generateStartingGrid(
        numPositions=globalParameters.startingGridPositions,
        spacing=globalParameters.startingGridSpacing
    )
else:
    startingGrid = []

## Racing-specific object types

class RacingCar(Car):
    """Abstract racing car class.
    
    This class defines the interface for racing cars but does not provide
    concrete implementations of racing-specific systems. Simulators must
    extend this class and implement the RacingSteers protocol.
    
    Properties:
        raceNumber: Car number for identification (1-999)
        team: Team name or identifier
        carType: Vehicle type for reference ("Dallara AV-24", "Custom", etc.)
        
        # Performance characteristics (configurable)
        maxSpeed: Maximum speed in m/s (default: 30 m/s = 108 km/h)
        acceleration: Acceleration capability in m/s² (default: 8.0)
        braking: Braking capability in m/s² (default: -12.0)
        
        # State properties
        fuelLevel: Current fuel level (0.0 to 1.0)
        tireWear: Tire wear level (0.0 = new, 1.0 = worn)
        
        # Autonomous racing capabilities
        waypointTolerance: Distance tolerance for waypoint following (default: 2.5)
        controllerAggressiveness: Controller aggressiveness (0.0-1.0, default: 0.5)
    """
    
    # Default racing properties
    speed: 25  # Higher default speed for racing (90 km/h)
    position: new Point on mainRacingRoad
    requireVisible: False
    
    # Racing identification
    raceNumber: Range(1, 999)
    team: None
    carType: "Racing Car"  # Default type
    
    # Performance characteristics (configurable)
    maxSpeed: 30.0  # ~108 km/h top speed
    acceleration: 8.0  # m/s² acceleration capability
    braking: -12.0  # m/s² braking capability
    
    # State properties
    fuelLevel: Range(0.5, 1.0)  # Start with reasonable fuel
    tireWear: 0.0  # Start with fresh tires
    
    # Autonomous capabilities
    waypointTolerance: 2.5  # Distance tolerance for waypoint following
    controllerAggressiveness: 0.5  # Controller aggressiveness (0.0-1.0)
    
    # Minimal racing API needed by behaviors
    ttl = racingLine  # Target line to drive on
    
    def setMaxSpeed(self, max_speed):
        raise NotImplementedError("Simulator must implement setMaxSpeed or accept property assignment")
    
    def setTTL(self, ttl):
        raise NotImplementedError("Simulator must implement setTTL or accept property assignment")

## Racing-specific utility functions

def carsInFormation(positions):
    """Create a formation of racing cars at the given positions.
    
    Example::
    
        cars = carsInFormation(startingGrid[:10])
    
    Args:
        positions: List of positions (typically from startingGrid)
        
    Returns:
        List of RacingCar objects
    """
    cars = []
    for i, pos in enumerate(positions):
        car = new RacingCar at pos, with raceNumber (i + 1)
        cars.append(car)
    return cars

def isOnRacingLine(car, tolerance=2.0):
    """Check if a car is on the optimal racing line.
    
    Args:
        car: The car to check
        tolerance: Distance tolerance in meters
        
    Returns:
        Boolean indicating if car is within tolerance of racing line
    """
    if _track.racingLine is None:
        return True  # No racing line defined, always on it
    
    # Check distance to racing line
    # This would need proper implementation with the actual racing line
    return True  # Placeholder

def distanceToSectorEnd(car):
    """Get distance from car's current position to the end of its current sector.
    
    Args:
        car: The car
        
    Returns:
        Distance in meters, or None if not in a sector
    """
    sector = _track.getSectorAt(car.position)
    if sector is None:
        return None

    current_distance = _track.distanceAlongTrack(car.position)
    if current_distance is None:
        return None
    
    return sector.endDistance - current_distance

def carsAheadInSector(car, sector=None):
    """Get all cars ahead of the given car in the specified sector.
    
    Args:
        car: The reference car
        sector: The sector to check (default: car's current sector)
        
    Returns:
        List of cars ahead in the sector
    """
    if sector is None:
        sector = _track.getSectorAt(car.position)

    if sector is None:
        return []

    current_distance = _track.distanceAlongTrack(car.position)
    if current_distance is None:
        return []
    
    ahead = []
    for obj in simulation().objects:
        if obj is car or not isinstance(obj, RacingCar):
            continue
        
        obj_distance = _track.distanceAlongTrack(obj.position)
        if obj_distance is None:
            continue
        
        # Check if ahead in the same sector
        if (obj_distance > current_distance and 
            sector.startDistance <= obj_distance < sector.endDistance):
            ahead.append(obj)
    
    return ahead

