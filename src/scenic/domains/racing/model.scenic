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

## Create racing track from the network

#: The racing track, extending the road network with racing features
track: RacingTrack = createRacingTrack(
    globalParameters.map, 
    direction=globalParameters.trackDirection,
    pitLaneRoadId=globalParameters.pitLaneRoadId,
    pitLaneRoadName=globalParameters.pitLaneRoadName,
    mainLineRoadId=globalParameters.mainLineRoadId,
    **globalParameters.map_options
)

# Replace the generic network with our racing track's network
network = track.network

# Store road segment IDs in params for simulator access
param pitLaneRoadIds = [str(track.pitLaneRoad.id)] if track.pitLaneRoad else []
param mainRacingRoadIds = [str(r.id) for r in track._mainRacingRoads] if track._mainRacingRoads else []

## Racing-specific regions

#: The pit lane region (if track has a pit lane)
pitLane: Region = track.pitLane.region if track.pitLane else nowhere

#: The main racing line region (excluding pit lane)
racingLine: Region = road.difference(pitLane) if track.pitLane else road

#: Individual track segment regions (mutually exclusive)
#: Main racing road includes all non-pit roads (main line + parallel tracks)
mainRacingRoad: Region = track.mainRacingRoad if track.mainRacingRoad else road
#: Pit lane road (separate from main racing circuit) - create proper region from road lanes
pitLaneRoad: Region = UnionRegion(*[lane for lane in track.pitLaneRoad.lanes]) if track.pitLaneRoad and track.pitLaneRoad.lanes and len(track.pitLaneRoad.lanes) > 1 else (track.pitLaneRoad.lanes[0] if track.pitLaneRoad and track.pitLaneRoad.lanes else nowhere)

#: Start/finish line region
# TODO: Create actual start/finish line region from track data

## Starting grid

# Generate starting grid if requested
if globalParameters.generateStartingGrid:
    #: List of starting grid positions
    startingGrid = track.generateStartingGrid(
        numPositions=globalParameters.startingGridPositions,
        spacing=globalParameters.startingGridSpacing
    )
else:
    startingGrid = []

## Racing-specific object types

class RacingCar(Car):
    """A racing car optimized for track performance.
    
    Extends the standard Car with racing-specific properties.
    
    Properties:
        speed: Default speed higher than regular cars (20 m/s = 72 km/h)
        position: Default position is on the racing line
        requireVisible: False (multiple cars in close proximity on track)
        raceNumber: Number displayed on the car (for identification)
        team: Team name or identifier
        fuelLevel: Current fuel level (0.0 to 1.0)
        tireWear: Tire wear level (0.0 = new, 1.0 = worn out)
    """
    speed: 20  # Higher default speed for racing
    position: new Point on racingLine
    requireVisible: False
    
    # Racing-specific properties
    raceNumber: Range(1, 99)
    team: None
    fuelLevel: Range(0.5, 1.0)  # Start with reasonable fuel
    tireWear: 0.0  # Start with fresh tires

class FormulaCar(RacingCar):
    """An open-wheel formula racing car (F1, F2, IndyCar style).
    
    Properties:
        width: Narrower than standard cars (2.0m)
        length: Longer for aerodynamics (5.5m)  
        speed: Higher default speed (25 m/s = 90 km/h)
    """
    width: 2.0
    length: 5.5
    speed: 25

class GTCar(RacingCar):
    """A GT (Grand Touring) racing car.
    
    Properties:
        width: Similar to road cars (2.0m)
        length: Standard racing car length (4.8m)
    """
    width: 2.0
    length: 4.8

class PrototypeCar(RacingCar):
    """A prototype racing car (LMP1, LMP2, DPi style).
    
    Properties:
        width: Wider for stability (2.0m)
        length: Longer for aerodynamics (5.0m)
        speed: Very high default speed (30 m/s = 108 km/h)
    """
    width: 2.0
    length: 5.0
    speed: 30

class PitCrew(Pedestrian):
    """A member of a pit crew.
    
    Properties:
        position: Default position is in the pit lane
        team: Team identifier
    """
    position: new Point on pitLane if track.pitLane else new Point on sidewalk
    team: None

class TrackMarshal(Pedestrian):
    """A track marshal (safety official).
    
    Properties:
        position: Positioned near track boundaries or critical points
        stationNumber: Marshal post number
    """
    position: new Point on shoulder
    stationNumber: None

## Racing-specific utility functions

def carsInFormation(positions, carType=RacingCar):
    """Create a formation of cars at the given positions.
    
    Example::
    
        cars = carsInFormation(startingGrid[:10])
    
    Args:
        positions: List of positions (typically from startingGrid)
        carType: Type of car to create (default: RacingCar)
        
    Returns:
        List of car objects
    """
    cars = []
    for i, pos in enumerate(positions):
        car = new carType at pos, with raceNumber (i + 1)
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
    if track.racingLine is None:
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
    sector = track.getSectorAt(car.position)
    if sector is None:
        return None
    
    current_distance = track.distanceAlongTrack(car.position)
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
        sector = track.getSectorAt(car.position)
    
    if sector is None:
        return []
    
    current_distance = track.distanceAlongTrack(car.position)
    if current_distance is None:
        return []
    
    ahead = []
    for obj in simulation().objects:
        if obj is car or not isinstance(obj, RacingCar):
            continue
        
        obj_distance = track.distanceAlongTrack(obj.position)
        if obj_distance is None:
            continue
        
        # Check if ahead in the same sector
        if (obj_distance > current_distance and 
            sector.startDistance <= obj_distance < sector.endDistance):
            ahead.append(obj)
    
    return ahead

