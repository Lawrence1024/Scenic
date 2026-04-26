"""Scenic world model for racing scenarios.

This model extends :doc:`scenic.domains.driving.model` with racing-specific
objects, regions, and constraints.

The map file must be specified using the ``map`` global parameter, and racing-specific
parameters can be set:

Example::

    param map = localPath('laguna_seca.xodr')
    param use2DMap = True
    param trackDirection = 'counterclockwise'  # or 'clockwise'
    model scenic.domains.racing.model
"""

# Import everything from driving domain
from scenic.domains.driving.model import *
from scenic.domains.racing.tracks import RacingTrack, createRacingTrack
from scenic.domains.racing.behaviors import *
from scenic.domains.racing.actions import *
from scenic.core.regions import UnionRegion
from scenic.domains.racing.segments.track_regions import create_track_regions, create_ttl_region_from_file

## Racing-specific parameters

param trackDirection = 'counterclockwise'

# Track segment identification (optional)
param pitLaneRoadId = None  # e.g., "1545702203" for Laguna Seca
param pitLaneRoadName = "pit"  # Pattern to match pit lane name
param mainLineRoadId = None  # e.g., "2117817291" for Laguna Seca
# Junction assignment: OpenDRIVE road IDs of connecting roads for outer loop vs pit (e.g. (24, 34) and (25, 30))
param main_loop_connecting_road_ids = None  # e.g. (24, 34) for two junctions
param pit_connecting_road_ids = None  # e.g. (25, 30)

# Track regions are built from XODR road geometry by default (Phase B.5, 2026-04-26).
# `ttlFolder` is still required for racing-line CSVs (ttl_optimal_xodr.csv, etc.) and for
# the `on ttl` placement helper, but mainTrack / pitTrack come from the XODR road network
# (verified to match race_common's geofence within 0.83m mean). Set
# `preferTtlTrackRegions=True` to revert to the legacy TTL-CSV-buffered behavior.
param ttlFolder = None  # e.g. localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param ttlFileName = None  # default TTL file for "on ttl" (e.g. 'ttl_optimal_xodr.csv')
param mainTrackBuffer = 6.0   # meters on each side of main road centerline (±6 m)
param pitTrackBuffer = 1.5   # meters on each side of pit road centerline (±1.5 m)
param preferTtlTrackRegions = False  # legacy: True -> use TTL CSV centerlines (ttl_main_road.csv) instead of XODR

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

## Racing-specific regions (XODR-native by default; ±buffer around each road's centerline):
## - mainTrack: ±mainTrackBuffer around _track._mainRacingRoads centerlines (includes Corkscrew + Andretti links)
## - pitTrack:  ±pitTrackBuffer around _track._pitRoads centerlines (mutually exclusive with mainTrack -- main wins on overlap)
## - ttl: a single TTL centerline (param ttlFileName) buffered with mainTrackBuffer; for `new RacingCar on ttl`
## Set param preferTtlTrackRegions=True to revert to the legacy TTL-CSV-derived regions.

_mainTrack, _pitTrack, _ = create_track_regions(
    map_file=globalParameters.map,
    ttl_folder=globalParameters.ttlFolder if globalParameters.ttlFolder else None,
    track=_track,
    main_buffer_m=globalParameters.mainTrackBuffer,
    pit_buffer_m=globalParameters.pitTrackBuffer,
    prefer_ttl_track_regions=globalParameters.preferTtlTrackRegions,
    direction=globalParameters.trackDirection,
    pitLaneRoadId=globalParameters.pitLaneRoadId,
    pitLaneRoadName=globalParameters.pitLaneRoadName,
    mainLineRoadId=globalParameters.mainLineRoadId,
    main_loop_connecting_road_ids=globalParameters.main_loop_connecting_road_ids,
    pit_connecting_road_ids=globalParameters.pit_connecting_road_ids,
    **globalParameters.map_options
)
mainTrack: Region = _mainTrack
pitTrack: Region = _pitTrack

# ttl: region from param ttlFileName (random point on that TTL). Fallback to mainTrack if no ttlFolder/ttl.
_ttl = create_ttl_region_from_file(
    globalParameters.ttlFolder,
    globalParameters.ttlFileName if globalParameters.ttlFileName else 'ttl_main_road.csv',
    globalParameters.mainTrackBuffer
) if globalParameters.ttlFolder else None
ttl: Region = _ttl if _ttl else mainTrack

# Helper for per-car TTL file: new RacingCar on ttlRegion('ttl_optimal_xodr.csv')
def ttlRegion(ttlFileName): (create_ttl_region_from_file(
    globalParameters.ttlFolder, ttlFileName, globalParameters.mainTrackBuffer
) if globalParameters.ttlFolder else None) or mainTrack

# Backward compatibility: mainRacingRoad and pitLaneRoad alias to mainTrack and pitTrack
mainRacingRoad: Region = mainTrack
pitLaneRoad: Region = pitTrack

# Keep racingLine as the TTL default (can be overridden by actions)
racingLine: Region = _track.racingLine.region if hasattr(_track, 'racingLine') and _track.racingLine else mainTrack

#: Start/finish line region
# TODO: Create actual start/finish line region from track data

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
    # IAC Dallara AV-21 / AV-24 rules footprint (192 in × 76 in); center-based OBB in eval logs.
    carType: "Dallara IAC"
    width: 1.9304
    length: 4.8768
    
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
    
        cars = carsInFormation([mainTrack, mainTrack, mainTrack])  # 3 cars on main track
    
    Args:
        positions: List of positions or regions (e.g. mainTrack repeated, or a list of specific regions)
        
    Returns:
        List of RacingCar objects
    """
    cars = []
    for i, pos in enumerate(positions):
        car = new RacingCar at pos, with raceNumber (i + 1)
        cars.append(car)
    return cars

