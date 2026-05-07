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
from scenic.domains.racing.segments.track_regions import (
    create_track_regions,
    create_ttl_region_from_file,
    build_curve_straight_regions_from_opendrive,
    ttl_category as _ttl_category,
)

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

# SD-24c: expose the track regions via params so the simulator's placement
# code (placement.py:place_ego, place_fellow) can do polygon-membership tests
# (mainTrack.containsPoint vs pitTrack.containsPoint) for the contradiction
# warning without re-running unionAll. Set as params (not just module
# globals) because placement.py reaches the regions via sim.scene.params.
param mainTrackRegion = _mainTrack
param pitTrackRegion = _pitTrack

# SD-19c: union of main + pit so users can write `new RacingCar on raceTrack`
# (any drivable surface) without spelling out the union. We chose the alias
# approach over subclassing RacingTrack from PolygonalRegion because it
# satisfies the same user intent with one line and zero risk to RacingTrack's
# existing __init__ semantics. RacingTrack remains a Python wrapper; raceTrack
# is the Region for `on` specifiers. Uses UnionRegion (already imported above).
raceTrack: Region = UnionRegion(_mainTrack, _pitTrack) if (_mainTrack and _pitTrack) else (_mainTrack or _pitTrack or mainTrack)

# SD-24: curve / straight track regions, built from the per-station curvature
# classifier in segments/segment_map.py and sliced into per-segment polygons by
# segments/track_regions.py:slice_road_polygon_at_segments. Six top-level
# Regions plus the unified `trackRegion(ttlFileName, segment)` helper below.
# - curve / straight: full union (both pit + main) — use when no implicit track context.
# - mainCurve / mainStraight / pitCurve / pitStraight: cross-product polygons.
#   These are the building blocks `trackRegion(...)` references internally and
#   are also exposed for explicit composition.
_cs_regions = build_curve_straight_regions_from_opendrive(_track)
curve: Region        = _cs_regions['curve']
straight: Region     = _cs_regions['straight']
mainCurve: Region    = _cs_regions['mainCurve']
mainStraight: Region = _cs_regions['mainStraight']
pitCurve: Region     = _cs_regions['pitCurve']
pitStraight: Region  = _cs_regions['pitStraight']

# ttl: PolylineRegion from param ttlFileName (random point exactly on that TTL).
# Fallback to mainTrack if no ttlFolder/ttl.
# SD-19b: PolylineRegion (no buffer) -- samples land on the line, not in a corridor.
_ttl = create_ttl_region_from_file(
    globalParameters.ttlFolder,
    globalParameters.ttlFileName if globalParameters.ttlFileName else 'ttl_main_road.csv',
) if globalParameters.ttlFolder else None
ttl: Region = _ttl if _ttl else mainTrack

# SD-24: unified placement-region pipeline.
#
# Replaces the old `ttlRegion(name)` helper. Decision tree:
#
#     ttlFileName | segment   | result
#     ------------+-----------+-------------------------------------------
#     'optimal'   | None      | TTL polyline                  (today)
#     'optimal'   | 'curve'   | TTL polyline ∩ mainCurve
#     'optimal'   | 'straight'| TTL polyline ∩ mainStraight
#     'pit'       | None      | TTL polyline                  (today)
#     'pit'       | 'curve'   | TTL polyline ∩ pitCurve
#     'pit'       | 'straight'| TTL polyline ∩ pitStraight
#     None        | None      | mainTrack                     (today's fallback)
#     None        | 'curve'   | curve   (full union, both pit + main)
#     None        | 'straight'| straight
#
# All branches return a Region. RacingCar's `position` default routes through
# this helper (no-segment case = today's behaviour, byte-identical), and
# users can pass `segment='curve'` / `'straight'` to filter. The
# cross-product regions (`mainCurve` etc.) above are still exposed as
# top-level names for users who want explicit polygon access without the
# pipeline.
def trackRegion(ttlFileName, segment=None):
    cat = _ttl_category(ttlFileName)
    # Step 1+2: choose base region from TTL availability.
    if ttlFileName and globalParameters.ttlFolder:
        base = create_ttl_region_from_file(
            globalParameters.ttlFolder, ttlFileName
        )
        if base is None:
            base = mainTrack
    else:
        base = mainTrack
    # Step 3: optional segment filter.
    if segment is None:
        return base
    if segment == 'curve':
        if cat == 'main':
            return base.intersect(mainCurve)
        if cat == 'pit':
            return base.intersect(pitCurve)
        return curve
    if segment == 'straight':
        if cat == 'main':
            return base.intersect(mainStraight)
        if cat == 'pit':
            return base.intersect(pitStraight)
        return straight
    # Unknown segment string — fall back to base for forward-compat.
    return base

# Backwards-compat alias so legacy scenarios written against
# `on ttlRegion(self.ttlFileName)` keep working unchanged. SD-24 routes all
# callers through `trackRegion`; this is the same behaviour for the
# no-segment case.
def ttlRegion(ttlFileName):
    return trackRegion(ttlFileName)

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
    
    # Per-vehicle TTL configuration. Declared up-front so the `position`
    # default can self-reference `self.ttlFileName` (the Scenic compiler
    # records the dependency and samples ttlFileName before position).
    ttlFolder: None
    ttlFileName: None

    # Default racing properties
    speed: 25  # Higher default speed for racing (90 km/h)
    # SD-24: routed through the unified `trackRegion(ttlFileName, segment)`
    # pipeline. Behaviour for the no-segment default is byte-identical to
    # the pre-SD-24 `ttlRegion(self.ttlFileName)`: sample on the TTL polyline
    # if a ttlFileName is set, fall back to mainTrack otherwise. Users who
    # want a curve / straight filter can override the default with
    # `with position new Point on trackRegion(self.ttlFileName, 'curve')`.
    # Explicit `at (x, y)` / `on mainTrack` etc. still override normally.
    position: new Point on trackRegion(self.ttlFileName)
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
    # tireWear ∈ [0, 1]: 0 = fresh tires (full grip), 1 = fully worn (30% grip
    # loss per stability_guard's tire_wear_grip_loss). SD-44 Action C wired
    # this into the friction-circle brake-steer coupling: as tireWear rises,
    # the controller's available brake authority while steering shrinks
    # proportionally. Was a declared-but-unread property until SD-44 made it
    # physically meaningful.
    tireWear: 0.0
    
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

