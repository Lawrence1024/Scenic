"""Example: Fellow Car Following TTL 27

This example shows how to configure a fellow car to follow a specific TTL file.
The TTL file ttl27_v5.csv is located in the needs_refine folder.

Key points:
1. Use ttlFileName to specify the exact CSV file (instead of ttlIndex)
2. Set ttlFolder to point to the correct directory
3. Assign FollowRacingLineBehavior to make the car follow the TTL
4. The simulator automatically attaches the TTL when creating the fellow vehicle
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

# Global TTL defaults (can be overridden per vehicle)
param ttlDX = -53.6         # Global X offset to apply to TTL points
param ttlDY = -15.7         # Global Y offset to apply to TTL points

model scenic.simulators.dspace.model

# ============================================================================
# Ego vehicle (optional - can use default TTL or specify one)
# ============================================================================
ego = new RacingCar on mainRacingRoad, with raceNumber 1
ego.behavior = FollowRacingLineBehavior(target_speed=30, manage_gears=True, use_waypoints=True)

# ============================================================================
# Fellow car following TTL 27 (ttl27_v5.csv)
# ============================================================================
# Option 1: Using ttlFileName to specify the exact file
fellow1 = new RacingCar on mainRacingRoad, \
    with raceNumber 2, \
    ttlFileName 'ttl27_v5.csv', \
    ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
    ttlDX 0.0, \
    ttlDY 0.0

# Assign behavior to make the fellow follow the TTL
fellow1.behavior = FollowRacingLineBehavior(target_speed=25, manage_gears=True, use_waypoints=True)

# ============================================================================
# Debugging: Visualizing TTL Waypoints
# ============================================================================
# There are several ways to see the GPS points/waypoints:
#
# 1. VISUALIZATION TOOL (Recommended):
#    Run this command to visualize the TTL waypoints in a 2D plot:
#    python tools/visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/ttl27_v5.csv --dx 0 --dy 0
#
#    With track overlay:
#    python tools/visualize_ttl.py assets/ttls/LS_ENU_TTL_CSV/ttl27_v5.csv \
#        --xodr assets/maps/dSPACE/LagunaSeca.xodr --dx -53.6 --dy -15.7
#
# 2. LOG WAYPOINTS DURING SIMULATION:
#    Import and use the LogWaypointsBehavior:
#    import 'log_waypoints_behavior.scenic'
#    fellow1.behavior = LogWaypointsBehavior(print_interval=10)  # Log every 10 steps
#
# 3. ACCESS WAYPOINTS IN CODE:
#    After TTL is loaded, access via:
#    - fellow1.waypoints: List of (x, y) tuples representing the TTL path
#    - fellow1.ttl: PolylineRegion representing the TTL
#    
#    Example:
#    if hasattr(fellow1, 'waypoints'):
#        print(f"Loaded {len(fellow1.waypoints)} waypoints")
#        for i, (x, y) in enumerate(fellow1.waypoints[:10]):  # First 10
#            print(f"  Waypoint {i}: ({x:.2f}, {y:.2f})")

# ============================================================================
# Alternative: If you want to use ttlIndex instead (for files in 'usable' folder)
# ============================================================================
# fellow2 = new RacingCar on mainRacingRoad, \
#     with raceNumber 3, \
#     ttlIndex 17, \
#     ttlDX -53.6, \
#     ttlDY -15.7
# fellow2.behavior = FollowRacingLineBehavior(target_speed=28, manage_gears=True, use_waypoints=True)

