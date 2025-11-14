"""Example: Mixed TTL Configuration for Multiple Vehicles

This example demonstrates Option 3: Mixing global and per-vehicle TTL configuration.

Features demonstrated:
- Global TTL defaults (applied to vehicles without specific config)
- Per-vehicle TTL overrides (using object properties)
- Multiple vehicles with different TTLs
- Automatic TTL attachment for both ego and fellows

TTL Configuration Priority:
1. Object-specific properties (obj.ttlIndex, obj.ttlDX, obj.ttlDY, etc.)
2. Scene parameters (ttlIndex, ttlDX, ttlDY, etc.)
3. Default values (index=17, dx=-53.6, dy=-15.7)

Available TTL indices for Laguna Seca: 2, 3, 9, 15, 16, 17
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

# Global TTL defaults - these apply to vehicles without specific TTL config
param ttlNumber = 17        # Default TTL index (choose among: 2, 3, 9, 15, 16, 17)
param ttlDX = -53.6         # Global X offset to apply to TTL points
param ttlDY = -15.7         # Global Y offset to apply to TTL points
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV/usable')

model scenic.simulators.dspace.model

# ============================================================================
# Example 1: Ego with global TTL (uses scene params: ttlNumber=17)
# ============================================================================
ego = new RacingCar on mainRacingRoad, with raceNumber 1
# Ego automatically gets TTL 17 from global params
ego.behavior = FollowRacingLineBehavior(target_speed=30, manage_gears=True, use_waypoints=True)

# ============================================================================
# Example 2: Fellow with different TTL (object-specific override)
# ============================================================================
fellow1 = new RacingCar on mainRacingRoad, \
    with raceNumber 2, \
    ttlIndex 15, \
    ttlDX -53.6, \
    ttlDY -15.7
# Fellow1 gets TTL 15 (overrides global ttlNumber=17)
# Uses same offset as global (can be overridden if needed)
fellow1.behavior = FollowRacingLineBehavior(target_speed=25, manage_gears=True, use_waypoints=True)

# ============================================================================
# Example 3: Another fellow with yet another TTL
# ============================================================================
fellow2 = new RacingCar on mainRacingRoad, \
    with raceNumber 3, \
    ttlIndex 9, \
    ttlDX -53.6, \
    ttlDY -15.7
# Fellow2 gets TTL 9 (different from both ego and fellow1)
fellow2.behavior = FollowRacingLineBehavior(target_speed=20, manage_gears=True, use_waypoints=True)

# ============================================================================
# Example 4: Fellow using global TTL (no override)
# ============================================================================
fellow3 = new RacingCar on mainRacingRoad, with raceNumber 4
# Fellow3 uses global TTL 17 (same as ego) - no object-specific config
fellow3.behavior = FollowRacingLineBehavior(target_speed=28, manage_gears=True, use_waypoints=True)

# ============================================================================
# Example 5: Fellow with partial override (only ttlIndex, uses global offset)
# ============================================================================
fellow4 = new RacingCar on mainRacingRoad, \
    with raceNumber 5, \
    ttlIndex 16
# Fellow4 gets TTL 16, but uses global ttlDX and ttlDY from scene params
fellow4.behavior = FollowRacingLineBehavior(target_speed=22, manage_gears=True, use_waypoints=True)

# ============================================================================
# Summary:
# - ego:        TTL 17 (global default)
# - fellow1:    TTL 15 (object-specific)
# - fellow2:    TTL 9  (object-specific)
# - fellow3:    TTL 17 (global default, no override)
# - fellow4:    TTL 16 (object-specific index, global offset)
# ============================================================================

