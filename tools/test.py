# Create car with TTL configuration
fellow1 = new RacingCar on mainRacingRoad, \
    with raceNumber 2, \
    ttlFileName 'ttl27_v5.csv', \
    ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV/needs_refine'), \
    ttlDX -53.6, \
    ttlDY -15.7

# Make it follow the TTL
fellow1.behavior = FollowRacingLineBehavior(
    target_speed=25,      # 25 m/s (~90 km/h)
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True    # Use waypoint-based control
)
# Set TTL using action
take SetTTLAction(ttl_region)

# Then use the behavior
do FollowRacingLineBehavior(target_speed=30)