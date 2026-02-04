# Example: Ego vehicle using MPC controller for racing line following
#
# This example demonstrates how to use the MPC-based behavior for improved
# racing performance compared to PID controllers.

# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.05  
param batch_steps = 1   

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Racing domain model (brings in RacingCar/racing behaviors) ---
model scenic.domains.racing.model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)
ego = new RacingCar at (72.567889, 107.574718, 0.0), \
    with raceNumber 1, \
    # with ttlFileName 'ttl_17.csv', \
    with ttlFileName 'ttl_fellow_test_xodr_all.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV/transformed'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,      # 30 m/s (~108 km/h)
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    lookahead=20.0,       # 20m lookahead distance
    mpc_config_path=None  # Use default MPC config (debug_mpc/vehicle_mpc.yaml)
)

# Using main racing road centerline TTL (3541 waypoints, ~4.2km total length)
# Waypoints are in XODR coordinate system and guaranteed to be on-road
# TTL file: ttl_fellow_test_xodr_all.csv (XODR coordinates, transformed from dSPACE)
# Fellow vehicles placed every 100m starting from 200m (s = 200, 300, 400, ..., 3100)

fellow0 = new RacingCar at (55.766137, 88.269387), with regionContainedIn everywhere
fellow1 = new RacingCar at (-3.492303, 6.189075), with regionContainedIn everywhere
fellow2 = new RacingCar at (-62.884421, -75.794619), with regionContainedIn everywhere
fellow3 = new RacingCar at (-115.684056, -161.921028), with regionContainedIn everywhere
fellow4 = new RacingCar at (-155.148453, -255.104971), with regionContainedIn everywhere
fellow5 = new RacingCar at (-177.555348, -352.917631), with regionContainedIn everywhere
fellow6 = new RacingCar at (-174.479231, -454.098250), with regionContainedIn everywhere
fellow7 = new RacingCar at (-145.430430, -543.681887), with regionContainedIn everywhere
fellow8 = new RacingCar at (-99.975653, -481.458821), with regionContainedIn everywhere
fellow9 = new RacingCar at (-115.985953, -382.001237), with regionContainedIn everywhere
fellow10 = new RacingCar at (-97.793578, -283.512705), with regionContainedIn everywhere
fellow11 = new RacingCar at (-3.518108, -276.422083), with regionContainedIn everywhere
fellow12 = new RacingCar at (95.769228, -298.296819), with regionContainedIn everywhere
fellow13 = new RacingCar at (181.372434, -344.774728), with regionContainedIn everywhere
fellow14 = new RacingCar at (193.724497, -444.928062), with regionContainedIn everywhere
fellow15 = new RacingCar at (194.903362, -545.902247), with regionContainedIn everywhere
fellow16 = new RacingCar at (171.239669, -644.262831), with regionContainedIn everywhere
fellow17 = new RacingCar at (144.328113, -741.816551), with regionContainedIn everywhere
fellow18 = new RacingCar at (172.962818, -826.465840), with regionContainedIn everywhere
fellow19 = new RacingCar at (273.403955, -824.407752), with regionContainedIn everywhere
fellow20 = new RacingCar at (373.273279, -805.916141), with regionContainedIn everywhere
fellow21 = new RacingCar at (473.336847, -788.496042), with regionContainedIn everywhere
fellow22 = new RacingCar at (559.565826, -748.487066), with regionContainedIn everywhere
fellow23 = new RacingCar at (597.583400, -655.118158), with regionContainedIn everywhere
fellow24 = new RacingCar at (603.858052, -554.244525), with regionContainedIn everywhere
fellow25 = new RacingCar at (605.095199, -453.045086), with regionContainedIn everywhere
fellow26 = new RacingCar at (602.921578, -351.954314), with regionContainedIn everywhere
fellow27 = new RacingCar at (604.896597, -260.312319), with regionContainedIn everywhere
fellow28 = new RacingCar at (560.555962, -185.786077), with regionContainedIn everywhere
fellow29 = new RacingCar at (552.977085, -85.646447), with regionContainedIn everywhere