# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0
param use2DMap = True

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at fixed coordinates ---
# These coordinates are from ttl_fellow_test_xodr.csv (XODR coordinate system)
# Total of 26 waypoints, using each waypoint as a fellow car position
# Using regionContainedIn everywhere to bypass strict validation since these are
# centerline waypoints that should theoretically be on-road but may fail strict
# container validation due to vehicle bounding box checks
ego = new RacingCar at (72.567889, 107.574718)
fellow0 = new RacingCar at (55.7661373818, 88.2693869080), with regionContainedIn everywhere
fellow1 = new RacingCar at (41.1449578427, 67.6113956345), with regionContainedIn everywhere
fellow2 = new RacingCar at (26.3599794929, 47.0694094007), with regionContainedIn everywhere
fellow3 = new RacingCar at (11.4636123361, 26.6075103770), with regionContainedIn everywhere
fellow4 = new RacingCar at (-3.4923031503, 6.1890751806), with regionContainedIn everywhere
fellow5 = new RacingCar at (-18.4565417466, -14.2232905053), with regionContainedIn everywhere
fellow6 = new RacingCar at (-33.3777258257, -34.6667589842), with regionContainedIn everywhere
fellow7 = new RacingCar at (-48.2043583544, -55.1783905109), with regionContainedIn everywhere
fellow8 = new RacingCar at (-62.8844214822, -75.7946194335), with regionContainedIn everywhere
fellow9 = new RacingCar at (-77.3649294828, -96.5505886570), with regionContainedIn everywhere
fellow10 = new RacingCar at (-91.5914181644, -117.4814491857), with regionContainedIn everywhere
fellow11 = new RacingCar at (-104.5288820054, -139.2148481080), with regionContainedIn everywhere
fellow12 = new RacingCar at (-115.6840555702, -161.9210281646), with regionContainedIn everywhere
fellow13 = new RacingCar at (-125.6932239111, -185.1577120926), with regionContainedIn everywhere
fellow14 = new RacingCar at (-135.2321288878, -208.5907848523), with regionContainedIn everywhere
fellow15 = new RacingCar at (-144.9833916092, -231.9328794205), with regionContainedIn everywhere
fellow16 = new RacingCar at (-155.1484531503, -255.1049714528), with regionContainedIn everywhere
fellow17 = new RacingCar at (-164.5374651440, -278.5943361484), with regionContainedIn everywhere
fellow18 = new RacingCar at (-172.0360212680, -302.7400201382), with regionContainedIn everywhere
fellow19 = new RacingCar at (-176.3754013495, -327.6394554609), with regionContainedIn everywhere
fellow20 = new RacingCar at (-177.5553482125, -352.9176310599), with regionContainedIn everywhere
fellow21 = new RacingCar at (-177.6938728293, -378.2274249216), with regionContainedIn everywhere
fellow22 = new RacingCar at (-177.0750349789, -403.5335737714), with regionContainedIn everywhere
fellow23 = new RacingCar at (-175.9277664245, -428.8233285756), with regionContainedIn everywhere
fellow24 = new RacingCar at (-174.4792314600, -454.0982500996), with regionContainedIn everywhere
fellow25 = new RacingCar at (-172.9549552708, -479.3684330664), with regionContainedIn everywhere

