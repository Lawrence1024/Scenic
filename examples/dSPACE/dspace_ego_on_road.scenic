# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LS_converted.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Simple straight road (center @ origin, heading +X, width 8 m, length 120 m) ---
road = RectangularRegion(0@0, 0 deg, 8, 120)

# --- Ego on the road, aligned with it, with a simple avoidance behavior ---
ego = new Car on road

# --- Another fellow car ahead of ego, to interact with ---
fellow = new Car ahead of ego by 20
# fellow = new Car with roadS 1020, with roadT 3.5