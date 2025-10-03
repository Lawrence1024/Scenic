# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at validated coordinates ---
fellow1 = new Car on road
fellow2 = new Car on road
fellow3 = new Car on road
fellow4 = new Car on road
fellow5 = new Car on road
fellow6 = new Car on road
fellow7 = new Car on road
fellow8 = new Car on road
fellow9 = new Car on road

