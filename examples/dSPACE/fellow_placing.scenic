# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/Laguna_seca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model
# fellow = new Car with roadS 1280, with roadT 0.0, with md_v 0.0
fellow = new Car on road
fellow2 = new Car ahead of fellow by 14
fellow3 = new Car behind fellow by 14