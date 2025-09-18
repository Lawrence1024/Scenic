# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/Laguna_Seca_OuterLoop_Optimized.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model
fellow = new Car on road
fellow2 = new Car behind fellow by 30
fellow3 = new Car ahead of fellow by 20
fellow4 = new Car left of fellow by 2
fellow5 = new Car right of fellow2 by 2