# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LS_converted.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model
fellow = new Car with roadS 1020, with roadT 3.5, with md_v 50.0