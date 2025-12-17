# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at fixed coordinates ---
# These coordinates should be validated to be on-road
ego = new RacingCar at (163.545, 48.302, 5.822)

