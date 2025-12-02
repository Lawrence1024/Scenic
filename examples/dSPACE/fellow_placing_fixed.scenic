# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at fixed coordinates ---
# These coordinates should be validated to be on-road
fellow1 = new Car at (-115.049129, -360.688095, 0.000000)
fellow2 = new Car at (601.412408, -432.289069, 0.000000)
fellow3 = new Car at (-101.919263, -457.524908, 0.000000)
fellow4 = new Car at (-88.818771, -260.414600, 0.000000)
fellow5 = new Car at (24.153787, -282.000078, 0.000000)
fellow6 = new Car at (121.433853, -305.520242, 0.000000)
fellow7 = new Car at (190.656480, -367.454840, 0.000000)
fellow8 = new Car at (196.063775, -467.757338, 0.000000)
fellow9 = new Car at (190.721348, -567.074749, 0.000000)

