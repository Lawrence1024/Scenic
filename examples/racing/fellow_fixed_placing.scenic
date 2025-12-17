# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at fixed coordinates ---
# These coordinates should be validated to be on-road
# Using 10 centerline points extracted from XODR file
ego = new RacingCar at (163.545, 48.302, 5.822)
fellow1 = new RacingCar at (-101.919263, -457.524908, 0.0)
fellow2 = new RacingCar at (0.948038, -272.443171, 0.0)
fellow3 = new RacingCar at (191.994781, -418.905118, 0.0)
fellow4 = new RacingCar at (162.256104, -693.627649, 0.0)
fellow5 = new RacingCar at (302.064561, -815.646205, 0.0)
fellow6 = new RacingCar at (557.639219, -737.139638, 0.0)
fellow7 = new RacingCar at (599.646200, -466.416118, 0.0)
fellow8 = new RacingCar at (438.050679, -47.247026, 0.0)
fellow9 = new RacingCar at (211.589136, -18.727096, 0.0)


