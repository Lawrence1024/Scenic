# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/Laguna_Seca_OuterLoop_Optimized.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at specific coordinates ---
# Location 1: (57.466713, 79.726326, 4.488719)
fellow1 = new Car at (57.466713, 79.726326, 4.488719)

# Location 2: (-57.365067, -79.575279, 4.479858)
fellow2 = new Car at (-57.365067, -79.575279, 4.479858)

# Location 3: (-167.860733, -453.353821, 4.644358)
fellow3 = new Car at (-167.860733, -453.353821, 4.644358)

# Location 4: (-92.086708, -284.781311, 0.448285)
fellow4 = new Car at (-92.086708, -284.781311, 0.448285)
