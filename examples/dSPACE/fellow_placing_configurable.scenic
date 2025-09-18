# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/Laguna_Seca_OuterLoop_Optimized.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Example showing configurable relative positioning ---
# The system uses resolved coordinates from Scenic, not file parameters

# Reference car
fellow = new Car on road

# Car to the left - the system will detect this as lateral positioning
fellow_left = new Car left of fellow by 2

# Car behind - the system will detect this as longitudinal positioning  
fellow_behind = new Car behind fellow by 20

# The dSPACE simulator can be configured via the model file or simulation parameters
# Configuration options include:
# - lateral_calibration_factor: Calibration for t-coordinate interpretation
# - duplicate_position_threshold: Threshold for detecting identical positions
# - relative_distance_threshold: Maximum world distance for relative positioning
# - s_coordinate_threshold: Maximum s-coordinate difference for relative positioning
# - heading_difference_threshold: Maximum heading difference for relative positioning
# - lateral_pattern_*_threshold: Various thresholds for lateral pattern detection

# Example of how to configure (if needed):
# param lateral_calibration_factor = 0.5
# param relative_distance_threshold = 50.0
