# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0/10

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

import math

# def left_point_off(obj, distance):
#    return obj.position offset by distance * Vector(-1.0, 0.0) @ obj.orientation

# --- Fellow cars placed at validated coordinates ---

fellow1 = new Car on road
fellow2 = new Car left of fellow1 by 3

# fellow3 = new Car right of fellow1 by 10, facing 90 deg
# fellow2 = new Car left of fellow1 by 5
# fellow3 = new Car right of fellow1 by 5
# fellow4 = new Car on road
# fellow5 = new Car on road
# fellow6 = new Car on road
# fellow7 = new Car on road
# fellow8 = new Car on road
# fellow9 = new Car on road

