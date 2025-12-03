"""
Square racetrack example using the robotics domain with VerifAI parameters.

This demonstrates a robot following a square race track with inner and outer boundaries.
Parameters marked with VerifaiRange can be varied by VerifAI falsification.

Uses Frenet frame: "s" parameter represents arc length along the track centerline.
"""

model scenic.simulators.webots.robotics_model
from scenic.domains.robotics.behaviors import SquareTrackBehavior
from scenic.core.external_params import VerifaiRange
import math

# Define track centerline waypoints (same as SquareTrackBehavior)
# Waypoints form a square: [(-1.5, 1.5), (1.5, 1.5), (1.5, -1.5), (-1.5, -1.5)]
track_waypoints = [(-1.5, 1.5), (1.5, 1.5), (1.5, -1.5), (-1.5, -1.5)]
# Side length of each segment (meters)
side_length = 3.0
# Total track perimeter (meters)
track_perimeter = 4 * side_length  # 12.0 meters

# Define track centerline as a closed polyline (add first point at end to close loop)
track_points = track_waypoints + [track_waypoints[0]]
track_centerline = PolylineRegion(track_points)

# Define a workspace region (4.5x4.5 meter area centered at origin to accommodate robot size)
workspace_region = RectangularRegion((0, 0, 0.016), 0, 4.5, 4.5)
workspace = Workspace(workspace_region)

# Parameter: arc length along track centerline (s) - using Frenet frame
# Normalize s to be within track perimeter (wraps around)
param s = VerifaiRange(0, track_perimeter)

# Create a Pololu robot that follows the square track
# Position determined by arc length "s" along the track centerline using pointAlongBy
# This method works with distributions - Scenic handles it properly
# Normalize s to track perimeter and set z coordinate to 0.016 (robot height)
s_normalized = globalParameters.s % track_perimeter
s_pos_2d = track_centerline.pointAlongBy(s_normalized, normalized=False)
robot = new WebotsPololuRobot at (s_pos_2d.x, s_pos_2d.y, 0.016), 
    with behavior SquareTrackBehavior(
        forwardSpeed=VerifaiRange(50, 100), 
        turnSpeed=VerifaiRange(40, 80), 
        headingOffset=-90 deg
    )

# Terminate after 120 seconds
terminate after 120 seconds
