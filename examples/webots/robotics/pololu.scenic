"""
Square racetrack example using the robotics domain.

This demonstrates a robot following a square race track with inner and outer boundaries.
"""

model scenic.simulators.webots.robotics_model
from scenic.domains.robotics.behaviors import SquareTrackBehavior

# Define a workspace region (4.5x4.5 meter area centered at origin to accommodate robot size)
workspace_region = RectangularRegion((0, 0, 0.016), 0, 4.5, 4.5)
workspace = Workspace(workspace_region)

# Create a Pololu robot that follows the square track
# Position at first waypoint to start the track properly
robot = new WebotsPololuRobot at (-1.5, -1.5, 0.016), with behavior SquareTrackBehavior(forwardSpeed=80, turnSpeed=60, headingOffset=-90 deg)

# Terminate after 120 seconds
terminate after 120 seconds
