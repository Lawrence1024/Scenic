"""
Square racetrack example with VerifAI parameters for falsification.

This scenario uses VerifaiRange parameters that can be systematically
varied by VerifAI falsification to find parameter values that cause
specification violations.

To run falsification on this scenario:
    python falsify_pololu.py --scenario pololu_falsify.scenic
"""

model scenic.simulators.webots.robotics_model
from scenic.domains.robotics.behaviors import SquareTrackBehavior
from scenic.core.external_params import VerifaiRange

# Set VerifAI sampler type for falsification
param verifaiSamplerType = 'ce'

# Define parameters that will be varied during falsification
# These use VerifaiRange instead of Range to enable VerifAI control
param ROBOT_X = VerifaiRange(-2.0, 2.0)        # Initial X position
param ROBOT_Y = VerifaiRange(-2.0, 2.0)        # Initial Y position  
param FORWARD_SPEED = VerifaiRange(50, 100)    # Robot forward speed (0-100)
param TURN_SPEED = VerifaiRange(40, 80)        # Robot turn speed (0-100)
param HEADING_OFFSET = VerifaiRange(-180, 180) # Initial heading offset (degrees)

# Define workspace region (4.5x4.5 meter area centered at origin)
workspace_region = RectangularRegion((0, 0, 0.016), 0, 4.5, 4.5)
workspace = Workspace(workspace_region)

# Create a Pololu robot with falsifiable parameters
# Access parameters via globalParameters to ensure they're available
# Assign to ego so it's the first object (required for VerifAI)
ego = new WebotsPololuRobot at (globalParameters.ROBOT_X, globalParameters.ROBOT_Y, 0.016), 
    facing globalParameters.HEADING_OFFSET deg,
    with behavior SquareTrackBehavior(
        forwardSpeed=globalParameters.FORWARD_SPEED, 
        turnSpeed=globalParameters.TURN_SPEED, 
        headingOffset=-90 deg
    )

# Terminate after 120 seconds
terminate after 120 seconds

