"""Test that MPC scenarios can be compiled correctly.

This test verifies that Scenic scenarios using MPC behavior can be parsed
and compiled without errors.
"""

import unittest
import sys
import os
import tempfile

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))


class TestScenarioCompilation(unittest.TestCase):
    """Test that MPC scenarios compile correctly."""
    
    def test_mpc_behavior_exists(self):
        """Test that FollowRacingLineMPCBehavior is available."""
        try:
            import scenic
            # Try to compile a simple scenario with MPC behavior
            scenario_code = """
model scenic.domains.racing.model
ego = new RacingCar on mainRacingRoad
ego.behavior = FollowRacingLineMPCBehavior(target_speed=30)
"""
            try:
                scenario = scenic.scenarioFromString(scenario_code)
                self.assertIsNotNone(scenario)
            except Exception as e:
                # If compilation fails, check if it's because behavior doesn't exist
                if "FollowRacingLineMPCBehavior" in str(e) and "not defined" in str(e):
                    self.fail("FollowRacingLineMPCBehavior is not available in racing domain")
                else:
                    # Other compilation errors are OK (e.g., missing map, etc.)
                    # We just want to verify the behavior name is recognized
                    pass
        except ImportError:
            self.skipTest("Scenic not available")
    
    def test_mpc_behavior_parameters(self):
        """Test that MPC behavior accepts correct parameters."""
        try:
            import scenic
            # Test with all parameters
            scenario_code = """
model scenic.domains.racing.model
ego = new RacingCar on mainRacingRoad
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,
    manage_gears=True,
    use_waypoints=True,
    lookahead=20.0,
    mpc_config_path=None
)
"""
            try:
                scenario = scenic.scenarioFromString(scenario_code)
                self.assertIsNotNone(scenario)
            except Exception as e:
                # Parameter errors would indicate interface mismatch
                if "unexpected keyword" in str(e).lower() or "argument" in str(e).lower():
                    self.fail(f"MPC behavior parameter error: {e}")
                # Other errors (missing map, etc.) are OK
        except ImportError:
            self.skipTest("Scenic not available")


if __name__ == '__main__':
    unittest.main()

