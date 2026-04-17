"""Integration tests for simulation-level MPC integration.

Tests that DSpaceSimulation can create and return MPC controllers.
"""

import unittest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))


class TestSimulationIntegration(unittest.TestCase):
    """Test simulation-level MPC integration."""

    def test_get_racing_controllers_signature(self):
        """Test that getRacingControllers accepts use_mpc parameter."""
        try:
            from scenic.simulators.dspace.simulator import DSpaceSimulation

            # Check method exists and has correct signature
            import inspect
            sig = inspect.signature(DSpaceSimulation.getRacingControllers)
            params = list(sig.parameters.keys())

            # Should have agent, use_mpc, mpc_config_path parameters
            self.assertIn('agent', params)
            self.assertIn('use_mpc', params)
            self.assertIn('mpc_config_path', params)

        except ImportError:
            self.skipTest("dSPACE simulator not available")


if __name__ == '__main__':
    unittest.main()
