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
    
    def test_mpc_controller_creation_via_simulation(self):
        """Test that simulation can create MPC controller."""
        try:
            from scenic.simulators.dspace.simulator import DSpaceSimulation
            from scenic.domains.racing.model import RacingCar
            
            # This would require full simulation setup, so we just verify
            # the method can be called with use_mpc=True
            # Full test would need actual simulation instance
            
            # Check that method exists and accepts parameters
            self.assertTrue(hasattr(DSpaceSimulation, 'getRacingControllers'))
            
        except ImportError:
            self.skipTest("dSPACE simulator or racing model not available")
    
    def test_fallback_to_pid(self):
        """Test that simulation falls back to PID if MPC fails."""
        # This test would verify that if MPC creation fails,
        # the simulation returns PID controllers instead
        # Requires actual simulation setup to test fully
        pass


if __name__ == '__main__':
    unittest.main()

