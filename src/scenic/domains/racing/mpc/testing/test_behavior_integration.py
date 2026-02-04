"""Integration tests for MPC behavior integration.

Tests that MPC behavior can be instantiated and works with simulation.
"""

import unittest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from scenic.domains.racing.mpc import MPCLateralController, load_mpc_config
from scenic.domains.driving.controllers import PIDLongitudinalController


class TestBehaviorIntegration(unittest.TestCase):
    """Test MPC behavior integration with simulation."""
    
    def test_mpc_controller_creation(self):
        """Test that MPC controller can be created with config."""
        try:
            config = load_mpc_config()
            mpc = MPCLateralController(config, timestep=0.05)
            
            self.assertIsNotNone(mpc)
            self.assertIsNotNone(mpc.config)
            self.assertEqual(mpc.timestep, 0.05)
        except FileNotFoundError:
            # Config file doesn't exist - skip test
            self.skipTest("MPC config file not found (expected in src/scenic/domains/racing/mpc/vehicle_mpc.yaml)")
        except Exception as e:
            self.fail(f"Failed to create MPC controller: {e}")
    
    def test_mpc_controller_interface(self):
        """Test that MPC controller has correct interface for behavior."""
        try:
            config = load_mpc_config()
            mpc = MPCLateralController(config, timestep=0.05)
            
            # Test that run_step exists and has correct signature
            self.assertTrue(hasattr(mpc, 'run_step'))
            self.assertTrue(callable(mpc.run_step))
            
            # Test with simple inputs
            vehicle_state = {
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'speed': 10.0,
            }
            waypoints = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]
            
            steering = mpc.run_step(vehicle_state, waypoints)
            
            # Should return normalized steering
            self.assertGreaterEqual(steering, -1.0)
            self.assertLessEqual(steering, 1.0)
        except FileNotFoundError:
            self.skipTest("MPC config file not found")
        except Exception as e:
            if "osqp" in str(e).lower():
                self.skipTest("osqp not installed")
            else:
                raise
    
    def test_simulation_controller_method(self):
        """Test that simulation can create MPC controllers."""
        try:
            # Mock simulation object
            class MockSimulation:
                def __init__(self):
                    self.timestep = 0.05
            
            # Import the actual method (would need real simulation for full test)
            from scenic.simulators.dspace.simulator import DSpaceSimulation
            
            # This test verifies the method signature exists
            # Full integration test would require actual simulation setup
            self.assertTrue(hasattr(DSpaceSimulation, 'getRacingControllers'))
            
        except ImportError:
            self.skipTest("dSPACE simulator not available")
    
    def test_mpc_pid_compatibility(self):
        """Test that MPC and PID controllers can coexist."""
        try:
            config = load_mpc_config()
            mpc = MPCLateralController(config, timestep=0.05)
            pid_lon = PIDLongitudinalController(K_P=0.8, K_D=0.15, K_I=0.9, dt=0.05)
            
            # Both should be callable
            self.assertTrue(callable(mpc.run_step))
            self.assertTrue(callable(pid_lon.run_step))
            
            # Both should return normalized values
            vehicle_state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'speed': 10.0}
            waypoints = [(0.0, 0.0), (10.0, 0.0)]
            
            steering = mpc.run_step(vehicle_state, waypoints)
            throttle = pid_lon.run_step(5.0)  # speed error
            
            self.assertGreaterEqual(steering, -1.0)
            self.assertLessEqual(steering, 1.0)
            self.assertGreaterEqual(throttle, -1.0)
            self.assertLessEqual(throttle, 1.0)
            
        except FileNotFoundError:
            self.skipTest("MPC config file not found")
        except Exception as e:
            if "osqp" in str(e).lower():
                self.skipTest("osqp not installed")
            else:
                raise


if __name__ == '__main__':
    unittest.main()

