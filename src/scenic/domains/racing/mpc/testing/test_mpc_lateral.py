"""Unit tests for MPCLateralController.

Tests MPC controller state computation, QP formulation, and control output.
"""

import unittest
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from scenic.domains.racing.mpc.mpc_lateral import MPCLateralController
from scenic.domains.racing.mpc.config import MPCConfig


class TestMPCLateralController(unittest.TestCase):
    """Test cases for MPCLateralController."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create minimal config
        config_dict = {
            'ctrl_period': 0.05,
            'mpc_prediction_horizon': 10,  # Smaller for testing
            'mpc_prediction_dt': 0.05,
            'wheel_base': 2.9718,
            'max_steer_angle': 0.2816,
            'steer_tau': 0.3,
            'steer_rate_lim': 1.0,
            'steer_cmd_max': 70,
            'w_ey': 2.0,
            'w_epsi': 0.5,
            'w_u': 0.2,
            'w_du': 5.0,
            'wT_ey': 5.0,
            'wT_epsi': 1.0,
            'admissible_position_error': 5.0,
            'admissible_yaw_error_rad': 1.57,
            'max_invalid_count': 10,
            'steering_lpf_cutoff_hz': 3.0,
            'traj_resample_dist': 0.2,
        }
        self.config = MPCConfig(config_dict)
        self.controller = MPCLateralController(self.config, timestep=0.05)
    
    def test_fallback_steering(self):
        """Test fallback steering behavior."""
        # Initially should return 0.0
        steer = self.controller._fallback_steering()
        self.assertEqual(steer, 0.0)
        
        # After setting last valid steering, should return that
        self.controller.last_valid_steering = 0.5
        self.controller.invalid_count = 5  # Below threshold
        steer = self.controller._fallback_steering()
        self.assertEqual(steer, 0.5)
        
        # After max invalid count, should return 0.0
        self.controller.invalid_count = 15  # Above threshold
        steer = self.controller._fallback_steering()
        self.assertEqual(steer, 0.0)
    
    def test_run_step_basic(self):
        """Test basic run_step with simple scenario."""
        vehicle_state = {
            'x': 5.0,
            'y': 0.0,
            'yaw': 0.0,
            'speed': 10.0,
        }
        waypoints = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)]
        
        # This will fail if osqp is not installed, but structure should be correct
        try:
            steering = self.controller.run_step(vehicle_state, waypoints)
            
            # Should return normalized steering in [-1, 1]
            self.assertGreaterEqual(steering, -1.0)
            self.assertLessEqual(steering, 1.0)
        except ImportError:
            # osqp not installed - skip this test
            self.skipTest("osqp not installed")
        except Exception as e:
            # Other errors are test failures
            if "osqp" in str(e).lower():
                self.skipTest("osqp not installed")
            else:
                raise


if __name__ == '__main__':
    unittest.main()

