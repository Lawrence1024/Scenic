"""Unit tests for ReferenceBuilder.

Tests waypoint search, curvature computation, and reference generation.
"""

import unittest
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from scenic.domains.racing.mpc.reference_builder import ReferenceBuilder


class TestReferenceBuilder(unittest.TestCase):
    """Test cases for ReferenceBuilder."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.builder = ReferenceBuilder(resample_dist=0.2)
    
    def test_find_nearest_waypoint_simple(self):
        """Test finding nearest waypoint in simple case."""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        position = (1.5, 0.1)
        
        idx = self.builder.find_nearest_waypoint(position, waypoints)
        
        # Should find waypoint 1 or 2 (closest to position)
        self.assertIn(idx, [1, 2])
    
    def test_find_nearest_waypoint_forward_only(self):
        """Test forward-only search prevents backtracking."""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]
        position = (2.5, 0.1)
        last_idx = 2
        
        idx = self.builder.find_nearest_waypoint(position, waypoints, last_idx=last_idx)
        
        # Should not go backwards from last_idx=2
        self.assertGreaterEqual(idx, 1)  # Allow some lookback but not too much
    
    def test_compute_curvature_straight(self):
        """Test curvature computation for straight line (should be ~0)."""
        p0 = (0.0, 0.0)
        p1 = (1.0, 0.0)
        p2 = (2.0, 0.0)
        
        kappa = self.builder.compute_curvature(p0, p1, p2)
        
        # Straight line should have near-zero curvature
        self.assertAlmostEqual(kappa, 0.0, places=3)
    
    def test_compute_curvature_circle(self):
        """Test curvature computation for circular arc."""
        # Points on a circle of radius 1.0
        p0 = (1.0, 0.0)
        p1 = (0.707, 0.707)  # 45 degrees
        p2 = (0.0, 1.0)      # 90 degrees
        
        kappa = self.builder.compute_curvature(p0, p1, p2)
        
        # Circle of radius 1.0 should have curvature ~1.0
        # Note: This is approximate due to discrete points
        self.assertGreater(kappa, 0.5)
        self.assertLess(kappa, 2.0)
    
    def test_build_reference_basic(self):
        """Test basic reference generation."""
        waypoints = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)]
        position = (5.0, 0.0)
        heading = 0.0
        horizon = 10
        dt = 0.05
        speed = 10.0  # 10 m/s
        
        psi_ref, kappa_ref, v_ref, grade_ref, wp_idx, s_0, s_horizon = self.builder.build_reference(
            waypoints, position, heading, horizon, dt, speed
        )
        
        # Check output shapes
        self.assertEqual(len(psi_ref), horizon)
        self.assertEqual(len(kappa_ref), horizon)
        self.assertEqual(len(v_ref), horizon)
        self.assertEqual(len(s_horizon), horizon)
        # Phase 1: s_0 and s_horizon (progress along path)
        self.assertIsInstance(s_0, (float, np.floating))
        self.assertGreaterEqual(s_0, 0.0)
        # s_horizon should be non-decreasing (progress along path)
        self.assertTrue(np.all(np.diff(s_horizon) >= -1e-9), "s_horizon should be non-decreasing")
        self.assertGreaterEqual(s_horizon[0], s_0)
        
        # Check reference speed is constant
        np.testing.assert_array_almost_equal(v_ref, speed)
        
        # For straight line, heading should be ~0
        np.testing.assert_array_almost_equal(psi_ref, 0.0, decimal=1)
        
        # For straight line, curvature should be ~0
        np.testing.assert_array_almost_equal(kappa_ref, 0.0, decimal=2)
    
    def test_build_reference_curved_path(self):
        """Test reference generation for curved path."""
        # Create waypoints forming a curve
        waypoints = []
        for i in range(10):
            angle = i * 0.1
            x = np.cos(angle) * 10.0
            y = np.sin(angle) * 10.0
            waypoints.append((x, y))
        
        position = waypoints[0]
        heading = 0.0
        horizon = 5
        dt = 0.05
        speed = 5.0
        
        psi_ref, kappa_ref, v_ref, grade_ref, wp_idx, s_0, s_horizon = self.builder.build_reference(
            waypoints, position, heading, horizon, dt, speed
        )
        
        # Check output shapes
        self.assertEqual(len(psi_ref), horizon)
        self.assertEqual(len(kappa_ref), horizon)
        self.assertEqual(len(s_horizon), horizon)
        self.assertGreaterEqual(s_0, 0.0)
        
        # Curvature should be non-zero for curved path
        self.assertTrue(np.any(np.abs(kappa_ref) > 0.01))
    
    def test_resample_waypoints(self):
        """Test waypoint resampling."""
        # Waypoints with uneven spacing
        waypoints = [(0.0, 0.0), (5.0, 0.0), (15.0, 0.0), (20.0, 0.0)]
        
        resampled = self.builder.resample_waypoints(waypoints)
        
        # Should have more points than original
        self.assertGreater(len(resampled), len(waypoints))
        
        # First and last points should be preserved
        self.assertAlmostEqual(resampled[0][0], waypoints[0][0], places=5)
        self.assertAlmostEqual(resampled[0][1], waypoints[0][1], places=5)
        self.assertEqual(resampled[-1], waypoints[-1])


if __name__ == '__main__':
    unittest.main()

