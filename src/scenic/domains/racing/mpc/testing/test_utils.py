"""Unit tests for utility functions.

Tests low-pass filter and other utilities.
"""

import unittest
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from scenic.domains.racing.mpc.utils import LowPassFilter


class TestLowPassFilter(unittest.TestCase):
    """Test cases for LowPassFilter."""
    
    def test_filter_step_response(self):
        """Test filter step response."""
        filter = LowPassFilter(cutoff_hz=1.0, dt=0.01, initial_value=0.0)
        
        # Step input
        output = filter.update(1.0)
        
        # Output should be between 0 and 1
        self.assertGreater(output, 0.0)
        self.assertLessEqual(output, 1.0)
    
    def test_filter_smoothing(self):
        """Test that filter smooths noisy input."""
        filter = LowPassFilter(cutoff_hz=1.0, dt=0.01, initial_value=0.0)
        
        # Noisy input
        inputs = [1.0, 0.5, 1.5, 0.8, 1.2]
        outputs = [filter.update(inp) for inp in inputs]
        
        # Outputs should be smoother (less variation) than inputs
        input_variance = np.var(inputs)
        output_variance = np.var(outputs)
        
        self.assertLess(output_variance, input_variance)
    
    def test_filter_reset(self):
        """Test filter reset functionality."""
        filter = LowPassFilter(cutoff_hz=1.0, dt=0.01, initial_value=0.0)
        
        # Update filter
        filter.update(1.0)
        
        # Reset
        filter.reset(0.5)
        
        # Next output should start from reset value
        output = filter.update(0.0)
        self.assertGreater(output, 0.0)  # Should be influenced by reset value


if __name__ == '__main__':
    unittest.main()

