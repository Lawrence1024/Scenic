"""Utility functions for MPC controller."""

import numpy as np
from typing import Optional


class LowPassFilter:
    """First-order low-pass filter for smoothing control signals."""
    
    def __init__(self, cutoff_hz: float, dt: float, initial_value: float = 0.0):
        """Initialize low-pass filter.
        
        Args:
            cutoff_hz: Cutoff frequency (Hz)
            dt: Time step (seconds)
            initial_value: Initial filter output value
        """
        # Compute filter coefficient
        # y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        # alpha = dt * 2*pi*fc / (1 + dt * 2*pi*fc)
        omega_c = 2.0 * np.pi * cutoff_hz
        self.alpha = dt * omega_c / (1.0 + dt * omega_c)
        self.prev_output = initial_value
    
    def update(self, input_value: float) -> float:
        """Update filter with new input.
        
        Args:
            input_value: New input value
            
        Returns:
            Filtered output value
        """
        output = self.alpha * input_value + (1.0 - self.alpha) * self.prev_output
        self.prev_output = output
        return output
    
    def reset(self, value: float = 0.0):
        """Reset filter state.
        
        Args:
            value: Value to reset to
        """
        self.prev_output = value

