"""Unit tests for MPCConfig.

Tests configuration loading and parameter validation.
"""

import unittest
import sys
import os
import tempfile
import yaml

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))

from scenic.domains.racing.mpc.config import MPCConfig, load_mpc_config


class TestMPCConfig(unittest.TestCase):
    """Test cases for MPCConfig."""
    
    def test_config_defaults(self):
        """Test that config has reasonable defaults."""
        config_dict = {}
        config = MPCConfig(config_dict)
        
        # Check that defaults are set
        self.assertEqual(config.ctrl_period, 0.05)
        self.assertEqual(config.wheel_base, 2.9718)
        self.assertIsNotNone(config.w_ey)
    
    def test_config_custom_values(self):
        """Test config with custom values."""
        config_dict = {
            'ctrl_period': 0.1,
            'wheel_base': 3.0,
            'w_ey': 5.0,
        }
        config = MPCConfig(config_dict)
        
        self.assertEqual(config.ctrl_period, 0.1)
        self.assertEqual(config.wheel_base, 3.0)
        self.assertEqual(config.w_ey, 5.0)
    
    def test_config_adapt_to_timestep(self):
        """Test timestep adaptation."""
        config_dict = {'ctrl_period': 0.05}
        config = MPCConfig(config_dict)
        
        config.adapt_to_timestep(0.1)
        
        self.assertEqual(config.ctrl_period, 0.1)
    
    def test_load_mpc_config_yaml(self):
        """Test loading config from YAML file."""
        # Create temporary YAML file
        config_dict = {
            '/**:': {
                'ros__parameters': {
                    'ctrl_period': 0.05,
                    'wheel_base': 2.9718,
                    'w_ey': 2.0,
                }
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(config_dict, f)
            temp_path = f.name
        
        try:
            config = load_mpc_config(temp_path)
            
            self.assertEqual(config.ctrl_period, 0.05)
            self.assertEqual(config.wheel_base, 2.9718)
            self.assertEqual(config.w_ey, 2.0)
        finally:
            os.unlink(temp_path)


if __name__ == '__main__':
    unittest.main()

