"""Test runner script for MPC module tests.

Provides convenient way to run all tests with proper output formatting.
"""

import unittest
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'))


def run_all_tests():
    """Run all test suites."""
    # Discover and run all tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test modules
    test_modules = [
        'test_config',
        'test_utils',
        'test_reference_builder',
        'test_mpc_lateral',
    ]
    
    for module_name in test_modules:
        try:
            module = __import__(module_name, fromlist=[''])
            tests = loader.loadTestsFromModule(module)
            suite.addTests(tests)
            print(f"[OK] Loaded {module_name}")
        except ImportError as e:
            print(f"[FAIL] Failed to load {module_name}: {e}")
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Skipped: {len(result.skipped)}")
    
    if result.wasSuccessful():
        print("\n[SUCCESS] All tests passed!")
        return 0
    else:
        print("\n[FAILURE] Some tests failed!")
        return 1


if __name__ == '__main__':
    # Change to test directory
    os.chdir(os.path.dirname(__file__))
    sys.exit(run_all_tests())

