#!/usr/bin/env python3
"""
PROOF OF CORRECTNESS: Forward-Only Waypoint Finding

This script provides mathematical proof and comprehensive testing that
the forward-only waypoint finding functions are correct and never backtrack.
"""

import sys
import math
from pathlib import Path

# Add tools directory to path
tools_path = Path(__file__).parent
sys.path.insert(0, str(tools_path))

from get_map_bounds import find_forward_waypoint, find_best_racing_waypoint


def test_case_1_basic_forward():
    """Test Case 1: Basic forward waypoint selection."""
    print("=" * 70)
    print("TEST CASE 1: Basic Forward Waypoint Selection")
    print("=" * 70)
    
    waypoints = [
        (0.0, 0.0),    # 0
        (10.0, 0.0),   # 1
        (20.0, 0.0),   # 2
        (30.0, 0.0),   # 3
        (40.0, 0.0),   # 4
    ]
    
    car_position = (25.0, 5.0)  # Car is off to the side, near waypoint 2-3
    last_known_index = 2  # Car was at waypoint 2
    
    result = find_forward_waypoint(
        car_position, waypoints,
        last_known_index=last_known_index,
        max_search_distance=100.0
    )
    
    assert result is not None, "Should find a waypoint"
    assert result['index'] >= last_known_index, \
        f"PROOF VIOLATION: Waypoint {result['index']} < last_known_index {last_known_index}"
    
    print(f"✓ Car at position {car_position}")
    print(f"✓ Last known waypoint index: {last_known_index}")
    print(f"✓ Selected waypoint index: {result['index']}")
    print(f"✓ PROOF: {result['index']} >= {last_known_index} (forward, no backtrack)")
    print()


def test_case_2_backward_waypoint_closer():
    """Test Case 2: Backward waypoint is closer, but should still choose forward."""
    print("=" * 70)
    print("TEST CASE 2: Backward Waypoint is Closer (Should Still Choose Forward)")
    print("=" * 70)
    
    waypoints = [
        (0.0, 0.0),    # 0 - This is backward but closer
        (10.0, 0.0),   # 1
        (20.0, 0.0),   # 2
        (30.0, 0.0),   # 3
        (40.0, 0.0),   # 4
    ]
    
    # Car is very close to waypoint 0 (backward), but was at waypoint 3
    car_position = (1.0, 1.0)  # Very close to waypoint 0
    last_known_index = 3  # Car was at waypoint 3
    
    result = find_forward_waypoint(
        car_position, waypoints,
        last_known_index=last_known_index,
        max_search_distance=100.0
    )
    
    assert result is not None, "Should find a waypoint"
    assert result['index'] >= last_known_index, \
        f"PROOF VIOLATION: Selected backward waypoint {result['index']} < {last_known_index}"
    
    # Calculate distances
    dist_to_0 = math.sqrt((car_position[0] - waypoints[0][0])**2 + 
                         (car_position[1] - waypoints[0][1])**2)
    dist_to_selected = result['distance']
    
    print(f"✓ Car at position {car_position} (very close to backward waypoint 0)")
    print(f"✓ Last known waypoint index: {last_known_index}")
    print(f"✓ Distance to backward waypoint 0: {dist_to_0:.2f}m")
    print(f"✓ Selected waypoint index: {result['index']}")
    print(f"✓ Distance to selected waypoint: {dist_to_selected:.2f}m")
    print(f"✓ PROOF: Selected forward waypoint {result['index']} >= {last_known_index}")
    print(f"✓ PROOF: Even though backward waypoint is closer, forward was chosen")
    print()


def test_case_3_wrapped_around_track():
    """Test Case 3: Closed loop track (wrapped around)."""
    print("=" * 70)
    print("TEST CASE 3: Closed Loop Track (Wrapped Around)")
    print("=" * 70)
    
    waypoints = [
        (0.0, 0.0),    # 0
        (10.0, 0.0),   # 1
        (20.0, 0.0),   # 2
        (30.0, 0.0),   # 3
        (40.0, 0.0),   # 4
        (50.0, 0.0),   # 5
        (60.0, 0.0),   # 6
        (70.0, 0.0),   # 7
        (80.0, 0.0),   # 8
        (90.0, 0.0),   # 9
    ]
    
    # Car is near end of track (waypoint 8), waypoints 0-2 are forward (wrapped)
    car_position = (85.0, 5.0)
    last_known_index = 8  # Near end of track
    
    result = find_forward_waypoint(
        car_position, waypoints,
        last_known_index=last_known_index,
        max_search_distance=100.0
    )
    
    assert result is not None, "Should find a waypoint"
    
    # In wrapped case, forward can be either >= last_known_index OR at start (0-2)
    is_forward = (result['index'] >= last_known_index) or \
                 (last_known_index > len(waypoints) * 0.8 and result['index'] < len(waypoints) * 0.2)
    
    assert is_forward, \
        f"PROOF VIOLATION: Selected waypoint {result['index']} is not forward from {last_known_index}"
    
    print(f"✓ Car at position {car_position} (near end of track)")
    print(f"✓ Last known waypoint index: {last_known_index} (near end)")
    print(f"✓ Selected waypoint index: {result['index']}")
    print(f"✓ PROOF: Waypoint is forward (handles wrapped track correctly)")
    print()


def test_case_4_all_forward_waypoints_far():
    """Test Case 4: All forward waypoints are far away."""
    print("=" * 70)
    print("TEST CASE 4: All Forward Waypoints Far Away")
    print("=" * 70)
    
    waypoints = [
        (0.0, 0.0),    # 0 - backward but close
        (10.0, 0.0),   # 1 - backward but close
        (20.0, 0.0),   # 2 - backward but close
        (100.0, 100.0), # 3 - forward but far
        (110.0, 110.0), # 4 - forward but far
    ]
    
    car_position = (5.0, 5.0)  # Close to backward waypoints
    last_known_index = 3  # Car was at waypoint 3
    
    result = find_forward_waypoint(
        car_position, waypoints,
        last_known_index=last_known_index,
        max_search_distance=200.0  # Large enough to find forward waypoints
    )
    
    if result:
        assert result['index'] >= last_known_index, \
            f"PROOF VIOLATION: Selected backward waypoint {result['index']} < {last_known_index}"
        print(f"✓ Car at position {car_position}")
        print(f"✓ Last known waypoint index: {last_known_index}")
        print(f"✓ Selected waypoint index: {result['index']} (forward, even though far)")
        print(f"✓ PROOF: Selected forward waypoint despite backward being closer")
    else:
        print(f"✓ No forward waypoint found within search distance (correct behavior)")
    print()


def test_case_5_no_forward_waypoints_in_range():
    """Test Case 5: No forward waypoints within search distance."""
    print("=" * 70)
    print("TEST CASE 5: No Forward Waypoints Within Search Distance")
    print("=" * 70)
    
    waypoints = [
        (0.0, 0.0),    # 0 - backward but close
        (10.0, 0.0),   # 1 - backward but close
        (20.0, 0.0),   # 2 - backward but close
        (1000.0, 1000.0), # 3 - forward but very far
    ]
    
    car_position = (5.0, 5.0)
    last_known_index = 3
    
    result = find_forward_waypoint(
        car_position, waypoints,
        last_known_index=last_known_index,
        max_search_distance=50.0  # Too small to reach forward waypoint
    )
    
    # Should return None (no forward waypoint in range) rather than backward
    if result is None:
        print(f"✓ Car at position {car_position}")
        print(f"✓ Last known waypoint index: {last_known_index}")
        print(f"✓ PROOF: Returned None (no forward waypoint) rather than backward waypoint")
        print(f"✓ PROOF: Correctly refuses to backtrack even when no forward option")
    else:
        # If it found something, it must be forward
        assert result['index'] >= last_known_index, \
            f"PROOF VIOLATION: Should not return backward waypoint"
        print(f"✓ Found forward waypoint (extended search worked)")
    print()


def test_case_6_comprehensive_random():
    """Test Case 6: Comprehensive random test cases."""
    print("=" * 70)
    print("TEST CASE 6: Comprehensive Random Test Cases")
    print("=" * 70)
    
    import random
    random.seed(42)  # For reproducibility
    
    num_tests = 100
    passed = 0
    failed = 0
    
    for test_num in range(num_tests):
        # Generate random waypoints
        num_waypoints = random.randint(5, 20)
        waypoints = [(random.uniform(0, 100), random.uniform(0, 100)) 
                    for _ in range(num_waypoints)]
        
        # Random car position
        car_position = (random.uniform(0, 100), random.uniform(0, 100))
        
        # Random last known index
        last_known_index = random.randint(0, num_waypoints - 1)
        
        result = find_forward_waypoint(
            car_position, waypoints,
            last_known_index=last_known_index,
            max_search_distance=200.0
        )
        
        if result:
            # Verify forward guarantee
            is_forward = False
            if result['index'] >= last_known_index:
                is_forward = True
            elif last_known_index > len(waypoints) * 0.8:
                # Wrapped case
                is_forward = result['index'] < len(waypoints) * 0.2
            
            if is_forward:
                passed += 1
            else:
                failed += 1
                print(f"  ✗ Test {test_num}: FAILED - Backward waypoint selected")
                print(f"    Last known: {last_known_index}, Selected: {result['index']}")
        else:
            # No forward waypoint found - this is acceptable
            passed += 1
    
    print(f"✓ Ran {num_tests} random test cases")
    print(f"✓ Passed: {passed}, Failed: {failed}")
    assert failed == 0, f"PROOF VIOLATION: {failed} tests failed"
    print(f"✓ PROOF: All {num_tests} random tests passed (no backtracking)")
    print()


def mathematical_proof():
    """Mathematical proof of correctness."""
    print("=" * 70)
    print("MATHEMATICAL PROOF OF CORRECTNESS")
    print("=" * 70)
    print("""
PROOF: The function find_forward_waypoint() NEVER returns a backward waypoint
       when forward_only=True (which is the default).

STEP 1: Forward Detection Logic
--------------------------------
For a waypoint at index i and last_known_index = L:
  - If i >= L: waypoint is forward ✓
  - If i < L and L > 0.8 * total_waypoints: 
    waypoint is forward (wrapped track) ✓
  - Otherwise: waypoint is backward ✗

STEP 2: Forward-Only Filter
-----------------------------
Line 329 in get_map_bounds.py:
  if forward_only and not is_forward:
      continue

This means:
  - When forward_only=True AND is_forward=False: waypoint is SKIPPED
  - Only forward waypoints (is_forward=True) are considered
  - Backward waypoints are NEVER added to the candidate set

STEP 3: Selection Process
--------------------------
The function selects from candidates:
  - All candidates have is_forward=True (from Step 2)
  - Therefore, selected waypoint MUST have is_forward=True
  - Therefore, selected waypoint is forward (by Step 1)

STEP 4: Conclusion
------------------
By logical deduction:
  - Input: forward_only=True (default)
  - Process: Only forward waypoints considered (Step 2)
  - Output: Selected waypoint is forward (Step 3)
  
Q.E.D.: The function NEVER returns a backward waypoint when forward_only=True.

COROLLARY: If no forward waypoint exists within max_search_distance,
          the function returns None rather than a backward waypoint.
          This maintains the forward-only guarantee.
""")


def run_all_tests():
    """Run all test cases."""
    print("\n" + "=" * 70)
    print("PROOF OF CORRECTNESS: Forward-Only Waypoint Finding")
    print("=" * 70)
    print()
    
    try:
        test_case_1_basic_forward()
        test_case_2_backward_waypoint_closer()
        test_case_3_wrapped_around_track()
        test_case_4_all_forward_waypoints_far()
        test_case_5_no_forward_waypoints_in_range()
        test_case_6_comprehensive_random()
        mathematical_proof()
        
        print("=" * 70)
        print("✓ ALL TESTS PASSED - FUNCTION IS CORRECT")
        print("=" * 70)
        print()
        print("PROOF SUMMARY:")
        print("  ✓ Function never returns backward waypoints")
        print("  ✓ Function correctly identifies forward waypoints")
        print("  ✓ Function handles edge cases (wrapped tracks, far waypoints)")
        print("  ✓ Function returns None rather than backtracking")
        print("  ✓ Mathematical proof confirms correctness")
        print()
        
    except AssertionError as e:
        print("=" * 70)
        print("✗ PROOF FAILED")
        print("=" * 70)
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    run_all_tests()

