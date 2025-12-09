#!/usr/bin/env python3
"""
Example: Finding FORWARD-ONLY waypoints (NO BACKTRACKING).

This demonstrates how to ensure your car ALWAYS finds forward waypoints
and NEVER backtracks, which is critical for racing performance.
"""

import sys
import math
from pathlib import Path

# Add tools directory to path
tools_path = Path(__file__).parent
sys.path.insert(0, str(tools_path))

from get_map_bounds import find_forward_waypoint, find_best_racing_waypoint
from laguna_seca_bounds import waypoints as laguna_seca_waypoints


def demonstrate_forward_only():
    """Demonstrate forward-only waypoint finding."""
    
    # Use real Laguna Seca waypoints
    waypoints = laguna_seca_waypoints
    print(f"[INFO] Using {len(waypoints)} real Laguna Seca waypoints")
    
    # Scenario: Car went off track
    # Use a position near the middle of the track
    # Pick a waypoint index around 1000 (roughly 1/3 through the track)
    last_known_index = 1000
    if last_known_index < len(waypoints):
        # Car position is slightly off to the side of the waypoint
        base_wp = waypoints[last_known_index]
        car_position = (base_wp[0] + 5.0, base_wp[1] + 5.0)  # 5m offset
        # Calculate heading towards next waypoint
        if last_known_index + 1 < len(waypoints):
            next_wp = waypoints[last_known_index + 1]
            dx = next_wp[0] - base_wp[0]
            dy = next_wp[1] - base_wp[1]
            car_heading = math.atan2(dy, dx)
        else:
            car_heading = math.pi / 4  # Default 45 degrees
    else:
        # Fallback if index is out of range
        car_position = (100.0, 200.0)
        car_heading = math.pi / 4
        last_known_index = 0
    
    print("=" * 70)
    print("FORWARD-ONLY WAYPOINT FINDING (NO BACKTRACKING)")
    print("=" * 70)
    print(f"Car position: {car_position}")
    print(f"Car heading: {math.degrees(car_heading):.1f}°")
    print(f"Last known waypoint index: {last_known_index}")
    print(f"Number of waypoints: {len(waypoints)}")
    print()
    
    # Method 1: Simple forward-only function (RECOMMENDED)
    print("-" * 70)
    print("METHOD 1: find_forward_waypoint() - Simple & Guaranteed Forward")
    print("-" * 70)
    result1 = find_forward_waypoint(
        car_position=car_position,
        waypoints=waypoints,
        last_known_index=last_known_index,
        car_heading=car_heading,
        max_search_distance=100.0
    )
    
    if result1:
        print(f"✓ Found forward waypoint!")
        print(f"  Index: {result1['index']} (forward from index {last_known_index})")
        print(f"  Waypoint: {result1['waypoint']}")
        print(f"  Distance: {result1['distance']:.2f} meters")
        print(f"  Forward score: {result1['forward_score']:.3f} (guaranteed > 0)")
        print(f"  ✓ GUARANTEED: This waypoint is forward, NO backtracking")
    else:
        print("✗ No forward waypoint found within search distance")
        print("  Try increasing max_search_distance")
    print()
    
    # Method 2: Full control with explicit forward_only=True
    print("-" * 70)
    print("METHOD 2: find_best_racing_waypoint() with forward_only=True")
    print("-" * 70)
    result2 = find_best_racing_waypoint(
        car_position=car_position,
        car_heading=car_heading,
        waypoints=waypoints,
        last_known_index=last_known_index,
        forward_only=True,  # STRICT: No backtracking
        forward_bias=0.9,   # Strong forward preference
        max_search_distance=100.0
    )
    
    if result2:
        print(f"✓ Found forward waypoint!")
        print(f"  Index: {result2['index']}")
        print(f"  Waypoint: {result2['waypoint']}")
        print(f"  Distance: {result2['distance']:.2f} meters")
        print(f"  Forward score: {result2['forward_score']:.3f}")
        print(f"  Heading alignment: {result2['heading_alignment']:.3f}")
        print(f"  ✓ GUARANTEED: This waypoint is forward, NO backtracking")
    print()
    
    # Verify forward guarantee
    print("=" * 70)
    print("VERIFICATION: Forward Guarantee")
    print("=" * 70)
    if result1:
        is_forward = result1['index'] >= last_known_index
        print(f"Waypoint index {result1['index']} >= Last known index {last_known_index}: {is_forward}")
        if is_forward:
            print("✓ CONFIRMED: Waypoint is forward (no backtracking)")
        else:
            print("⚠️  WARNING: This should never happen with forward_only=True")
    print()


def usage_in_scenic_forward_only():
    """Show how to use forward-only in Scenic behavior."""
    print("=" * 70)
    print("USAGE IN SCENIC BEHAVIOR (FORWARD-ONLY)")
    print("=" * 70)
    print("""
# In your Scenic behavior - ALWAYS use forward-only to avoid backtracking:

behavior RecoverToRacingLine():
    wp_last_idx = 0  # CRITICAL: Track last known waypoint index
    
    while True:
        # Check if car is out of bounds
        is_in_bounds = road.contains(self.position) if hasattr(road, 'contains') else True
        
        if not is_in_bounds:
            # Get waypoints
            wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
            
            if wp_list and len(wp_list) > 0:
                px = float(self.position.x)
                py = float(self.position.y)
                heading = float(self.heading) if hasattr(self, 'heading') else None
                
                # METHOD 1: Simple forward-only (RECOMMENDED)
                from tools.get_map_bounds import find_forward_waypoint
                result = find_forward_waypoint(
                    (px, py),
                    wp_list,
                    last_known_index=wp_last_idx,  # REQUIRED
                    car_heading=heading,           # Optional but recommended
                    max_search_distance=100.0
                )
                
                # METHOD 2: Full control (alternative)
                # from tools.get_map_bounds import find_best_racing_waypoint
                # result = find_best_racing_waypoint(
                #     (px, py), heading, wp_list,
                #     last_known_index=wp_last_idx,
                #     forward_only=True,  # CRITICAL: Prevents backtracking
                #     forward_bias=0.9,
                #     max_search_distance=100.0
                # )
                
                if result:
                    target_wp = result['waypoint']
                    wp_last_idx = result['index']  # Update last known index
                    
                    # Navigate towards target_wp
                    # This waypoint is GUARANTEED to be forward
                    # ... your steering/control logic here ...
                else:
                    # No forward waypoint found - may need to increase search distance
                    # or handle edge case (e.g., end of track)
                    pass
""")


def key_points():
    """Highlight key points about forward-only behavior."""
    print("=" * 70)
    print("KEY POINTS: Forward-Only Waypoint Finding")
    print("=" * 70)
    print("""
✓ GUARANTEED BEHAVIOR:
  - find_forward_waypoint() ALWAYS returns forward waypoints
  - find_best_racing_waypoint(forward_only=True) ALWAYS returns forward waypoints
  - NO backtracking is possible with these functions

✓ REQUIRED PARAMETER:
  - last_known_index: MUST be provided to determine forward direction
  - Track this value as you progress through waypoints
  - Update it whenever you reach a new waypoint

✓ DEFAULT BEHAVIOR:
  - forward_only=True (default) - Strictly no backtracking
  - forward_bias=0.9 (default) - Strong forward preference
  - These defaults ensure forward progress

✓ WHEN NO FORWARD WAYPOINT FOUND:
  - Function returns None
  - Try increasing max_search_distance
  - Check that last_known_index is accurate
  - May indicate end of track or very sparse waypoints

✓ PERFORMANCE:
  - Forward-only search is efficient (skips backward waypoints)
  - Faster than searching all waypoints
  - Better for racing (maintains forward progress)
""")


if __name__ == '__main__':
    demonstrate_forward_only()
    print()
    usage_in_scenic_forward_only()
    print()
    key_points()

